#include "PointPerfectClientMavlink.hpp"
#include <mavsdk/log_callback.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ctime>

#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <openssl/ssl.h>
#include <openssl/err.h>

static std::string base64_encode(const std::string& in)
{
	static const char* tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	std::string out;
	int val = 0, bits = -6;

	for (unsigned char c : in) {
		val = (val << 8) + c;
		bits += 8;

		while (bits >= 0) {
			out.push_back(tbl[(val >> bits) & 0x3F]);
			bits -= 6;
		}
	}

	if (bits > -6) {
		out.push_back(tbl[((val << 8) >> (bits + 8)) & 0x3F]);
	}

	while (out.size() % 4) {
		out.push_back('=');
	}

	return out;
}

PointPerfectClientMavlink::PointPerfectClientMavlink(const PointPerfectClientMavlink::Settings& settings)
	: _settings(settings)
{
	// Disable mavsdk noise
	mavsdk::log::subscribe([](...) {
		// https://mavsdk.mavlink.io/main/en/cpp/guide/logging.html
		return true;
	});

	const bool use_spartn = (_settings.correction_format == "spartn");

	// An explicit mountpoint wins; otherwise derive it from the format. The
	// *-MGA mountpoints prepend AssistNow ephemeris (UBX-MGA) for faster TTFF.
	if (!_settings.ntrip_mountpoint.empty()) {
		_mountpoint = _settings.ntrip_mountpoint;

	} else {
		_mountpoint = use_spartn ? "NEAR-SPARTN" : "NEAR-RTCM";

		if (_settings.use_mga) {
			_mountpoint += "-MGA";
		}
	}

	// An explicitly configured *-MGA mountpoint carries assistance too.
	_mga_expected = _mountpoint.ends_with("-MGA");

	std::cout << "Correction format: " << (use_spartn ? "SPARTN" : "RTCM") << std::endl;

	if (_mga_expected) {
		std::cout << "MGA assistance data enabled (u-blox receivers only)" << std::endl;

		if (!_settings.send_gga) {
			std::cout << "WARNING: the MGA mountpoints only start the correction stream "
				  << "after receiving a GGA report; enable send_gga." << std::endl;
		}
	}
}

PointPerfectClientMavlink::~PointPerfectClientMavlink()
{
	ntrip_disconnect();
}

void PointPerfectClientMavlink::stop()
{
	_should_exit = true;
}

void PointPerfectClientMavlink::run()
{
	std::cout << "Waiting for MAVSDK connection: " << _settings.mavsdk_connection_url << std::endl;

	while (!wait_for_mavsdk_connection(3)) {
		if (_should_exit) {
			return;
		}
	}

	// Track GPS_RAW_INT for (1) receiver-up gate before NTRIP and (2) GGA.
	_mavlink_passthrough->subscribe_message(
		MAVLINK_MSG_ID_GPS_RAW_INT,
	[this](const mavlink_message_t& message) {
		handle_gps_raw_int(message);
	});

	// Do not open the caster until the GPS driver is publishing. On MGA
	// mountpoints the assistance burst is one-shot at connect (and billed);
	// if inject is still blocked mid-receiver-config those bytes are dropped.
	if (!wait_for_gps_receiver()) {
		return;
	}

	uint8_t buffer[4096];

	while (!_should_exit) {

		_report_connect_failure = _settings.verbose || (_connect_failures % kConnectFailureLogEvery == 0);

		if (!ntrip_connect()) {
			_connect_failures++;

			if (_report_connect_failure) {
				std::cerr << "NTRIP connect failed (attempt " << _connect_failures
					  << "), retrying in 5s" << std::endl;
			}

			for (int i = 0; i < 5 && !_should_exit; i++) {
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}

			continue;
		}

		_connect_failures = 0;

		// Report position immediately: the caster only starts (and keeps)
		// streaming corrections once it has a GGA from this connection.
		auto last_gga = std::chrono::steady_clock::now() - std::chrono::seconds(10);
		_last_stream_data = std::chrono::steady_clock::now();
		_last_stats_log = _last_stream_data;
		_mga_messages_forwarded = 0;
		_mga_bytes_forwarded = 0;
		_mga_burst_reported = false;
		_stats = {};

		// Stream corrections until the connection drops or we're asked to exit.
		while (!_should_exit) {
			auto now = std::chrono::steady_clock::now();

			if (_settings.send_gga && now - last_gga >= std::chrono::seconds(10)) {
				send_gga();
				last_gga = now;
			}

			if (now - _last_stats_log >= kStatsInterval) {
				log_stats();
				_last_stats_log = now;
			}

			int n = socket_recv(buffer, sizeof(buffer));

			if (n > 0) {
				forward_corrections(buffer, n);

			} else if (n < 0) {
				std::cerr << "NTRIP connection lost" << std::endl;
				break;

			} else if (now - _last_stream_data >= kScannerIdleFlush) {
				// Idle: a partial frame candidate is not going to complete.
				if (_scanner.buffered() > 0) {
					_scanner.flush([this](const uint8_t* data, size_t length) { forward_raw(data, length); });
				}

				// The caster holds the corrections back until it has a GGA, so
				// the burst is usually followed by silence, not by the raw bytes
				// that would otherwise mark its end.
				report_mga_burst();
			}
		}

		// A burst cut short by the drop is still worth reporting.
		report_mga_burst();

		// Buffered partial data belongs to the old stream; drop it.
		_scanner.reset();
		ntrip_disconnect();
	}
}

bool PointPerfectClientMavlink::wait_for_mavsdk_connection(double timeout_s)
{
	auto config = mavsdk::Mavsdk::Configuration(1, MAV_COMP_ID_ONBOARD_COMPUTER2, true); // Emit heartbeats (Client)
	_mavsdk = std::make_shared<mavsdk::Mavsdk>(config);

	auto result = _mavsdk->add_any_connection(_settings.mavsdk_connection_url);

	if (result != mavsdk::ConnectionResult::Success) {
		return false;
	}

	auto system = _mavsdk->first_autopilot(timeout_s);

	if (!system) {
		return false;
	}

	std::cout << "Connected to autopilot" << std::endl;
	_mavlink_passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(system.value());

	return true;
}

bool PointPerfectClientMavlink::wait_for_gps_receiver()
{
	if (_gps_receiver_seen.load()) {
		return true;
	}

	std::cout << "Waiting for GPS_RAW_INT (receiver up) before NTRIP connect..." << std::endl;

	while (!_should_exit && !_gps_receiver_seen.load()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	if (_should_exit) {
		return false;
	}

	std::cout << "GPS receiver seen; safe to open NTRIP (MGA can be injected)" << std::endl;
	return true;
}

bool PointPerfectClientMavlink::ntrip_connect()
{
	if (_report_connect_failure) {
		std::cout << "Connecting to NTRIP caster: " << _settings.ntrip_host << ":" << _settings.ntrip_port
			  << "/" << _mountpoint << std::endl;
	}

	// Resolve and connect the TCP socket
	struct addrinfo hints = {};
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	struct addrinfo* result = nullptr;
	const std::string port_str = std::to_string(_settings.ntrip_port);

	if (getaddrinfo(_settings.ntrip_host.c_str(), port_str.c_str(), &hints, &result) != 0) {
		connect_log() << "Failed to resolve " << _settings.ntrip_host << std::endl;
		return false;
	}

	for (struct addrinfo* rp = result; rp != nullptr; rp = rp->ai_next) {
		_socket_fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);

		if (_socket_fd < 0) {
			continue;
		}

		if (connect(_socket_fd, rp->ai_addr, rp->ai_addrlen) == 0) {
			break;
		}

		close(_socket_fd);
		_socket_fd = -1;
	}

	freeaddrinfo(result);

	if (_socket_fd < 0) {
		connect_log() << "Failed to connect to caster" << std::endl;
		return false;
	}

	// Optional TLS handshake
	if (_settings.use_tls) {
		_ssl_ctx = SSL_CTX_new(TLS_client_method());

		if (!_ssl_ctx) {
			connect_log() << "Failed to create SSL context" << std::endl;
			ntrip_disconnect();
			return false;
		}

		SSL_CTX_set_default_verify_paths(_ssl_ctx);
		_ssl = SSL_new(_ssl_ctx);
		SSL_set_fd(_ssl, _socket_fd);
		SSL_set_tlsext_host_name(_ssl, _settings.ntrip_host.c_str()); // SNI

		if (SSL_connect(_ssl) != 1) {
			connect_log() << "TLS handshake failed" << std::endl;

			if (_report_connect_failure) {
				ERR_print_errors_fp(stderr);
			}

			ntrip_disconnect();
			return false;
		}
	}

	// Send the NTRIP request
	const std::string auth = base64_encode(_settings.ntrip_username + ":" + _settings.ntrip_password);
	std::string request =
		"GET /" + _mountpoint + " HTTP/1.1\r\n"
		"Host: " + _settings.ntrip_host + ":" + port_str + "\r\n"
		"Ntrip-Version: Ntrip/2.0\r\n"
		"User-Agent: NTRIP pointperfect-client-mavlink/1.0\r\n"
		"Authorization: Basic " + auth + "\r\n"
		"Connection: keep-alive\r\n"
		"\r\n";

	if (socket_send(request.data(), request.size()) <= 0) {
		connect_log() << "Failed to send NTRIP request" << std::endl;
		ntrip_disconnect();
		return false;
	}

	std::string leftover;

	if (!read_ntrip_response(leftover)) {
		ntrip_disconnect();
		return false;
	}

	std::cout << "NTRIP stream established" << std::endl;

	// Any bytes read past the response header are already correction data.
	if (!leftover.empty()) {
		forward_corrections(reinterpret_cast<const uint8_t*>(leftover.data()), leftover.size());
	}

	return true;
}

bool PointPerfectClientMavlink::read_ntrip_response(std::string& leftover)
{
	// Read until the end of the HTTP/ICY header ("\r\n\r\n").
	std::string header;
	uint8_t buffer[1024];
	auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);

	while (std::chrono::steady_clock::now() < deadline) {
		int n = socket_recv(buffer, sizeof(buffer));

		if (n < 0) {
			connect_log() << "Connection closed while reading NTRIP response" << std::endl;
			return false;
		}

		if (n == 0) {
			continue; // timeout, keep waiting until deadline
		}

		header.append(reinterpret_cast<const char*>(buffer), n);
		size_t end = header.find("\r\n\r\n");

		if (end != std::string::npos) {
			leftover = header.substr(end + 4);
			header.resize(end);
			break;
		}
	}

	// NTRIP v2 responds with "HTTP/1.1 200", v1 with "ICY 200 OK".
	if (header.find("200") == std::string::npos) {
		connect_log() << "NTRIP caster rejected the request:\n" << header << std::endl;
		return false;
	}

	return true;
}

void PointPerfectClientMavlink::ntrip_disconnect()
{
	if (_ssl) {
		SSL_shutdown(_ssl);
		SSL_free(_ssl);
		_ssl = nullptr;
	}

	if (_ssl_ctx) {
		SSL_CTX_free(_ssl_ctx);
		_ssl_ctx = nullptr;
	}

	if (_socket_fd >= 0) {
		close(_socket_fd);
		_socket_fd = -1;
	}
}

int PointPerfectClientMavlink::socket_send(const void* buf, size_t len)
{
	if (_socket_fd < 0) {
		return -1;
	}

	if (_ssl) {
		return SSL_write(_ssl, buf, len);
	}

	return ::send(_socket_fd, buf, len, MSG_NOSIGNAL);
}

int PointPerfectClientMavlink::socket_recv(void* buf, size_t len)
{
	if (_socket_fd < 0) {
		return -1;
	}

	// If TLS already has buffered plaintext, read it without blocking on select.
	bool data_ready = _ssl && SSL_pending(_ssl) > 0;

	if (!data_ready) {
		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(_socket_fd, &rfds);
		struct timeval tv = { 1, 0 }; // 1 second

		int s = select(_socket_fd + 1, &rfds, nullptr, nullptr, &tv);

		if (s == 0) {
			return 0; // timeout

		} else if (s < 0) {
			return (errno == EINTR) ? 0 : -1;
		}
	}

	if (_ssl) {
		int n = SSL_read(_ssl, buf, len);

		if (n > 0) {
			return n;
		}

		int err = SSL_get_error(_ssl, n);

		if (err == SSL_ERROR_WANT_READ || err == SSL_ERROR_WANT_WRITE) {
			return 0;
		}

		return -1;
	}

	ssize_t n = recv(_socket_fd, buf, len, 0);

	if (n > 0) {
		return n;

	} else if (n == 0) {
		return -1; // peer closed
	}

	if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
		return 0;
	}

	return -1;
}

void PointPerfectClientMavlink::handle_gps_raw_int(const mavlink_message_t& message)
{
	mavlink_gps_raw_int_t msg;
	mavlink_msg_gps_raw_int_decode(&message, &msg);

	// Any GPS_RAW_INT means the GPS driver is alive (even fix_type 0/1). Set
	// once so wait_for_gps_receiver() can release; do not require a fix.
	_gps_receiver_seen.store(true);

	// Require a continuous >=2D fix for kStableFixDuration before reporting
	// position to the NTRIP caster. Any drop below 2D resets the window.
	const auto now = std::chrono::steady_clock::now();

	if (msg.fix_type < kMinFixType) {
		if (_fix_reported) {
			std::cout << "GPS fix lost (fix_type=" << int(msg.fix_type) << ")" << std::endl;

		} else if (_settings.verbose && _fix_ok_since.has_value()) {
			std::cout << "GPS fix dropped before stable (fix_type=" << int(msg.fix_type)
				  << "), resetting stable-fix timer" << std::endl;
		}

		_fix_ok_since.reset();
		_fix_reported = false;

		std::lock_guard<std::mutex> lock(_position.lock);
		_position.valid = false;
		return;
	}

	if (!_fix_ok_since.has_value()) {
		_fix_ok_since = now;

		if (_settings.verbose) {
			std::cout << "GPS >=2D fix acquired (fix_type=" << int(msg.fix_type)
				  << "), waiting " << kStableFixDuration.count()
				  << "s for stability" << std::endl;
		}

		return;
	}

	if (now - *_fix_ok_since < kStableFixDuration) {
		return;
	}

	if (!_fix_reported) {
		_fix_reported = true;
		std::cout << "GPS fix stable (fix_type=" << int(msg.fix_type)
			  << "), reporting position to the caster" << std::endl;
	}

	std::lock_guard<std::mutex> lock(_position.lock);
	_position.lat_deg = double(msg.lat) / 1e7;
	_position.lon_deg = double(msg.lon) / 1e7;
	_position.alt_m = double(msg.alt) / 1e3;
	_position.fix_type = msg.fix_type;
	_position.satellites_visible = msg.satellites_visible;
	_position.eph = msg.eph;
	_position.valid = true;
}

uint8_t PointPerfectClientMavlink::nmea_quality_from_fix_type(uint8_t fix_type)
{
	// GPS_RAW_INT fix_type -> NMEA GGA quality indicator
	// 0-1: no fix, 2: 2D, 3: 3D, 4: DGPS, 5: RTK float, 6: RTK fixed
	switch (fix_type) {
	case 2:
	case 3:
		return 1; // GPS SPS

	case 4:
		return 2; // DGPS

	case 5:
		return 5; // RTK float

	case 6:
		return 4; // RTK fixed

	default:
		return 0; // invalid / no fix
	}
}

bool PointPerfectClientMavlink::build_gga(std::string& sentence)
{
	double lat, lon, alt;
	uint8_t fix_type;
	uint8_t satellites_visible;
	uint16_t eph;
	{
		std::lock_guard<std::mutex> lock(_position.lock);

		if (!_position.valid) {
			return false;
		}

		lat = _position.lat_deg;
		lon = _position.lon_deg;
		alt = _position.alt_m;
		fix_type = _position.fix_type;
		satellites_visible = _position.satellites_visible;
		eph = _position.eph;
	}

	const uint8_t quality = nmea_quality_from_fix_type(fix_type);

	// satellites_visible is UINT8_MAX when unknown
	const unsigned sats = (satellites_visible == UINT8_MAX) ? 0u
			      : std::min<unsigned>(satellites_visible, 99u);

	// eph is HDOP * 100; UINT16_MAX when unknown
	const double hdop = (eph == UINT16_MAX) ? 99.9 : (double(eph) / 100.0);

	// UTC time hhmmss.ss
	std::time_t t = std::time(nullptr);
	std::tm tm_utc;
	gmtime_r(&t, &tm_utc);

	char utc[16];
	snprintf(utc, sizeof(utc), "%02d%02d%02d.00", tm_utc.tm_hour, tm_utc.tm_min, tm_utc.tm_sec);

	// Latitude ddmm.mmmm
	char ns = (lat >= 0) ? 'N' : 'S';
	double abs_lat = std::fabs(lat);
	int lat_deg = int(abs_lat);
	double lat_min = (abs_lat - lat_deg) * 60.0;

	// Longitude dddmm.mmmm
	char ew = (lon >= 0) ? 'E' : 'W';
	double abs_lon = std::fabs(lon);
	int lon_deg = int(abs_lon);
	double lon_min = (abs_lon - lon_deg) * 60.0;

	char body[128];
	snprintf(body, sizeof(body),
		 "GPGGA,%s,%02d%07.4f,%c,%03d%07.4f,%c,%u,%02u,%.1f,%.1f,M,0.0,M,,",
		 utc, lat_deg, lat_min, ns, lon_deg, lon_min, ew,
		 unsigned(quality), sats, hdop, alt);

	// NMEA checksum: XOR of all bytes between '$' and '*'
	uint8_t checksum = 0;

	for (const char* p = body; *p; p++) {
		checksum ^= uint8_t(*p);
	}

	char full[160];
	snprintf(full, sizeof(full), "$%s*%02X\r\n", body, checksum);
	sentence = full;
	return true;
}

void PointPerfectClientMavlink::send_gga()
{
	std::string sentence;

	if (!build_gga(sentence)) {
		return; // no valid position yet
	}

	if (socket_send(sentence.data(), sentence.size()) <= 0) {
		std::cerr << "Failed to send GGA to caster" << std::endl;
	}
}

void PointPerfectClientMavlink::forward_corrections(const uint8_t* data, size_t length)
{
	if (length == 0) {
		return;
	}

	if (_settings.verbose) {
		std::cout << "\nReceived " << length << " correction bytes" << std::endl;
	}

	_stats.rx_bytes += length;
	_last_stream_data = std::chrono::steady_clock::now();

	if (!_mga_expected) {
		forward_raw(data, length); // no assistance in this stream to separate out
		return;
	}

	// A read can hold the tail of the assistance burst and the first correction
	// bytes. Stage them apart so each GPS_RTCM_DATA sequence carries one type
	// and the receiver never gets a correction fragment wedged between two
	// assistance messages.
	_mga_staging.clear();
	_mga_frame_lengths.clear();
	_raw_staging.clear();

	// UBX-MGA assistance (MGA mountpoints) is forwarded message-aligned and
	// paced; everything else passes through unmodified.
	_scanner.feed(data, length,
	[this](const uint8_t* frame, size_t frame_length) {
		_mga_staging.insert(_mga_staging.end(), frame, frame + frame_length);
		_mga_frame_lengths.push_back(frame_length);
	},
	[this](const uint8_t* raw, size_t raw_length) {
		_raw_staging.insert(_raw_staging.end(), raw, raw + raw_length);
	});

	// Assistance goes first: it is one-shot, billed, and paced to survive the
	// autopilot's injection queue, which unpaced correction bytes would crowd.
	size_t offset = 0;

	for (size_t frame_length : _mga_frame_lengths) {
		forward_ubx(&_mga_staging[offset], frame_length);
		offset += frame_length;
	}

	if (!_raw_staging.empty()) {
		forward_raw(_raw_staging.data(), _raw_staging.size());
	}
}

void PointPerfectClientMavlink::forward_ubx(const uint8_t* frame, size_t length)
{
	// The autopilot writes GPS_RTCM_DATA payloads to the receiver as-is, so
	// UBX-MGA reaches the receiver on the same path as the corrections.
	_mga_messages_forwarded++;
	_mga_bytes_forwarded += length;

	if (_settings.verbose) {
		char msg_id[16];
		snprintf(msg_id, sizeof(msg_id), "UBX-%02X-%02X", frame[2], frame[3]);
		std::cout << "Forwarding MGA message " << _mga_messages_forwarded
			  << " (" << msg_id << ", " << length << " bytes)" << std::endl;
	}

	send_sequence(frame, length);

	if (!_should_exit) {
		std::this_thread::sleep_for(kMgaMessageInterval);
	}
}

void PointPerfectClientMavlink::forward_raw(const uint8_t* data, size_t length)
{
	// The assistance burst is one-shot at connect, ahead of the corrections:
	// the first correction bytes mark its end.
	report_mga_burst();

	size_t offset = 0;

	while (offset < length) {
		const size_t current_length = std::min(length - offset, kMaxSequenceLength);
		send_sequence(data + offset, current_length);
		offset += current_length;
	}
}

void PointPerfectClientMavlink::send_sequence(const uint8_t* data, size_t length)
{
	mavlink_gps_rtcm_data_t msg = {};

	if (length <= MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN) {
		msg.len = length;
		msg.flags = (_sequence_id & 0x1F) << 3;
		memcpy(msg.data, data, length);
		send_mavlink_gps_rtcm_data(msg);

	} else {

		uint8_t fragment_id = 0;
		size_t start = 0;

		while (start < length) {
			size_t current_length = std::min(length - start, size_t(MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN));
			msg.flags = 1; // LSB set indicates message is fragmented
			msg.flags |= (fragment_id++ & 0x3) << 1; // Next 2 bits are fragment id
			msg.flags |= (_sequence_id & 0x1F) << 3; // Next 5 bits are sequence id
			msg.len = current_length;
			memcpy(&msg.data, data + start, current_length);
			send_mavlink_gps_rtcm_data(msg);
			start += current_length;
		}

		// A fragmented sequence ends with a fragment shorter than the field
		// size; append an empty one when the length is an exact multiple.
		if (length % MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN == 0 && fragment_id <= 3) {
			msg.flags = 1;
			msg.flags |= (fragment_id & 0x3) << 1;
			msg.flags |= (_sequence_id & 0x1F) << 3;
			msg.len = 0;
			send_mavlink_gps_rtcm_data(msg);
		}
	}

	_sequence_id++;

	if (_sequence_id >= 32) {
		_sequence_id = 0;
	}
}

void PointPerfectClientMavlink::send_mavlink_gps_rtcm_data(const mavlink_gps_rtcm_data_t& msg)
{
	if (_settings.verbose) {
		std::cout << "Sending GPS_RTCM_DATA: " << int(msg.len) << std::endl;
	}

	_stats.tx_messages++;
	_mavlink_passthrough->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
		mavlink_message_t message;

		mavlink_msg_gps_rtcm_data_encode_chan(
			mavlink_address.system_id,
			mavlink_address.component_id,
			channel,
			&message,
			&msg);
		return message;
	});
}

void PointPerfectClientMavlink::log_stats()
{
	// Logged unconditionally: a window with no corrections means the caster
	// went quiet, which is exactly what a stalled stream looks like.
	std::cout << "Corrections: " << _stats.rx_bytes << " bytes received, "
		  << _stats.tx_messages << " GPS_RTCM_DATA sent (last "
		  << kStatsInterval.count() << "s)" << std::endl;

	_stats = {};
}

std::ostream& PointPerfectClientMavlink::connect_log()
{
	// A stream with no buffer swallows everything written to it.
	static std::ostream discarded(nullptr);
	return _report_connect_failure ? std::cerr : discarded;
}

void PointPerfectClientMavlink::report_mga_burst()
{
	if (_mga_burst_reported || _mga_messages_forwarded == 0) {
		return;
	}

	_mga_burst_reported = true;
	std::cout << "MGA assistance: forwarded " << _mga_messages_forwarded << " messages ("
		  << _mga_bytes_forwarded << " bytes)" << std::endl;
}
