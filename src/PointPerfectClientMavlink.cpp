#include "PointPerfectClientMavlink.hpp"
#include <mavsdk/log_callback.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <cerrno>
#include <cmath>
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

	// An explicit mountpoint wins; otherwise derive it from the format.
	if (!_settings.ntrip_mountpoint.empty()) {
		_mountpoint = _settings.ntrip_mountpoint;

	} else {
		_mountpoint = use_spartn ? "NEAR-SPARTN" : "NEAR-RTCM";
	}

	std::cout << "Correction format: " << (use_spartn ? "SPARTN" : "RTCM") << std::endl;

	if (use_spartn) {
		std::cout << "WARNING: PX4 currently only injects/parses RTCM. SPARTN requires "
			  << "receiver/firmware support for GPS_RTCM_DATA-carried SPARTN." << std::endl;
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

	// Track the vehicle position so we can report it to the NTRIP caster.
	_mavlink_passthrough->subscribe_message(
		MAVLINK_MSG_ID_GPS_RAW_INT,
	[this](const mavlink_message_t& message) {
		handle_gps_raw_int(message);
	});

	uint8_t buffer[4096];
	auto last_gga = std::chrono::steady_clock::now() - std::chrono::seconds(10);

	while (!_should_exit) {

		if (!ntrip_connect()) {
			std::cerr << "NTRIP connect failed, retrying in 5s" << std::endl;

			for (int i = 0; i < 5 && !_should_exit; i++) {
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}

			continue;
		}

		// Stream corrections until the connection drops or we're asked to exit.
		while (!_should_exit) {
			auto now = std::chrono::steady_clock::now();

			if (_settings.send_gga && now - last_gga >= std::chrono::seconds(10)) {
				send_gga();
				last_gga = now;
			}

			int n = socket_recv(buffer, sizeof(buffer));

			if (n > 0) {
				forward_corrections(buffer, n);

			} else if (n < 0) {
				std::cerr << "NTRIP connection lost" << std::endl;
				break;
			}
		}

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

bool PointPerfectClientMavlink::ntrip_connect()
{
	std::cout << "Connecting to NTRIP caster: " << _settings.ntrip_host << ":" << _settings.ntrip_port
		  << "/" << _mountpoint << std::endl;

	// Resolve and connect the TCP socket
	struct addrinfo hints = {};
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	struct addrinfo* result = nullptr;
	const std::string port_str = std::to_string(_settings.ntrip_port);

	if (getaddrinfo(_settings.ntrip_host.c_str(), port_str.c_str(), &hints, &result) != 0) {
		std::cerr << "Failed to resolve " << _settings.ntrip_host << std::endl;
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
		std::cerr << "Failed to connect to caster" << std::endl;
		return false;
	}

	// Optional TLS handshake
	if (_settings.use_tls) {
		_ssl_ctx = SSL_CTX_new(TLS_client_method());

		if (!_ssl_ctx) {
			std::cerr << "Failed to create SSL context" << std::endl;
			ntrip_disconnect();
			return false;
		}

		SSL_CTX_set_default_verify_paths(_ssl_ctx);
		_ssl = SSL_new(_ssl_ctx);
		SSL_set_fd(_ssl, _socket_fd);
		SSL_set_tlsext_host_name(_ssl, _settings.ntrip_host.c_str()); // SNI

		if (SSL_connect(_ssl) != 1) {
			std::cerr << "TLS handshake failed" << std::endl;
			ERR_print_errors_fp(stderr);
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
		std::cerr << "Failed to send NTRIP request" << std::endl;
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
			std::cerr << "Connection closed while reading NTRIP response" << std::endl;
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
		std::cerr << "NTRIP caster rejected the request:\n" << header << std::endl;
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

	if (msg.lat == 0 && msg.lon == 0) {
		return;
	}

	std::lock_guard<std::mutex> lock(_position.lock);
	_position.lat_deg = double(msg.lat) / 1e7;
	_position.lon_deg = double(msg.lon) / 1e7;
	_position.alt_m = double(msg.alt) / 1e3;
	_position.valid = true;
}

bool PointPerfectClientMavlink::build_gga(std::string& sentence)
{
	double lat, lon, alt;
	{
		std::lock_guard<std::mutex> lock(_position.lock);

		if (!_position.valid) {
			return false;
		}

		lat = _position.lat_deg;
		lon = _position.lon_deg;
		alt = _position.alt_m;
	}

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
		 "GPGGA,%s,%02d%07.4f,%c,%03d%07.4f,%c,1,12,1.0,%.1f,M,0.0,M,,",
		 utc, lat_deg, lat_min, ns, lon_deg, lon_min, ew, alt);

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

	std::cout << "\nReceived " << length << " correction bytes" << std::endl;

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
	}

	_sequence_id++;

	if (_sequence_id >= 32) {
		_sequence_id = 0;
	}
}

void PointPerfectClientMavlink::send_mavlink_gps_rtcm_data(const mavlink_gps_rtcm_data_t& msg)
{
	std::cout << "Sending GPS_RTCM_DATA: " << int(msg.len) << std::endl;
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
