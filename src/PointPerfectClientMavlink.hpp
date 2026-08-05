#pragma once

#include <atomic>
#include <chrono>
#include <iosfwd>
#include <mutex>
#include <optional>
#include <string>
#include <memory>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "UbxFrameScanner.hpp"

// Forward declarations to keep OpenSSL out of the header
typedef struct ssl_st SSL;
typedef struct ssl_ctx_st SSL_CTX;

class PointPerfectClientMavlink
{
public:
	struct Settings {
		std::string mavsdk_connection_url;
		std::string ntrip_host;
		int ntrip_port;
		// Correction format: "rtcm" (default, supported by PX4) or "spartn".
		std::string correction_format;
		// Optional explicit mountpoint. When empty it is derived from
		// correction_format (NEAR-RTCM / NEAR-SPARTN).
		std::string ntrip_mountpoint;
		std::string ntrip_username;
		std::string ntrip_password;
		bool use_tls;
		bool send_gga;
		// Request AssistNow (MGA) startup assistance for faster TTFF by using
		// the *-MGA mountpoints. u-blox receivers only.
		bool use_mga;
		// Log every correction chunk and every forwarded message instead of a
		// periodic throughput summary.
		bool verbose;
	};

	PointPerfectClientMavlink(const Settings& settings);
	~PointPerfectClientMavlink();

	void run();
	void stop();

private:
	// GPS_RAW_INT fix_type: 0-1 no fix, 2 = 2D, 3 = 3D, ...
	static constexpr uint8_t kMinFixType = 2;
	static constexpr std::chrono::seconds kStableFixDuration{3};

	// GPS_RTCM_DATA carries at most 4 fragments of 180 bytes per sequence.
	static constexpr size_t kMaxSequenceLength = 4 * MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN;

	// Pacing for the connect-time UBX-MGA burst (several KB at once): PX4
	// drains gps_inject_data from an 8-deep queue, so an unpaced burst drops
	// chunks before they reach the receiver.
	static constexpr std::chrono::milliseconds kMgaMessageInterval{20};

	// A partial frame candidate is flushed as raw data if the stream idles.
	static constexpr std::chrono::seconds kScannerIdleFlush{2};

	// How often the throughput summary is logged while streaming. It doubles as
	// a liveness signal, so it is logged even when nothing arrived.
	static constexpr std::chrono::seconds kStatsInterval{30};

	// Connect failures repeat every 5s (no network at boot, bad credentials);
	// log one attempt per minute rather than each one.
	static constexpr unsigned kConnectFailureLogEvery = 12;

	bool wait_for_mavsdk_connection(double timeout_s);
	// Blocks until the first GPS_RAW_INT (any fix_type), meaning the GPS
	// driver is up and inject is safe. Returns false if exit was requested.
	bool wait_for_gps_receiver();
	void handle_gps_raw_int(const mavlink_message_t& message);

	// NTRIP transport
	bool ntrip_connect();
	void ntrip_disconnect();
	int socket_send(const void* buf, size_t len);
	int socket_recv(void* buf, size_t len); // >0 bytes, 0 timeout, -1 error/closed
	bool read_ntrip_response(std::string& leftover);

	// NMEA GGA position report sent to the caster
	static uint8_t nmea_quality_from_fix_type(uint8_t fix_type);
	bool build_gga(std::string& sentence);
	void send_gga();

	// Corrections forwarding
	void forward_corrections(const uint8_t* data, size_t length);
	void forward_ubx(const uint8_t* frame, size_t length);
	void forward_raw(const uint8_t* data, size_t length);
	void send_sequence(const uint8_t* data, size_t length); // length <= kMaxSequenceLength
	void send_mavlink_gps_rtcm_data(const mavlink_gps_rtcm_data_t& msg);

	// Logging
	void log_stats();         // periodic throughput summary, resets the counters
	void report_mga_burst();  // one-shot, once the assistance burst is over
	std::ostream& connect_log(); // stderr, or discarded while a retry is throttled

	// MAVSDK
	std::shared_ptr<mavsdk::Mavsdk> _mavsdk;
	std::shared_ptr<mavsdk::MavlinkPassthrough> _mavlink_passthrough;
	uint8_t _sequence_id = 0;

	// NTRIP socket / TLS
	unsigned _connect_failures = 0; // consecutive; reset once a stream is up
	bool _report_connect_failure = true; // this attempt's turn to be logged
	int _socket_fd = -1;
	SSL* _ssl = nullptr;
	SSL_CTX* _ssl_ctx = nullptr;

	// Latest vehicle position + fix metadata from GPS_RAW_INT (for GGA)
	struct GlobalPosition {
		double lat_deg;
		double lon_deg;
		double alt_m;
		uint8_t fix_type;
		uint8_t satellites_visible;
		uint16_t eph; // HDOP * 100; UINT16_MAX if unknown
		bool valid;
		std::mutex lock;
	} _position = {};

	// Tracks continuous >=2D fix so we only report position after a stable window.
	std::optional<std::chrono::steady_clock::time_point> _fix_ok_since;
	// True once the stable window elapsed and the position is being reported.
	// Fix acquisition flaps until the receiver settles, so only the transitions
	// of this flag are logged; the rest is verbose-only.
	bool _fix_reported = false;

	// Set on the first GPS_RAW_INT of any fix_type. Used to delay NTRIP connect
	// (and therefore the one-shot MGA burst) until the receiver is configured
	// enough that the autopilot will inject GPS_RTCM_DATA to the device.
	std::atomic<bool> _gps_receiver_seen{false};

	// Splits UBX-MGA assistance out of the correction stream (MGA mountpoints).
	UbxFrameScanner _scanner;
	std::chrono::steady_clock::time_point _last_stream_data{};
	unsigned _mga_messages_forwarded = 0;
	size_t _mga_bytes_forwarded = 0;
	bool _mga_burst_reported = false;

	// Throughput since the last summary. Only touched from the run() thread.
	struct Stats {
		size_t rx_bytes;
		unsigned tx_messages;
	} _stats = {};
	std::chrono::steady_clock::time_point _last_stats_log{};

	Settings _settings;
	std::string _mountpoint; // resolved from settings (explicit or format-derived)
	std::atomic<bool> _should_exit = false;
};
