#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <memory>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

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
	};

	PointPerfectClientMavlink(const Settings& settings);
	~PointPerfectClientMavlink();

	void run();
	void stop();

private:
	// GPS_RAW_INT fix_type: 0-1 no fix, 2 = 2D, 3 = 3D, ...
	static constexpr uint8_t kMinFixType = 2;
	static constexpr std::chrono::seconds kStableFixDuration{3};

	bool wait_for_mavsdk_connection(double timeout_s);
	void handle_gps_raw_int(const mavlink_message_t& message);

	// NTRIP transport
	bool ntrip_connect();
	void ntrip_disconnect();
	int socket_send(const void* buf, size_t len);
	int socket_recv(void* buf, size_t len); // >0 bytes, 0 timeout, -1 error/closed
	bool read_ntrip_response(std::string& leftover);

	// NMEA GGA position report sent to the caster
	bool build_gga(std::string& sentence);
	void send_gga();

	// Corrections forwarding
	void forward_corrections(const uint8_t* data, size_t length);
	void send_mavlink_gps_rtcm_data(const mavlink_gps_rtcm_data_t& msg);

	// MAVSDK
	std::shared_ptr<mavsdk::Mavsdk> _mavsdk;
	std::shared_ptr<mavsdk::MavlinkPassthrough> _mavlink_passthrough;
	uint8_t _sequence_id = 0;

	// NTRIP socket / TLS
	int _socket_fd = -1;
	SSL* _ssl = nullptr;
	SSL_CTX* _ssl_ctx = nullptr;

	// Latest vehicle position (WGS-84), used to build the GGA sentence
	struct GlobalPosition {
		double lat_deg;
		double lon_deg;
		double alt_m;
		bool valid;
		std::mutex lock;
	} _position = {};

	// Tracks continuous >=2D fix so we only report position after a stable window.
	std::optional<std::chrono::steady_clock::time_point> _fix_ok_since;

	Settings _settings;
	std::string _mountpoint; // resolved from settings (explicit or format-derived)
	std::atomic<bool> _should_exit = false;
};
