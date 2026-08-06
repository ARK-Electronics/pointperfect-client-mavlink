#pragma once

#include <atomic>
#include <chrono>
#include <iosfwd>
#include <mutex>
#include <optional>
#include <string>
#include <memory>
#include <vector>

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

	// The autopilot's inject queue is 8-16 deep, drains only between receiver
	// reads, and drops silently; 50ms covers a 16-deep queue across an 800ms stall.
	static constexpr std::chrono::milliseconds kMgaMessageInterval{50};

	// A partial frame candidate is flushed as raw data if the stream idles.
	static constexpr std::chrono::seconds kScannerIdleFlush{2};

	// Logged even when nothing arrived, so a stalled caster shows.
	static constexpr std::chrono::seconds kStatsInterval{30};

	// Failed connects retry every 5s; log one per minute.
	static constexpr unsigned kConnectFailureLogEvery = 12;

	// Receiver gone once GPS_RAW_INT stops this long (PX4 publishes only on a
	// parsed report).
	static constexpr std::chrono::seconds kGpsReceiverTimeout{3};

	// AssistNow ephemeris only lives a few hours; older caches are refetched.
	static constexpr std::chrono::hours kMgaCacheMaxAge{2};

	// A stable fix lost this long means lost ephemeris; a flap recovers faster.
	static constexpr std::chrono::seconds kFixLostReplayDelay{10};

	bool wait_for_mavsdk_connection(double timeout_s);
	// Blocks until GPS_RAW_INT arrives (driver up, inject safe); false on exit.
	bool wait_for_gps_receiver();
	void handle_gps_raw_int(const mavlink_message_t& message);
	bool gps_receiver_up() const;

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

	// The burst is billed per fetch: fetch once, serve restarts from memory.
	void mark_mga_cache_complete();
	void replay_mga_cache(); // paced; leaves the retry pending if cut short

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

	// Continuous >=2D fix window gating the GGA report.
	std::optional<std::chrono::steady_clock::time_point> _fix_ok_since;
	// Position is being reported; only transitions are logged (acquisition flaps).
	bool _fix_reported = false;

	// steady_clock ticks at the last real GPS_RAW_INT, 0 before the first —
	// the receiver-presence signal. Callback thread writes, run() reads.
	std::atomic<int64_t> _last_gps_raw_int{0};

	// Splits UBX-MGA assistance out of the correction stream (MGA mountpoints).
	UbxFrameScanner _scanner;
	bool _fetching_assistance = false; // this connection is on the MGA mountpoint
	// Per-read staging keeping assistance and corrections in separate sequences.
	std::vector<uint8_t> _mga_staging;
	std::vector<size_t> _mga_frame_lengths;
	std::vector<uint8_t> _raw_staging;
	std::chrono::steady_clock::time_point _last_stream_data{};
	unsigned _mga_messages_forwarded = 0;
	size_t _mga_bytes_forwarded = 0;
	bool _mga_burst_reported = false;

	// The fetched burst, one UBX-MGA message per entry; complete only once the
	// stream moved past it (a fetch cut short is refetched).
	std::vector<std::vector<uint8_t>> _mga_cache;
	bool _mga_cache_complete = false;
	std::chrono::steady_clock::time_point _mga_cache_time{};
	// Receiver restarted; it is owed a replay.
	std::atomic<bool> _mga_replay_pending{false};
	// When a stable fix was lost, 0 while fixed; outlasting kFixLostReplayDelay
	// schedules a replay.
	std::atomic<int64_t> _fix_lost_since{0};
	// Receiver time seen; time going away again means it restarted (a flap
	// keeps time).
	bool _gps_time_seen = false;

	// Throughput since the last summary. Only touched from the run() thread.
	struct Stats {
		size_t rx_bytes;
		unsigned tx_messages;
	} _stats = {};
	std::chrono::steady_clock::time_point _last_stats_log{};

	Settings _settings;
	std::string _mountpoint;     // corrections mountpoint (explicit or format-derived)
	std::string _mountpoint_mga; // assistance mountpoint; empty when MGA is off
	std::atomic<bool> _should_exit = false;
};
