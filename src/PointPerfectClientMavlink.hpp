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

	// Pacing for the connect-time UBX-MGA burst (several KB at once). The
	// autopilot buffers injected corrections in a uORB queue - 8 deep on PX4
	// 1.16 and earlier (gps_inject_data), 16 since (rtcm_corrections) - and the
	// GPS driver only drains it between receiver reads, which stalls entirely
	// while the driver is configuring the receiver. Overflow is silent and
	// costs the oldest message in the queue, which for this burst is the
	// UBX-MGA-INI the rest of the assistance is useless without. 50ms keeps a
	// 16-deep queue covered across an 800ms stall.
	static constexpr std::chrono::milliseconds kMgaMessageInterval{50};

	// A partial frame candidate is flushed as raw data if the stream idles.
	static constexpr std::chrono::seconds kScannerIdleFlush{2};

	// How often the throughput summary is logged while streaming. It doubles as
	// a liveness signal, so it is logged even when nothing arrived.
	static constexpr std::chrono::seconds kStatsInterval{30};

	// Connect failures repeat every 5s (no network at boot, bad credentials);
	// log one attempt per minute rather than each one.
	static constexpr unsigned kConnectFailureLogEvery = 12;

	// The receiver is gone once GPS_RAW_INT stops for this long. PX4 publishes
	// only on a parsed report, so the stream stops outright when the receiver
	// drops or the driver goes back to configuring it — that is distinct from a
	// receiver that is up and reporting fix_type 0.
	static constexpr std::chrono::seconds kGpsReceiverTimeout{3};

	// AssistNow ephemeris is only valid for a few hours; a cache past this age
	// is refetched (billed) instead of replayed.
	static constexpr std::chrono::hours kMgaCacheMaxAge{2};

	bool wait_for_mavsdk_connection(double timeout_s);
	// Blocks until the first GPS_RAW_INT (any fix_type), meaning the GPS
	// driver is up and inject is safe. Returns false if exit was requested.
	bool wait_for_gps_receiver();
	void handle_gps_raw_int(const mavlink_message_t& message);
	// True while GPS_RAW_INT has arrived within kGpsReceiverTimeout.
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

	// MGA cache: the assistance burst is billed per fetch, so it is fetched once
	// and receiver restarts are served from memory over the plain mountpoint.
	void mark_mga_cache_complete(); // the stream moved past the burst: it is whole
	void replay_mga_cache();        // paced replay; leaves the retry pending if cut short

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

	// steady_clock ticks at the last GPS_RAW_INT of any fix_type, 0 before the
	// first one. Written from the MAVSDK callback thread, read by run(). Whether
	// it is recent is the whole receiver-presence signal: NTRIP stays closed
	// until the driver is publishing, so the one-shot MGA burst is not injected
	// while the autopilot is still configuring the receiver and dropping it.
	std::atomic<int64_t> _last_gps_raw_int{0};

	// Splits UBX-MGA assistance out of the correction stream (MGA mountpoints).
	UbxFrameScanner _scanner;
	bool _fetching_assistance = false; // this connection is on the MGA mountpoint
	// Per-read staging that keeps assistance and correction bytes in separate
	// GPS_RTCM_DATA sequences. Reused so a read costs no allocation.
	std::vector<uint8_t> _mga_staging;
	std::vector<size_t> _mga_frame_lengths;
	std::vector<uint8_t> _raw_staging;
	std::chrono::steady_clock::time_point _last_stream_data{};
	unsigned _mga_messages_forwarded = 0;
	size_t _mga_bytes_forwarded = 0;
	bool _mga_burst_reported = false;

	// The assistance burst, one complete UBX-MGA message per entry, as received
	// from the fetch. Complete only once the stream moved past the burst — a
	// fetch cut short (receiver away, connection drop) is refetched instead.
	std::vector<std::vector<uint8_t>> _mga_cache;
	bool _mga_cache_complete = false;
	std::chrono::steady_clock::time_point _mga_cache_time{};
	// The receiver restarted and came back without ephemeris; serve it from the
	// cache. Set from the MAVSDK callback thread, consumed by run().
	std::atomic<bool> _mga_replay_pending{false};
	// GPS_RAW_INT carried a nonzero time_usec: the receiver knows UTC time. A
	// report without time after this means the receiver restarted — it keeps
	// time across a fix flap, and PX4's driver can reconfigure a reset receiver
	// faster than the publication-gap watchdog can notice.
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
