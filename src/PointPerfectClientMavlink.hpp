#pragma once

#include <atomic>
#include <string>
#include <memory>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mqtt/async_client.h>

class PointPerfectClientMavlink : public virtual mqtt::callback
{
public:
	struct Settings {
		std::string mavsdk_connection_url;
		std::string mqtt_server_uri;
		std::string mqtt_client_id;
		std::string client_cert_path;
		std::string client_key_path;
		std::string root_ca_path;
		std::string region;
		bool enable_key_distribution;
	};

	PointPerfectClientMavlink(const Settings& settings);

	void run();

	// Signal-safe: only sets the exit flag. Cleanup happens in run().
	void stop();

	// mqtt::callback interface
	void connected(const std::string& cause) override;
	void connection_lost(const std::string& cause) override;
	void message_arrived(mqtt::const_message_ptr msg) override;

private:
	// GPS_RTCM_DATA carries a 2-bit fragment id, so one sequence is at most
	// 4 fragments of 180 bytes. Frames of this size or larger cannot be
	// transported (an exact multiple of 180 also needs a zero-length
	// terminating fragment).
	static constexpr size_t MAX_SEQUENCE_LENGTH = 4 * MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN;

	bool wait_for_mavsdk_connection(double timeout_s);
	bool connect_pointperfect();
	void disconnect_pointperfect();
	void subscribe_topics();
	void print_statistics();

	// Forward a single frame (SPARTN or UBX) to the autopilot as one
	// GPS_RTCM_DATA sequence, fragmenting if necessary.
	void forward_frame(const uint8_t* data, size_t length);
	void send_mavlink_gps_rtcm_data(const mavlink_gps_rtcm_data_t& msg);

	// MAVSDK
	std::shared_ptr<mavsdk::Mavsdk> _mavsdk;
	std::shared_ptr<mavsdk::MavlinkPassthrough> _mavlink_passthrough;
	uint8_t _sequence_id = 0;
	// PointPerfect MQTT
	std::shared_ptr<mqtt::async_client> _mqtt_client;
	// Statistics, written from the MQTT callback thread and read in run()
	std::atomic<uint32_t> _messages_received {0};
	std::atomic<uint32_t> _frames_forwarded {0};
	std::atomic<uint32_t> _frames_dropped {0};
	std::atomic<uint64_t> _bytes_forwarded {0};
	// Other
	Settings _settings;
	std::atomic<bool> _should_exit = false;
};
