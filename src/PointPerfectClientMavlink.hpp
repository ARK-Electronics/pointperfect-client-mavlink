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
	void stop();

	// mqtt::callback interface
	void connected(const std::string& cause) override;
	void connection_lost(const std::string& cause) override;
	void message_arrived(mqtt::const_message_ptr msg) override;

private:
	bool wait_for_mavsdk_connection(double timeout_s);
	bool connect_pointperfect();
	void subscribe_topics();

	// Forward a blob of correction bytes (SPARTN data or UBX keys) to the
	// autopilot as one or more GPS_RTCM_DATA messages.
	void forward_corrections(const uint8_t* data, size_t length);
	void send_mavlink_gps_rtcm_data(const mavlink_gps_rtcm_data_t& msg);

	// MAVSDK
	std::shared_ptr<mavsdk::Mavsdk> _mavsdk;
	std::shared_ptr<mavsdk::MavlinkPassthrough> _mavlink_passthrough;
	uint8_t _sequence_id = 0;
	// PointPerfect MQTT
	std::shared_ptr<mqtt::async_client> _mqtt_client;
	// Other
	Settings _settings;
	std::atomic<bool> _should_exit = false;
};
