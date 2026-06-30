#include "PointPerfectClientMavlink.hpp"
#include <mavsdk/log_callback.h>
#include <iostream>

PointPerfectClientMavlink::PointPerfectClientMavlink(const PointPerfectClientMavlink::Settings& settings)
	: _settings(settings)
{
	// Disable mavsdk noise
	mavsdk::log::subscribe([](...) {
		// https://mavsdk.mavlink.io/main/en/cpp/guide/logging.html
		return true;
	});
}

void PointPerfectClientMavlink::stop()
{
	if (_mqtt_client && _mqtt_client->is_connected()) {
		try {
			_mqtt_client->disconnect()->wait();

		} catch (const mqtt::exception& e) {
			std::cerr << "MQTT disconnect error: " << e.what() << std::endl;
		}
	}

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

	if (!connect_pointperfect()) {
		std::cerr << "Failed to connect to PointPerfect" << std::endl;
		return;
	}

	// Corrections are pushed to us asynchronously through message_arrived().
	while (!_should_exit) {
		std::this_thread::sleep_for(std::chrono::seconds(1));
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

bool PointPerfectClientMavlink::connect_pointperfect()
{
	std::cout << "Connecting to PointPerfect: " << _settings.mqtt_server_uri << std::endl;

	_mqtt_client = std::make_shared<mqtt::async_client>(_settings.mqtt_server_uri, _settings.mqtt_client_id);
	_mqtt_client->set_callback(*this);

	auto ssl_opts = mqtt::ssl_options_builder()
			.trust_store(_settings.root_ca_path)
			.key_store(_settings.client_cert_path)
			.private_key(_settings.client_key_path)
			.enable_server_cert_auth(true)
			.finalize();

	auto conn_opts = mqtt::connect_options_builder()
			 .mqtt_version(MQTTVERSION_3_1_1)
			 .ssl(std::move(ssl_opts))
			 .clean_session(true)
			 .keep_alive_interval(std::chrono::seconds(60))
			 .automatic_reconnect(std::chrono::seconds(1), std::chrono::seconds(30))
			 .finalize();

	try {
		_mqtt_client->connect(conn_opts)->wait();

	} catch (const mqtt::exception& e) {
		std::cerr << "MQTT connect error: " << e.what() << std::endl;
		return false;
	}

	return true;
}

void PointPerfectClientMavlink::subscribe_topics()
{
	// SPARTN correction data for the configured region.
	const std::string data_topic = "/pp/ip/" + _settings.region;
	std::cout << "Subscribing to " << data_topic << std::endl;
	_mqtt_client->subscribe(data_topic, 0);

	// Dynamic decryption keys (UBX-RXM-SPARTNKEY) that the receiver needs to
	// decode the SPARTN stream.
	if (_settings.enable_key_distribution) {
		const std::string key_topic = "/pp/ubx/0236/ip";
		std::cout << "Subscribing to " << key_topic << std::endl;
		_mqtt_client->subscribe(key_topic, 0);
	}
}

void PointPerfectClientMavlink::connected(const std::string& cause)
{
	std::cout << "Connected to PointPerfect" << std::endl;
	subscribe_topics();
}

void PointPerfectClientMavlink::connection_lost(const std::string& cause)
{
	std::cerr << "PointPerfect connection lost";

	if (!cause.empty()) {
		std::cerr << ": " << cause;
	}

	std::cerr << std::endl;
}

void PointPerfectClientMavlink::message_arrived(mqtt::const_message_ptr msg)
{
	const std::string& payload = msg->to_string();
	std::cout << "\nReceived " << payload.size() << " bytes on " << msg->get_topic() << std::endl;
	forward_corrections(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
}

void PointPerfectClientMavlink::forward_corrections(const uint8_t* data, size_t length)
{
	if (length == 0) {
		return;
	}

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
