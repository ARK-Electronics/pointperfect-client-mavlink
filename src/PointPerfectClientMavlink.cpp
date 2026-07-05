#include "PointPerfectClientMavlink.hpp"
#include <mavsdk/log_callback.h>
#include <iostream>

namespace
{

// Read 'num_bits' bits starting at 'bit_offset' (MSB first) from 'data'.
uint32_t get_bits(const uint8_t* data, size_t bit_offset, size_t num_bits)
{
	uint32_t value = 0;

	for (size_t i = 0; i < num_bits; i++) {
		size_t bit = bit_offset + i;
		value = (value << 1) | ((data[bit / 8] >> (7 - (bit % 8))) & 1);
	}

	return value;
}

// Total length of the SPARTN frame starting at data[0], or 0 if it cannot
// be determined from the available bytes.
size_t spartn_frame_length(const uint8_t* data, size_t available)
{
	if (available < 8 || data[0] != 0x73) {
		return 0;
	}

	const uint32_t payload_length = get_bits(data, 15, 10); // TF003
	const uint32_t eaf = get_bits(data, 25, 1);             // TF004
	const uint32_t crc_type = get_bits(data, 26, 2);        // TF005
	const uint32_t time_tag_type = get_bits(data, 36, 1);   // TF008

	// TF001-TF011: preamble + message description + payload description
	size_t header_bytes = time_tag_type ? 10 : 8;

	size_t auth_bytes = 0;

	if (eaf) {
		header_bytes += 2; // TF012-TF015

		if (available < header_bytes) {
			return 0;
		}

		// TF014 authentication indicator, TF015 embedded auth length
		const size_t tf014_offset = time_tag_type ? 90 : 74;
		const uint32_t auth_indicator = get_bits(data, tf014_offset, 3);

		if (auth_indicator > 1) {
			const uint32_t auth_length = get_bits(data, tf014_offset + 3, 3);

			switch (auth_length) {
			case 0: auth_bytes = 8; break;

			case 1: auth_bytes = 12; break;

			case 2: auth_bytes = 16; break;

			case 3: auth_bytes = 32; break;

			case 4: auth_bytes = 64; break;

			default: return 0;
			}
		}
	}

	const size_t crc_bytes = crc_type + 1;

	return header_bytes + payload_length + auth_bytes + crc_bytes;
}

// Total length of the UBX frame starting at data[0], or 0 if it cannot
// be determined from the available bytes.
size_t ubx_frame_length(const uint8_t* data, size_t available)
{
	if (available < 8 || data[0] != 0xB5 || data[1] != 0x62) {
		return 0;
	}

	const size_t payload_length = data[4] | (data[5] << 8);

	// Sync (2) + class/id (2) + length (2) + payload + checksum (2)
	return 8 + payload_length;
}

} // namespace

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
	// May be called from a signal handler: an atomic store is the only
	// async-signal-safe thing we can do here. MQTT teardown happens in run().
	_should_exit = true;
}

void PointPerfectClientMavlink::run()
{
	std::cout << "Waiting for MAVSDK connection: " << _settings.mavsdk_connection_url << std::endl;

	while (!wait_for_mavsdk_connection(3)) {
		if (_should_exit) {
			return;
		}

		// Avoid spinning when the connection cannot even be created
		// (e.g. the port is already in use).
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	if (!connect_pointperfect()) {
		std::cerr << "Failed to connect to PointPerfect" << std::endl;
		return;
	}

	// Corrections are pushed to us asynchronously through message_arrived().
	int seconds = 0;

	while (!_should_exit) {
		std::this_thread::sleep_for(std::chrono::seconds(1));

		if (++seconds >= 10) {
			seconds = 0;
			print_statistics();
		}
	}

	disconnect_pointperfect();
}

void PointPerfectClientMavlink::print_statistics()
{
	const uint32_t messages = _messages_received.exchange(0);
	const uint32_t frames = _frames_forwarded.exchange(0);
	const uint32_t dropped = _frames_dropped.exchange(0);
	const uint64_t bytes = _bytes_forwarded.exchange(0);

	if (messages == 0 && dropped == 0) {
		return;
	}

	std::cout << "Last 10s: " << messages << " messages, "
		  << frames << " frames / " << bytes << " bytes forwarded";

	if (dropped > 0) {
		std::cout << ", " << dropped << " frames dropped (too large)";
	}

	std::cout << std::endl;
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

void PointPerfectClientMavlink::disconnect_pointperfect()
{
	if (_mqtt_client && _mqtt_client->is_connected()) {
		try {
			_mqtt_client->disconnect()->wait();

		} catch (const mqtt::exception& e) {
			std::cerr << "MQTT disconnect error: " << e.what() << std::endl;
		}
	}
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
	const uint8_t* data = reinterpret_cast<const uint8_t*>(payload.data());
	const size_t length = payload.size();

	_messages_received++;

	// A single MQTT message can bundle several SPARTN (or UBX) frames and
	// regularly exceeds the 720 bytes one GPS_RTCM_DATA sequence can carry,
	// so split it at frame boundaries and send each frame as its own sequence.
	size_t offset = 0;

	while (offset < length && !_should_exit) {
		size_t frame_length = spartn_frame_length(data + offset, length - offset);

		if (frame_length == 0) {
			frame_length = ubx_frame_length(data + offset, length - offset);
		}

		if (frame_length == 0 || frame_length > length - offset) {
			// Unknown framing or truncated frame: forward the remainder as-is
			frame_length = length - offset;
		}

		if (frame_length >= MAX_SEQUENCE_LENGTH) {
			// Cannot be transported over GPS_RTCM_DATA (2-bit fragment id;
			// exactly 720 bytes would need a fifth, zero-length terminating
			// fragment). Dropping it lets the receiver resync on the next
			// frame instead of feeding it corrupted data.
			_frames_dropped++;

		} else {
			forward_frame(data + offset, frame_length);
		}

		offset += frame_length;
	}
}

void PointPerfectClientMavlink::forward_frame(const uint8_t* data, size_t length)
{
	if (length == 0 || length >= MAX_SEQUENCE_LENGTH) {
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
			memcpy(msg.data, data + start, current_length);
			send_mavlink_gps_rtcm_data(msg);
			start += current_length;
		}

		// The receiver detects the last fragment by len < 180, so a message
		// that is an exact multiple of the fragment size needs a zero-length
		// terminating fragment.
		if (length % MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN == 0) {
			msg.flags = 1;
			msg.flags |= (fragment_id++ & 0x3) << 1;
			msg.flags |= (_sequence_id & 0x1F) << 3;
			msg.len = 0;
			send_mavlink_gps_rtcm_data(msg);
		}
	}

	_frames_forwarded++;
	_bytes_forwarded += length;
	_sequence_id = (_sequence_id + 1) & 0x1F;
}

void PointPerfectClientMavlink::send_mavlink_gps_rtcm_data(const mavlink_gps_rtcm_data_t& msg)
{
	// queue_message invokes the composer synchronously, so capturing msg by
	// reference is safe.
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
