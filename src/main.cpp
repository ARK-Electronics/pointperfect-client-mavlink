#include "PointPerfectClientMavlink.hpp"
#include <filesystem>
#include <signal.h>
#include <iostream>
#include <toml.hpp>
#include <unistd.h>
#include <utility>
#include <sys/types.h>

static void signal_handler(int signum);

std::shared_ptr<PointPerfectClientMavlink> _pointperfect_client_mavlink;

int main(int argc, char** argv)
{
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	setbuf(stdout, NULL); // Disable stdout buffering

	// Config lookup: --config <path> (or --config=<path>) overrides everything;
	// otherwise user override > deb-installed default.
	const std::string home = getenv("HOME") ? getenv("HOME") : "/tmp";
	const auto user_config = std::filesystem::path(home) / ".config/ark/pointperfect/config.toml";
	const auto default_config = std::filesystem::path("/opt/ark/share/pointperfect/config.toml");
	std::string config_path = (std::filesystem::exists(user_config) ? user_config : default_config).string();

	for (int i = 1; i < argc; i++) {
		std::string arg = argv[i];

		if (arg == "--config" && i + 1 < argc) {
			config_path = argv[++i];

		} else if (arg.rfind("--config=", 0) == 0) {
			config_path = arg.substr(std::string("--config=").size());
		}
	}

	toml::table config;

	try {
		config = toml::parse_file(config_path);

	} catch (const toml::parse_error& err) {
		std::cerr << "Parsing failed:\n" << err << "\n";
		return -1;

	} catch (const std::exception& err) {
		std::cerr << "Error: " << err.what() << "\n";
		return -1;
	}

	PointPerfectClientMavlink::Settings settings = {
		.mavsdk_connection_url = config["connection_url"].value_or("udp://:14552"),
		.mqtt_server_uri = config["mqtt_server_uri"].value_or("ssl://pp.services.u-blox.com:8883"),
		.mqtt_client_id = config["mqtt_client_id"].value_or(""),
		.client_cert_path = config["client_cert_path"].value_or(""),
		.client_key_path = config["client_key_path"].value_or(""),
		.root_ca_path = config["root_ca_path"].value_or(""),
		.region = config["region"].value_or("us"),
		.enable_key_distribution = config["enable_key_distribution"].value_or(true)
	};

	// Fail fast with a clear message instead of a cryptic TLS/MQTT error later.
	if (settings.mqtt_client_id.empty() || settings.mqtt_client_id.find("your_client_id") != std::string::npos) {
		std::cerr << "mqtt_client_id is not set in " << config_path << "\n"
			  "Copy the Client ID from the Thingstream portal: your Thing > Credentials > MQTT Credentials\n";
		return -1;
	}

	const std::pair<const char*, const std::string*> credential_files[] = {
		{"client_cert_path", &settings.client_cert_path},
		{"client_key_path", &settings.client_key_path},
		{"root_ca_path", &settings.root_ca_path},
	};

	for (const auto& [name, path] : credential_files) {
		if (path->empty() || !std::filesystem::exists(*path)) {
			std::cerr << name << " \"" << *path << "\" does not exist (configured in " << config_path << ")\n"
				  "Download the MQTT credentials from the Thingstream portal: your Thing > Credentials\n";
			return -1;
		}
	}

	_pointperfect_client_mavlink = std::make_shared<PointPerfectClientMavlink>(settings);

	_pointperfect_client_mavlink->run();

	std::cout << "exiting" << std::endl;

	return 0;
}

static void signal_handler(int signum)
{
	if (_pointperfect_client_mavlink.get()) _pointperfect_client_mavlink->stop();
}
