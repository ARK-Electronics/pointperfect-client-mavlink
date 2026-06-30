## u-blox PointPerfect Client for MAVLink
This application connects to u-blox's [PointPerfect](https://www.u-blox.com/en/product/pointperfect) GNSS corrections service (delivered via the Thingstream MQTT broker) and forwards the SPARTN correction data to a flight controller via MAVLink.

You must first create a Thingstream account, activate a PointPerfect plan on a *Thing*, and copy the MQTT credentials (client certificate, client key, and root CA) into the config file. <br>
https://portal.thingstream.io/

The **config.toml** file is used to configure the program settings. <br>
https://github.com/ARK-Electronics/pointperfect-client-mavlink/blob/main/config.toml

### Behavior
The application waits for a MAVSDK connection. Once connected, it opens a TLS connection to the PointPerfect MQTT broker (`pp.services.u-blox.com:8883`) using the client certificate/key from the config and subscribes to:

- `/pp/ip/<region>` — the SPARTN correction stream for your region (`us` or `eu`)
- `/pp/ubx/0236/ip` — the dynamic decryption keys (`UBX-RXM-SPARTNKEY`), when `enable_key_distribution` is set

Both the SPARTN corrections and the key messages are forwarded, unmodified, to the flight controller as [GPS_RTCM_DATA](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) MAVLink messages (fragmented when larger than the message payload). The autopilot injects these bytes directly into the u-blox receiver, which uses the keys to decrypt the SPARTN stream and compute a high-precision fix.

### Credentials
From the Thingstream portal, open your PointPerfect *Thing* → **Credentials** → **MQTT Credentials** and download the Client Certificate, Client Key, and Root Certificate. Point the config at their paths:
```toml
client_cert_path = "/opt/ark/share/pointperfect/device-pp-cert.crt"
client_key_path  = "/opt/ark/share/pointperfect/device-pp-key.pem"
root_ca_path     = "/opt/ark/share/pointperfect/root.crt"
```

### Build
Pre-requisites
```
sudo apt install libssl-dev cmake build-essential
```
Install MAVSDK if you haven't already, the latest releases can be found at https://github.com/mavlink/MAVSDK/releases
```
sudo dpkg -i libmavsdk-dev_2.4.1_debian12_arm64.deb
```
Initialize submodules (tomlplusplus)
```
git submodule update --init --recursive
```
Build (the Eclipse Paho MQTT C++ client is fetched and built automatically)
```
make
```
Run
```
./build/pointperfect-client-mavlink
```

### Configuration lookup
The config file is resolved in this order:
1. `--config <path>` (or `--config=<path>`) command-line argument
2. `~/.config/ark/pointperfect/config.toml`
3. `/opt/ark/share/pointperfect/config.toml` (installed by `make install`)
