## u-blox PointPerfect Client for MAVLink
This application connects to u-blox's [PointPerfect](https://www.u-blox.com/en/product/pointperfect) GNSS corrections service over **NTRIP** and forwards the correction data to a flight controller via MAVLink.

> NTRIP is u-blox's currently supported distribution method for PointPerfect (the older MQTT distribution is deprecated).

### Correction format
The client defaults to **RTCM**, which is the format PX4 natively injects and parses. PointPerfect also offers **SPARTN**, which is recommended for u-blox receivers when the receiver/firmware supports it. Select the format with `correction_format` in the config:
```toml
correction_format = "rtcm"    # default; PX4-compatible
# correction_format = "spartn"  # requires receiver/firmware SPARTN support
```
The format selects the caster mountpoint (`NEAR-RTCM` / `NEAR-SPARTN`) automatically; set `ntrip_mountpoint` explicitly only if you need to override it.

You must first create a Thingstream account, activate a PointPerfect plan on a *Thing*, and copy the NTRIP credentials into the config file. <br>
https://portal.thingstream.io/

The **config.toml** file is used to configure the program settings. <br>
https://github.com/ARK-Electronics/pointperfect-client-mavlink/blob/main/config.toml

### Behavior
The application waits for a MAVSDK connection. Once connected, it opens an NTRIP connection to the PointPerfect caster (`ppntrip.services.u-blox.com`, TLS on port `2102` or plain TCP on `2101`), authenticates with the per-Thing username/password, and requests the mountpoint for the configured correction format.

The caster provides *localized* corrections, so the client periodically reports the vehicle's position to it. The [GPS_RAW_INT](https://mavlink.io/en/messages/common.html#GPS_RAW_INT) MAVLink messages from the flight controller are converted to an NMEA-GGA sentence and sent to the caster every 10 seconds.

The incoming correction stream is forwarded, unmodified, to the flight controller as [GPS_RTCM_DATA](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) MAVLink messages (fragmented when larger than the message payload). The autopilot injects these bytes directly into the u-blox receiver to compute a high-precision fix.

### Credentials
From the Thingstream portal, open your PointPerfect *Thing* → **Credentials**. Copy the NTRIP server, port, username, and password into the config:
```toml
ntrip_host = "ppntrip.services.u-blox.com"
ntrip_port = 2102
ntrip_username = "your_username_goes_here"
ntrip_password = "your_password_goes_here"
```
Each Thing allows a single simultaneous NTRIP connection. The mountpoint is derived from `correction_format`; override it with `ntrip_mountpoint` if your credentials show a different name.

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
Build
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
