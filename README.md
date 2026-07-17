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

### AssistNow start-up assistance (MGA)
For u-blox receivers, PointPerfect offers dedicated mountpoints (`NEAR-RTCM-MGA` / `NEAR-SPARTN-MGA`) that deliver AssistNow assistance data — live global GPS and Galileo ephemeris as UBX-MGA messages — once, immediately on connection, ahead of the correction stream. Fed to the receiver, this significantly reduces the time to first fix, which in turn lets the client report a valid position (GGA) to the caster sooner; the caster starts the correction stream only after it receives a valid GGA. Enable it with:
```toml
use_mga = true    # u-blox receivers only
```
The client extracts the UBX-MGA messages from the stream and forwards each one to the flight controller in its own `GPS_RTCM_DATA` sequence, paced 20 ms apart so the burst of assistance data (several KB at connect) does not overflow the autopilot's injection queue on its way to the receiver. `use_mga` requires `send_gga`, and selects the `-MGA` mountpoint automatically unless `ntrip_mountpoint` is set explicitly (an explicit `*-MGA` mountpoint gets the same forwarding treatment).

You must first create a Thingstream account, activate a PointPerfect plan on a *Thing*, and copy the NTRIP credentials into the config file. <br>
https://portal.thingstream.io/

The **config.toml** file is used to configure the program settings. <br>
https://github.com/ARK-Electronics/pointperfect-client-mavlink/blob/main/config.toml

### Behavior
The application waits for a MAVSDK connection, then for the first [GPS_RAW_INT](https://mavlink.io/en/messages/common.html#GPS_RAW_INT) of any `fix_type` (including no-fix). That signals the GPS driver has finished configuring the receiver and will inject `GPS_RTCM_DATA`, which matters for the one-shot AssistNow (MGA) burst billed at NTRIP connect. Only then does it open an NTRIP connection to the PointPerfect caster (`ppntrip.services.u-blox.com`, TLS on port `2102` or plain TCP on `2101`), authenticate with the per-Thing username/password, and request the mountpoint for the configured correction format.

The caster provides *localized* corrections, so the client periodically reports the vehicle's position to it. After a continuous ≥2D fix for a few seconds, GPS_RAW_INT is converted to an NMEA-GGA sentence and sent to the caster every 10 seconds. MGA forwarding itself is not gated on fix — only on the receiver being up.

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
