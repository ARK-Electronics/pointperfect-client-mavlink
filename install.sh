#!/bin/bash
set -euo pipefail

THIS_DIR="$(dirname "$(realpath "$BASH_SOURCE")")"

# Build
pushd "$THIS_DIR" > /dev/null
make
popd > /dev/null

# Install binary
sudo mkdir -p /opt/ark/bin
sudo cp "$THIS_DIR/build/pointperfect-client-mavlink" /opt/ark/bin/

# Install default config, preserving an existing one (it carries the user's
# MQTT client id and credential paths)
sudo mkdir -p /opt/ark/share/pointperfect
if [ -f /opt/ark/share/pointperfect/config.toml ]; then
	echo "Existing config preserved: /opt/ark/share/pointperfect/config.toml"
else
	sudo cp "$THIS_DIR/config.toml" /opt/ark/share/pointperfect/
fi
