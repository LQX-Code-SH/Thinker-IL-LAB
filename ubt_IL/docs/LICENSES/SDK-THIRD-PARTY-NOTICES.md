# UBT Robot SDK third-party notices

This notice applies to the SDK archives distributed in this repository under
`ubt_IL/docs/walker_s2/assets/`:

- `ubt_api_tiny.20260203.zip` — C/C++ SDK (static libraries, headers, and
  examples for x86_64 and aarch64);
- `ubt_api_tiny_python_20260407.tar.xz` — Python SDK (the two `ubt_robot`
  1.0.0 wheels for cp310).

Third-party components remain subject to their own license terms. These
notices do not grant any rights to UBTECH proprietary portions of the SDK.

## Components distributed inside the SDK

### nlohmann/json 3.12.0

- Project: https://github.com/nlohmann/json
- License: MIT
- License text: `MIT-nlohmann-json.txt`
- Form: header code compiled into `libcc_api_client` (both the static
  `libcc_api_client.a` in the zip archive and the shared
  `libcc_api_client.so` inside the wheels).

The nlohmann/json headers also include Hedley by Evan Nemerson under CC0-1.0.
The corresponding text is provided in `CC0-1.0.txt`.

### cpp-tbox

- Project: https://github.com/cpp-main/cpp-tbox
- License: MIT
- License text: `MIT-cpp-tbox.txt`
- Upstream release version: not identified in the SDK metadata.

The SDK contains the following cpp-tbox components. In the Python wheels they
are bundled as shared libraries; in the C/C++ zip archive the same components
are bundled as static libraries (`libtbox_*.a`). The numbers shown are the
bundled library file versions and do not establish an overall cpp-tbox
release version:

- `libtbox_base.so.1.0.1`
- `libtbox_event.so.1.1.2`
- `libtbox_eventx.so.1.0.2`
- `libtbox_jsonrpc.so.0.0.1`
- `libtbox_network.so.1.0.0`
- `libtbox_util.so.0.0.2`

## Runtime dependency not distributed in the SDK

### Eclipse Mosquitto

- Project: https://github.com/eclipse-mosquitto/mosquitto
- Required ABI: `libmosquitto.so.1`
- Selected license option for this notice: Eclipse Distribution License 1.0
  (BSD-3-Clause)
- License text: `EDL-1.0-mosquitto.txt`

The Mosquitto library itself is not included in this repository or in the
SDK archives. It must be installed separately from the target operating
system's package repository.
