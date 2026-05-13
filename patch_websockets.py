"""
Pre-build patch for links2004/WebSockets library.

Problem:
  WebSocketsServer.cpp sets WiFiClient::setTimeout() on accept, but that only
  affects READ operations (readBytesUntil etc.). The library's writes go
  through WiFiClient::write() → lwip_send(), which respects SO_SNDTIMEO. With
  SO_SNDTIMEO=0 (default), a slow client can block sendBIN/sendTXT for
  multiple seconds when the TCP send buffer fills.

Fix:
  Right after setTimeout() on a new client, also call setsockopt() on the
  underlying socket fd with SO_SNDTIMEO set to WEBSOCKETS_TCP_TIMEOUT.

  This makes write() return short / fail when the buffer's been full for
  longer than the timeout — caps the broadcast-loop stall at ~50 ms.

The patch is idempotent: if the marker is already present we do nothing.
PlatformIO calls this script before every build via `extra_scripts =`.
"""

import os

Import("env")  # type: ignore   noqa: F821 - provided by SCons/PlatformIO

# Path is computed from PROJECT_DIR so it works on any user's machine.
PROJECT_DIR = env["PROJECT_DIR"]                              # type: ignore  noqa: F821
LIB_FILE = os.path.join(
    PROJECT_DIR,
    ".pio", "libdeps", env["PIOENV"],                         # type: ignore  noqa: F821
    "WebSockets", "src", "WebSocketsServer.cpp",
)

MARKER = "// SO_SNDTIMEO patch applied"

# The block to find, verbatim, in the original library source.
ORIG = (
    "#if (WEBSOCKETS_NETWORK_TYPE != NETWORK_ESP8266_ASYNC)\n"
    "            // set Timeout for readBytesUntil and readStringUntil\n"
    "            client->tcp->setTimeout(WEBSOCKETS_TCP_TIMEOUT);\n"
    "#endif"
)

# Replacement: keep the original setTimeout() (it's still correct for reads),
# and add SO_SNDTIMEO so writes also honour the timeout. lwIP exposes
# setsockopt() via the standard BSD socket API on ESP32.
REPLACEMENT = (
    "#if (WEBSOCKETS_NETWORK_TYPE != NETWORK_ESP8266_ASYNC)\n"
    "            // set Timeout for readBytesUntil and readStringUntil\n"
    "            client->tcp->setTimeout(WEBSOCKETS_TCP_TIMEOUT);\n"
    "            " + MARKER + "\n"
    "            {\n"
    "                int _fd = client->tcp->fd();\n"
    "                if (_fd >= 0) {\n"
    "                    struct timeval _tv;\n"
    "                    _tv.tv_sec  = WEBSOCKETS_TCP_TIMEOUT / 1000;\n"
    "                    _tv.tv_usec = (WEBSOCKETS_TCP_TIMEOUT % 1000) * 1000;\n"
    "                    setsockopt(_fd, SOL_SOCKET, SO_SNDTIMEO, &_tv, sizeof(_tv));\n"
    "                }\n"
    "            }\n"
    "#endif"
)

# Header for sys/time.h / socket APIs. The library already includes platform
# networking but not <sys/time.h> on ESP32 explicitly. We add it once.
HEADER_MARKER = "// SO_SNDTIMEO patch header"
HEADER_ORIG = '#include "WebSocketsServer.h"'
HEADER_NEW = (
    '#include "WebSocketsServer.h"\n'
    f"{HEADER_MARKER}\n"
    "#include <sys/socket.h>\n"
    "#include <sys/time.h>"
)


def apply_patch() -> None:
    if not os.path.exists(LIB_FILE):
        # First build: libdeps not yet installed. PIO will install, then call
        # this script again on the next build phase. Nothing to do this pass.
        print(f"[patch] WebSocketsServer.cpp not yet at {LIB_FILE} — skipping (will retry next build).")
        return

    with open(LIB_FILE, "r", encoding="utf-8") as f:
        src = f.read()

    if MARKER in src:
        print("[patch] WebSocketsServer.cpp SO_SNDTIMEO patch already present.")
        return

    if ORIG not in src:
        print("[patch] WARNING: expected setTimeout block not found in WebSocketsServer.cpp.")
        print("[patch] Library version may have changed — patch skipped.")
        return

    new_src = src.replace(ORIG, REPLACEMENT, 1)

    if HEADER_MARKER not in new_src and HEADER_ORIG in new_src:
        new_src = new_src.replace(HEADER_ORIG, HEADER_NEW, 1)

    with open(LIB_FILE, "w", encoding="utf-8") as f:
        f.write(new_src)

    print(f"[patch] Applied SO_SNDTIMEO patch to {LIB_FILE}")


apply_patch()
