#pragma once

// Per-board mDNS hostname. The board is reachable at http://<HOSTNAME>.local
// Use a unique name per physical board (e.g. for 3 boards with different slopes:
// "imuboard350", "imuboard500", "imuboard750"). Keep it lowercase, no spaces.
#define WIFI_HOSTNAME "imuboard350"

// List of WiFi networks to try (in order of priority).
// The ESP will attempt each one and connect to the first that responds.
#define WIFI_CREDENTIALS { \
    {"ACT-ai_102801145059", "36358643"},\
}
