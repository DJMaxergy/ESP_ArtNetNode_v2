# ESP32_ArtNetNode_v2
ESP32 based WiFi ArtNet V4 to DMX, RDM and LED Pixels

This is a complete rewrite of mtongnz's [project](https://github.com/mtongnz/ESP8266_ArtNetNode_v2.git) for Platformio using [NeoPixelBus](https://github.com/Makuna/NeoPixelBus) and [ESPUI](https://github.com/s00500/ESPUI).

An Instructable detailing how to setup pixel mapping using Jinx can be found here: https://www.instructables.com/id/Artnet-LED-Pixels-With-ESP8266/

## Getting Started
### First Boot
On your first boot, the device will start a hotspot called "espArtNetNode" with a password of "ArtNet2023" (case sensitive).  Login to the hotspot and a browser should open automatically (Captive Portal) or goto 2.0.0.1 in a browser.

Note that Stand Alone mode using the hotspot is enabled by default. You can change that in the web UI if you want to send ArtNet to the device only in an infrastructure WLAN.
### Web UI
In hotspot mode, goto 2.0.0.1 and in Wifi mode goto whatever the device IP might be - either static or assigned by DHCP.

In the Wifi tab, enter your SSID and password.  Click save (it should go green and say Settings Saved).  Now click reboot and the device should connect to your Wifi.

If the device can't connect to the wifi or get a DHCP assigned address within (Start Delay) seconds, it will start the hotspot and wait for 30 seconds for you to connect.  If a client doesn't connect to the hotspot in time, the device will restart and try again.

### Restore Factory Defaults
I have allowed for 2 methods to restore the factory default settings: using a dedicated factory reset button or by Web UI.

Method 1: Hold GPIO14 to GND while the device boots.  This method isn't available for the ESP01 or NO_RESET builds.

Method 2: Go to WebUI and press the factory reset button in the general settings tab.

### DMX Workshop
I have implemented as many DMX Workshop/ArtNet V4 features as I possibly could.  You can change settings such as merge mode, IP address, DHCP, port addresses, node name...  Most of these are also available via the web UI.

## Features
 - sACN and ArtNet V4 support
 - 2 full universes of DMX output with full RDM support for both outputs
 - Up to 1360 ws2812(b) pixels - 8 full universes
 - DMX/RDM out one port, ws2812(b) out the other
 - DMX in - send to any ArtNet device
 - Web UI with mobile support
 - Web UI uses AJAX & JSON to minimize network traffic used & decrease latency
 - Full DMX Workshop support
 - Pixel FX - a 12 channel mode for ws2812 LED pixel control

## Pixel FX
To enable this mode, select WS2812 in the port settings and enter the number of pixels you wish to control.  Select '12 Channel FX'. 'Start Channel' is the DMX address of the first channel below.

Note: You still need to set the Artnet net, subnet and universe correctly.

| DMX Channel | Function | Values (0-255) |  |
|----|----|----|----|
| 1 | Intensity | 0 - 255   |               |
| 2 | FX Select | 0 - 49    | Static        |
|   |           | 50 - 74   | Rainbow       |
|   |           | 75 - 99   | Theatre Chase |
|   |           | 100 - 124 | Twinkle       |
| 3 | Speed     | 0 - 19    | Stop - Index Reset |
|   |           | 20 - 122  | Slow - Fast CW |
|   |           | 123 - 130 | Stop |
|   |           | 131 - 234 | Fast - Slow CCW |
|   |           | 235 - 255 | Stop |
| 4 | Position  | 0 - 127 - 255 | Left - Centre - Right |
| 5 | Size      | 0 - 255   | Small - Big   |
| 6 | Colour 1  | 0 - 255   | Red |
| 7 |           | 0 - 255   | Green |
| 8 |           | 0 - 255   | Blue |
| 9 | Colour 2  | 0 - 255   | Red |
| 10 |          | 0 - 255   | Green |
| 11 |          | 0 - 255   | Blue |
| 12 | Modify   | 0 - 255   | *Modify FX |

Modify FX is only currently used for the Static effect and is used to resize colour 1 within the overall size.

## Wemos Pins

| Wemos | ESP32 GPIO | Purpose |
|-----------------|--------------|---------|
| TX | GPIO1 | PIN_PORT_A |
| D4 | GPIO16 | PIN_PORT_B |
| - | GPIO22 | PIN_PORT_A_RX |
| D7 | GPIO23 | PIN_DMX_DIR_A |
| D0 | GPIO26 | PIN_DMX_DIR_B |

