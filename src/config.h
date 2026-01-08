#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif

#define CONF_NODENAME_DEF     "espArtNetNode"
#define CONF_LONGNAME_DEF     "espArtNetNode DMX/Pixel interface"
#define CONF_NET_AP_SSID_DEF  CONF_NODENAME_DEF
#define CONF_NET_AP_PW_DEF    "ArtNet2023"

#define CONF_ART_FIRM_VER     FIRMARE_VER // Firmware given over Artnet (2 bytes)
#define CONF_ARTNET_OEM       0x0123      // Artnet OEM Code
#define CONF_ESTA_MAN         0x08DD      // ESTA Manufacturer Code
#define CONF_ESTA_DEV         0xEE000000  // RDM Device ID (used with Man Code to make 48bit UID)

enum config_pixel_fx_mode {
  PIXEL_FX_MODE_MAP = 0,
  PIXEL_FX_MODE_12 = 1
};

enum config_port_type {
  PORT_TYPE_DMX_OUT = 0,
  PORT_TYPE_RDM_OUT = 1,
  PORT_TYPE_DMX_IN = 2,
  PORT_TYPE_WS2812 = 3
};

enum config_port_protocol {
  PORT_PROT_ARTNET = 0,
  PORT_PROT_ARTNET_SACN = 1
};

enum config_port_merge {
  PORT_MERGE_LTP = 0,
  PORT_MERGE_HTP = 1
};


struct Config {
  char gen_version[7];
  IPAddress net_sta_ip, net_sta_subnet, net_sta_gateway, net_sta_broadcast, net_ap_ip, net_ap_subnet, net_ap_broadcast, net_dmxIn_broadcast;
  bool net_sta_dhcp, net_ap_standalone;
  char gen_nodeName[18], gen_longName[64], net_sta_ssid[32], net_sta_password[64], net_ap_ssid[32], net_ap_password[64];
  uint16_t net_ap_delay;
  uint8_t portA_mode, portB_mode, portA_prot, portB_prot, portA_merge, portB_merge;
  uint8_t portA_net, portA_subnet, portA_uni[4], portB_net, portB_subnet, portB_uni[4];
  uint16_t portA_SACNuni[4], portB_SACNuni[4];
  uint16_t portA_pixel_count, portB_pixel_count, portA_pixel_config, portB_pixel_config;
  uint8_t portA_pixel_mode, portB_pixel_mode;
  uint16_t portA_pixel_startFX, portB_pixel_startFX;
  uint8_t resetCounter, wdtCounter;
};

void config_init(void);
bool config_load(void);
bool config_save(void);

#endif // _CONFIG_H_