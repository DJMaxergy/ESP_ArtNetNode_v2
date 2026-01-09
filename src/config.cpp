#include "config.h"
#include <ArduinoJson.h>
#include <LittleFS.h>

extern Config config, configActive;

void config_init(void)
{
    char fwVersion[7];
    snprintf(fwVersion, sizeof(fwVersion),"v%1u.%1u.%1u", (uint8_t)((CONF_ART_FIRM_VER & 0x0F00) >> 8), (uint8_t)((CONF_ART_FIRM_VER & 0x00F0) >> 4), (uint8_t)(CONF_ART_FIRM_VER & 0x000F));
    strcpy(config.gen_version, fwVersion);
    strcpy(config.gen_nodeName, CONF_NODENAME_DEF);
    strcpy(config.gen_longName, CONF_LONGNAME_DEF);

    strcpy(config.net_sta_ssid, "");
    strcpy(config.net_sta_password, "");
    config.net_sta_dhcp = true;
    config.net_sta_ip.fromString("2.0.0.1");
    config.net_sta_subnet.fromString("255.255.255.0");
    config.net_sta_gateway.fromString("2.0.0.1");
    config.net_sta_broadcast.fromString("2.0.0.255");

    strcpy(config.net_ap_ssid, CONF_NET_AP_SSID_DEF);
    strcpy(config.net_ap_password, CONF_NET_AP_PW_DEF);
    config.net_ap_ip.fromString("2.0.0.1");
    config.net_ap_subnet.fromString("255.255.255.0");
    config.net_ap_broadcast.fromString("2.0.0.255");
    config.net_ap_standalone = true;
    config.net_ap_delay = 15;

    config.net_dmxIn_broadcast.fromString("2.0.0.255");

    config.portA_mode = 0;
    config.portA_prot = 0;
    config.portA_merge = 1;
    config.portA_net = 0;
    config.portA_subnet = 0;
    memcpy(config.portA_uni, (const uint8_t[]){0, 1, 2, 3}, 4);
    memcpy(config.portA_SACNuni, (const uint16_t[]){1, 2, 3, 4}, 4 * sizeof(uint16_t));
    config.portA_pixel_count = 680;
    config.portA_pixel_config = 0;
    config.portA_pixel_mode = 0;
    config.portA_pixel_startFX = 1;

    config.portB_mode = 0;
    config.portB_prot = 0;
    config.portB_merge = 1;
    config.portB_net = 0;
    config.portB_subnet = 0;
    memcpy(config.portB_uni, (const uint8_t[]){4, 5, 6, 7}, 4);
    memcpy(config.portB_SACNuni, (const uint16_t[]){5, 6, 7, 8}, 4 * sizeof(uint16_t));
    config.portB_pixel_count = 680;
    config.portB_pixel_config = 0;
    config.portB_pixel_mode = 0;
    config.portB_pixel_startFX = 1;

    config.resetCounter = 0;
    config.wdtCounter = 0;
}

bool config_load(void)
{
    File configFile = LittleFS.open("/config.json", "r");
    if (!configFile) {
        return false;
    }

    size_t size = configFile.size();
    if (size == 0 || size > 1536) {
        configFile.close();
        return false;
    }

    StaticJsonDocument<1536> doc;
    DeserializationError error = deserializeJson(doc, configFile);
    configFile.close();

    if (error) {
        return false;
    }

    // copy values from JsonDocument to Config objects:
    strlcpy(config.gen_version, doc["general"]["version"], sizeof(config.gen_version));
    char fwVersion[7];
    sprintf(fwVersion, "v%1u.%1u.%1u", (uint8_t)((CONF_ART_FIRM_VER & 0x0F00) >> 8), (uint8_t)((CONF_ART_FIRM_VER & 0x00F0) >> 4), (uint8_t)(CONF_ART_FIRM_VER & 0x000F));
    if (strcmp(config.gen_version, fwVersion) != 0) {
        strcpy(config.gen_version, fwVersion);
    }
    strlcpy(config.gen_nodeName, doc["general"]["nodeName"], sizeof(config.gen_nodeName));
    strlcpy(config.gen_longName, doc["general"]["longName"], sizeof(config.gen_longName));
    
    strlcpy(config.net_sta_ssid, doc["network"]["sta"]["ssid"], sizeof(config.net_sta_ssid));
    strlcpy(config.net_sta_password, doc["network"]["sta"]["password"], sizeof(config.net_sta_password));
    config.net_sta_dhcp = doc["network"]["sta"]["dhcp"];
    config.net_sta_ip.fromString(doc["network"]["sta"]["ip"].as<const char*>());
    config.net_sta_subnet.fromString(doc["network"]["sta"]["subnet"].as<const char*>());
    config.net_sta_gateway.fromString(doc["network"]["sta"]["gateway"].as<const char*>());
    config.net_sta_broadcast.fromString(doc["network"]["sta"]["broadcast"].as<const char*>());
    
    strlcpy(config.net_ap_ssid, doc["network"]["ap"]["ssid"], sizeof(config.net_ap_ssid));
    strlcpy(config.net_ap_password, doc["network"]["ap"]["password"], sizeof(config.net_ap_password));
    config.net_ap_ip.fromString(doc["network"]["ap"]["ip"].as<const char*>());
    config.net_ap_subnet.fromString(doc["network"]["ap"]["subnet"].as<const char*>());
    config.net_ap_broadcast.fromString(doc["network"]["ap"]["broadcast"].as<const char*>());
    config.net_ap_standalone = doc["network"]["ap"]["standalone"];
    config.net_ap_delay = doc["network"]["ap"]["delay"];

    config.net_dmxIn_broadcast.fromString(doc["network"]["dmxIn"]["broadcast"].as<const char*>());

    config.portA_mode = doc["portA"]["mode"];
    config.portA_prot = doc["portA"]["protocol"];
    config.portA_merge = doc["portA"]["merge"];
    config.portA_net = doc["portA"]["net"];
    config.portA_subnet = doc["portA"]["subnet"];
    copyArray(doc["portA"]["universe"].as<JsonArrayConst>(), config.portA_uni);
    copyArray(doc["portA"]["SACN_universe"].as<JsonArrayConst>(), config.portA_SACNuni);
    config.portA_pixel_count = doc["portA"]["pixel"]["count"];
    config.portA_pixel_config = doc["portA"]["pixel"]["config"];
    config.portA_pixel_mode = doc["portA"]["pixel"]["mode"];
    config.portA_pixel_startFX = doc["portA"]["pixel"]["startFx"];

    config.portB_mode = doc["portB"]["mode"];
    config.portB_prot = doc["portB"]["protocol"];
    config.portB_merge = doc["portB"]["merge"];
    config.portB_net = doc["portB"]["net"];
    config.portB_subnet = doc["portB"]["subnet"];
    copyArray(doc["portB"]["universe"].as<JsonArrayConst>(), config.portB_uni);
    copyArray(doc["portB"]["SACN_universe"].as<JsonArrayConst>(), config.portB_SACNuni);
    config.portB_pixel_count = doc["portB"]["pixel"]["count"];
    config.portB_pixel_config = doc["portB"]["pixel"]["config"];
    config.portB_pixel_mode = doc["portB"]["pixel"]["mode"];
    config.portB_pixel_startFX = doc["portB"]["pixel"]["startFx"];

    config.resetCounter = doc["debug"]["resetCounter"];
    config.wdtCounter = doc["debug"]["wdtCounter"];

    return true;
}

bool config_save(void)
{
    bool ret_val = true;
    File configFile = LittleFS.open("/config.json", "w+");

    if (!configFile) {
        ret_val = false;
    }

    size_t size = configFile.size();
    if (size > 1536) {
        ret_val = false;
    }

    StaticJsonDocument<1536> doc;

    if (ret_val == true) {
        // copy values from Config objects to JsonDocument:
        JsonObject general = doc["general"].to<JsonObject>();
        general["version"] = config.gen_version;
        general["nodeName"] = config.gen_nodeName;
        general["longName"] = config.gen_longName;

        JsonObject network = doc["network"].to<JsonObject>();

        JsonObject network_sta = doc["sta"].to<JsonObject>();
        network_sta["ssid"] = config.net_sta_ssid;
        network_sta["password"] = config.net_sta_password;
        network_sta["dhcp"] = config.net_sta_dhcp;
        network_sta["ip"] = config.net_sta_ip;
        network_sta["subnet"] = config.net_ap_subnet;
        network_sta["gateway"] = config.net_sta_gateway;
        network_sta["broadcast"] = config.net_sta_broadcast;

        JsonObject network_ap = doc["ap"].to<JsonObject>();
        network_ap["ssid"] = config.net_ap_ssid;
        network_ap["password"] = config.net_ap_password;
        network_ap["ip"] = config.net_ap_ip;
        network_ap["subnet"] = config.net_ap_subnet;
        network_ap["broadcast"] = config.net_ap_broadcast;
        network_ap["standalone"] = config.net_ap_standalone;
        network_ap["delay"] = config.net_ap_delay;

        network["dmxIn"]["broadcast"] = config.net_dmxIn_broadcast;

        JsonObject portA = doc["portA"].to<JsonObject>();
        portA["mode"] = config.portA_mode;
        portA["protocol"] = config.portA_prot;
        portA["merge"] = config.portA_merge;
        portA["net"] = config.portA_net;
        portA["subnet"] = config.portA_subnet;

        JsonArray portA_universe = portA["universe"].to<JsonArray>();
        portA_universe.add(config.portA_uni[0]);
        portA_universe.add(config.portA_uni[1]);
        portA_universe.add(config.portA_uni[2]);
        portA_universe.add(config.portA_uni[3]);

        JsonArray portA_SACN_universe = portA["SACN_universe"].to<JsonArray>();
        portA_SACN_universe.add(config.portA_SACNuni[0]);
        portA_SACN_universe.add(config.portA_SACNuni[1]);
        portA_SACN_universe.add(config.portA_SACNuni[2]);
        portA_SACN_universe.add(config.portA_SACNuni[3]);

        JsonObject portA_pixel = portA["pixel"].to<JsonObject>();
        portA_pixel["count"] = config.portA_pixel_count;
        portA_pixel["config"] = config.portA_pixel_config;
        portA_pixel["mode"] = config.portA_pixel_mode;
        portA_pixel["startFx"] = config.portA_pixel_startFX;

        JsonObject portB = doc["portB"].to<JsonObject>();
        portB["mode"] = config.portB_mode;
        portB["protocol"] = config.portB_prot;
        portB["merge"] = config.portB_merge;
        portB["net"] = config.portB_net;
        portB["subnet"] = config.portB_subnet;

        JsonArray portB_universe = portB["universe"].to<JsonArray>();
        portB_universe.add(config.portB_uni[0]);
        portB_universe.add(config.portB_uni[1]);
        portB_universe.add(config.portB_uni[2]);
        portB_universe.add(config.portB_uni[3]);

        JsonArray portB_SACN_universe = portB["SACN_universe"].to<JsonArray>();
        portB_SACN_universe.add(config.portB_SACNuni[0]);
        portB_SACN_universe.add(config.portB_SACNuni[1]);
        portB_SACN_universe.add(config.portB_SACNuni[2]);
        portB_SACN_universe.add(config.portB_SACNuni[3]);

        JsonObject portB_pixel = portB["pixel"].to<JsonObject>();
        portB_pixel["count"] = config.portB_pixel_count;
        portB_pixel["config"] = config.portB_pixel_config;
        portB_pixel["mode"] = config.portB_pixel_mode;
        portB_pixel["startFx"] = config.portB_pixel_startFX;

        JsonObject debug = doc["debug"].to<JsonObject>();
        debug["resetCounter"] = config.resetCounter;
        debug["wdtCounter"] = config.wdtCounter;
    }

    // Serialize JSON to file
    if (serializeJson(doc, configFile) == 0) {
        ret_val = false;
    }

    configFile.flush();
    configFile.close();

    return ret_val;
}