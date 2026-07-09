/**
 * @file options.cpp
 * @brief Firmware options implementation for ELRS on SiW917
 *
 * Provides configuration storage and initialization for the receiver.
 */

#include "options.h"
#include "logging.h"
#include <string.h>

// NVM3 persistent config integration
extern "C" {
#include "elrs_config.h"
}

// Frequency domain enum (matches upstream OTA.h)
// Defined here before use in firmwareOptions initializer
enum : uint8_t {
    AU915 = 0,
    FCC915 = 1,
    EU868 = 2,
    IN866 = 3,
    AU433 = 4,
    EU433 = 5,
    US433 = 6,
    US433W = 7,
    ISM2G4 = 8,
    CE2G4 = 9,
};

// Target identification
const unsigned char target_name[] = "SiW917_ELRS_RX";
const uint8_t target_name_size = sizeof(target_name) - 1;
const char commit[] = "dev";
const char version[] = "4.0.0";

// WiFi configuration defaults (not used on SiW917 in same way as ESP)
const char *wifi_hostname = "elrs_siw917";
const char *wifi_ap_ssid = "ExpressLRS RX";
const char *wifi_ap_password = "expresslrs";
const char *wifi_ap_address = "10.0.0.1";

#ifndef OPTIONS_UID_DIAG
#define OPTIONS_UID_DIAG 0
#endif

static constexpr uint8_t FLASHED_UID[6] = {0xBE, 0x93, 0x67,
                                           0x27, 0xD6, 0x9C};

static void loadFlashedUid()
{
    firmwareOptions.hasUID = 1;
    memcpy(firmwareOptions.uid, FLASHED_UID, sizeof(firmwareOptions.uid));
}

// Device name storage
char device_name[ELRSOPTS_DEVICENAME_SIZE] = "SiW917 RX";
char product_name[ELRSOPTS_PRODUCTNAME_SIZE] = "ExpressLRS SiW917";
uint32_t logo_image = 0;

// Firmware options with defaults
firmware_options_t firmwareOptions = {
    ._magic_ = {'E', 'L', 'R', 'S', 'O', 'P', 'T', 'S'},
    ._version_ = 1,
    .domain = FCC915,
    .hasUID = 0,
    .uid = {0, 0, 0, 0, 0, 0},
    .flash_discriminator = 0,
    .fan_min_runtime = 0,
    .wifi_auto_on_interval = -1,  // Disabled
    .home_wifi_ssid = "",
    .home_wifi_password = "",
#if defined(TARGET_RX)
    .uart_baud = 420000,
    ._unused1 = false,
    .lock_on_first_connection = true,
    .dji_permanently_armed = false,
    .is_airport = false,
#endif
};

bool options_init()
{
    elrs_config_t *cfg = elrs_config_get();
    firmwareOptions.domain = cfg ? elrs_config_get_web_domain()
                                 : static_cast<uint8_t>(FCC915);
#if SIW917_ELRS_LR2021_FORCE_2G4_DOMAIN
    firmwareOptions.domain = static_cast<uint8_t>(ISM2G4);
#endif
    firmwareOptions.uart_baud = cfg ? elrs_config_get_uart_baud() : 420000;
#if defined(TARGET_RX)
    firmwareOptions.wifi_auto_on_interval =
        cfg ? elrs_config_get_wifi_on_interval() : 60;
    memset(firmwareOptions.home_wifi_ssid, 0, sizeof(firmwareOptions.home_wifi_ssid));
    memset(firmwareOptions.home_wifi_password, 0, sizeof(firmwareOptions.home_wifi_password));
    if (cfg != nullptr) {
        strncpy(firmwareOptions.home_wifi_ssid, cfg->wifi_ssid,
                sizeof(firmwareOptions.home_wifi_ssid) - 1);
        strncpy(firmwareOptions.home_wifi_password, cfg->wifi_password,
                sizeof(firmwareOptions.home_wifi_password) - 1);
    }
    firmwareOptions.lock_on_first_connection =
        cfg ? elrs_config_get_lock_on_first_connection() : true;
    firmwareOptions.dji_permanently_armed =
        cfg ? elrs_config_get_dji_permanently_armed() : false;
    firmwareOptions.is_airport =
        cfg ? elrs_config_get_is_airport() : false;
#endif
    loadFlashedUid();

    // firmwareOptions.uid is the flashed/home UID. Runtime binding is loaded
    // from elrs_config by rx_main, matching upstream Returnable Bind Storage.
#if OPTIONS_UID_DIAG
    if (cfg != nullptr) {
        DBGLN("Config UID in NVM3: %02X:%02X:%02X:%02X:%02X:%02X",
              cfg->uid[0], cfg->uid[1], cfg->uid[2], cfg->uid[3],
              cfg->uid[4], cfg->uid[5]);
    }
    DBGLN("Flashed UID: %02X:%02X:%02X:%02X:%02X:%02X",
          firmwareOptions.uid[0], firmwareOptions.uid[1],
          firmwareOptions.uid[2], firmwareOptions.uid[3],
          firmwareOptions.uid[4], firmwareOptions.uid[5]);
#endif

#if SIW917_ELRS_LR2021_FORCE_2G4_DOMAIN
    DBGLN("LR2021 diag: forcing ISM2G4 domain for 2.4GHz mode scan");
#endif
    DBGLN("Options initialized - domain=%d, baud=%lu",
          firmwareOptions.domain, firmwareOptions.uart_baud);

    return true;
}

// Placeholder implementations for flash storage (not fully implemented yet)
static char optionsJson[ELRSOPTS_OPTIONS_SIZE] = "{}";
static char hardwareJson[ELRSOPTS_HARDWARE_SIZE] = "{}";

String optionsString(optionsJson);
String hardwareString(hardwareJson);

String& getOptions()
{
    return optionsString;
}

String& getHardware()
{
    return hardwareString;
}

void saveOptions()
{
    // Save options to NVM3 persistent storage
    elrs_config_t* cfg = elrs_config_get();
    if (cfg) {
        // Save to flash
        int result = elrs_config_save();
        if (result == 0) {
            DBGLN("Options saved to NVM3");
        } else {
            DBGLN("ERROR: Failed to save options to NVM3");
        }
    } else {
        DBGLN("ERROR: Config not available for save");
    }
}

void setOptions(String &options)
{
    strncpy(optionsJson, options.c_str(), sizeof(optionsJson) - 1);
    optionsJson[sizeof(optionsJson) - 1] = '\0';
}

void options_SetTrueDefaults()
{
    // Reset to compiled defaults
    firmwareOptions.domain = FCC915;
    loadFlashedUid();
    firmwareOptions.flash_discriminator = 0;
    firmwareOptions.fan_min_runtime = 0;
    firmwareOptions.wifi_auto_on_interval = -1;
    memset(firmwareOptions.home_wifi_ssid, 0, sizeof(firmwareOptions.home_wifi_ssid));
    memset(firmwareOptions.home_wifi_password, 0, sizeof(firmwareOptions.home_wifi_password));
#if defined(TARGET_RX)
    firmwareOptions.uart_baud = 420000;
    firmwareOptions._unused1 = false;
    firmwareOptions.lock_on_first_connection = true;
    firmwareOptions.dji_permanently_armed = false;
    firmwareOptions.is_airport = false;
#endif
}
