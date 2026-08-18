/**
 * @file options.cpp
 * @brief Firmware options implementation for ELRS on SiW917
 *
 * Provides configuration storage and initialization for the receiver/transmitter.
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
#if defined(TARGET_TX)
const unsigned char target_name[] = "SiW917_ELRS_TX";
#else
const unsigned char target_name[] = "SiW917_ELRS_RX";
#endif
const uint8_t target_name_size = sizeof(target_name) - 1;
const char commit[] = "dev";
const char version[] = "4.0.0";

// WiFi configuration defaults (not used on SiW917 in same way as ESP)
const char *wifi_hostname = "elrs_siw917";
#if defined(TARGET_TX)
const char *wifi_ap_ssid = "ExpressLRS TX";
#else
const char *wifi_ap_ssid = "ExpressLRS RX";
#endif
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

static uint8_t domainFromConfig(const elrs_config_t *cfg)
{
    if (cfg == nullptr) {
        return FCC915;
    }

    switch (cfg->reg_domain_low) {
    case ELRS_DOMAIN_AU_915:
        return AU915;
    case ELRS_DOMAIN_FCC_915:
        return FCC915;
    case ELRS_DOMAIN_EU_868:
        return EU868;
    case ELRS_DOMAIN_IN_866:
        return IN866;
    case ELRS_DOMAIN_AU_433:
        return AU433;
    case ELRS_DOMAIN_EU_433:
        return EU433;
    case ELRS_DOMAIN_ISM_2400:
        return ISM2G4;
    case ELRS_DOMAIN_CE_2400:
        return CE2G4;
    default:
        return FCC915;
    }
}

// Device name storage
#if defined(TARGET_TX)
char device_name[ELRSOPTS_DEVICENAME_SIZE] = "SiW917 TX";
#else
char device_name[ELRSOPTS_DEVICENAME_SIZE] = "SiW917 RX";
#endif
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
#if defined(TARGET_TX)
    .tlm_report_interval = 240,
    ._unused1 = false,
    .unlock_higher_power = false,
    .is_airport = false,
    .uart_baud = 400000,
#endif
};

bool options_init()
{
    elrs_config_t *cfg = elrs_config_get();

    firmwareOptions.domain = domainFromConfig(cfg);
#if defined(TARGET_TX) && defined(SIW917_ELRS_TX_PC_BENCH)
    // Bench mode keeps the LR1121 primary FHSS table on FCC915. The startup
    // rate is still applied from NVM by the upstream TX wrapper.
    firmwareOptions.domain = FCC915;
    DBGLN("[TXBENCH] Bench FHSS map: primary=FCC915 dual=ISM2G4");
#endif
#if defined(TARGET_TX)
    if (cfg != nullptr) {
        firmwareOptions.uart_baud = (cfg->serial_protocol == ELRS_SERIAL_CRSF)
                                        ? 400000U
                                        : firmwareOptions.uart_baud;
    }
#endif
    loadFlashedUid();
    /* Runtime UID is the last Lua/WebUI bind phrase if one was saved. */
    if (cfg != nullptr && elrs_config_is_bound()) {
      memcpy(firmwareOptions.uid, cfg->uid, sizeof(firmwareOptions.uid));
      firmwareOptions.hasUID = 1;
    }

    // firmwareOptions.uid is the flashed/home UID. Runtime binding is loaded
    // from elrs_config by rx_main, matching upstream Returnable Bind Storage.
#if OPTIONS_UID_DIAG
    elrs_config_t* cfg = elrs_config_get();
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

bool options_HasStringInFlash(EspFlashStream &strmFlash)
{
    (void)strmFlash;
    return false;
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
#if defined(TARGET_TX)
    firmwareOptions.tlm_report_interval = 240;
    firmwareOptions._unused1 = false;
    firmwareOptions.unlock_higher_power = false;
    firmwareOptions.is_airport = false;
    firmwareOptions.uart_baud = 400000;
#endif
}
