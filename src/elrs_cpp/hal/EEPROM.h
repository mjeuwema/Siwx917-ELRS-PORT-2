#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern "C" {
#include "nvm3.h"
#include "nvm3_default.h"
}

class EEPROMClass
{
public:
    bool begin(size_t size)
    {
        activeSize = size > sizeof(storage) ? sizeof(storage) : size;
        if (!initialized)
        {
            memset(storage, 0xFF, sizeof(storage));
            nvmReady = initNvm3();
            if (nvmReady)
            {
                loadFromNvm3();
            }
            initialized = true;
        }
        return true;
    }

    uint8_t read(size_t address) const
    {
        return address < activeSize ? storage[address] : 0xFF;
    }

    void write(size_t address, uint8_t value)
    {
        if (address < activeSize)
        {
            if (storage[address] == value)
            {
                return;
            }
            storage[address] = value;
            dirty[address / CHUNK_SIZE] = true;
        }
    }

    bool commit()
    {
        if (!nvmReady)
        {
            return true;
        }

        bool ok = true;
        for (size_t chunk = 0; chunk < CHUNK_COUNT; ++chunk)
        {
            if (!dirty[chunk])
            {
                continue;
            }

            const size_t offset = chunk * CHUNK_SIZE;
            if (offset >= activeSize)
            {
                dirty[chunk] = false;
                continue;
            }

            size_t len = activeSize - offset;
            if (len > CHUNK_SIZE)
            {
                len = CHUNK_SIZE;
            }

            Ecode_t status = nvm3_writeData(nvm3_defaultHandle,
                                            KEY_BASE + (uint32_t)chunk,
                                            &storage[offset],
                                            len);
            if (status == ECODE_NVM3_OK)
            {
                dirty[chunk] = false;
            }
            else
            {
                ok = false;
            }
        }

        if (ok && nvm3_repackNeeded(nvm3_defaultHandle))
        {
            (void)nvm3_repack(nvm3_defaultHandle);
        }

        return ok;
    }

private:
    static constexpr uint32_t KEY_BASE = 0x00040;
    static constexpr size_t CHUNK_SIZE = 128;
    static constexpr size_t EEPROM_SIZE = 1024;
    static constexpr size_t CHUNK_COUNT =
        (EEPROM_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE;

    bool initNvm3()
    {
        const Ecode_t status = nvm3_initDefault();
        return status == ECODE_NVM3_OK;
    }

    void loadFromNvm3()
    {
        for (size_t chunk = 0; chunk < CHUNK_COUNT; ++chunk)
        {
            const size_t offset = chunk * CHUNK_SIZE;
            if (offset >= activeSize)
            {
                break;
            }

            uint32_t objectType = 0;
            size_t objectLen = 0;
            Ecode_t status = nvm3_getObjectInfo(nvm3_defaultHandle,
                                                KEY_BASE + (uint32_t)chunk,
                                                &objectType,
                                                &objectLen);
            if (status != ECODE_NVM3_OK || objectType != NVM3_OBJECTTYPE_DATA)
            {
                continue;
            }

            size_t len = objectLen;
            const size_t available = activeSize - offset;
            if (len > CHUNK_SIZE)
            {
                len = CHUNK_SIZE;
            }
            if (len > available)
            {
                len = available;
            }

            (void)nvm3_readData(nvm3_defaultHandle,
                                KEY_BASE + (uint32_t)chunk,
                                &storage[offset],
                                len);
        }
    }

    bool initialized = false;
    bool nvmReady = false;
    size_t activeSize = EEPROM_SIZE;
    uint8_t storage[EEPROM_SIZE] = {};
    bool dirty[CHUNK_COUNT] = {};
};

static EEPROMClass EEPROM;
