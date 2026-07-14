#pragma once

#include <stdint.h>

class TwoWire {
public:
    void begin(int sda = -1, int scl = -1) {
        (void)sda;
        (void)scl;
    }

    void setClock(uint32_t frequency) {
        (void)frequency;
    }
};

extern TwoWire Wire;
