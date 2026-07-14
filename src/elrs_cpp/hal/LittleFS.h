#pragma once

class LittleFSClass {
public:
    bool exists(const char *path) const {
        (void)path;
        return false;
    }
};

static LittleFSClass LittleFS;
