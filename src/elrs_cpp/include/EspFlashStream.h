#pragma once

#include "Arduino.h"

// SiW917 stores runtime config through NVM3 rather than ESP flash streams.
// Keep the API surface parse-compatible for shared ELRS code.
class EspFlashStream : public Stream {
public:
  EspFlashStream() = default;
  ~EspFlashStream() override = default;

  bool availableForWrite() const { return false; }
};
