#pragma once

#include <stdint.h>

template <typename SumT, typename ValueT, int DefaultPrevious>
class MeanAccumulator {
public:
  MeanAccumulator() { reset(); }

  void reset() {
    sum_ = 0;
    count_ = 0;
    previousMean_ = static_cast<ValueT>(DefaultPrevious);
  }

  void add(ValueT value) {
    sum_ += static_cast<SumT>(value);
    ++count_;
  }

  uint32_t getCount() const { return count_; }

  ValueT mean() {
    if (count_ == 0) {
      return previousMean_;
    }

    previousMean_ = static_cast<ValueT>(sum_ / static_cast<SumT>(count_));
    sum_ = 0;
    count_ = 0;
    return previousMean_;
  }

  ValueT previousMean() const { return previousMean_; }

private:
  SumT sum_;
  uint32_t count_;
  ValueT previousMean_;
};
