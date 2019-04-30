#ifndef UTILITIES_FILTER_H
#define UTILITIES_FILTER_H

namespace utilities
{
  /**
  * A simple low pass filter from the VESC firmware
  *
  * @param value
  * The filtered value.
  *
  * @param sample
  * Next sample.
  *
  * @param filter_constant
  * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
  */
  template <typename type>
  void simpleLowPassFilter(type &value, type sample, type filter_constant)
  {
    value -= filter_constant * (value - sample);
  }
}

#endif //UTILITIES_FILTER_H
