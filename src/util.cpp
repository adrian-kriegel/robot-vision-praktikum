
#include "util.hpp"

#include <algorithm>

float util::clamp(float val, float minmax)
{
  return std::min(std::max(val, -minmax), minmax);
}