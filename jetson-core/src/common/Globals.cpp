// File: src/common/Globals.cpp
// Global runtime flags shared across pipeline components
#include "adas/common/Globals.hpp"

namespace adas {

// Global verbose mode for debug output
// Toggle via menu option 8 at runtime
std::atomic<bool> g_verbose_mode{false};

} // namespace adas
