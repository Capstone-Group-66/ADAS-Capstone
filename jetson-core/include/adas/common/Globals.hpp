// File: include/adas/common/Globals.hpp
// Global runtime flags accessible across the pipeline
// NOTE: These are runtime-configurable via the interactive menu.
#pragma once

#include <atomic>

namespace adas {

/// Global verbose mode for debug output.
/// Toggle via menu option 8 at runtime.
/// Use this flag for any component that needs verbose debug logging.
extern std::atomic<bool> g_verbose_mode;

} // namespace adas
