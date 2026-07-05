#pragma once

#include <stdint.h>

// Measure CPU load over a period in milliseconds.
// Returns CPU usage percentage (0.0 - 100.0) or a negative value on error (e.g., run-time stats not enabled).
float measure_cpu_load_ms(uint32_t period_ms);
