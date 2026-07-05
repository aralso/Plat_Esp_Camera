#include "cpu_load.h"

#if defined(configGENERATE_RUN_TIME_STATS) && (configGENERATE_RUN_TIME_STATS == 1)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <string.h>

float measure_cpu_load_ms(uint32_t period_ms)
{
    // Get number of tasks
    UBaseType_t nTasks = uxTaskGetNumberOfTasks();
    if (nTasks == 0) return -1.0f;

    TaskStatus_t *start = (TaskStatus_t*)pvPortMalloc(nTasks * sizeof(TaskStatus_t));
    TaskStatus_t *end   = (TaskStatus_t*)pvPortMalloc(nTasks * sizeof(TaskStatus_t));
    if (!start || !end) {
        if (start) vPortFree(start);
        if (end) vPortFree(end);
        return -1.0f;
    }

    uint32_t totalStart = 0;
    UBaseType_t gotStart = uxTaskGetSystemState(start, nTasks, &totalStart);
    if (gotStart == 0 || totalStart == 0) {
        vPortFree(start); vPortFree(end);
        return -1.0f;
    }

    // Wait observation period
    vTaskDelay(pdMS_TO_TICKS(period_ms));

    uint32_t totalEnd = 0;
    UBaseType_t gotEnd = uxTaskGetSystemState(end, nTasks, &totalEnd);
    if (gotEnd == 0 || totalEnd <= totalStart) {
        vPortFree(start); vPortFree(end);
        return -1.0f;
    }

    uint32_t totalDelta = totalEnd - totalStart;
    uint64_t idleDelta = 0;

    // Sum idle task deltas (IDLE0/IDLE1). Compare by handle.
    for (UBaseType_t i = 0; i < gotEnd; ++i) {
        TaskStatus_t *tEnd = &end[i];
        // Check task name commonly used for idle tasks
        if (tEnd->pcTaskName && (strcmp(tEnd->pcTaskName, "IDLE") == 0 || strcmp(tEnd->pcTaskName, "IDLE0") == 0 || strcmp(tEnd->pcTaskName, "IDLE1") == 0)) {
            // find matching start entry by handle
            for (UBaseType_t j = 0; j < gotStart; ++j) {
                if (start[j].xHandle == tEnd->xHandle) {
                    uint32_t delta = tEnd->ulRunTimeCounter - start[j].ulRunTimeCounter;
                    idleDelta += delta;
                    break;
                }
            }
        }
    }

    if (totalDelta == 0) {
        vPortFree(start); vPortFree(end);
        return -1.0f;
    }

    double idleFrac = (double)idleDelta / (double)totalDelta;
    if (idleFrac < 0.0) idleFrac = 0.0;
    if (idleFrac > 1.0) idleFrac = 1.0;

    float cpuLoad = (float)((1.0 - idleFrac) * 100.0);

    vPortFree(start);
    vPortFree(end);
    return cpuLoad;
}

#else

// Run-time stats not enabled: stub implementation
#include <Arduino.h>
float measure_cpu_load_ms(uint32_t period_ms)
{
    (void)period_ms;
    Serial.println("measure_cpu_load_ms: run-time stats not enabled in FreeRTOS config");
    return -1.0f;
}

#endif
