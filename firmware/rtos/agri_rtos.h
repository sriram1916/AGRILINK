#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace agri {

constexpr UBaseType_t AGRILINK_TASK_PRIORITY_SAFETY = configMAX_PRIORITIES - 1;
constexpr UBaseType_t AGRILINK_TASK_PRIORITY_MISSION = configMAX_PRIORITIES - 2;
constexpr UBaseType_t AGRILINK_TASK_PRIORITY_NAVIGATION = configMAX_PRIORITIES - 3;
constexpr UBaseType_t AGRILINK_TASK_PRIORITY_CONTROL = configMAX_PRIORITIES - 4;
constexpr UBaseType_t AGRILINK_TASK_PRIORITY_COMMUNICATION = configMAX_PRIORITIES - 5;
constexpr UBaseType_t AGRILINK_TASK_PRIORITY_DIAGNOSTICS = configMAX_PRIORITIES - 6;
constexpr UBaseType_t AGRILINK_TASK_PRIORITY_SIMULATION = configMAX_PRIORITIES - 7;
constexpr UBaseType_t AGRILINK_TASK_PRIORITY_BOOT = configMAX_PRIORITIES - 8;

}  // namespace agri
