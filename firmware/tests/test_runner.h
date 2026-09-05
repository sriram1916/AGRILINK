#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace agri::tests {

// Phase 8 — deterministic validation layer.
//
// The test runner is an observer. It only reads existing Phase 2-7
// observables and emits clearly-prefixed [TEST] log lines. It does
// not modify any subsystem state and does not bypass any layer.

void test_runner_init(void);
void vTestRunnerTask(void *arg);

}  // namespace agri::tests
