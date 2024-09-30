#pragma once
// The maximum number of tasks which can be created within the lifetime of the program
#define MAX_NUM_TASKS 100

// The provided function will be run once when the task starts. The next task will be run after taskTime_ms
void addTimedTask(void (*task)(), int taskTime_ms);

// The provided function will be run once when the task starts. The next function will not be run until the second providedfunction returns true. 
void addWaitingTask(void (*task)(), bool (*isComplete)());

// Call this in the main loop to process the task sequence.
void tickTaskSequence();