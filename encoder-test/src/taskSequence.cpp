#include "taskSequence.h"
#include <Arduino.h>

enum SequenceStepEndType {
    TIMED_TASK,
    WAITING_TASK
};

struct SequenceStep {
    void (*task)();
    enum SequenceStepEndType endType;
    bool (*endCondition)();
    int length;
};

int currentTaskStartTime;

SequenceStep tasks[MAX_NUM_TASKS];

bool taskInProgress = false;

int currentTask = -1;
int lastTaskIndex = -1;

void addTimedTask(void (*task)(), int taskTime_ms) {
    lastTaskIndex++;
    tasks[lastTaskIndex] = (SequenceStep){task, TIMED_TASK, nullptr, taskTime_ms};
}

void addWaitingTask(void (*task)(), bool (*isComplete)()) {
    lastTaskIndex++;
    tasks[lastTaskIndex] = (SequenceStep){task, WAITING_TASK, isComplete, -1};
}

void beginNextTask() {
    currentTask++;
    currentTaskStartTime = millis();
    taskInProgress = true;
    (tasks[currentTask].task)();
}

void finishTask() {
    if (lastTaskIndex > currentTask) {
        beginNextTask();
    }
    else {
        taskInProgress = false;
    }
}

void tickTaskSequence() {
    if (taskInProgress) {
        switch (tasks[currentTask].endType) {
            case TIMED_TASK:
                if (millis() > currentTaskStartTime + tasks[currentTask].length) {
                    finishTask();
                }
                break;
                
            case WAITING_TASK:
                if ((tasks[currentTask].endCondition)()) {
                    finishTask();
                }
                break;
        }
    }
    else {
        if (lastTaskIndex > currentTask) {
            beginNextTask();
        }
    }
}

