#include "taskSequence.h"
#include <Arduino.h>

enum SequenceStepEndType {
    TIMED_TASK,
    WAITING_TASK,
    WAITING_TIMED_TASK
};

struct SequenceStep {
    void (*task)();
    enum SequenceStepEndType endType;
    bool (*endCondition)();
    int length;
    bool timedOut;
    bool runIfTimedOut;
};

int currentTaskStartTime;

SequenceStep tasks[MAX_NUM_TASKS];

bool taskInProgress = false;

int currentTask = -1;
int lastTaskIndex = -1;

void addTimedTask(void (*task)(), int taskTime_ms) {
    lastTaskIndex++;
    tasks[lastTaskIndex] = (SequenceStep){task, TIMED_TASK, nullptr, taskTime_ms, false, false};
}

void addTimedTask(void (*task)(), int taskTime_ms, bool runIfTimedOut) {
    lastTaskIndex++;
    tasks[lastTaskIndex] = (SequenceStep){task, TIMED_TASK, nullptr, taskTime_ms, false, runIfTimedOut};
}

void addWaitingTask(void (*task)(), bool (*isComplete)()) {
    lastTaskIndex++;
    tasks[lastTaskIndex] = (SequenceStep){task, WAITING_TASK, isComplete, -1, false, false};
}

void addWaitingTask(void (*task)(), bool (*isComplete)(), bool runIfTimedOut) {
    lastTaskIndex++;
    tasks[lastTaskIndex] = (SequenceStep){task, WAITING_TASK, isComplete, -1, false, runIfTimedOut};
}

void addWaitingTimedTask(void (*task)(), bool (*isComplete)(), int taskTime_ms) {
    lastTaskIndex++;
    tasks[lastTaskIndex] = (SequenceStep){task, WAITING_TIMED_TASK, isComplete, taskTime_ms, false, false};
}

void addWaitingTimedTask(void (*task)(), bool (*isComplete)(), int taskTime_ms, bool runIfTimedOut) {
    lastTaskIndex++;
    tasks[lastTaskIndex] = (SequenceStep){task, WAITING_TIMED_TASK, isComplete, taskTime_ms, false, runIfTimedOut};
}

void beginNextTask() {

    currentTask++;
    while (tasks[currentTask].runIfTimedOut && !tasks[currentTask - 1].timedOut) {
        tasks[currentTask].timedOut = true;
        currentTask++;
    }

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
                    tasks[currentTask].timedOut = true;
                    finishTask();
                }
                break;
                
            case WAITING_TASK:
                if ((tasks[currentTask].endCondition)()) {
                    tasks[currentTask].timedOut = false;
                    finishTask();
                }
                break;
            case WAITING_TIMED_TASK:
                if (millis() > currentTaskStartTime + tasks[currentTask].length) {
                    tasks[currentTask].timedOut = true;
                    finishTask();
                } else if ((tasks[currentTask].endCondition)()) {
                    tasks[currentTask].timedOut = false;
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
