#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <stdint.h>
#include <functional>

#define MAX_TRANSITIONS 3
#define MAX_ACTIONS 3
#define MAX_STATES 6

class StateMachine;

struct Transition {
    std::function<bool()> condition;
    uint8_t targetId;
};

class State {
public:
    State() : id(0), numTransitions(0), numEntryActions(0), numExitActions(0), subMachine(nullptr) {}
    State(uint8_t id) : id(id), numTransitions(0), numEntryActions(0), numExitActions(0), subMachine(nullptr) {}
    uint8_t getId() const { return id; }
    void setId(uint8_t id) { this->id = id; }

    bool addTransition(std::function<bool()> condition, uint8_t targetId);
    uint8_t checkTransitions() const;

    bool addEntryAction(std::function<void()> action);
    void runEntryActions();

    bool addExitAction(std::function<void()> action);
    void runExitActions();

    void setSubMachine(StateMachine *sm) { subMachine = sm; }
    StateMachine *getSubMachine() const { return subMachine; }
    void updateSubMachine();

private:
    uint8_t id;
    Transition transitions[MAX_TRANSITIONS];
    uint8_t numTransitions;
    std::function<void()> entryActions[MAX_ACTIONS];
    uint8_t numEntryActions;
    std::function<void()> exitActions[MAX_ACTIONS];
    uint8_t numExitActions;
    StateMachine *subMachine;
};

class StateMachine {
public:
    static StateMachine& get();

    StateMachine();

    bool addState(uint8_t id, uint8_t index);
    State *getState(uint8_t index);
    uint8_t getNumStates() const;

    bool addTransition(uint8_t stateIndex, std::function<bool()> condition, uint8_t targetId);
    bool addEntryAction(uint8_t stateIndex, std::function<void()> action);
    bool addExitAction(uint8_t stateIndex, std::function<void()> action);
    void setCurrentState(uint8_t index);
    uint8_t getCurrentState() const;
    void update();

private:
    uint8_t findIndexById(uint8_t id) const;

    State states[MAX_STATES];
    uint8_t numStates;
    uint8_t currentState;
};

#endif
