#include "state_machine.hpp"

/* State IDs — redefine or extend in your own code */
enum : uint8_t {
    SM_INIT = 0,
    SM_IDLE,
    SM_FAULT,
    SM_STANDBY,
    SM_ACTION,
    SM_PAUSE
};

StateMachine& StateMachine::get()
{
    static StateMachine root;
    static StateMachine sub;
    static bool init = false;

    if (!init) {
        init = true;

        root.addState(SM_INIT, 0);
        root.addState(SM_IDLE, 1);
        root.addState(SM_FAULT, 2);

        sub.addState(SM_STANDBY, 0);
        sub.addState(SM_ACTION, 1);
        sub.addState(SM_PAUSE, 2);

        root.getState(1)->setSubMachine(&sub);

        /* Example transitions — replace with your own conditions:
        root.addTransition(0, [] { return flagGet(COMMUNICATION_OK_FLAG); }, SM_IDLE);
        root.addTransition(1, [] { return flagGet(ERROR_FLAG); }, SM_FAULT);
        sub.addTransition(0, [] { return flagGet(ACTION_MOVE_FLAG); }, SM_ACTION);
        sub.addTransition(1, [] { return flagGet(ACTION_STOP_FLAG); }, SM_PAUSE);
        sub.addTransition(2, [] { return flagGet(ACTION_RESUME_FLAG); }, SM_ACTION);
        sub.addTransition(2, [] { return flagGet(ACTION_IDLE_FLAG); }, SM_STANDBY);
        sub.addTransition(1, [] { return flagGet(ACTION_IDLE_FLAG); }, SM_STANDBY);
        */

        /* Example entry/exit actions — replace with your own:
        root.addEntryAction(1, [] { idlePIN.write(true); });
        root.addExitAction(1, [] { idlePIN.write(false); });
        root.addEntryAction(2, [] { faultPIN.write(true); });
        root.addExitAction(2, [] { faultPIN.write(false); });
        */
    }

    return root;
}

bool State::addTransition(std::function<bool()> condition, uint8_t targetId)
{
    if (numTransitions >= MAX_TRANSITIONS)
        return false;
    transitions[numTransitions].condition = condition;
    transitions[numTransitions].targetId = targetId;
    numTransitions++;
    return true;
}

uint8_t State::checkTransitions() const
{
    for (uint8_t i = 0; i < numTransitions; i++) {
        if (transitions[i].condition())
            return transitions[i].targetId;
    }
    return 0xFF;
}

bool State::addEntryAction(std::function<void()> action)
{
    if (numEntryActions >= MAX_ACTIONS)
        return false;
    entryActions[numEntryActions] = action;
    numEntryActions++;
    return true;
}

void State::runEntryActions()
{
    for (uint8_t i = 0; i < numEntryActions; i++)
        entryActions[i]();
    if (subMachine != nullptr) {
        subMachine->setCurrentState(0);
        subMachine->getState(0)->runEntryActions();
    }
}

bool State::addExitAction(std::function<void()> action)
{
    if (numExitActions >= MAX_ACTIONS)
        return false;
    exitActions[numExitActions] = action;
    numExitActions++;
    return true;
}

void State::runExitActions()
{
    if (subMachine != nullptr)
        subMachine->getState(subMachine->getCurrentState())->runExitActions();
    for (uint8_t i = 0; i < numExitActions; i++)
        exitActions[i]();
}

void State::updateSubMachine()
{
    if (subMachine != nullptr)
        subMachine->update();
}

StateMachine::StateMachine()
    : numStates(0), currentState(0)
{
}

bool StateMachine::addState(uint8_t id, uint8_t index)
{
    if (index >= MAX_STATES)
        return false;
    states[index] = State(id);
    if (index >= numStates)
        numStates = index + 1;
    return true;
}

State *StateMachine::getState(uint8_t index)
{
    if (index >= MAX_STATES)
        return nullptr;
    return &states[index];
}

uint8_t StateMachine::getNumStates() const
{
    return numStates;
}

bool StateMachine::addTransition(uint8_t stateIndex, std::function<bool()> condition, uint8_t targetId)
{
    if (stateIndex >= MAX_STATES)
        return false;
    return states[stateIndex].addTransition(condition, targetId);
}

bool StateMachine::addEntryAction(uint8_t stateIndex, std::function<void()> action)
{
    if (stateIndex >= MAX_STATES)
        return false;
    return states[stateIndex].addEntryAction(action);
}

bool StateMachine::addExitAction(uint8_t stateIndex, std::function<void()> action)
{
    if (stateIndex >= MAX_STATES)
        return false;
    return states[stateIndex].addExitAction(action);
}

void StateMachine::setCurrentState(uint8_t index)
{
    if (index < MAX_STATES)
        currentState = index;
}

uint8_t StateMachine::getCurrentState() const
{
    return currentState;
}

uint8_t StateMachine::findIndexById(uint8_t id) const
{
    for (uint8_t i = 0; i < numStates; i++) {
        if (states[i].getId() == id)
            return i;
    }
    return 0xFF;
}

void StateMachine::update()
{
    uint8_t targetId = states[currentState].checkTransitions();
    if (targetId != 0xFF) {
        uint8_t idx = findIndexById(targetId);
        if (idx < MAX_STATES) {
            states[currentState].runExitActions();
            currentState = idx;
            states[currentState].runEntryActions();
        }
    }
    states[currentState].updateSubMachine();
}
