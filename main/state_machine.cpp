#include "state_machine.hpp"
#include "sm_events.hpp"
#include "Arm.hpp"

extern Arm arm;

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

        /* Raiz */
        root.addTransition(0, [] { return sm_isReady(); }, SM_IDLE);                  /* INIT   -> IDLE */
        root.addTransition(1, [] { return sm_consume(SmEvent::STOP); }, SM_FAULT);    /* IDLE   -> FAULT (seta) */
        root.addTransition(2, [] { return sm_consume(SmEvent::RESUME); }, SM_IDLE);   /* FAULT  -> IDLE (reanudar) */

        /* Submaquina de IDLE */
        sub.addTransition(0, [] { return sm_consume(SmEvent::MOVE); }, SM_ACTION);    /* STANDBY -> ACTION */
        sub.addTransition(1, [] { return !arm.isMoving(); }, SM_STANDBY);             /* ACTION  -> STANDBY (fin de mov) */
        sub.addTransition(1, [] { return sm_consume(SmEvent::PAUSE); }, SM_PAUSE);    /* ACTION  -> PAUSE */
        sub.addTransition(2, [] { return sm_consume(SmEvent::RESUME); }, SM_ACTION);  /* PAUSE   -> ACTION */
        sub.addTransition(2, [] { return !arm.isMoving(); }, SM_STANDBY);             /* PAUSE   -> STANDBY (fin de mov) */
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

bool StateMachine::canDoAction() const
{
    if (currentState != 1)   /* solo en IDLE */
        return false;
    StateMachine *sub = states[1].getSubMachine();
    return sub && sub->getCurrentState() == 0;   /* submáquina en STANDBY */
}

uint8_t StateMachine::getSubState() const
{
    if (currentState != 1)
        return 0xFF;
    StateMachine *sub = states[1].getSubMachine();
    return sub ? sub->getCurrentState() : 0xFF;
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
