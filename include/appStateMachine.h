#ifndef APP_STATE_MACHINE_H
#define APP_STATE_MACHINE_H

#include "types.h"

class AppStateMachine {
public:
    static AppStateMachine& getInstance() {
        static AppStateMachine instance;
        return instance;
    }
    
    void init();
    void update();
    void onEvent(const event_t& event);
    
    app_state_t getState() const { return current_state; }
    const char* getStateString() const;
    
private:
    AppStateMachine();
    
    app_state_t current_state;
    app_state_t previous_state;
    uint32_t state_start_time;
    uint32_t still_since_detecting;  // tracks stillness timer across DETECTING re-entries
    
    // State transitions
    void transitionTo(app_state_t new_state);
    
    // State handlers (virtual-like via templates)
    void onStateInit();
    void onStateIdle();
    void onStateDetecting();
    void onStateAlert();
    void onStateSleeping();
};

#endif
