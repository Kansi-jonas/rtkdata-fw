#ifndef STATE_H
#define STATE_H

typedef enum {
    ADMINMODE,
    HOSTINGMODE
} State_t;

extern State_t currentState;

void init_state();

void set_state(State_t state);

State_t get_state();

#endif // STATE_H
