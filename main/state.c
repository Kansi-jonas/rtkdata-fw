#include <stdio.h>
#include <stdint.h> 
#include "state.h"
#include "config.h"

State_t currentState; 

void init_state(){

    uint16_t state          = config_get_u16(CONF_ITEM(KEY_CONFIG_MODE_ACTIVE));

    currentState   = state;
}

void set_state(State_t state) {

    currentState = state;

    // set the state data in the memory
    config_set_u16(KEY_CONFIG_MODE_ACTIVE,state);

}

State_t get_state(){

    return config_get_u16(CONF_ITEM(KEY_CONFIG_MODE_ACTIVE));
}
