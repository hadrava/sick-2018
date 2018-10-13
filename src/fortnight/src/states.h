#ifndef STATE_H_
#define STATE_H_

#define STATE_DISABLED           0 // no action

#define STATE_START              1
#define STATE_WAIT_FOR_OBJECT    2
#define STATE_ROTATING           3
#define STATE_FOLLOWING          4
#define STATE_PERFORMING_ACTION  5
#define STATE_LEAVING            6
#define STATE_STORNO             7 // same as leaving but transits to cancelled

#define STATE_CANCELLED          8 // no action
#define STATE_FINISHED           9 // no action


#define COMMAND_DISABLE      0 // will slowly leave transporter and end, if it fails, feel free to perform reset
#define COMMAND_ENABLE       1 // sending during whole action
#define COMMAND_RESET        2 // always resets to STATE_DISABLED



#define D_STATE_DISABLED      0 // no action

// order is important!
#define D_STATE_START         1 // ends immediately, resets history
#define D_STATE_ROTATE_TO_1   2 // rotating to first point
#define D_STATE_MOVE_TO_1     3 // moving to first point
#define D_STATE_ROTATE_TO_2   4 // rotating to second point
#define D_STATE_MOVE_TO_2     5 // moving to second point
#define D_STATE_DISPOSE_START 6 // sending release signal, changed when not at home
#define D_STATE_DISPOSE_WAIT  7 // sending release signal, waiting
#define D_STATE_DISPOSE_END   8 // sending GRABBER_DISABLED, waiting for at_home
#define D_STATE_MOVE_TO_3     9 // moving to third point

#define D_STATE_FINISHED      10 // no action
#define D_STATE_CANCELLED     11 // no action

#endif
