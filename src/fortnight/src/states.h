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

#endif
