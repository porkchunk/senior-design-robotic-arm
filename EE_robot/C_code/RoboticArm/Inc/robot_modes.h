#ifndef ROBOT_MODES_H
#define ROBOT_MODES_H

#define AUTO_MANUAL_SWITCH_PIN 15
#define AUTO_START_SWITCH 0
#define STARTING_X 25.75
#define STARTING_Y 0
#define STARTING_Z 19.9
#define STARTING_PITCH 0

void manual_mode();
void automatic_mode();
void initialize_auto_manual_pin();
void initialize_start_auto_mode_pin();

#endif