#ifndef ROBOT_MODES_H
#define ROBOT_MODES_H

#define AUTO_MANUAL_SWITCH_PIN 15
#define AUTO_START_SWITCH 0
#define STARTING_X 61
#define STARTING_Y 0
#define STARTING_Z 6
#define STARTING_PITCH 0
#define STARTING_YAW 0

#define BUTTON_LEFT 14 //prev 14
#define BUTTON_RIGHT 15 //prev 15

void manual_mode();
void automatic_mode();
void initialize_auto_manual_pin();
void initialize_start_auto_mode_pin();
void button_initialization();
void move_to_preset_location(int configuration_number);

#endif