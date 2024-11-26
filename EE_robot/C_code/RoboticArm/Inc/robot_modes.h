#ifndef ROBOT_MODES_H
#define ROBOT_MODES_H

#define AUTO_MANUAL_SWITCH_PIN 15
#define AUTO_START_SWITCH 0
#define STARTING_X 25.75
#define STARTING_Y 0
#define STARTING_Z 19.9
#define STARTING_PITCH 0

#define DIP_bit0 6
#define DIP_bit1 7
#define DIP_bit2 8
#define DIP_bit3 9

void manual_mode();
void automatic_mode();
void initialize_auto_manual_pin();
void initialize_start_auto_mode_pin();
int get_DIP_number();
void initialize_DIP_switch_pins();

#endif