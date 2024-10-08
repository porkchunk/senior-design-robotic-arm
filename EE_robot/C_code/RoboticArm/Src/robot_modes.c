#include "robot_modes.h"
#include "main.h"
#include "robot_commands.h"
#include "hardware/adc.h"
#include "pwm.h"
#include "pico/stdlib.h"

const float starting_x = 25;
const float starting_y = 0;
const float starting_z = 19.9;
const float starting_pitch = 0;

float diff_x = 0;
float diff_y = 0;
float diff_z = 0;

float countY = 0;
float countX = 0;
float countZ = 0;

uint16_t result;

void initialize_auto_manual_pin(){
    gpio_init(AUTO_MANUAL_SWITCH_PIN);
    gpio_set_input_enabled(AUTO_MANUAL_SWITCH_PIN, true);
    gpio_pull_down(AUTO_MANUAL_SWITCH_PIN);
}

void manual_mode(){

    diff_x = xyzpitch[0];
    diff_y = xyzpitch[1];
    diff_z = xyzpitch[2];

    adc_select_input(0);
    result = adc_read();
    if(result < 1500){
        countY += 0.03;
    }
    if(result > 2500){
        countY -= 0.03;
    }
    adc_select_input(1);
    result = adc_read();
    if(result < 1900){
        countZ += 0.02;
    }
    if(result > 2100){
        countZ -= 0.02;
    }
    adc_select_input(2);
    result = adc_read();
    if(result < 1500){
        countX += 0.03;
    }
    if(result > 2500){
        countX -= 0.03;
    }
    
    xyzpitch[0] = starting_x + countX;
    xyzpitch[1] = starting_y + countY;
    xyzpitch[2] = starting_z + countZ;
    xyzpitch[3] = starting_pitch;
    
    if(xyzpitch[0] != diff_x || xyzpitch[1] != diff_y || xyzpitch[2] != diff_z){
        robot_move(xyzpitch, slice_motors, chan_motors);
    }

    claw_move(slice_motors, chan_motors);
}

void automatic_mode()
{
    xyzpitch[0] = starting_x + 5;
    xyzpitch[1] = starting_y;
    xyzpitch[2] = starting_z;
    xyzpitch[3] = starting_pitch;

    set_initial_position();

    sleep_ms(500);

    robot_move(xyzpitch, slice_motors, chan_motors);

    sleep_ms(500);

    xyzpitch[0] = starting_x;

    robot_move(xyzpitch, slice_motors, chan_motors);

    sleep_ms(500);
}