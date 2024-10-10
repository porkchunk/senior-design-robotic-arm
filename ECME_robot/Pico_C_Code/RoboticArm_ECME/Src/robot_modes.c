#include "robot_modes.h"
#include "main.h"
#include "robot_commands.h"
#include "hardware/adc.h"
#include "pwm.h"
#include "pico/stdlib.h"

float diff_x = 0;
float diff_y = 0;
float diff_z = 0;

float countY = 0;
float countX = 0;
float countZ = 0;

uint16_t result;

void initialize_start_auto_mode_pin(){
    gpio_init(AUTO_START_SWITCH);
    gpio_set_input_enabled(AUTO_START_SWITCH, true);
    gpio_pull_down(AUTO_START_SWITCH);
}

void initialize_auto_manual_pin(){
    gpio_init(AUTO_MANUAL_SWITCH_PIN);
    gpio_set_input_enabled(AUTO_MANUAL_SWITCH_PIN, true);
    gpio_pull_down(AUTO_MANUAL_SWITCH_PIN);
}

void manual_mode(){
    diff_x = xyzpitch[0];
    diff_y = xyzpitch[1]; 
    diff_z = xyzpitch[2];

    //Read y-axis input for joystick
    adc_select_input(0);
    result = adc_read();
    if(result < 1500){
        countY += 0.03;
    }
    if(result > 2500){
        countY -= 0.03;
    }

    //Read z-axis input for joystick
    adc_select_input(1);
    result = adc_read();
    if(result < 1900){
        countZ += 0.02;
    }
    if(result > 2100){
        countZ -= 0.02;
    }

    //Read x-axis input for joystick
    adc_select_input(2);
    result = adc_read();
    if(result < 1500){
        countX += 0.03;
    }
    if(result > 2500){
        countX -= 0.03;
    }
    
    xyzpitch[0] = STARTING_X + countX;
    xyzpitch[1] = STARTING_Y + countY;
    xyzpitch[2] = STARTING_Z + countZ;
    xyzpitch[3] = STARTING_PITCH;
    
    //If joystick moved, move robot
    if(xyzpitch[0] != diff_x || xyzpitch[1] != diff_y || xyzpitch[2] != diff_z){
        robot_move(xyzpitch);
    }

    claw_move();
}

void automatic_mode()
{ 
    //If left joystick button is pressed run auto mode
    if(gpio_get(AUTO_START_SWITCH) == false){
        xyzpitch[0] = STARTING_X + 3;
        xyzpitch[1] = STARTING_Y - 9;
        xyzpitch[2] = STARTING_Z - 9;
        xyzpitch[3] = STARTING_PITCH;
        robot_move(xyzpitch);

        claw_position = true;
        claw_move();

        sleep_ms(500);

        set_initial_position();

        claw_position = false;
        claw_move();

        sleep_ms(500);
    }
}