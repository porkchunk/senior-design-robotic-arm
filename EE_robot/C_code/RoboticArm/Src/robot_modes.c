#include "robot_modes.h"
#include "main.h"
#include "robot_commands.h"
#include "hardware/adc.h"
#include "pwm.h"
#include <math.h>
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

    float current_position[4];

    //Read y-axis input for joystick
    adc_select_input(0);
    result = adc_read();
    if(result < 1500){
        xyzpitch[1] = xyzpitch[1] + 0.03;
    }
    if(result > 2500){
        xyzpitch[1] = xyzpitch[1] + -0.03;
    }

    //Read z-axis input for joystick
    adc_select_input(1);
    result = adc_read();
    if(result < 1900){
        xyzpitch[2] = xyzpitch[2] + 0.03;
    }
    if(result > 2100){
        xyzpitch[2] = xyzpitch[2] + -0.03;
    }

    //Read x-axis input for joystick
    adc_select_input(2);
    result = adc_read();
    if(result < 1500){
        xyzpitch[0] = xyzpitch[0] + 0.03;
    }
    if(result > 2500){
        xyzpitch[0] = xyzpitch[0] + -0.03;
    }

    float theta1 = map_function(20*duty_cycle_motors.motor1, 0.5, 2.5, -M_PI/2, M_PI/2);
    float theta2 = map_function(20*duty_cycle_motors.motor2, 0.5, 2.5, M_PI, 0);
    float theta3 = map_function(20*duty_cycle_motors.motor3, 0.56, 2.06, (-3*M_PI)/4, 0);
    float theta4 = map_function(20*duty_cycle_motors.motor4, 0.5, 2.5, -M_PI/2, M_PI/2);
    
    bool did_robot_move = xyzpitch[0] != diff_x || xyzpitch[1] != diff_y || xyzpitch[2] != diff_z;
    bool is_robot_in_boundary = ((powf(xyzpitch[0],2) + powf(xyzpitch[1],2) + powf(xyzpitch[2] - 19.9,2) - 900) < 0) && xyzpitch[2] > 7 && xyzpitch[2] < 23 && (xyzpitch[0] > 3);
    if(xyzpitch[2] < 7 && xyzpitch[0] < 10) {is_robot_in_boundary = false;}

    bool x_lower = xyzpitch[0] < diff_x;
    bool y_lower = xyzpitch[1] < diff_y;
    bool z_lower = xyzpitch[2] < diff_z;
    bool x_higher = xyzpitch[0] > diff_x;
    bool y_higher = xyzpitch[1] > diff_y;
    bool z_higher = xyzpitch[2] > diff_z;

    if(did_robot_move && is_robot_in_boundary){
        robot_move(xyzpitch); 

        forward_kinematics(theta1, theta2, theta3, theta4, current_position);
        printf("X: %0.3f\n", current_position[0]);
        printf("Y: %0.3f\n", current_position[1]);
        printf("Z: %0.3f\n", current_position[2]);
        printf("PITCH: %0.3f\n", current_position[3]);
    }
    if(!is_robot_in_boundary){
        if(x_lower){
            xyzpitch[0] = xyzpitch[0]/0.9;
        }
        if(x_higher){
            xyzpitch[0] = xyzpitch[0]/1.1;
        }
        if(y_lower){
            xyzpitch[1] = xyzpitch[1]/0.9;
        }
        if(y_higher){
            xyzpitch[1] = xyzpitch[1]/1.1;
        }
        if(z_lower){
            xyzpitch[2] = xyzpitch[2]/0.9;
        }
        if(z_higher){
            xyzpitch[2] = xyzpitch[2]/1.1;
        }
        
        robot_move(xyzpitch);

        forward_kinematics(theta1, theta2, theta3, theta4, current_position);
        printf("X: %0.3f\n", current_position[0]);
        printf("Y: %0.3f\n", current_position[1]);
        printf("Z: %0.3f\n", current_position[2]);
        printf("PITCH: %0.3f\n", current_position[3]);
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