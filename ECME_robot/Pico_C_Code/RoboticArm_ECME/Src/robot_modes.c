#include "robot_modes.h"
#include "main.h"
#include "motor.h"
#include "robot_commands.h"
#include "hardware/adc.h"
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
    diff_x = position[0];
    diff_y = position[1]; 
    diff_z = position[2];

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
    
    position[0] = STARTING_X + countX;
    position[1] = STARTING_Y + countY;
    position[2] = STARTING_Z + countZ;
    position[3] = STARTING_PITCH;
    
    //If joystick moved, move robot
    if(position[0] != diff_x || position[1] != diff_y || position[2] != diff_z){
        //robot_move(position);
    }

    claw_move();
}

void move_to_preset_location(int configuration_number){
    float initial_position[6] = {0,0,0,0,0,0};
    float intermediate_position[6] = {0};
    float dropping_capsule_position[6] = {0}; 
    float selected_configuration[6] = {0};

    switch(configuration_number){
        case 1:
            for(int i=0; i<6; i++){
                selected_configuration[i] = initial_position[i];
            }
            break;

        case 2: 
            for(int i=0; i<6; i++){
                selected_configuration[i] = intermediate_position[i];
            }
            break;

        case 3:
            for(int i=0; i<6; i++){
                selected_configuration[i] = dropping_capsule_position[i];
            }
            break;

        default:
            for(int i=0; i<6; i++){
                selected_configuration[i] = initial_position[i];
            }
    }

    float current_position[6] = {0};
    int increments = 500;

    for(int i=0; i<increments; i++){
        for(int j=0; j<6; j++){
            current_position[j] = selected_configuration[j]/increments;
            target_position[j] = map_function(current_position[j],0,0,0,0); //Input here what the conversion is for angles to signal on the encoders
        }
        sleep_ms(10);
    }
}

void automatic_mode()
{ 
    //If left joystick button is pressed run auto mode
    if(gpio_get(AUTO_START_SWITCH) == false){
        move_to_preset_location(1);
    }
}

void button_initialization(){
    gpio_init(BUTTON_LEFT);
    gpio_set_input_enabled(BUTTON_LEFT, true);
    gpio_pull_down(BUTTON_LEFT);

    gpio_init(BUTTON_RIGHT);
    gpio_set_input_enabled(BUTTON_RIGHT, true);
    gpio_pull_down(BUTTON_RIGHT);
}