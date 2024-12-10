#include "robot_modes.h"
#include "main.h"
#include "robot_commands.h"
#include "hardware/adc.h"
#include "pwm.h"
#include <math.h>
#include "pico/stdlib.h"

float initial_x = 0;
float initial_y = 0;
float initial_z = 0;

float countY = 0;
float countX = 0;
float countZ = 0;

float manual_theta1 = 0;
float manual_theta2 = 2*M_PI/3;
float manual_theta3 = -2*M_PI/3;
float manual_theta4 = 0;

uint16_t result;
float move_location_1_x[12] = {13.6, 22, 27.5, 28, 
                               13.6, 22, 27.5, 28,
                               13.6, 22, 27.5, 28};

float move_location_1_y[12] = {25, 18, 7.5, -3.75,
                               25, 18, 8.5, -3.75,
                               25, 18, 8.5, -3.75};

float move_location_1_z[12] = {8.5, 8.5, 8.25, 8,
                               14.5, 14.5, 14.25, 14,
                               19, 19.5, 19.25, 19};

float move_location_2_x[12] = {13.4, 18, 22, 24,
                               13.4, 18, 22, 24,
                               13.4, 18, 22, 24};

float move_location_2_y[12] = {20, 14, 7.5, -3.75,
                               20, 14, 7.5, -3.75,
                               20, 14, 7.5, -3.75};

float move_location_2_z[12] = {11, 13, 11, 11,
                               15.5, 16.5, 15.5, 15.5,
                               21, 21, 21, 21};

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

void initialize_DIP_switch_pins(){
    gpio_init(DIP_bit0);
    gpio_set_input_enabled(DIP_bit0, true);
    gpio_pull_down(DIP_bit0);

    gpio_init(DIP_bit1);
    gpio_set_input_enabled(DIP_bit1, true);
    gpio_pull_down(DIP_bit1);

    gpio_init(DIP_bit2);
    gpio_set_input_enabled(DIP_bit2, true);
    gpio_pull_down(DIP_bit2);

    gpio_init(DIP_bit3);
    gpio_set_input_enabled(DIP_bit3, true);
    gpio_pull_down(DIP_bit3);

    gpio_init(10);
    gpio_set_input_enabled(5, true);
    gpio_pull_down(10);
}

int get_DIP_number(){
    int bit0 = gpio_get(DIP_bit0);
    int bit1 = gpio_get(DIP_bit1);
    int bit2 = gpio_get(DIP_bit2);
    int bit3 = gpio_get(DIP_bit3);

    int number = bit0 + (bit1<<1) + (bit2<<2) + (bit3<<3);
    return number;
}

void default_manual_theta(){
    manual_theta1 = 0;
    manual_theta2 = 2*M_PI/3;
    manual_theta3 = -2*M_PI/3;
    manual_theta4 = 0;
}

void manual_mode(){
    int column_position = 0;
    bool flag = false;            
 
    if(gpio_get(10) == false){
        default_manual_theta();
        initial_x = xyzpitch[0];
        initial_y = xyzpitch[1]; 
        initial_z = xyzpitch[2];

        //Read y-axis input for joystick
        adc_select_input(2);
        result = adc_read();
        if(result < 1950){
            xyzpitch[1] = xyzpitch[1] - 0.03;
        }
        else if(result > 2250){
            xyzpitch[1] = xyzpitch[1] + 0.03;
        }
        
        //Read z-axis input for joystick
        adc_select_input(0);
        result = adc_read();
        if(result < 1950){
            xyzpitch[2] = xyzpitch[2] + 0.025;
        }
        else if(result > 2250){
            xyzpitch[2] = xyzpitch[2] + -0.025;
        }

        //Read x-axis input for joystick
        adc_select_input(1);
        result = adc_read();
        if(result < 1950){
            xyzpitch[0] = xyzpitch[0] + 0.03;
        }
        else if(result > 2250){
            xyzpitch[0] = xyzpitch[0] + -0.03;
        }

        bool did_robot_move = xyzpitch[0] != initial_x || xyzpitch[1] != initial_y || xyzpitch[2] != initial_z;
        bool is_robot_in_boundary = ((powf(xyzpitch[0],2) + powf(xyzpitch[1],2) + powf(xyzpitch[2] - 9.5,2) - 1000) < 0) && xyzpitch[2] > 7 && xyzpitch[2] < 23.5 && (xyzpitch[0] > 2);
        if(xyzpitch[2] < 12 && ((powf(xyzpitch[0],2) + powf(xyzpitch[1],2) - 515) < 0)) {is_robot_in_boundary = false;}
        if(xyzpitch[0] < 7 && fabs(xyzpitch[1]) < 5) {is_robot_in_boundary = false;}

        bool x_lower = fabs(xyzpitch[0]) < fabs(initial_x); 
        bool y_lower = fabs(xyzpitch[1]) < fabs(initial_y);
        bool z_lower = fabs(xyzpitch[2]) < fabs(initial_z);
        bool x_higher = fabs(xyzpitch[0]) > fabs(initial_x);
        bool y_higher = fabs(xyzpitch[1]) > fabs(initial_y);
        bool z_higher = fabs(xyzpitch[2]) > fabs(initial_z);


        if(did_robot_move && is_robot_in_boundary){
            robot_move(xyzpitch, 1.5); 
        }
        
        if(did_robot_move && !is_robot_in_boundary){
            if(x_lower && xyzpitch[0] < 3){
                xyzpitch[0] = xyzpitch[0]/0.8;
            }
            else if(x_lower && xyzpitch[0]){
                xyzpitch[0] = xyzpitch[0]/0.98;
            }
            if(x_higher){
                xyzpitch[0] = xyzpitch[0]/1.015;
            }
            if(y_lower){
                xyzpitch[1] = xyzpitch[1]/0.98;
            }
            if(y_higher){
                xyzpitch[1] = xyzpitch[1]/1.015;
            }
            if(z_lower){
                xyzpitch[2] = xyzpitch[2]/0.98;
            }
            if(z_higher){
                xyzpitch[2] = xyzpitch[2]/1.015;
            }
            robot_move(xyzpitch, 1.5);
        }
        printf("X: %0.2f\n", xyzpitch[0]);
        printf("Y: %0.2f\n", xyzpitch[1]);
        printf("Z: %0.2f\n\n", xyzpitch[2]);
        claw_move(true);
    }
    //Controlling motor angles directly
    else{
        if((gpio_get(AUTO_START_SWITCH) == false)){
            adc_select_input(0);
            result = adc_read();
            if(result < 1950){
                manual_theta1 = manual_theta1 - 0.00002;
            }
            else if(result > 2250){
                manual_theta1 = manual_theta1 + 0.00002;
            }
         
            if(manual_theta1 > M_PI/2){
                manual_theta1 = M_PI/2;
            }
            else if(manual_theta1 < -M_PI/2){
                manual_theta1 = -M_PI/2;
            }

            duty_cycle_set(manual_theta1, manual_theta2, manual_theta3, manual_theta4, 0);

            motor_move();

            claw_move(true);
        }
        else{
            adc_select_input(2);
            result = adc_read();
            if(result < 1950){
                manual_theta3 = manual_theta3 - 0.00002;
            }
            else if(result > 2250){
                manual_theta3 = manual_theta3 + 0.00002;
            }
            
            adc_select_input(0);
            result = adc_read();
            if(result < 1950){
                manual_theta4 = manual_theta4 + 0.00002;

            }
            else if(result > 2250){
                manual_theta4 = manual_theta4 - 0.00002;

            }

            adc_select_input(1);
            result = adc_read();
            if(result < 1950){
                manual_theta2 = manual_theta2 - 0.00002;
            }
            else if(result > 2250){
                manual_theta2 = manual_theta2 + 0.00002;
            }

            //Safety limits
            if(manual_theta4 > M_PI/2){
                manual_theta4 = M_PI/2;
            }
            else if(manual_theta4 < -M_PI/2){
                manual_theta4 = -M_PI/2;
            }

            if(manual_theta3 < -3*M_PI/4){
                manual_theta3 = -3*M_PI/4;
            }
            else if(manual_theta3 > 0){
                manual_theta3 = 0;
            }

            if(manual_theta2 > M_PI){
                manual_theta2 = M_PI;
            }
            else if(manual_theta2 < 0){
                manual_theta2 = 0;
            }

        duty_cycle_set(manual_theta1, manual_theta2, manual_theta3, manual_theta4, 0);

        motor_move();

        claw_move(true);
        }
        
    }
}

void automatic_mode()
{ 
    default_manual_theta();
    int block_number = get_DIP_number();
    if(block_number > 12){
        block_number = 12;
    }

    set_initial_position();

    //If left joystick button is pressed run auto mode
    if((gpio_get(AUTO_START_SWITCH) == false) && (block_number >= 1)){
        claw_position = false;
        if((block_number == 3) || (block_number == 8)){
            claw_move(true);
        }
        else{
            claw_move(false);
        }

        if(block_number == 3){
            xyzpitch[0] = 20.55;
            xyzpitch[1] = 6.24;
            xyzpitch[2] = 18.51;
            robot_move(xyzpitch, 2);
        }

        sleep_ms(500);

        xyzpitch[0] =  move_location_1_x[block_number - 1];
        xyzpitch[1] =  move_location_1_y[block_number - 1];
        xyzpitch[2] =  move_location_1_z[block_number - 1];
        xyzpitch[3] = STARTING_PITCH;
        robot_move(xyzpitch, 2);

        sleep_ms(500);

        claw_position = true;
        claw_move(true);
        
        sleep_ms(500);

        xyzpitch[0] = move_location_2_x[block_number - 1];
        xyzpitch[1] = move_location_2_y[block_number - 1];
        xyzpitch[2] = move_location_2_z[block_number - 1];
        robot_move(xyzpitch, 2);

        set_initial_position();

        sleep_ms(50);

        xyzpitch[0] = 13;
        xyzpitch[1] = -16;
        xyzpitch[2] = 4;
        xyzpitch[3] = 0.9;

        robot_move(xyzpitch, 2);

        sleep_ms(300);

        claw_position = false;
        claw_move(false);

        set_initial_position();

        sleep_ms(1500);
    }

}