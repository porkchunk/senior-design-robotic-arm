#include "motor.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "robot_commands.h"
#include "robot_modes.h"
#include "interrupts.h"
#include "SPI.h"
#include "adc.h"
#include <math.h>
#include <stdio.h>

uint chan_motors[NUMBER_OF_MOTORS];
uint slice_motors[NUMBER_OF_MOTORS];
struct pwmSignal duty_cycle_motors;

uint32_t current_time = 0;
float delta_time = 0;
float previous_time = 0;
float error_value[6] = {0};
float previous_error[6]= {0};
int actual_position[6] = {0};
int target_position[6] = {0};
float error_derivative[6] = {0};
float error_integral[6] = {0};
float control_signal[6] = {0};

float duty_other_motors = 0;
float duty_current_motor = 0;
int current_direction_pin = 0;
int motor_number = 1;
int adc_result = 0;
float current_duty = 0;
bool flag = false;
float increment_value = 0;

//Add direction pins here
const int PIN_DIRECTION[6] = {DIRECTION_MOTOR_1, 2, DIRECTION_MOTOR_3, DIRECTION_MOTOR_4, DIRECTION_MOTOR_5, DIRECTION_MOTOR_6};

//PID terms for user to adjust
float proportional[6] = {110,70,110,4.5,1,40};
float integral[6] = {55,60,110,1.5,1.5,20};
float derivative[6] = {0,0,0,0,0,0};

void calculate_PID(){
    current_time = time_us_32();
    delta_time = (current_time - previous_time) / 1e6;
    previous_time = current_time;

    error_value[MOTOR_2] = (((actual_position[MOTOR_2] - target_position[MOTOR_2] + 1325) % 2680) + 2680) % 2680 - 1325;
    error_value[MOTOR_4] = (((actual_position[MOTOR_4] - target_position[MOTOR_4] + 1332) % 2690) + 2690) % 2690 - 1332;
    error_value[MOTOR_5] = (((actual_position[MOTOR_5] - target_position[MOTOR_5] + 1350) % 2720) + 2720) % 2720 - 1350;

    for(int i=0; i<6; ++i){
        //Error value is calculated to be the smallest angular distance and the direction of the 
        //shortest distance between actual and target position
        error_derivative[i] = (error_value[i] - previous_error[i]) / delta_time;
        error_integral[i] = error_integral[i] + (error_value[i] * delta_time);
        control_signal[i] = (proportional[i] * error_value[i]) + (derivative[i] * error_derivative[i]) + (integral[i] * error_integral[i]);
        previous_error[i] = error_value[i];
    }
    //printf("Error value: %0.2f\n", error_value[MOTOR_5]);
}
///Set direction of motors based on error
void set_pin_directions(){  
    //Not set
    if(control_signal[MOTOR_1] < 0){
        gpio_put(PIN_DIRECTION[MOTOR_1], false);
    }
    else if (control_signal[MOTOR_1] > 0){
        gpio_put(PIN_DIRECTION[MOTOR_1], true);
    }
    
    //Set correctly
    if(control_signal[MOTOR_2] < 0){
        gpio_put(PIN_DIRECTION[MOTOR_2], true);
    }
    else if (control_signal[MOTOR_2] > 0){
        gpio_put(PIN_DIRECTION[MOTOR_2], false);
    }

    //Not set
    if(control_signal[MOTOR_3] < 0){
        gpio_put(PIN_DIRECTION[MOTOR_3], false);
    }
    else if (control_signal[MOTOR_3] > 0){
        gpio_put(PIN_DIRECTION[MOTOR_3], true);
    }

    //Set correctly
    if(control_signal[MOTOR_4] < 0){
        gpio_put(PIN_DIRECTION[MOTOR_4], false);
    }
    else if (control_signal[MOTOR_4] > 0){
        gpio_put(PIN_DIRECTION[MOTOR_4], true);
    }

    //Not set
    if(control_signal[MOTOR_5] < 0){
        gpio_put(PIN_DIRECTION[MOTOR_5], true);
    }
    else if (control_signal[MOTOR_5] > 0){
        gpio_put(PIN_DIRECTION[MOTOR_5], false);
    }

    //Not set
    if(control_signal[MOTOR_6] < 0){
        gpio_put(PIN_DIRECTION[MOTOR_6], false);
    }
    else if (control_signal[MOTOR_6] > 0){
        gpio_put(PIN_DIRECTION[MOTOR_6], true);
    }
}

void calculate_control_signals(){
    control_signal[MOTOR_2] = fabs(control_signal[MOTOR_2])/2680; //Divide here by whatever (Vmaxencoder/3.3)*4096 equals
    control_signal[MOTOR_4] = fabs(control_signal[MOTOR_4])/2680;
    control_signal[MOTOR_5] = fabs(control_signal[MOTOR_5])/2720;

    //MOTOR1
    if(control_signal[MOTOR_1] > 0.55){
        control_signal[MOTOR_1] = 0.55;
    }
    
    if(control_signal[MOTOR_1] < 0.3 && error_value[MOTOR_1] != 0){
        control_signal[MOTOR_1] = 0.3;
    }

    //MOTOR2
    if(control_signal[MOTOR_2] > 0.55){
        control_signal[MOTOR_2] = 0.55;
    }
    
    if(control_signal[MOTOR_2] < 0.3 && error_value[MOTOR_2] != 0){
        control_signal[MOTOR_2] = 0.3;
    }

    //MOTOR3
    if(control_signal[MOTOR_3] > 0.55){
        control_signal[MOTOR_3] = 0.55;
    }
    
    if(control_signal[MOTOR_3] < 0.3 && error_value[MOTOR_3] != 0){
        control_signal[MOTOR_3] = 0.3;
    }

    //MOTOR4
    if(control_signal[MOTOR_4] > 0.55){
        control_signal[MOTOR_4] = 0.55;
    }
    
    if(control_signal[MOTOR_4] < 0.3 && error_value[MOTOR_4] != 0){
        control_signal[MOTOR_4] = 0.3;
    }

    //MOTOR5
    if(control_signal[MOTOR_5] > 0.12){
        control_signal[MOTOR_5] = 0.12;
    }
    
    if(control_signal[MOTOR_5] < 0 && error_value[MOTOR_5] != 0){
        control_signal[MOTOR_5] = 0;
    }

    //MOTOR6
    if(control_signal[MOTOR_6] > 0.55){
        control_signal[MOTOR_6] = 0.55;
    }
    
    if(control_signal[MOTOR_6] < 0.3 && error_value[MOTOR_6] != 0){
        control_signal[MOTOR_6] = 0.3;
    }
    printf("Control signal: %0.2f\n", control_signal[MOTOR_5]);

    //These steps are necessary since motor 2 uses a different control signal from the Sparkmax controller with 1ms to 2ms being the control signal forward to reverse
    control_signal[MOTOR_2] = map_function(control_signal[MOTOR_2], 0.3, 0.55, 0, 0.25);

    if(gpio_get(PIN_DIRECTION[1]) == true){
        control_signal[MOTOR_2] = (1.5 + control_signal[MOTOR_2])/20;
    }
    else{
        control_signal[MOTOR_2] = (1.5 - control_signal[MOTOR_2])/20;
    }
}

void update_motor_speed(){
    /*UNUSED NOW
    duty_cycle_motors.motor1 = control_signal[MOTOR_1];
    duty_cycle_motors.motor2 = control_signal[MOTOR_2];
    duty_cycle_motors.motor3 = control_signal[MOTOR_3];
    duty_cycle_motors.motor4 = control_signal[MOTOR_4];
    duty_cycle_motors.motor5 = control_signal[MOTOR_5];
    duty_cycle_motors.motor6 = control_signal[MOTOR_6];
    */

    pwm_set_freq_duty(slice_motors[MOTOR_1], chan_motors[MOTOR_1], PWM_FREQ, duty_cycle_motors.motor1);
    pwm_set_freq_duty(slice_motors[MOTOR_2], chan_motors[MOTOR_2], PWM_FREQ, duty_cycle_motors.motor2);
    pwm_set_freq_duty(slice_motors[MOTOR_3], chan_motors[MOTOR_3], PWM_FREQ, duty_cycle_motors.motor3);
    pwm_set_freq_duty(slice_motors[MOTOR_4], chan_motors[MOTOR_4], PWM_FREQ, duty_cycle_motors.motor4);
    pwm_set_freq_duty(slice_motors[MOTOR_5], chan_motors[MOTOR_5], PWM_FREQ, duty_cycle_motors.motor5);
    pwm_set_freq_duty(slice_motors[MOTOR_6], chan_motors[MOTOR_6], PWM_FREQ, duty_cycle_motors.motor6);
    
    pwm_set_enabled(slice_motors[MOTOR_1], false);
    pwm_set_enabled(slice_motors[MOTOR_2], false);
    pwm_set_enabled(slice_motors[MOTOR_3], false);
    pwm_set_enabled(slice_motors[MOTOR_4], false);
    pwm_set_enabled(slice_motors[MOTOR_5], true);
    pwm_set_enabled(slice_motors[MOTOR_6], false);
}

void move_motor_with_controller(){
    switch(motor_number){
        case 0:
            duty_cycle_motors.motor1 = 0.2;
            duty_cycle_motors.motor2 = 0;
            duty_cycle_motors.motor3 = 0;
            duty_cycle_motors.motor4 = 0;
            duty_cycle_motors.motor5 = 0;
            duty_cycle_motors.motor6 = 0;
            break;
        case 1: 
            duty_cycle_motors.motor1 = 0;
            duty_cycle_motors.motor2 = 0;
            duty_cycle_motors.motor3 = 0;
            duty_cycle_motors.motor4 = 0;
            duty_cycle_motors.motor5 = 0;
            duty_cycle_motors.motor6 = 0;
            increment_value = 0.15;
            break;
        case 2: 
            duty_cycle_motors.motor1 = 0;
            duty_cycle_motors.motor2 = 0;
            duty_cycle_motors.motor3 = 0.2;
            duty_cycle_motors.motor4 = 0;
            duty_cycle_motors.motor5 = 0;
            duty_cycle_motors.motor6 = 0;
            break;
        case 3:
            duty_cycle_motors.motor1 = 0;
            duty_cycle_motors.motor2 = 0;
            duty_cycle_motors.motor3 = 0;
            duty_cycle_motors.motor4 = 0.25;
            duty_cycle_motors.motor5 = 0;
            duty_cycle_motors.motor6 = 0;
            break;
        case 4: 
            duty_cycle_motors.motor1 = 0;
            duty_cycle_motors.motor2 = 0;
            duty_cycle_motors.motor3 = 0;
            duty_cycle_motors.motor4 = 0;
            duty_cycle_motors.motor5 = 0.08;
            duty_cycle_motors.motor6 = 0;
            break;
        case 5:
            duty_cycle_motors.motor1 = 0;
            duty_cycle_motors.motor2 = 0;
            duty_cycle_motors.motor3 = 0;
            duty_cycle_motors.motor4 = 0;
            duty_cycle_motors.motor5 = 0;
            duty_cycle_motors.motor6 = 0.1;
            break;
        default: 
            duty_cycle_motors.motor1 = 0;
            duty_cycle_motors.motor2 = 0;
            duty_cycle_motors.motor3 = 0;
            duty_cycle_motors.motor4 = 0;
            duty_cycle_motors.motor5 = 0;
            duty_cycle_motors.motor6 = 0;
    }

    pwm_set_enabled(slice_motors[MOTOR_1], false);
    pwm_set_enabled(slice_motors[MOTOR_2], false);
    pwm_set_enabled(slice_motors[MOTOR_3], false);
    pwm_set_enabled(slice_motors[MOTOR_4], false);
    pwm_set_enabled(slice_motors[MOTOR_5], false);
    pwm_set_enabled(slice_motors[MOTOR_6], false);

    current_direction_pin = PIN_DIRECTION[motor_number];

    adc_select_input(0);
    adc_result = adc_read();
    if(adc_result > 2500){
        gpio_put(current_direction_pin, false);
        pwm_set_enabled(slice_motors[motor_number], true);
        flag = true;
    }
    else if (adc_result < 1500){
        gpio_put(current_direction_pin, true);
        pwm_set_enabled(slice_motors[motor_number], true);
        flag = true;
    }
    else{
        pwm_set_enabled(slice_motors[motor_number], false);
        flag = false;
    }

    if(gpio_get(PIN_DIRECTION[1]) == true){
        duty_cycle_motors.motor2 = (1.5 + 0.10)/20;
    }
    else{
        duty_cycle_motors.motor2 = (1.5 - 0.4)/20;
    }

    adc_select_input(1);
    adc_result = adc_read();
    if(adc_result > 2500){
        claw_position = false;
    }
    else if(adc_result < 1500){
        claw_position = true;
    }

    pwm_set_freq_duty(slice_motors[MOTOR_1], chan_motors[MOTOR_1], PWM_FREQ, duty_cycle_motors.motor1);
    pwm_set_freq_duty(slice_motors[MOTOR_2], chan_motors[MOTOR_2], PWM_FREQ, duty_cycle_motors.motor2);
    pwm_set_freq_duty(slice_motors[MOTOR_3], chan_motors[MOTOR_3], PWM_FREQ, duty_cycle_motors.motor3);
    pwm_set_freq_duty(slice_motors[MOTOR_4], chan_motors[MOTOR_4], PWM_FREQ, duty_cycle_motors.motor4);
    pwm_set_freq_duty(slice_motors[MOTOR_5], chan_motors[MOTOR_5], PWM_FREQ, duty_cycle_motors.motor5);
    pwm_set_freq_duty(slice_motors[MOTOR_6], chan_motors[MOTOR_6], PWM_FREQ, duty_cycle_motors.motor6);

    if(motor_number == 0){
        current_duty = duty_cycle_motors.motor1;
    }
    else if(motor_number == 1){
        current_duty = duty_cycle_motors.motor2;
    }
    else if(motor_number == 2){
        current_duty = duty_cycle_motors.motor3;
    }
    else if(motor_number == 3){
        current_duty = duty_cycle_motors.motor4;
    }
    else if(motor_number == 4){
        current_duty = duty_cycle_motors.motor5;
    }
    else if(motor_number == 5){
        current_duty = duty_cycle_motors.motor6;
    }

    claw_move();
    printf("Current_motor: %d\n\n", motor_number + 1);
   // printf("duty_cycle motor %d: %0.2f and is direction %d and on value is %d\n; claw is %d\n", motor_number + 1, current_duty, gpio_get(current_direction_pin), flag, claw_position);
}


void motor_move(){
    set_pin_directions();
    calculate_control_signals();
    update_motor_speed();
}

void motor_initialization(){
    for(int i=0; i<6; ++i){
        gpio_init(PIN_DIRECTION[i]);
        gpio_set_dir(PIN_DIRECTION[i], GPIO_OUT);
    }

    gpio_set_function(PIN_MOTOR_1, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_2, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_3, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_4, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_5, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_6, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_7, GPIO_FUNC_PWM);

    slice_motors[MOTOR_1] = pwm_gpio_to_slice_num(PIN_MOTOR_1);
    slice_motors[MOTOR_2] = pwm_gpio_to_slice_num(PIN_MOTOR_2);
    slice_motors[MOTOR_3] = pwm_gpio_to_slice_num(PIN_MOTOR_3);
    slice_motors[MOTOR_4] = pwm_gpio_to_slice_num(PIN_MOTOR_4);
    slice_motors[MOTOR_5] = pwm_gpio_to_slice_num(PIN_MOTOR_5);
    slice_motors[MOTOR_6] = pwm_gpio_to_slice_num(PIN_MOTOR_6);
    slice_motors[MOTOR_7] = pwm_gpio_to_slice_num(PIN_MOTOR_7);

    chan_motors[MOTOR_1] = pwm_gpio_to_channel(PIN_MOTOR_1);
    chan_motors[MOTOR_2] = pwm_gpio_to_channel(PIN_MOTOR_2);
    chan_motors[MOTOR_3] = pwm_gpio_to_channel(PIN_MOTOR_3);
    chan_motors[MOTOR_4] = pwm_gpio_to_channel(PIN_MOTOR_4);
    chan_motors[MOTOR_5] = pwm_gpio_to_channel(PIN_MOTOR_5);
    chan_motors[MOTOR_6] = pwm_gpio_to_channel(PIN_MOTOR_6);
    chan_motors[MOTOR_7] = pwm_gpio_to_channel(PIN_MOTOR_7);

    gpio_init(DIRECTION_MOTOR_1);
    gpio_init(DIRECTION_MOTOR_3);
    gpio_init(DIRECTION_MOTOR_4);
    gpio_init(DIRECTION_MOTOR_5);
    gpio_init(DIRECTION_MOTOR_6);

    gpio_set_dir(DIRECTION_MOTOR_1, GPIO_OUT);
    gpio_set_dir(DIRECTION_MOTOR_3, GPIO_OUT);
    gpio_set_dir(DIRECTION_MOTOR_4, GPIO_OUT);
    gpio_set_dir(DIRECTION_MOTOR_5, GPIO_OUT);  
    gpio_set_dir(DIRECTION_MOTOR_6, GPIO_OUT);
}

void pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d)
{
    uint32_t clock = SYS_FREQ_KHZ * 1000;
    uint16_t wrapval = 65535;
    uint16_t chan_level = wrapval * d;
    float clkdiv = (double)clock/(wrapval * f);
    pwm_set_clkdiv(slice_num, clkdiv);
    pwm_set_wrap(slice_num, wrapval);
    pwm_set_chan_level(slice_num, chan, chan_level);
}

void pwm_initialization(){
    gpio_set_function(PIN_MOTOR_1, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_2, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_3, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_4, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_5, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_6, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_7, GPIO_FUNC_PWM);

    slice_motors[MOTOR_1] = pwm_gpio_to_slice_num(PIN_MOTOR_1);
    slice_motors[MOTOR_2] = pwm_gpio_to_slice_num(PIN_MOTOR_2);
    slice_motors[MOTOR_3] = pwm_gpio_to_slice_num(PIN_MOTOR_3);
    slice_motors[MOTOR_4] = pwm_gpio_to_slice_num(PIN_MOTOR_4);
    slice_motors[MOTOR_5] = pwm_gpio_to_slice_num(PIN_MOTOR_5);
    slice_motors[MOTOR_6] = pwm_gpio_to_slice_num(PIN_MOTOR_6);
    slice_motors[MOTOR_7] = pwm_gpio_to_slice_num(PIN_MOTOR_7);

    chan_motors[MOTOR_1] = pwm_gpio_to_channel(PIN_MOTOR_1);
    chan_motors[MOTOR_2] = pwm_gpio_to_channel(PIN_MOTOR_2);
    chan_motors[MOTOR_3] = pwm_gpio_to_channel(PIN_MOTOR_3);
    chan_motors[MOTOR_4] = pwm_gpio_to_channel(PIN_MOTOR_4);
    chan_motors[MOTOR_5] = pwm_gpio_to_channel(PIN_MOTOR_5);
    chan_motors[MOTOR_6] = pwm_gpio_to_channel(PIN_MOTOR_6);
    chan_motors[MOTOR_7] = pwm_gpio_to_channel(PIN_MOTOR_7);
}
