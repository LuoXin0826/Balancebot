/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motor drivers
*
*******************************************************************************/
#include <stdio.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"
#include <math.h>

// preposessor macros
#define unlikely(x) __builtin_expect (!!(x), 0)

// global initialized flag
static int init_flag = 0;

/*******************************************************************************
* int mb_motor_init()
* 
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init(){
    rc_gpio_init(MDIR1_CHIP, MDIR1_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(MDIR2_CHIP, MDIR2_PIN, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(MOT_BRAKE_EN, GPIOHANDLE_REQUEST_OUTPUT);
    init_flag = 1;
    return mb_motor_init_freq(MB_MOTOR_DEFAULT_PWM_FREQ);
}

/*******************************************************************************
* int mb_motor_init_freq()
* 
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz){
    rc_pwm_init(1, pwm_freq_hz);
    rc_pwm_set_duty(1, 'A', 0.0);
    rc_pwm_set_duty(1, 'B', 0.0);
    return 0;
}

/*******************************************************************************
* mb_motor_cleanup()
* 
*******************************************************************************/
int mb_motor_cleanup(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying cleanup before motors have been initialized\n");
        return -1;
    }
    else{
        rc_pwm_cleanup(1);
        rc_gpio_cleanup(MDIR1_CHIP, MDIR1_PIN);
        rc_gpio_cleanup(MDIR2_CHIP, MDIR2_PIN);
        rc_gpio_cleanup(MOT_BRAKE_EN);
    }
    return 0;
}

/*******************************************************************************
* mb_motor_brake()
* 
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }
    else{
    rc_gpio_set_value(MOT_BRAKE_EN, brake_en);
    }
   return 0;
}

/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_motor_disable(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }
    else{
        rc_pwm_set_duty(1, 'A', 0.0);
        rc_pwm_set_duty(1, 'B', 0.0);
    }
    return 0;
}


/*******************************************************************************
* int mb_motor_set(int motor, double duty)
* 
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* uses the defines in mb_defs.h
* returns 0 on success
*******************************************************************************/
int mb_motor_set(int motor, double duty){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    if (fabs(duty) > 1)
        duty = duty/fabs(duty);

    int dir = 0;
    if(motor == 1 ){
        if (duty < 0.0){
            dir = MOT_1_POL*-1;
        }
        else {
            dir = MOT_1_POL;
        }
        if (dir < 0){
            dir = 0;
        }
        else{
            dir = 1;
        }
        printf("%0.2f", fabs(duty));
        rc_gpio_set_value(MDIR1_CHIP, MDIR1_PIN, dir);
        rc_pwm_set_duty(1, 'A', fabs(duty));
        
    }
    else if(motor == 2){

        if (duty < 0.0){
            dir = MOT_2_POL*-1;
        }
        else {
            dir = MOT_2_POL;
        }
        if (dir < 0){
            dir = 0;
        }
        else{
            dir = 1;
        }
        rc_gpio_set_value(MDIR2_CHIP,MDIR2_PIN, dir);
        rc_pwm_set_duty(1, 'B', fabs(duty));
    }
    return 0;
}

/*******************************************************************************
* int mb_motor_set_all(double duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty){

    if(unlikely(!init_flag)){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    else{
        int dir1 = 0;
        int dir2 = 0;
        if (duty < 0.0){
            dir1 = MOT_1_POL*-1;
            dir2 = MOT_2_POL*-1;
        }
        else {
            dir1 = MOT_1_POL;
            dir2 = MOT_2_POL;
        }
        if (dir1 < 0){
            dir1 = 0;
        }
        else{
            dir1 = 1;
        }
        if (dir2 < 0){
            dir2 = 0;
        }
        else{
            dir2 = 1;
        }
        rc_gpio_set_value(MDIR1_CHIP,MDIR1_PIN, dir1);
        rc_gpio_set_value(MDIR2_CHIP,MDIR2_PIN, dir2);
        rc_pwm_set_duty(1, 'A', fabs(duty));
        rc_pwm_set_duty(1, 'B', fabs(duty));
    }
    return 0;
}


/*******************************************************************************
* int mb_motor_read_current(int motor)
* 
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor){
    //DRV8801 driver board CS pin puts out 500mV/A
    float current = 0;
    if(motor == 1){
        current = rc_adc_read_volt(MOT_1_CS)/0.5;
    }
    else if(motor == 2){
        current = rc_adc_read_volt(MOT_2_CS)/0.5;
    }
    return current;
}