#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/math/filter.h>
#include "mb_controller.h"
#include "mb_defs.h"
#include "mb_structs.h"


float overallGain1_o = 1;
float kp1_o = 0;
float ki1_o;
float kd1_o;

float overallGain2_o = 1;
float kp2_o; 
float ki2_o; 
float kd2_o;

float kp3_o;
float kd3_o;


float bodyAngleOffset_o;

float actVolt_o = 11.5;

float speed_o = 0;


rc_filter_t F1_o = RC_FILTER_INITIALIZER;
rc_filter_t F2_o = RC_FILTER_INITIALIZER;
rc_filter_t F3_o = RC_FILTER_INITIALIZER;

int mb_controller_odometry_load_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening %s\n", CFG_PATH );
    }
    /* TODO parse your config file here*/

    fscanf(file, "%f", &overallGain1_o);
    fscanf(file, "%f", &kp1_o);
	fscanf(file, "%f", &ki1_o);
	fscanf(file, "%f", &kd1_o);
    fscanf(file, "%f", &overallGain2_o);
	fscanf(file, "%f", &kp2_o);
	fscanf(file, "%f", &ki2_o);
	fscanf(file, "%f", &kd2_o);
    fscanf(file, "%f", &kp3_o);
    fscanf(file, "%f", &kd3_o);
    fscanf(file, "%f", &bodyAngleOffset_o);
    fscanf(file, "%f", &actVolt_o);
    fscanf(file, "%f", &speed_o);


    kp1_o = kp1_o/overallGain1_o;
    ki1_o = ki1_o/overallGain1_o;
    kd1_o = kd1_o/overallGain1_o;

    kp2_o = kp2_o/overallGain2_o;
    ki2_o = ki2_o/overallGain2_o;
    kd2_o = kd2_o/overallGain2_o;


    fclose(file);
    return 0;
}


int mb_controller_init_odometry(){
    mb_controller_odometry_load_config();

    if(rc_filter_pid(&F1_o, kp1_o, ki1_o, kd1_o, 2*DT, DT)){
    fprintf(stderr, "ERROR in mb_controller, failed to make inner loop filter\n");
        return -1;
    }
    if(rc_filter_pid(&F2_o,  kp2_o, ki2_o, kd2_o, 2*DT, DT)){
    fprintf(stderr, "ERROR in mb_controller, failed to make outer loop filter\n");
        return -1;
    }
    if(rc_filter_pid(&F3_o, kp3_o, 0, kd3_o, 2*DT, DT)){
    fprintf(stderr, "ERROR in mb_controller, failed to make steering loop filter\n");
        return -1;
    }
    return 0;
}


int mb_controller_update_odometry(mb_state_t* mb_state, mb_odometry_t*mb_odometry){
    float u_1 = 0.0;
    float u_2 = 0.0;
    float u_3 = 0.0;
    float dist_error;
    float theta_error;
    float head_error;

    mb_state->speed = speed_o;

    dist_error = mb_state->spDist - mb_state->phi;
    mb_state->spTheta = rc_filter_march(&F2_o, dist_error) + bodyAngleOffset_o;
    theta_error = mb_state->spTheta - mb_state->theta;
    u_1 = rc_filter_march(&F1_o, theta_error);
    

    // if(mb_state->turn)
    // {
    //     mb_state->diff = u_1;
    // }
    // else
    // {
    //     head_error = mb_state->spHeading - mb_odometry->heading;
    //      // mb_state->diff = rc_filter_march(&F3_o, head_error);
    //     mb_state->diff = kp3_o*(head_error);
    // }

    head_error = mb_state->spHeading - mb_odometry->heading;
    mb_state->diff = rc_filter_march(&F3_o, head_error);
    mb_state->diff = kp3_o*(head_error);
    

    mb_state->left_cmd = u_1 - mb_state->diff;
    mb_state->right_cmd = u_1 + mb_state->diff;

     FILE* fp = fopen("pidData.csv", "a");
    fprintf(fp, "%7.3f", mb_state->theta);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->thetadot);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->P1);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->I1);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->D1);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->torque);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->left_cmd);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->right_cmd);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%d", mb_state->left_encoder);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%d", mb_state->right_encoder);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->spTheta);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->diff);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->speedLeft);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->speedRight);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_odometry->x);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_odometry->y);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_odometry->heading);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->spHeading);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->opti_x);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->opti_y);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->opti_yaw);
    fprintf(fp, "%s", "\n");
    fclose(fp);


}


int mb_controller_odometry_cleanup(){
    rc_filter_free(&F1_o);
    rc_filter_free(&F2_o);
    rc_filter_free(&F3_o);

    return 0;
}
