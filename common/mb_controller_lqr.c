#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "mb_controller.h"
#include "mb_defs.h"
#include "mb_structs.h"

/*******************************************************************************
* int mb_controller_init()
*
* this initializes the controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/

float K1;
float K2;
float K3;
float K4;
float overallGain;
float bodyAngleOffset;


int mb_controller_lqr_init(){
    mb_controller_load_config();
    /* TODO initialize your controllers here*/
    

    return 0;
}

/*******************************************************************************
* int mb_controller_load_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/


int mb_controller_lqr_load_config(){
    FILE* file = fopen("lqr.cfg", "r");
    if (file == NULL){
        printf("Error opening %s\n", "lqr.cfg" );
    }
    /* TODO parse your config file here*/

    fscanf(file, "%f", &K1);
	fscanf(file, "%f", &K2);
	fscanf(file, "%f", &K3);
    fscanf(file, "%f", &K4);
    fscanf(file, "%f", &overallGain);
    fscanf(file, "%f", &bodyAngleOffset);

    K1 = K1*overallGain;
    K2 = K2*overallGain;
    K3 = K3*overallGain;
    K4 = K4*overallGain;


    fclose(file);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* 
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* this should only be called in the imu call back function, no mutex needed
*
* return 0 on success
*
*******************************************************************************/

float dutyLeft = 0;
float dutyRight = 0;
float theta = 0;
float torque = 0;

float actVolt = 11.5;


int mb_controller_lqr_update(mb_state_t* mb_state){

    float thetaErr = 0 - mb_state->theta + bodyAngleOffset;
    float thetadotErr = 0 - mb_state->thetadot;
    float phiErr = mb_state->spPhi - mb_state->phi;
    float phidotErr = mb_state->spPhidot - mb_state->phidot;
    
    float duty = K1*thetaErr + K2*thetadotErr + K3*phiErr + K4*phidotErr;

    mb_state->left_cmd = duty*(NOM_VOLTAGE/actVolt); //This accounts for the actual voltage being less than the nominal that we used to measured the motor parameters
    mb_state->right_cmd = duty*(NOM_VOLTAGE/actVolt);

    FILE* fp = fopen("pidData.csv", "a");
    fprintf(fp, "%7.3f", mb_state->theta);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->thetadot);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->phi);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->phidot);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->spPhi);
    fprintf(fp, "%s", ",");
    fprintf(fp, "%7.3f", mb_state->spPhidot);
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
    fprintf(fp, "%7.3f", mb_state->setPointBodyAngle);
    fprintf(fp, "%s", "\n");
    fclose(fp);

    return 0;
}


/*******************************************************************************
* int mb_controller_cleanup()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_controller_lqr_cleanup(){
    return 0;
}
