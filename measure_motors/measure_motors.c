/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
* TODO: Capture encoder readings, current readings, timestamps etc. to a file
*       to analyze and determine motor parameters
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>	// for mkdir and chmod
#include <sys/types.h>	// for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"

FILE* f1;
FILE* fp;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	
	// make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)<0){
        fprintf(stderr,"Failed to set governor to PERFORMANCE\n");
        return -1;
    }

	// initialize enocders
    if(rc_encoder_eqep_init()==-1){
        fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if(rc_adc_init()==-1){
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    // initialize motors
    if(mb_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze mb_motors\n");
        return -1;
    }

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();


    rc_set_state(RUNNING);

    printf("\nRaw encoder positions\n");
    printf("      E1   |");
    printf("      E2   |");
    //printf(" CS_0 |");
    //printf(" CS_1 |");
    printf(" \n");

    mb_motor_set(RIGHT_MOTOR, 1);
    mb_motor_set(LEFT_MOTOR, 1);


    while(rc_get_state()!=EXITING){
        fp = fopen("motor_data.csv", "a");
        printf("\r");
        int i = 1;
        int polarity = 1;
        for(i=1;i<=2;i++){

            if (i == 1){
                polarity = ENC_1_POL;
            }
            else
            {
                polarity = ENC_2_POL;
            }
            int enc = rc_encoder_eqep_read(i)*polarity;
            printf("%10d |", enc);
            fprintf(fp, "%f", rc_nanos_since_boot()/1.0e9);
            fprintf(fp, "%s", ",");
            fprintf(fp, "%d", enc);
            fprintf(fp, "%s", ",");
        }
        // float curr1 = mb_motor_read_current(1);
        // float curr2 = mb_motor_read_current(2);
        // printf("%6.2f |", curr1);
        // printf("%6.2f |", curr2);
        // fprintf(fp, "%f", curr1);
        // fprintf(fp, "%s", ",");
        // fprintf(fp, "%f", curr2);
        fprintf(fp, "%s", "\n");
        
        fflush(stdout);
        rc_usleep(500);

        fclose(fp);
    }
    printf("\n");

	// exit cleanly
    rc_adc_cleanup();
	rc_encoder_eqep_cleanup();
    mb_motor_cleanup();
	rc_remove_pid_file();   // remove pid file LAST
	return 0;
}