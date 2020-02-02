/*******************************************************************************
* balancebot.c
*
* Main template code for the BalanceBot Project
* based on rc_balance
* 
*******************************************************************************/

#include <math.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/cpu.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/pthread.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <math.h>


#include "balancebot.h"
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"
#include "../common/mb_controller.h"
#include "../common/mb_controller_odometry.h"
#include "../common/mb_controller_lqr.h"
#include "../common/mb_odometry.h"


#define PI                        3.14159265

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

    if(rc_dsm_init()==-1){
		fprintf(stderr,"failed to start initialize DSM\n");
		return -1;
	}

	printf("initializing xbee... \n");
	//initalize XBee Radio
	int baudrate = BAUDRATE;
	if(XBEE_init(baudrate)==-1){
		fprintf(stderr,"Error initializing XBee\n");
		return -1;
	};

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	printf("starting print thread... \n");
	pthread_t  printf_thread;
	rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	rc_pthread_create(&setpoint_control_thread, setpoint_control_loop, (void*) NULL, SCHED_FIFO, 50);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Z_DOWN;
	mpu_config.dmp_fetch_accel_gyro = 1;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

	//rc_nanosleep(5E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

	//initialize state values that otherwise aren't initialized before referenced
	mb_state.left_encoder = 0;
	mb_state.right_encoder = 0;
	mb_state.I1 = 0;
	mb_state.I2 = 0;
	mb_state.diff = 0;
	mb_setpoints.bodyAngleAchieved = 0;
	mb_setpoints.count = 0;
	
	mb_state.spPhi = 0;
	mb_state.spPhidot = 0;
	
	mb_state.dist = 0;
	mb_state.spHeading = 0;
	mb_state.speed = 0;
	mb_state.boxSide = 1;



	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_controller_init();

	printf("initializing motors...\n");
	mb_motor_init();
	//mb_motor_brake(1);

	printf("resetting encoders...\n");
	rc_encoder_eqep_write(1, 0);
	rc_encoder_eqep_write(2, 0);

	printf("initializing odometry...\n");
	mb_odometry_init(&mb_odometry, 0.0,0.0,0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&balancebot_controller);

	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
		// always sleep at some point
		rc_nanosleep(1E9);
	}
	
	// exit cleanly
	rc_mpu_power_off();
	mb_motor_cleanup();
	mb_controller_cleanup();
	mb_controller_lqr_cleanup();
	mb_controller_odometry_cleanup();
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_remove_pid_file(); // remove pid file LAST 
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/

float yaw;
float yaw_error;
float dist_tr;
float dist_tr_est;
float dist_tr_y;

void balancebot_controller(){
	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// Read IMU
	mb_state.theta = mpu_data.dmp_TaitBryan[TB_PITCH_X];
	mb_state.thetadot = mpu_data.gyro[0]*PI/180.0;
	// Read encoders
	mb_state.prev_left_enc = mb_state.left_encoder;
	mb_state.prev_right_enc = mb_state.right_encoder;
	mb_state.left_encoder = ENC_1_POL*rc_encoder_eqep_read(1);
	mb_state.right_encoder = ENC_2_POL*rc_encoder_eqep_read(2);
	mb_state.phi = (1/2.0)*(2*PI)*(mb_state.left_encoder + mb_state.right_encoder)/(GEAR_RATIO*ENCODER_RES);
	mb_state.speedLeft = (mb_state.left_encoder - mb_state.prev_left_enc)/(DT*GEAR_RATIO*ENCODER_RES);
	mb_state.speedRight = (mb_state.right_encoder - mb_state.prev_right_enc)/(DT*GEAR_RATIO*ENCODER_RES);
	mb_state.phidot = (1/2.0)*(2*PI)*(mb_state.speedLeft + mb_state.speedRight);
    // Update odometry

	static int loopNum;

	if(loopNum < 600)
	{
		mb_state.spPhi = 0;
		mb_state.spPhidot = 0;
	}
	else
	{
		mb_state.spPhi = distance;
		// mb_state.spPhidot = speed;
	}
	
	
	mb_controller_update(&mb_state);


    if(!mb_setpoints.manual_ctl){
		// if (loopNum < 300)							
		//This code is for getting step response data
		// 	mb_state.spPhi = 0.0;
		// else
		// 	mb_state.spPhi = 0.3*2/WHEEL_DIAMETER;

		mb_state.spPhi = 0.0;

		mb_setpoints.turn_velocity = 0;
		mb_setpoints.fwd_velocity = 0;

		// Calculate controller outputs
		mb_controller_update(&mb_state, &mb_setpoints);

    	//send motor commands
		mb_motor_set(LEFT_MOTOR, mb_state.left_cmd);
		mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd);
   	}

    if(mb_setpoints.manual_ctl){
    	// Calculate controller outputs
		mb_controller_update(&mb_state, &mb_setpoints);

    	//send motor commands
		mb_motor_set(LEFT_MOTOR, mb_state.left_cmd);
		mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd);
   	}

     if(!mb_setpoints.box_ctl){
		mb_state.spPhi = 0.0;
		mb_setpoints.turn_velocity = 0;
		mb_setpoints.fwd_velocity = 0;
		mb_state.spDist += mb_state.speed;
		mb_state.dist = 0.0;

		/* This code is to gather data of a step response to a heading change */
		// if (loopNum < 300)
		// 	mb_state.spHeading = 0;
		// else
		// 	mb_state.spHeading = PI/2.0;
		
		// Calculate controller outputs
		mb_controller_update_odometry(&mb_state, &mb_odometry);

    	//send motor commands
		mb_motor_set(LEFT_MOTOR, mb_state.left_cmd);
		mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd);
   	}

    if(mb_setpoints.box_ctl){
    	// Calculate controller outputs
		mb_state.spDist += mb_state.speed;
		float dist = 0.93;


		if ((mb_state.boxSide == 1) && (mb_odometry.x > dist - WHEEL_BASE/2.0))
		{
			mb_state.spHeading += PI/2.0;
			mb_state.boxSide = 2;
		}
		else if (mb_state.boxSide == 2 && mb_odometry.y > dist - WHEEL_BASE/2.0)
		{
			mb_state.spHeading += PI/2.0;
			mb_state.boxSide = 3;
		}
		else if (mb_state.boxSide == 3 && mb_odometry.x < 0 + WHEEL_BASE/2.0)
		{
			mb_state.spHeading += PI/2.0;
			mb_state.boxSide = 4;
		}
		else if (mb_state.boxSide == 4 && mb_odometry.y < 0 + WHEEL_BASE/2.0)
		{
			mb_state.spHeading += PI/2.0;
			mb_state.boxSide = 1;
		}

			mb_controller_update_odometry(&mb_state, &mb_odometry);
			mb_motor_set(LEFT_MOTOR, mb_state.left_cmd);
			mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd);

		
   	}


	XBEE_getData();
	double q_array[4] = {xbeeMsg.qw, xbeeMsg.qx, xbeeMsg.qy, xbeeMsg.qz};
	double tb_array[3] = {0, 0, 0};
	rc_quaternion_to_tb_array(q_array, tb_array);
	mb_state.opti_x = xbeeMsg.x;
	mb_state.opti_y = -xbeeMsg.y;	    //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_roll = tb_array[0];
	mb_state.opti_pitch = -tb_array[1]; //xBee quaternion is in Z-down, need Z-up
	mb_state.opti_yaw = -tb_array[2];   //xBee quaternion is in Z-down, need Z-up
	
	
   	//unlock state mutex
    pthread_mutex_unlock(&state_mutex);

	loopNum += 1;
}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/


void* setpoint_control_loop(void* ptr){


	while(1){


		if(rc_dsm_is_new_data()){
				// TODO: Handle the DSM data from the Spektrum radio reciever
				// You may should implement switching between manual and autonomous mode
				// using channel 5 of the DSM data.

				float deadZoneWidth = 0.2;
				float manSwitch = rc_dsm_ch_normalized(5); // left toggle (3-pos)
				float dragSwitch = rc_dsm_ch_normalized(1); // right toggle (2-pos)
				float boxSwitch = rc_dsm_ch_normalized(7)
				float tVel;
				float fVel;
				if (manSwitch < 0.1)
					mb_setpoints.manual_ctl = 0;
				else if(manSwitch > 0.1)
					mb_setpoints.manual_ctl = 1;

				if (dragSwitch < -0.5)
				{
					mb_setpoints.drag_race = 0;
					mb_setpoints.count = 0;
				}
				else
					mb_setpoints.drag_race = 1;
					
				if(boxSwitch < 0.1)
					mb_setpoints.box_ctl = 0;
				else if(boxSwitch > 0.1)
					mb_setpoints.box_ctl = 1;

				

				
				if(mb_setpoints.manual_ctl){

					tVel = rc_dsm_ch_normalized(4);
					fVel = rc_dsm_ch_normalized(3);

					if(fabs(tVel) <= deadZoneWidth)
						tVel = 0;
					else {
						if(tVel < 0)
							tVel = tVel + deadZoneWidth;
						else
							tVel = tVel - deadZoneWidth;
						
					}

					if(fabs(fVel) <= deadZoneWidth)
						fVel = 0;
					else {
						if(fVel < 0)
							fVel = fVel + deadZoneWidth;
						else
							fVel = fVel - deadZoneWidth;
						
					}

					mb_setpoints.turn_velocity = tVel;
					mb_setpoints.fwd_velocity = fVel;
					// if (fVel < 0.3)
					// 	mb_setpoints.fwd_velocity = 0.0;
					// else
					// {
					// 	mb_setpoints.fwd_velocity = 0.15;
					// }
					
				}

				if(mb_setpoints.drag_race){


				}

		}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
	return NULL;
}




/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();

		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |                                     PID		                       	|");
			printf("\n");
			printf("    θ    |");
			printf("   θset  |");
			printf("    θ.   |");
			printf("    φ    |");
			printf("   φset  |");
			printf("  forwV  |");
			printf("  turnV  |");
			printf("   diff  |");
			printf("  L vel  |");
			printf("  R vel  |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("  DutyL  |");
			printf("  DutyR  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    ψ    |");
			printf("  count  |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			pthread_mutex_lock(&state_mutex);
			printf("%7.3f  |", mb_state.theta);
			printf("%7.3f  |", mb_state.spTheta);
			printf("%7.3f  |", mb_state.thetadot);
			printf("%7.3f  |", mb_state.phi);
			printf("%7.3f  |", mb_state.spPhi);
			printf("%7.3f  |", mb_setpoints.fwd_velocity);
			printf("%7.3f  |", mb_setpoints.turn_velocity);
			printf("%7.3f  |", mb_state.diff);
			printf("%7.3f  |", mb_state.speedLeft);
			printf("%7.3f  |", mb_state.speedRight);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			printf("%7.3f  |", mb_state.left_cmd);
			printf("%7.3f  |", mb_state.right_cmd);
			printf("%7.3f  |", mb_state.opti_x);
			printf("%7.3f  |", mb_state.opti_y);
			printf("%7.3f  |", mb_state.opti_yaw);
			printf("%7d  |", mb_setpoints.count);

			pthread_mutex_unlock(&state_mutex);
			fflush(stdout);
		}
		rc_nanosleep(1E9/PRINTF_HZ);
	}
	return NULL;
} 