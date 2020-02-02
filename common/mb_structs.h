#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   theta;             // body angle (rad)
    float   thetadot;          // body angular velocity (rad/s)
    float   phi;               // average wheel angle (rad)
    float   phidot;            // average wheel angular velocity (rad/s)
    int     left_encoder;      // total left encoder counts
    int     right_encoder;     // total right encoder counts
    int     prev_left_enc;     // left encoder counts since last timestep
    int     prev_right_enc;    // right encoder counts since last timestep
    float   speedLeft;         // left wheel speed in rad/s
    float   speedRight;        // right wheel speed in rad/s
    float   voltage;           // current battery voltage
    double  dist;              // distance
    float   speed;             // this is used to determine how far to tell the outer loop to move each period
    int     turn;              // for box mode: if 1, we will turn, if 0, drive straight
    int     boxSide;           // which side of the box we are on


    //outputs
    float   P1;
    float   I1;
    float   D1;
    float   P2;
    float   I2;
    float   D2;
    float   prevI1;
    float   prevI2;
    float   torque;
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]
    float   diff; // commanded differential velocity between the two wheels

    //commanded values
    float   spTheta;
    float   spPhi;
    float   spDist;
    float   spHeading;



    float opti_x;
    float opti_y;
    float opti_roll;
    float opti_pitch;
    float opti_yaw;

    //TODO: Add more variables to this state as needed
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
    int drag_race;
    int box_ctl;
    int bodyAngleAchieved;
    int count;
 };

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float psi;      //orientation from initialization in rad
    float heading;      //orientation from initialization in rad
};

#endif