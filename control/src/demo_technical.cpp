#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "control/pid.hpp"
#include "vision/XY_cor_msg.h"

#define DEG_2_RAD 0.01745329251
#define meter_per_pixel 0.01
//=====================PID SETUP===================================//
double dt = 1/20.0;
double max_val = 1.5;
double min_val = -1.5;
double Kp = 0.45;
double Ki = 0.005;
double Kd = 0.0007;
double theta_max_val=2.0;
double theta_min_val=-2.0;
PID mypid_x_back(dt, max_val, min_val, 0.750, 0.600, 0.00000);
PID mypid_y_back(dt, max_val, min_val, 0.750, 0.600, 0.00000);
PID mypid_y_pseudo(dt, max_val, min_val, 0.750, 0.600, 0.00000);
PID mypid_y(dt, max_val*0.25, min_val*0.25, 0.7, 0.001, 0.007);
PID mypid_z(dt, max_val, min_val, Kp+0.39, Kd+0.02, Ki);
PID mypid_z_takeoff(dt, max_val, min_val, 1, 0.355, 0.015);
PID mypid_z_vision_gate(dt, max_val, min_val, 0.30, 0.0031, 0);
PID mypid_y_vision_gate(dt, max_val, min_val, 0.30, 0.0031, 0);
PID mypid_y_vision_PAD(dt, max_val, min_val, 0.30, 0.0031, 0);
PID mypid_x_vision_PAD(dt, max_val, min_val, 0.30, 0.0031, 0);
float threshold1 = 170.0;
float threshold2 = 180.0 - threshold1;
double remvel = 0.2;
double gasval = 1;

//===========================FUNCTIONS============================//
geometry_msgs::Pose current_pose;
void pose_cb(const geometry_msgs::Pose::ConstPtr msg){
    current_pose = *msg;
}

vision::XY_cor_msg landing_pad_pos;
void landing_cb(vision::XY_cor_msgConstPtr msg){
    landing_pad_pos = *msg;
}

/*Receive msg about keadaan landing pad*/
std_msgs::Bool there_is_landing_pad;
void detect_pad_cb(std_msgs::BoolConstPtr msg){
    there_is_landing_pad = *msg;
}

std_msgs::Bool is_centered;
void centering_cb(std_msgs::BoolConstPtr msg){
    is_centered = *msg;
}

std_msgs::Bool greenLight;
void gas_cb(std_msgs::BoolConstPtr msg){
    greenLight = *msg;
}

/*Receive msg about keadaan gate*/
std_msgs::Bool there_is_gate;
void detect_gate_cb(std_msgs::BoolConstPtr msg){
    there_is_gate = *msg;
}

std_msgs::Bool area_is_ok;
void area_cb(std_msgs::BoolConstPtr msg){
    area_is_ok = *msg;
}

/*variable for gate coordinate*/
vision::XY_cor_msg gate_coor;
void gate_coor_cb(const vision::XY_cor_msgConstPtr msg){
    gate_coor = *msg;
}

vision::XY_cor_msg left_vs_right;
void ori_cb(const vision::XY_cor_msgConstPtr msg){
    left_vs_right = *msg;
}

vision::XY_cor_msg initGateCor;
void pseudo_cb(const vision::XY_cor_msgConstPtr msg){
    initGateCor = *msg;
}

void print_drone_pos(){
    ROS_INFO("x = %f, y = %f, z = %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
}

/*Function to extract yaw from quarternion*/
double calculateYaw(double z, double w){
    /*Convert quarternion z, w to radian*/
    double yaw = atan2(z, w)*2.0;
    /*Map degree to -180 til 180*/
    if(yaw <= 0){
        yaw += M_PI;
    }else{
        yaw -= M_PI;
    }
    return yaw;
}

/*Func to calc constant meter per pixel*/
double getMeterPerPixel(){
   double x = current_pose.position.z;
   return 0.00195*x - 0.00042;
}

double getMinVal(double a, double b){
  return a < b ? a: b;
}

double jarak_titik(geometry_msgs::Pose tujuan, geometry_msgs::Pose asal){
    double jarak_x = tujuan.position.x - asal.position.x;
    double jarak_y = tujuan.position.y - asal.position.y;
    double jarak_z = tujuan.position.z- asal.position.z;
    double jarak_total = sqrt(pow(jarak_x, 2) + pow(jarak_y, 2) + pow(jarak_z, 2));
    return jarak_total;
}

/*Handle take off*/
void take_off(geometry_msgs::Twist *vel, ros::Publisher *cmd_pub, bool *takeoff, ros::Time *waktu, double initHeight){
    //calculate speed for take off
    (*vel).linear.x = 0;
    (*vel).linear.y = 0;
    (*vel).linear.z = mypid_z_takeoff.calculate(initHeight, current_pose.position.z);
    (*cmd_pub).publish(*vel);
    if (current_pose.position.z < initHeight){
        *waktu = ros::Time::now();
        ROS_INFO("Taking off! | z: %.6f", current_pose.position.z);
    }
    else if (ros::Time::now() - (*waktu) > ros::Duration(1)){
        *takeoff = true;
        ROS_INFO("Ready to Move");
    }else{
        ROS_INFO("Wait 1 second to make sure drone's position is stable");
    }
}

/*Has logic for going forward*/
void go_forward(geometry_msgs::Twist *vel, double thevel){
    (*vel).angular.z = 0;
    (*vel).linear.x = thevel;
}

/*Logic for correcting yaw*/
void change_yaw(geometry_msgs::Twist *vel, PID *mypid_yaw, double init_yaw){
    //stop
    ROS_INFO("fix yaw");
    (*vel).angular.z = (*mypid_yaw).calculate(init_yaw, calculateYaw(current_pose.orientation.z, current_pose.orientation.w));
}

/*Logic for correcting yaw*/
void change_yaw_toGate(geometry_msgs::Twist *vel, PID *mypid_yaw){
    //stop
    ROS_INFO("GATE ORIENTATION AS GUIDE");
    (*vel).angular.z = (*mypid_yaw).calculate(left_vs_right.x*0.01, left_vs_right.y*0.01);
}

void hold_z(geometry_msgs::Twist *vel, double locked_cor){
    /*trying to stay in locked z m*/
    (*vel).linear.z = mypid_z.calculate(locked_cor, current_pose.position.z);
}

/*HANDLE CENTERING TO LANDING PAD*/
void center_to_pad(geometry_msgs::Twist *vel, bool *is_center_with_pad){
    ROS_INFO("CENTERING DI ATAS");
    double meterPerPixel = getMeterPerPixel();
    double center_x = 320.0*meterPerPixel;
    double center_y = 180*meterPerPixel;
    double cur_x = landing_pad_pos.x*meterPerPixel;
    double cur_y = landing_pad_pos.y*meterPerPixel;
    
    vel->linear.y = mypid_y_vision_PAD.calculate(center_x, cur_x);
    vel->linear.x = mypid_x_vision_PAD.calculate(center_y, cur_y);
    
   if(abs(center_x - cur_x) <= 0.5 && abs(center_y- cur_y) <= 0.1){
       ROS_INFO("CENTERED: OK");
       *is_center_with_pad = true;
   }
}

/*HANDLE CENTERING TO GATE*/
void center_to_gate(geometry_msgs::Twist *vel){
  ROS_INFO("centering to gate");

 /*UKURAN FRAME CAMERA : 640x360*/
  vel->linear.z = mypid_z_vision_gate.calculate(180.0*meter_per_pixel, gate_coor.y*meter_per_pixel);
  vel->linear.y = mypid_y_vision_gate.calculate(320.0*meter_per_pixel, gate_coor.x*meter_per_pixel);
  vel->linear.x = 0;
}

void center_to_pseudoGate(geometry_msgs::Twist *vel){
  ROS_INFO("centering to  SMALL PSEUDO gate");
 /*UKURAN FRAME CAMERA : 640x360*/
  vel->linear.y = mypid_y_pseudo.calculate(320.0*meter_per_pixel*2, initGateCor.x*meter_per_pixel*2);
}

/*HANDLE LANDING LOGIC*/
void landing(geometry_msgs::Twist *vel, ros::Publisher *cmd_pub, ros::Publisher *land_pub, bool *isDone){
    (*vel).linear.z = mypid_z.calculate(0, current_pose.position.z);
    
    (*cmd_pub).publish(*vel);
    if (current_pose.position.z > 0.5){
        ROS_INFO("going down!");
        print_drone_pos();
    }
    else{
        ROS_INFO("SENDING LAND TRIGGER");
        int i=0;
        std_msgs::Empty empty;
        land_pub->publish(empty);
        if(current_pose.position.z <= 0.3){
            *isDone = true;
        }
    }
}

//=================================================MAIN-MAIN-MAIN===================================================================//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_technical_node");
    ros::NodeHandle nh;
    
    //takeoff api
    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>
        ("/drone/takeoff", 5);
    //movement api
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
        ("/cmd_vel", 5);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>
        ("/drone/gt_pose", 5, pose_cb);
    //gating api
    ros::Subscriber detecting_gate_sub = nh.subscribe<std_msgs::Bool>
        ("/cntr", 5, detect_gate_cb);
    ros::Subscriber centering_sub = nh.subscribe<std_msgs::Bool>
        ("/ctr", 5, centering_cb);
    ros::Subscriber tc_sub = nh.subscribe<vision::XY_cor_msg>
        ("/tc", 5, gate_coor_cb);
    ros::Subscriber ori_sub = nh.subscribe<vision::XY_cor_msg>
        ("/ori", 5, ori_cb);
    ros::Subscriber pseudo_gate_sub = nh.subscribe<vision::XY_cor_msg>
        ("/pseudo", 5, pseudo_cb);        
    ros::Subscriber area_sub = nh.subscribe<std_msgs::Bool>
        ("/areaok", 5, area_cb);
    ros::Subscriber gas_sub = nh.subscribe<std_msgs::Bool>
        ("/gas", 5, gas_cb);
    //landing api
    ros::Publisher land_pub = nh.advertise<std_msgs::Empty>
        ("/drone/land", 5);
    ros::Subscriber landing_cor_sub = nh.subscribe<vision::XY_cor_msg>
        ("/landing_pos", 5, landing_cb);   
    ros::Subscriber detecting_pad_sub = nh.subscribe<std_msgs::Bool>
        ("/pad_detected", 5, detect_pad_cb);  

    ros::Rate rate(1/dt);
    
    std_msgs::Empty empty;
    geometry_msgs::Twist vel;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }
    //Helper variable
    ros::Time waktu = ros::Time::now();
    bool done_takeoff= false;
    
    double p, i, d;
    p = 0.9; i = 0.011; d = 0;
    PID mypid_yaw(dt, theta_max_val, theta_min_val, p, i, d);

    double init_yaw_orient = calculateYaw(current_pose.orientation.z, current_pose.orientation.w);
    /*Set drone to be ready for takeoff*/
    ROS_INFO("Set drone to take off");
    takeoff_pub.publish(empty);
    ROS_INFO("Done");

    ros::Time t0 = ros::Time::now();
    geometry_msgs::Pose above_pad_pose;
    geometry_msgs::Pose before_gate_pose;
    geometry_msgs::Pose last_non_center_pose;
    short counter = 0;
    double z_lock_cor = 2;
    bool isDone = false;
    bool is_centered_with_pad = false;

    /*Take off now [takeoff loop]*/
    while(ros::ok() && !done_takeoff){
        take_off(&vel, &vel_pub, &done_takeoff, &waktu, z_lock_cor);
        //Crucial to call callback func
        ros::spinOnce();
        rate.sleep(); 
    }
    
    ROS_INFO("TAKE OFF SUCCESS");
    above_pad_pose = current_pose;
    above_pad_pose.position.z=0;

    /*MAIN LOOP FOR MOVEMENT*/
    bool doneFirstTime = false;
    for(counter = 0; counter < 3 && ros::ok(); counter++){
        bool centering = false;
        bool lock_yaw = false;
        double yaw_locked = 0.0;
        bool lock_maju = false;
        bool is_ready_land = false;
        bool succedd= false;
        bool back_to_school = false;
        bool switch_yaw = false;
        bool firstTimeCentered = false;
        double centerLockYaw = 0.0;
        ROS_INFO("Looping");
    
        /* GATING LOOP */
        while (ros::ok() && !back_to_school)
        {
            if(!there_is_gate.data && !centering && !lock_maju && !doneFirstTime){
                ROS_INFO("Not enough area to detect gate| Contour.x: %2f", initGateCor.x - 320.0);
                // go_forward(&vel, 1.1);
                vel.linear.x = 1.4*(abs(initGateCor.x - 320.0) < 250);
                //lock in 2 meter
                vel.linear.z = mypid_z.calculate(2, current_pose.position.z);
                center_to_pseudoGate(&vel);
            }
            else if(there_is_gate.data && !is_centered.data && !lock_maju){
                doneFirstTime = true;
                ROS_INFO("Gate detected");
                center_to_gate(&vel);
                change_yaw_toGate(&vel, &mypid_yaw);
                switch_yaw = true;
                centering = true;
                last_non_center_pose = current_pose;
            }
            else if(there_is_gate.data && is_centered.data && !area_is_ok.data){
                if(!firstTimeCentered){
                    ROS_INFO("FIRST TIME");
                    centerLockYaw = calculateYaw(current_pose.orientation.z, current_pose.orientation.w);
                    firstTimeCentered = true;
                }
                ROS_INFO("CENTERED:OK, READY TO GO FORWARD");
                center_to_gate(&vel);
                change_yaw(&vel, &mypid_yaw, centerLockYaw);
                switch_yaw = true;
                go_forward(&vel, 0.1);
            }
            else if((there_is_gate.data && is_centered.data && area_is_ok.data)){
                ROS_INFO("AREA: OK, WAIT SIGNAL FOR ULTIMATE THROTTLE");
                before_gate_pose.position.x = current_pose.position.x;
                vel.linear.x = 0.000;
            //Wait for gas signal
                if(greenLight.data){
                    //PASS THE GATE
                    ros::Time t1 = ros::Time::now();
                    while(ros::ok() && ros::Time::now() - t1 < ros::Duration(1.5)){
                        ROS_INFO("GO!, x0 = %f, x1 = %f", before_gate_pose.position.x, current_pose.position.x);
                        go_forward(&vel, 2.0);
                        vel_pub.publish(vel);
                        succedd = true;
                        ros::spinOnce();
                        rate.sleep();
                    }
                    back_to_school = true;
                    ROS_INFO("Exiting gate loop");
                }
            }
            else if(!succedd){
                ROS_INFO("Too low or too close| z=%.2f", current_pose.position.z);
                vel.linear.z = mypid_z.calculate(2, current_pose.position.z);
                go_forward(&vel, -0.1);
            }

            if(!switch_yaw){
                change_yaw(&vel, &mypid_yaw, init_yaw_orient);
            }
            switch_yaw = false;
            vel_pub.publish(vel);

            //END OF MOVEMENT LOGIC
            ros::spinOnce();
            rate.sleep();    
        }
        firstTimeCentered = false;
        double above_gate_pose = current_pose.position.z + 2;
        bool readytoDown = false;
        ROS_INFO("change loop");

        /* BACK-TO-last-centered-pos or above pad LOOP */
        while(ros::ok && back_to_school){
            if(abs(current_pose.position.z - above_gate_pose) > 0.1 && !readytoDown){
                vel.linear.x = 0;
                vel.linear.y = 0;
                vel.linear.z = mypid_z.calculate(above_gate_pose, current_pose.position.z);
                ROS_INFO("Going to the top of the gate %2f", current_pose.position.z);
            }
            //JIKA KURANG DARI 2, KE LAST NON CENTERED POSITION
            else if(counter < 2){
                readytoDown = true;
                vel.linear.z = 0;
                vel.linear.y = mypid_y_back.calculate(last_non_center_pose.position.x, current_pose.position.x);
                vel.linear.x = mypid_x_back.calculate(current_pose.position.y, last_non_center_pose.position.y);
                ROS_INFO("GO TO LAST NON CENTERED POSITION");
                ROS_INFO("x:%2f, y:%2f", vel.linear.x, vel.linear.y);
                if(abs(last_non_center_pose.position.x - current_pose.position.x) < 0.3 && abs(last_non_center_pose.position.y - current_pose.position.y) < 0.3){
                    ROS_INFO("Go down!|z: %2f", current_pose.position.z);
                    vel.linear.z = mypid_z.calculate(last_non_center_pose.position.z, current_pose.position.z);
                }
                back_to_school = jarak_titik(last_non_center_pose, current_pose) > 0.2;
            }
            //OK COUNTER == 2 BERARTI KE ABOVE PAD
            else{
                
                readytoDown = true;
                vel.linear.y = mypid_y_back.calculate(above_pad_pose.position.x, current_pose.position.x);
                vel.linear.x = mypid_x_back.calculate(current_pose.position.y, above_pad_pose.position.y);
                vel.linear.z = mypid_z.calculate(above_pad_pose.position.z, current_pose.position.z) * (abs(above_pad_pose.position.y - current_pose.position.y) < 0.75);
                back_to_school = abs(current_pose.position.z) > 0.2; 
                ROS_INFO("Back to take-off pad");     
            }
            change_yaw(&vel, &mypid_yaw, 1.5707963268);
            vel_pub.publish(vel);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("loop done");
    }
    /*END OF MAIN MOVEMENT LOOP*/
    ros::Duration t = (ros::Time::now() - t0);
    printf("========Total time: %lf========", t.toSec());
    return 0;
}