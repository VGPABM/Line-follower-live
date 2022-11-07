#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "../include/control/pid.hpp"
#include "../include/control/soeroYawUtils.hpp"
#include "vision/XY_cor_msg.h"

#define DEG_2_RAD 0.01745329251
#define pix_per_meter 0.01875

//=====================PID SETUP===================================//
double dt = 1/20.0;
double max_val = 1.0;
double min_val = -1.0;
double Kp = 0.45;
double Ki = 0.005;
double Kd = 0.0007;
double theta_max_val=2.0;
double theta_min_val=-2.0;

/*----------------------del time, interval val, kp,    kd,    ki*/
PID mypid_x(dt, max_val, min_val, Kp, Kd, Ki);
PID mypid_y(dt, max_val*0.25, min_val*0.25, 0.7, 0.001, 0.005);
PID mypid_z(dt, max_val, min_val, Kp+0.18, Kd, Ki);
PID mypid_z_vision_gate(dt, max_val, min_val, 0.0020, 0.0011, 0);
PID mypid_y_vision_gate(dt, max_val, min_val, 0.0020, 0.0011, 0);
//=====================PID SETUP===================================//

float lesser_threshold = 178.0;
float upper_threshold = 180.0 - lesser_threshold;
double slow_vel = 0.1;
double gas_vel = 0.3;


geometry_msgs::Pose current_pose;
void pose_cb(const geometry_msgs::Pose::ConstPtr msg){
    current_pose = *msg;
}

/*detected line angle*/
float line_angle;
void angle_cb(const std_msgs::Float32ConstPtr msg){
    line_angle = (*msg).data;
}

vision::XY_cor_msg line_xy;
void line_xy_cb(vision::XY_cor_msgConstPtr msg){
    line_xy = *msg;
}

uint8_t predict;
void assist_cb(std_msgs::Int8ConstPtr msg){
    predict = (*msg).data;
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

/*Receive msg about keadaan gate*/
std_msgs::Bool there_is_gate;
void detect_gate_cb(std_msgs::BoolConstPtr msg){
    there_is_gate = *msg;
}

/*variable for gate coordinate*/
vision::XY_cor_msg gate_coor;
void gate_coor_cb(const vision::XY_cor_msgConstPtr msg){
    gate_coor = *msg;
}

std_msgs::Float64 rasio;
void gate_rasio_cb(const std_msgs::Float64ConstPtr msg){
    rasio = *msg;
}

std_msgs::Int8 direction;
void yaw_direct_cb(const std_msgs::Int8ConstPtr msg){
    direction = *msg;
}

void print_drone_pos(){
    ROS_INFO("x = %f, y = %f, z = %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
}

double getMeterPerPixel(){
    /*Based on data on data.txt got linear equation:
    y = 0.00195X - 0.00042
    */
   double x = current_pose.position.z;
   return 0.00195*x - 0.00042;
}

// double rasioToYaw(double rasio){ 
//     return (-2.1*rasio) + 2.99;
// }

double getMinVal(double a, double b){
  return a < b ? a: b;
}

/*Handle take off*/
void take_off(geometry_msgs::Twist *vel, ros::Publisher *cmd_pub, bool *takeoff, ros::Time *waktu, double initHeight){
    (*vel).linear.x = 0;
    (*vel).linear.y = 0;
    (*vel).linear.z = mypid_z.calculate(initHeight, current_pose.position.z);
    (*cmd_pub).publish(*vel);
    
    if (current_pose.position.z < initHeight-0.2){
        *waktu = ros::Time::now();
        ROS_INFO("Taking off! | z: %.6f", current_pose.position.z);
    }
    else if (ros::Time::now() - (*waktu) > ros::Duration(2)){
        *takeoff = true;
        ROS_INFO("Ready to Move");
    }
    else{
        ROS_INFO("Wait 2 seconds to make sure drone's position is stable");
    }
}

/*Has logic for going forward*/
void go_forward(geometry_msgs::Twist *vel, double certain_vel=gas_vel){
    /*
    Moving around:
   -/-     w(+x)/    -/-
    a(+y)  s(-x)     d(-y)
    */
    (*vel).angular.z = 0;
    (*vel).linear.x = certain_vel;
    ROS_INFO("MAJU go_forward");
    
}

void change_yaw_gate(geometry_msgs::Twist *vel, PID *mypid_yaw){
    // cmd_vel.angular.z = soero::calcYawVelCompass(last_compass_heading, compass_heading.data*DEG_2_RAD, mypid_yaw_centering);
    // (*vel).angular.z = soero::CalcYawVelPID();
}

/*Logic for correcting yaw*/
void change_yaw(geometry_msgs::Twist *vel, PID *mypid_yaw, bool isCentering){
    //stop
    if(!isCentering){
        //if not centering, tamper with x and y, else dont tamper with the vel
        (*vel).linear.y = 0;
        (*vel).linear.x = 0;
    }
    if(line_angle > 100.0){
        //180 target
        (*vel).angular.z = (*mypid_yaw).calculate(M_PI, line_angle*DEG_2_RAD);
        ROS_INFO("drone muter KIRI");
    }
    else if(line_angle < 80.0){
        (*vel).angular.z = (*mypid_yaw).calculate(0, line_angle*DEG_2_RAD);
        ROS_INFO("drone muter KANAN");
    }
    else{
        //Cek guide
        ROS_INFO("===USING PREDICTION FROM PENNCIPI %d===", predict);
        if(predict == 1){
            (*vel).angular.z = (*mypid_yaw).calculate(0, line_angle*DEG_2_RAD);
            ROS_INFO("===PREDICT drone KANAN===");            
        }
        else if(predict == 2){
            (*vel).angular.z = (*mypid_yaw).calculate(M_PI, line_angle*DEG_2_RAD);
            ROS_INFO("==PREDICT drone KIRI===");           
        }
        else{
            ROS_INFO("SHIT, EXACTLY BALANCED, AS ALL THING SHOULD BE");
        }
    }
    

}

void hold_z(geometry_msgs::Twist *vel, double locked_cor){
    (*vel).linear.z = mypid_z.calculate(locked_cor, current_pose.position.z);
}

/*Fix drone's y relative to line*/
void fix_y(geometry_msgs::Twist *vel){
  // , ros::Publisher *dist_to_center_pub <-- extra argument for debuggin
    double meterPerPixel = getMeterPerPixel();
    double center = 320.0*meterPerPixel;
    double line_x_in_meter = line_xy.x * meterPerPixel;

    std_msgs::Float64 dist_line_x_to_center;
    dist_line_x_to_center.data = abs(line_x_in_meter-center);

    if(dist_line_x_to_center.data > 0.05 && (line_angle >= 140.0 || line_angle <= 40.0)){
        (*vel).linear.y = mypid_y.calculate(center, line_x_in_meter);
        ROS_INFO("line centering, dist_to_center: %.2f", dist_line_x_to_center.data);
        // (*dist_to_center_pub).publish(dist_line_x_to_center);
        (*vel).linear.x = slow_vel;
    }
}

/*HANDLE CENTERING TO LANDING PAD*/
void center_to_pad(geometry_msgs::Twist *vel, bool *is_center_with_pad){
    ROS_INFO("CENTERING DI ATAS");
    double meterPerPixel = getMeterPerPixel();
    double center_x = 320.0*meterPerPixel;
    double center_y = 240*meterPerPixel;
    double cur_x = landing_pad_pos.x*meterPerPixel;
    double cur_y = landing_pad_pos.y*meterPerPixel;
    
    vel->linear.y = mypid_y.calculate(center_x,  cur_x);
    vel->linear.x = mypid_x.calculate(center_y, cur_y);
    ROS_INFO("cx:%f -> %f | cy:%f -> %f", landing_pad_pos.x, 320.0, landing_pad_pos.y, 240.0);

    if(abs(center_x - cur_x) <= 0.05 && abs(center_y- cur_y) <= 0.05){
       ROS_INFO("DAH CUKUP TENGAH");
       *is_center_with_pad = true;
    }
}

/*HANDLE CENTERING TO GATE*/
void center_to_gate(geometry_msgs::Twist *vel, bool *is_centered_with_gate){
  /*
  translation of coordinate x-y vision to yz drone
  left-right = drone's y = vision's x
  up-down = drone's z = vision's y
  y(+) = left, y(-) = right
  z(+) = up, z(-) = down
  vision x (+++) = ada di kanan
  vision x (---) = ada di kiri
  vision y (+++) = ada di atas
  vision y (---) = ada di bawah 
  */
  /*UKURAN FRAME CAMERA : 640x360*/

  vel->linear.z = mypid_z_vision_gate.calculate(180.0, gate_coor.y);
  vel->linear.y = mypid_y_vision_gate.calculate(160.0, gate_coor.x);
  if(abs(180.0 - gate_coor.y) < 10 && abs(160.0 - gate_coor.x) < 10){
    *is_centered_with_gate = true;
  }
  ROS_INFO("centering to gate, x: %f y: %f", abs(160.0 - gate_coor.x), abs(180.0 - gate_coor.y));
}

/*HANDLE LANDING LOGIC*/
void landing(geometry_msgs::Twist *vel, ros::Publisher *cmd_pub, ros::Publisher *land_pub, bool *isDone){
    (*vel).linear.z = mypid_z.calculate(0, current_pose.position.z);
    (*cmd_pub).publish(*vel);

    if (current_pose.position.z > 0.5){
        ROS_INFO("Turun bosku!");
        print_drone_pos();
    }
    else{
        ROS_INFO("SENDING LAND TRIGGER");
        int i=0;
        std_msgs::Empty empty;
        land_pub->publish(empty);
        if(current_pose.position.z <= 0.1){
            *isDone = true;
        }
    }
}

//=================================================MAIN-MAIN-MAIN===================================================================//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "testing");
    ros::NodeHandle nh;

    //takeoff api
    ros::Publisher takeoff_pub              = nh.advertise<std_msgs::Empty>          ("/drone/takeoff", 5);
    //movement api
    ros::Publisher  vel_pub                 = nh.advertise<geometry_msgs::Twist>     ("/cmd_vel", 5);
    ros::Subscriber pose_sub                = nh.subscribe<geometry_msgs::Pose>      ("/drone/gt_pose", 5, pose_cb);
    ros::Subscriber angle_sub               = nh.subscribe<std_msgs::Float32>        ("/angle", 5, angle_cb);
    ros::Subscriber line_pos_sub            = nh.subscribe<vision::XY_cor_msg>       ("/line_pos", 5, line_xy_cb);
    ros::Subscriber move_assist_sub         = nh.subscribe<std_msgs::Int8>           ("/guide", 5, assist_cb);
    //gating api
    ros::Subscriber detecting_gate_sub      = nh.subscribe<std_msgs::Bool>           ("/cntr", 5, detect_gate_cb);
    ros::Subscriber tc_sub                  = nh.subscribe<vision::XY_cor_msg>       ("/tc", 5, gate_coor_cb);
    ros::Subscriber gate_rasio_sub          = nh.subscribe<std_msgs::Float64>        ("/rasio", 5, gate_rasio_cb);
    ros::Subscriber yaw_direct_sub          = nh.subscribe<std_msgs::Int8>           ("/direction", 5, yaw_direct_cb);
    //landing api
    ros::Publisher  land_stream_pub         = nh.advertise<std_msgs::Bool>           ("/stream_land", 5);
    ros::Publisher  land_pub                = nh.advertise<std_msgs::Empty>          ("/drone/land", 5);
    ros::Subscriber landing_cor_sub         = nh.subscribe<vision::XY_cor_msg>       ("/landing_pos", 5, landing_cb);   
    ros::Subscriber detecting_pad_sub       = nh.subscribe<std_msgs::Bool>           ("/pad_detected", 5, detect_pad_cb);  

    ros::Rate rate(1/dt);

    std_msgs::Empty empty;
    geometry_msgs::Twist vel;
    // vel.linear.x = 0;
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


    //ganti threshold
    printf("Threshold 1 (175 <= x <= 180) (default:178):  ");
    // scanf("%f", &lesser_threshold);
    upper_threshold = 180.0-lesser_threshold;

    printf("slow_vel & gas_vel (default: 0.3 0.1): ");
    // scanf("%lf %lf",&slow_vel, &gas_vel);

    printf("P, D , I for yaw (default: 0.9 0.011 0)? ");
    double p, i, d;
    p = 0.9;
    d = 0.011;
    i = 0.0;
    // scanf("%lf %lf %lf", &p, &d, &i);
    PID mypid_rasio(dt, theta_max_val, theta_min_val, p, d, i);

    ros::Time t0 = ros::Time::now();
    // std_msgs::Bool is_land_stream;
    bool is_centered_with_pad = false;
    bool is_centered_with_gate = false;
    bool lock_yaw = false;
    double yaw_locked = 0.0;
    bool isDone = false;
    bool is_line_tracked = false;
    bool is_close_enough = false;
    double z_lock_cor = 1.3;
    // bool is_centering_before_tracing = true;
    line_angle = -1.0;

    /*Take off now [takeoff loop]*/
    while(ros::ok()){
        vel.angular.z = (mypid_rasio.calculate(1.428, rasio.data)) * direction.data;
        ROS_INFO("Vel: %f, gap: %f, direction: %d", mypid_rasio.calculate(1.428, rasio.data), abs(rasio.data-1.428), direction.data);
        vel_pub.publish(vel);
       
        ros::spinOnce();
        rate.sleep(); 
    } 
    
    ros::Duration t = (ros::Time::now() - t0);
    printf("========Waktu: %lf========", t.toSec());
    return 0;
}