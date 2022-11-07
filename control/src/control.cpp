#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "control/pid.hpp"
#include "control/soeroYawUtils.hpp"
#include "control/pid.hpp"
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
double slow_vel = 0.2;
double gas_vel = 0.4;
int h_front_frame = 360;
int w_front_frame = 640;
int h_down_frame = 180;
int w_down_frame = 640;


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


/*Receive msg about keadaan line*/
std_msgs::Bool there_is_line;
void detect_line_cb(std_msgs::BoolConstPtr msg){
    there_is_line = *msg;
}

std_msgs::Bool there_is_kotak;
void detect_kotak_cb(std_msgs::BoolConstPtr msg){
    there_is_kotak = *msg;
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

std_msgs::UInt64 cntr_area;
void cntr_area_cb(const std_msgs::UInt64ConstPtr msg){
    cntr_area = *msg;
}

std_msgs::Int32 del_y;
void del_y_gate_cb(const std_msgs::Int32ConstPtr msg){
    del_y = *msg;
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
    else if (ros::Time::now() - (*waktu) > ros::Duration(1)){
        *takeoff = true;
        ROS_INFO("Ready to Move");
    }
    else{
        ROS_INFO("Wait 1 second to make sure drone's position is stable");
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
    ROS_INFO("MAJU -> %f ", (*vel).linear.x);
    
}

void change_yaw_gate(geometry_msgs::Twist *vel, PID *mypid_rasio){
    // (*vel).angular.z = soero::CalcYawVelPID();
    (*vel).angular.z = ((*mypid_rasio).calculate(1.428, rasio.data)) * direction.data;
    ROS_INFO("CHANGE YAW GATE -> ang_z: %f, gap: %f, direction: %d", (*vel).angular.z, abs(rasio.data-1.428), direction.data);
}

/*Logic for correcting yaw*/
void change_yaw(geometry_msgs::Twist *vel, PID *mypid_yaw, bool isCentering){
    //stop
    if(!isCentering){
        //if not centering, tamper with x and y, else dont tamper with the vel
        (*vel).linear.y = 0;
        (*vel).linear.x = 0.0;
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
    ROS_INFO("HOLDING ALT -> %f to %f", current_pose.position.z, locked_cor);
}

/*Fix drone's y relative to line*/
void fix_y(geometry_msgs::Twist *vel){
  // , ros::Publisher *dist_to_center_pub <-- extra argument for debuggin
    double meterPerPixel = getMeterPerPixel();
    double center = w_down_frame / 2 * meterPerPixel;
    double line_x_in_meter = line_xy.x * meterPerPixel;

    std_msgs::Float64 dist_line_x_to_center;
    dist_line_x_to_center.data = abs(line_x_in_meter-center);

    if(dist_line_x_to_center.data > 0.05 && (line_angle >= 140.0 || line_angle <= 40.0)){
        (*vel).linear.y = mypid_y.calculate(center, line_x_in_meter);
        ROS_INFO("LINE CENTERING -> dist_to_center: %.2f", dist_line_x_to_center.data);
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
    ROS_INFO("CENTERING DI ATAS -> cx:%f to %f | cy:%f to %f", landing_pad_pos.x, 320.0, landing_pad_pos.y, 240.0);

    if(abs(center_x - cur_x) <= 0.05 && abs(center_y- cur_y) <= 0.05){
       ROS_INFO("DAH CUKUP TENGAH");
       *is_center_with_pad = true;
    }
}

/*HANDLE CENTERING TO GATE*/
void centering_to_gate(geometry_msgs::Twist *vel, bool *is_centered_with_gate){
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
  double center_y = h_front_frame / 2;
  double center_x = w_front_frame / 2;
  
  vel->linear.x = 0;
  vel->linear.z = mypid_z_vision_gate.calculate(center_y, gate_coor.y);
  vel->linear.y = mypid_y_vision_gate.calculate(center_x, gate_coor.x);
  if(abs(center_y - gate_coor.y) < 10 && abs(center_x - gate_coor.x) < 10){
    *is_centered_with_gate = true;
  }
  else{
    *is_centered_with_gate = false;
  }
  ROS_INFO("CENTERING TO GATE -> x: %f y: %f", abs(center_x - gate_coor.x), abs(center_y- gate_coor.y));
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
    ros::init(argc, argv, "demo_line_gate_follower");
    ros::NodeHandle nh;

    //takeoff api
    ros::Publisher takeoff_pub              = nh.advertise<std_msgs::Empty>          ("/drone/takeoff", 10);
    //movement api
    ros::Publisher  vel_pub                 = nh.advertise<geometry_msgs::Twist>     ("/cmd_vel", 10);
    ros::Subscriber pose_sub                = nh.subscribe<geometry_msgs::Pose>      ("/drone/gt_pose", 10, pose_cb);
    ros::Subscriber angle_sub               = nh.subscribe<std_msgs::Float32>        ("/angle", 10, angle_cb);
    ros::Subscriber line_pos_sub            = nh.subscribe<vision::XY_cor_msg>       ("/line_pos", 10, line_xy_cb);
    ros::Subscriber move_assist_sub         = nh.subscribe<std_msgs::Int8>           ("/guide", 10, assist_cb);
    ros::Subscriber detecting_line_sub      = nh.subscribe<std_msgs::Bool>           ("/line", 10, detect_line_cb);
    //gating api
    ros::Subscriber detecting_gate_sub      = nh.subscribe<std_msgs::Bool>           ("/cntr", 10, detect_gate_cb);
    ros::Subscriber tc_sub                  = nh.subscribe<vision::XY_cor_msg>       ("/tc", 10, gate_coor_cb);
    ros::Subscriber gate_rasio_sub          = nh.subscribe<std_msgs::Float64>        ("/rasio", 10, gate_rasio_cb);
    ros::Subscriber yaw_direct_sub          = nh.subscribe<std_msgs::Int8>           ("/direction", 10, yaw_direct_cb);
    ros::Subscriber cntr_area_sub           = nh.subscribe<std_msgs::UInt64>         ("/ctr_area", 10, cntr_area_cb);
    ros::Subscriber del_y_gate_sub          = nh.subscribe<std_msgs::Int32>          ("/del_y", 10, del_y_gate_cb);
    ros::Subscriber detecting_kotak_sub     = nh.subscribe<std_msgs::Bool>           ("/kotak", 10, detect_kotak_cb);

    //landing api
    ros::Publisher  land_stream_pub         = nh.advertise<std_msgs::Bool>           ("/stream_land", 10);
    ros::Publisher  land_pub                = nh.advertise<std_msgs::Empty>          ("/drone/land", 10);
    ros::Subscriber landing_cor_sub         = nh.subscribe<vision::XY_cor_msg>       ("/landing_pos", 10, landing_cb);   
    ros::Subscriber detecting_pad_sub       = nh.subscribe<std_msgs::Bool>           ("/pad_detected", 10, detect_pad_cb);  

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
    ros::Time throttle_start_time = ros::Time::now();
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
    i = 0.011;
    d = 0.0;
    // scanf("%lf %lf %lf", &p, &d, &i);
    PID mypid_yaw(dt, theta_max_val, theta_min_val, p, i, d);
    PID mypid_rasio(dt, theta_max_val, theta_min_val, p, i, d);

    /*Set drone to be ready for takeoff*/
    ROS_INFO("Set drone to take off");
    takeoff_pub.publish(empty);
    ROS_INFO("Done");

    ros::Time t0 = ros::Time::now();
    // std_msgs::Bool is_land_stream;
    bool is_centered_with_pad = false;
    bool is_centered_with_gate = false;
    bool lock_yaw = false;
    double yaw_locked = 0.0;
    bool isDone = false;
    bool is_close_enough = false;
    double z_lock_cor = 1.5;
    bool is_full_throttle = false;
    bool ever_tracked_line = false;
    bool lagi_centering = false;
    double centered_y_pos;
    double centered_z_pos;
    // bool is_centering_before_tracing = true;
    // line_angle = -1.0;

    /*Take off now [takeoff loop]*/
    while(ros::ok() && !done_takeoff){
        take_off(&vel, &vel_pub, &done_takeoff, &waktu, z_lock_cor);
        //Crucial to call callback func
        ros::spinOnce();
        rate.sleep(); 
    }

    ROS_INFO("TAKE OFF SUCCESS");

    /*MAIN LOOP FOR MOVEMENT*/
    while (ros::ok() && !isDone){
        /* Search the track */
        if((line_angle == -1.0 && !there_is_line.data && !lagi_centering && !ever_tracked_line)){
            is_full_throttle = false;
            vel.linear.x = 0.2;
            vel.linear.y = 0.0;
            vel.angular.z = 0.0;
            hold_z(&vel, z_lock_cor);
            ROS_INFO("Searching the Track");

            vel_pub.publish(vel);
        }
        /* line or gate tracing */
        else if(!there_is_landing_pad.data || ros::Time::now() - t0 < ros::Duration(20) || there_is_gate.data || there_is_line.data){
          //Check ada gate
          if(there_is_gate.data && !is_full_throttle && rasio.data > 1.1){
            lagi_centering = true;
            centering_to_gate(&vel, &is_centered_with_gate);
            
            if(there_is_kotak.data){
                change_yaw_gate(&vel, &mypid_rasio);
                if(abs(del_y.data) < 5){
                    centering_to_gate(&vel, &is_centered_with_gate);
                }
                else{
                    vel.linear.y = 0;
                    vel.linear.z = 0;
                }
            }
            else{
                vel.angular.z = 0;
            }

            ROS_INFO("CHECKING VEL -> vx: %f, vy: %f, vz: %f", vel.linear.x,vel.linear.y, vel.linear.z);

            if(is_centered_with_gate){
              if(cntr_area.data < 50000){
                go_forward(&vel, slow_vel);
              }  
              else if(cntr_area.data > 50000 && abs(del_y.data) < 5){
                is_full_throttle = true;
                centered_y_pos = current_pose.position.y;
                centered_z_pos = current_pose.position.z;
                z_lock_cor = 1.0;
                throttle_start_time = ros::Time::now();
                ROS_INFO("Time to throttle");
              }
              z_lock_cor = getMinVal(current_pose.position.z, 1.8);
              hold_z(&vel, z_lock_cor);
              ROS_INFO("maju melewati gate while centering");
            } 
          }
          // full throttle
          else if(is_full_throttle && ros::Time::now() - throttle_start_time < ros::Duration(2.2)){
            go_forward(&vel, 0.8);
            vel.linear.z = mypid_z.calculate(centered_z_pos, current_pose.position.z);
            vel.linear.y = mypid_y.calculate(centered_y_pos, current_pose.position.y);
            ROS_INFO("FULL THROTTLE -> %f", vel.linear.x);
            lagi_centering = false;
            z_lock_cor = 1.0;
          }
          //line tracing
          else{
            is_centered_with_gate = false;
            is_full_throttle = false;
            ever_tracked_line = true;
            ROS_INFO("LINE TRACKING");
            //Follow the line while early or no landing pad
            if(line_angle >= lesser_threshold || line_angle <= upper_threshold){
                go_forward(&vel);
            }
            else{
                change_yaw(&vel, &mypid_yaw, false);

            }
            fix_y(&vel);
            hold_z(&vel, z_lock_cor);
          }

          vel_pub.publish(vel);
        }
        /* landing */
        else if(ever_tracked_line && there_is_landing_pad.data){
            ROS_INFO("H Tracking");
            //time to land!
            if(there_is_landing_pad.data || is_centered_with_pad){
                ROS_INFO("ADA PADD");
                hold_z(&vel, 1.8);
                //centering
                center_to_pad(&vel, &is_centered_with_pad);
                // change_yaw(&vel, &mypid_yaw, true);
                vel_pub.publish(vel);
                if(is_centered_with_pad){
                    landing(&vel, &vel_pub, &land_pub, &isDone);
                }
            }
        }
        ros::spinOnce();
        rate.sleep();    
    }
    
    ros::Duration t = (ros::Time::now() - t0);
    printf("========Waktu: %lf========", t.toSec());
    return 0;
}