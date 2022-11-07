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

PID mypid_x(dt, max_val, min_val, Kp, Kd, Ki);
// PID mypid_y(dt, max_val*0.25, min_val*0.25, Kp, Kd, Ki);
PID mypid_y(dt, max_val*0.25, min_val*0.25, 0.5, 0.001, 0.005);
PID mypid_z(dt, max_val, min_val, Kp+0.18, Kd, Ki);

float upper_threshold = 170.0;
float lesser_threshold = 180.0 - upper_threshold;

double slow_vel = 0;
double gas_vel = 0;


geometry_msgs::Pose current_pose;
void pose_cb(const geometry_msgs::Pose::ConstPtr msg){
    current_pose = *msg;
}

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

vision::XY_cor_msg gating_pos;
void gating_cb(vision::XY_cor_msgConstPtr msg){
    gating_pos = *msg;
}


std_msgs::Bool there_is_landing_pad;
void detect_pad_cb(std_msgs::BoolConstPtr msg){
    there_is_landing_pad = *msg;
}

std_msgs::Bool is_gated;
void detect_gate_cb(std_msgs::BoolConstPtr msg){
    is_gated = *msg;
}

std_msgs::Bool is_centered;
void centered_cb(std_msgs::BoolConstPtr msg){
    is_centered = *msg;
}

vision::XY_cor_msg cc;
void cc_cb(const vision::XY_cor_msgConstPtr msg){
    cc = *msg;
}

vision::XY_cor_msg tc;
void tc_cb(const vision::XY_cor_msgConstPtr msg){
    tc = *msg;
}

void print_drone_pos(){
    ROS_INFO("x = %f, y = %f, z = %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
}

double getYawFromQuaternion(double w, double x, double y, double z){
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp); 
    return yaw;
}

double getMeterPerPixel(){
    /*Based on data on data.txt got linear equation:
    y = 0.00195X - 0.00042
    */
   double x = current_pose.position.z;
   return 0.00195*x - 0.00042;
}

void take_off(geometry_msgs::Twist *vel, ros::Publisher *cmd_pub, bool *takeoff, ros::Time *waktu){
    (*vel).linear.x = 0;
    (*vel).linear.y = 0;
    (*vel).linear.z = mypid_z.calculate(2, current_pose.position.z);
    (*cmd_pub).publish(*vel);

    if (current_pose.position.z < 1.8){
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

double calc_x_to_center(){
    double meterPerPixel = getMeterPerPixel();
    double center = 320.0*meterPerPixel;
    double line_x_in_meter = line_xy.x * meterPerPixel;
    double dist_line_x_to_center = abs(line_x_in_meter-center);

    return dist_line_x_to_center;
}

void go_forward(geometry_msgs::Twist *vel){
    /*
    Moving around:
   -/-     w(+x)/    -/-
    a(+y)  s(-x)     d(-y)
    */
    (*vel).angular.z = 0;
    (*vel).linear.x = gas_vel;
    double dist_line_x_to_center = calc_x_to_center();

    ROS_INFO("MAJU dan dist_x_to_center: %.2f", dist_line_x_to_center);
}

void turning(geometry_msgs::Twist *vel, PID *mypid_yaw, bool isCentering){
    if(!isCentering){
        (*vel).linear.y=0;
        (*vel).linear.x=slow_vel;
    }

    if(line_angle > 100.0){
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
        }else if(predict == -1){
            (*vel).angular.z = (*mypid_yaw).calculate(M_PI, line_angle*DEG_2_RAD);
            ROS_INFO("==PREDICT drone KIRI===");           
        }   
    }
    
}

void fix_y(geometry_msgs::Twist *vel){
    double meterPerPixel = getMeterPerPixel();
    double center = 320.0*meterPerPixel;
    double line_x_in_meter = line_xy.x * meterPerPixel;
    double dist_line_x_to_center = calc_x_to_center();

    if(dist_line_x_to_center > 0.1 && (line_angle >= 150.0 || line_angle <= 30.0)){
        (*vel).linear.y = mypid_y.calculate(center, line_x_in_meter);
        ROS_INFO("line centering, dist_to_center: %.2f", dist_line_x_to_center);
        (*vel).linear.x = slow_vel;
    }
}

void center_to_pad(geometry_msgs::Twist *vel, bool *is_center_with_pad){
    ROS_INFO("CENTERING DI ATAS");
    double meterPerPixel = getMeterPerPixel();
    double center_x = 320.0*meterPerPixel;
    double center_y = 240*meterPerPixel;
    double cur_x = landing_pad_pos.x*meterPerPixel;
    double cur_y = landing_pad_pos.y*meterPerPixel;
    
    vel->linear.y = mypid_y.calculate(center_x,  cur_x);
    vel->linear.x = mypid_x.calculate(center_y, cur_y);

    ROS_INFO("cx:%f -> %f | cy:%f -> %f", cur_x, center_y, cur_y, center_x);
    ROS_INFO("cx:%f -> %f | cy:%f -> %f", landing_pad_pos.x, 320.0, landing_pad_pos.y, 240.0);

    if(abs(center_x - cur_x) <= 0.05 && abs(center_y- cur_y) <= 0.05){
       ROS_INFO("DAH CUKUP TENGAH");
       *is_center_with_pad = true;
    }
}

void landing(geometry_msgs::Twist *vel, ros::Publisher *cmd_pub, ros::Publisher *land_pub, bool *isDone){
    (*vel).linear.z = mypid_z.calculate(0, current_pose.position.z);
    (*cmd_pub).publish(*vel);

    if (current_pose.position.z > 0.5){
        ROS_INFO("Turun bosku!");
        print_drone_pos();
    }
    else{
        ROS_INFO("SENDING LAND TRIGGER");

        std_msgs::Empty empty;
        land_pub->publish(empty);

        if(current_pose.position.z <= 0.1){
            *isDone = true;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_line_follower");
    ros::NodeHandle nh;
    
    //takeoff api
    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>
        ("/drone/takeoff", 5);
        
    //movement api
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>
        ("/cmd_vel", 5);
    // ros::Publisher dist_to_center_pub = nh.advertise<std_msgs::Float64>
    //     ("/dist_center", 5); //line centering testing
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>
        ("/drone/gt_pose", 5, pose_cb);
    ros::Subscriber angle_sub = nh.subscribe<std_msgs::Float32>
        ("/angle", 5, angle_cb);
    ros::Subscriber line_pos_sub = nh.subscribe<vision::XY_cor_msg>
        ("/line_pos", 5, line_xy_cb);
    ros::Subscriber move_assist_sub = nh.subscribe<std_msgs::Int8>
        ("/guide", 5, assist_cb);

    //gating api
    ros::Subscriber detecting_gate_sub = nh.subscribe<std_msgs::Bool>
        ("/cntr", 5, detect_gate_cb);
    // ros::Subscriber centered_sub = nh.subscribe<std_msgs::Bool>
        // ("/ctr", 5, centered_cb);
    // ros::Subscriber cc_sub = nh.subscribe<vision::XY_cor_msg>
        // ("/cc", 5, cc_cb);
    ros::Subscriber tc_sub = nh.subscribe<vision::XY_cor_msg>
        ("/tc", 5, tc_cb);

    //landing api
    // ros::Publisher land_stream_pub = nh.advertise<std_msgs::Bool>
        // ("/stream_land", 5);
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

    for(int i = 10; ros::ok() && i > 0; --i){
        vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

  
    ros::Time waktu = ros::Time::now();
    bool done_takeoff= false;
    
    printf("Threshold 1 (175 <= x <= 180) (default:178):  ");
    scanf("%f", &upper_threshold);
    lesser_threshold = 180.0-upper_threshold;

    printf("rem_vel & gas_vel (default: 0.3 1): ");
    scanf("%lf %lf",&slow_vel, &gas_vel);

    printf("P, I , D for yaw (default: 0.9 0.011 0)? ");
    double p, i, d;
    scanf("%lf %lf %lf", &p, &i, &d);
    PID mypid_yaw(dt, theta_max_val, theta_min_val, p, i, d);

    
    ROS_INFO("Set drone to take off");
    takeoff_pub.publish(empty);
    ROS_INFO("Done");

    ros::Time t0 = ros::Time::now();
    // std_msgs::Bool is_land_stream;
    bool is_centered_with_pad = false;
    bool lock_yaw = false;
    double yaw_locked = 0.0;
    bool isDone = false;

    while (ros::ok() && !isDone)
    {
        if(!done_takeoff){
            take_off(&vel, &vel_pub, &done_takeoff, &waktu);
        }
        else{
            if(!there_is_landing_pad.data || ros::Time::now() - t0 < ros::Duration(8)){ //duration for escaping landing pad
                if(line_angle >= upper_threshold || line_angle <= lesser_threshold){ //around 180 deg
                    go_forward(&vel);
                }
                else{ //far from 180 deg
                    turning(&vel, &mypid_yaw, false);
                    
                }
                fix_y(&vel);
                vel_pub.publish(vel);
            }
            else{ //there is landing pad
                if(there_is_landing_pad.data || is_centered_with_pad){
                    ROS_INFO("ADA PADD");
                    center_to_pad(&vel, &is_centered_with_pad);
                    turning(&vel, &mypid_yaw, true);
                    vel_pub.publish(vel);
                    
                    if(is_centered_with_pad){
                        landing(&vel, &vel_pub, &land_pub, &isDone);
                    }
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
