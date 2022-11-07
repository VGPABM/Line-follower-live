#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "control/pid.hpp"
#include "control/soeroLog.hpp"
#include "control/soeroYawUtils.hpp"
#include "vision/RotatedRect.h"

#include <stdexcept>
#include <string>

#define _CRT_SECURE_NO_WARNINGS

#define DEG_2_RAD 0.01745329251
#define pix_per_meter 0.01875
#define HALF_PI 1.57079632679

//=====================PID SETUP===================================//
double dt = 1/32.0;
double max_val = 1.0;
double min_val = -1.0;
double Kp = 0.45;
double Ki = 0.000;
double Kd = 0.0007;
double theta_max_val=2.0;
double theta_min_val=-2.0;

PID mypid_x(dt, max_val, min_val, Kp, Kd, Ki);
PID mypid_y(dt, max_val*0.5, min_val*0.5, 0.7, 0.001, 0.000);
PID mypid_z(dt, max_val, min_val, Kp+2.5, Kd+0.002, Ki);
PID mypid_z_hrz(dt, max_val+1.5, min_val-1.5, Kp+5.5, Kd+1, Ki+0.01);
PID mypid_yaw(dt, 1.5, -1.5, 0.900, 0.100, 0.00000);
/*----------------------del time, interval val, kp,    kd,    ki*/
PID mypid_z_vision_gate(dt, max_val, min_val, 0.0040, 0.0011, 0);
PID mypid_y_vision_gate(dt, max_val, min_val, 0.0040, 0.0016, 0);
PID mypid_y_hor_gate(dt, max_val, min_val, 0.500, 0.0011, 0);
PID mypid_x_hor_gate(dt, max_val, min_val, 0.500, 0.0021, 0);

//=====================CALLBACKs===================================//
geometry_msgs::Pose current_pose;
void pose_cb(const geometry_msgs::Pose::ConstPtr msg){
    current_pose = *msg;
}

vision::RotatedRect upperLine;
void uline_cb(const vision::RotatedRect::ConstPtr msg){
    upperLine = *msg;
}

vision::RotatedRect lowerLine;
void lline_cb(const vision::RotatedRect::ConstPtr msg){
    lowerLine = *msg;
}

vision::RotatedRect landingPad;
void lpad_cb(const vision::RotatedRect::ConstPtr msg){
    landingPad = *msg;
}

vision::RotatedRect verticalGate;
void vgate_cb(const vision::RotatedRect::ConstPtr msg){
    verticalGate = *msg;
}

vision::RotatedRect horizontalGate;
void hgate_cb(const vision::RotatedRect::ConstPtr msg){
    horizontalGate = *msg;
}

vision::RotatedRect horizontalGateHelper;
void hgate_help_cb(const vision::RotatedRect::ConstPtr msg){
    horizontalGateHelper = *msg;
}

std_msgs::Float32MultiArray verticalGateHelper;
void vgate_help1_cb(const std_msgs::Float32MultiArray::ConstPtr msg){
    verticalGateHelper = *msg;
}

std_msgs::UInt8 specialVGateCmd;
void vgate_help2_cb(const std_msgs::UInt8::ConstPtr msg){
    specialVGateCmd = *msg;
}

// =====================HELPER =========================== //
/*
Drone movement:
        +x
         ^
         |
 +y <----+----> -y
         |
         v
        -x

+ang.z => clockwise         -> yaw ngecilin
-ang.z => counter clockwise -> yaw besar
*/

//  ===============BASIC==============HELPER===========
void zerofied_main_vel(geometry_msgs::Twist &vel){
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.z = 0;
}

bool isClose(double val1, double val2, double threshold){
    return std::fabs(val1 - val2) < threshold;
}

/*used to dynamically change the centering's threshold for vertical gate based on the w and h
imagine a vertical gate with size wxh
+-----------------+
|                 |
|                 |h
|        `        |
|                 |
+-----------------+
        w

default ratio_x is: 3/16, meaning the threshold for x or width is: w*ratio_x = w*3/16 = w*0.1875
got it from this thought:
width of gate is 1m according to rule.
from experimenting in gazebo, 1 square is 1m^2,(tiap grid is 1m)
and got drone size ~1/4 *1m^2 or 0.5m x 0.5m

put it in the middle of the gate: (ascii art is only some part of the gate)

|                         |
|--0.25m--<drone>--0.25m--|
|                         |

mayan 0.5m error tolerance, so berapa thresholdnya?, well range thresholdnya is panjang drone ==>0.5m
bagi dua buat sisi kanan + kiri ==> 0.25m
ok nice, so threshold = 0.25m or 1/4w, hmm, too risky maybe?, ok cut it down biar lebih akurat, kecilin sikit

range threshold
[---0.25m---|center|---0.25m---]

let's cut it 3/4 nya:
[-3/16m-|center|-3/16m-] --> so 3/16*w (kebetulan karena size gate=1m jadi rationya lsg sama ae)

how about the ratio_y?
same, but jalan pikirnya:
error tolerance = 0.125m, ( i mean dronenya tipis, so aman?)=>1/8h
meaning range threshold = h-2*1/8h = 6/8h = 3/4h
bagi dua kiri kanan = 3/4*0.5 = 3/8h
too risky? kali 3/4 => 3/8*3/4 = 9/32 => 0.28125h

*/
constexpr double ertol_x = 0.275;// satuan ==> width (not meter)
constexpr double risky_cutoff_x=0.75;
constexpr double final_ratio_x = (0.5-ertol_x)*risky_cutoff_x;
constexpr double ertol_y = 0.2;
constexpr double risky_cutoff_y=0.75;
constexpr double final_ratio_y = (0.5-ertol_y)*risky_cutoff_y;
bool isCenteredRatio(double cx, double cy, 
                     vision::RotatedRect &object,double ratio_x=final_ratio_x, double ratio_y=final_ratio_y){
    double tx = object.w*ratio_x;
    double ty = object.h*ratio_y;
    soero::Log::Info("tx: %f | ty: %f", tx, ty);
    return isClose(object.x, cx, tx/2) && isClose(object.y, cy, ty/2);
    // return isClose(object.x, cx, tx/2) && isClose(object.y, cy, ty/2);
}

// normal threshold, static, cek apakah abs(x1-x2) < threshold for x and y
bool isCenteredStatic(double x1, double y1, double x2, double y2, double threshold){
    return isClose(x1, x2, threshold) && isClose(y1, y2, threshold);
}

// does what you think it does :), nah jk, it convert a string to well integer... (carefull for overflow)
int convert_string_to_integer(std::string &arg){
    try {
    std::size_t pos;
    int x = std::stoi(arg, &pos);
    if (pos < arg.size()) {
        std::cerr << "Trailing characters after number: " << arg << '\n';
    }
    return x;
    } catch (std::invalid_argument const &ex) {
    std::cerr << "Invalid number: " << arg << '\n';
    } catch (std::out_of_range const &ex) {
    std::cerr << "Number out of range: " << arg << '\n';
    }
    return -1;// biar compiler ga ngasi warning
}
//=======================================================================//
//                    MATH STUFF                                        //
//=======================================================================
constexpr double vx_max = 0.64;                     // velocity maximum when line is align
constexpr double vx_min = 0.005;                    // minimum velocity when angular z exceed vaz_max
constexpr double vaz_max = 0.15;                    // angular velocity maximum to consider still moving/ do vmin
constexpr double _m_fx = (vx_max - vx_min)/vaz_max; // gradient for x that seamlessly transition between vmax and vmin
double line_activation_x(double vel_ang_z, double mult){
    double vel = -std::fabs(_m_fx*vel_ang_z) + vx_max;  // try this func in desmos to see visualization
    return std::max(vel, vx_min)*mult;                  // cap the min vel to vmin not negative and set to 0 if mult is 0
}

constexpr double area_min = 20.0;                   // minimum area to consider moving towards the gate, instead of centering only
constexpr double area_stop = 85.0;                  // maximum area to consider STOP, dah deket bang
constexpr double area_min_v = 0.5; // max velocity when gate area is == area_min | jangan 0.5 berbahaya suka gagal-->ditambal pakek if else room top h bottom h gate
constexpr double _m_avx = area_min_v/(area_min - area_stop);// calculate gradient for seamless transition of vel
constexpr double _b_avx = -area_stop*_m_avx;                // and the b too
double gate_activation_x(double gateArea, int mult=1){
    gateArea *= 0.001;// discaledown
    return std::max(_m_avx*gateArea + _b_avx, 0.0)*mult;// ensure min vel is 0 not negative and set 0 if mult is 0
}

/*
approximate pixel on down cam to meter using some data (udah tak ambil)
*/
double get_max_width_meter(){
/* ŷ = 1.09881X - 0.00682 -> x is Z a.k.a height/altitude, y is approximation range of camera

   <drone>
   /    \
  /      \
 /        \
a--ground--b --> nah berapa jarak a-b di camera? this is what function do/try to approximate
*/
    return 1.09881*current_pose.position.z - 0.00682;
}

/*
then use that 'meter' to calculate velocity using PID in this function
*/
double calc_vel_based_vision_z(double vision_data, double cam_length, double max_meter,PID &thePID, char label, int mult=1){
    double cam_center_meter = max_meter * 0.5;                                      // center of cam is always half of the range
    double vision_data_meter = vision_data/cam_length * max_meter;                  // convert vision pixel to 'meter'
    double calc_vel = thePID.calculate(cam_center_meter, vision_data_meter)*mult;   // calc the vel using PID
    soero::Log::Info("[z: %f, cal_vel:%c]: cam center=%f | vis raw=%f, res: %f", current_pose.position.z,label, cam_center_meter, vision_data, calc_vel);
    return calc_vel;

}

// return velocity needed to go from theta a to theta b with PID
double velAlignYawToLine(double line_rad){
    return soero::CalcYawVelPID(line_rad, HALF_PI, mypid_yaw);
}

// calculate duration needed to pass vertical gate, gathered using some math+intuition and trial and error
double calculate_needed_duration(double area){
    double duration = area/46316.5 * -0.219504061 + 1.941916177;
    return std::max(0.5, duration);//minimum duration is 0.5
}


// a^2+b^2=c^2 D:, yes u got it
double calc_distance_to_wp(geometry_msgs::Pose &wp){
    double dx = wp.position.x - current_pose.position.x;
    double dy = wp.position.y - current_pose.position.y;
    double dz = wp.position.z - current_pose.position.z;

    return std::sqrt(dx*dx + dy*dy + dz*dz);
}


// ============================================================================ //
//                        VISION HANDLER                                        //
// =============================================================================//

/*
calculate velocity for ang.z, lin.x, lin.y based on line
*/
void handleLine(geometry_msgs::Twist &vel, vision::RotatedRect &line_guide, double mult_x=1.0){
    double line_rad = line_guide.angle*DEG_2_RAD;
    vel.angular.z = velAlignYawToLine(line_rad);
    soero::Log::Info("target is: %f, yet we are here: %f, so ang.z: %f", HALF_PI, line_rad, vel.angular.z);
    // maju mundur
    vel.linear.x = line_activation_x(vel.angular.z, mult_x) + 0.0;
    // kiri kanan
    vel.linear.y = calc_vel_based_vision_z(line_guide.x, 640.0, get_max_width_meter(),mypid_y, 'y');
    soero::Log::Info("y garis: %f, yet we are here: %f, so vel y: %f", 320.0, line_guide.x, vel.linear.y);
}

// Handle velocity calculation for landing pad
void handleLandingPad(geometry_msgs::Twist &vel, double hold_yaw, double hold_z, bool withTurun){
    double curYaw = soero::getYawFromQuaternion(current_pose.orientation);
    vel.angular.z = soero::CalcYawVelPID(hold_yaw, curYaw, mypid_yaw);             // hold our yaw while centering, so that the drone aint going crazy
    double max_meter = get_max_width_meter();
    vel.linear.y = calc_vel_based_vision_z(landingPad.x, 640.0, max_meter, mypid_y_hor_gate, 'y')*1.2;        
    vel.linear.x = calc_vel_based_vision_z(landingPad.y, 360.0, max_meter*0.5625, mypid_x_hor_gate, 'x')*1.5;//agak majuan sikit, biar masuk
    
    // if else if want centering sambil turun
    if(withTurun){vel.linear.z = -3;}// turun ae
    else{
        vel.linear.z = mypid_z.calculate(hold_z, current_pose.position.z);
        soero::Log::Info("landing, ga turun, tapi ke %fm ae dulu", hold_z);
    }
    
    soero::Log::Info("target is: %f, yet we are here: %f, so ang.z: %f", hold_yaw, curYaw, vel.angular.z);
    soero::Log::Info("y tengah: %f, yet we are here: %f, so vel y: %f", 320.0, landingPad.x, vel.linear.y);
}

// calc angular velocity to handle vertical gate's left and right side
void  handleVGateSide(geometry_msgs::Twist &vel){
    double max_side = std::max(verticalGateHelper.data[0], verticalGateHelper.data[1]); // get the largest side
    double diff = verticalGateHelper.data[0] - verticalGateHelper.data[1];              // calc the diff, not absolute, cause kiri kanan matter
    diff = diff/std::max(1.0, max_side);//mencegah div by zero, and normalize it
    vel.angular.z = mypid_yaw.calculate(diff, 0);                                       // pid stuff 
    soero::Log::Info("cntr-vgate: lside=%f, rside=%f, norm_diff=%f, vel_ang.z=%f", 
    verticalGateHelper.data[0], verticalGateHelper.data[1], diff, vel.angular.z);
}

void handleVerticalGate(geometry_msgs::Twist &vel){
    double gateArea = verticalGate.w*verticalGate.h;                    // calculate gate area for gate_activation_x to calc desired vel x
    double half_h = verticalGate.h*0.5;                                 // temp var for h/2
    double top_h=verticalGate.y-half_h, bottom_h=verticalGate.y+half_h; // calc where the top and bottom of the gate on frame
    // basically if gate cukup sejajar with camera and ada room di atas, and ada room dibawah, maju, if not diem
    int eligible = isClose(verticalGateHelper.data[0], verticalGateHelper.data[1], 50.0) && top_h > 25.0 && bottom_h < 335.0;
    vel.linear.x = gate_activation_x(gateArea, eligible);
    // centerin
    vel.linear.y = mypid_y_vision_gate.calculate(320.0, verticalGate.x);
    vel.linear.z = mypid_z_vision_gate.calculate(180.0, verticalGate.y);
    // fix gate side
    handleVGateSide(vel);
    soero::Log::Info("cntr-vgate: tgt_y=%f, cur_y=%f, vel_y=%f", 320.0, verticalGate.x, vel.linear.y);
    soero::Log::Info("cntr-vgate: tgt_x=%f, cur_x=%f, vel_x=%f", 180.0, verticalGate.y, vel.linear.x);
    soero::Log::Info("gArea: %f, with side diff: %f, | vel_x =%f", gateArea, (verticalGateHelper.data[0] - verticalGateHelper.data[1]), vel.linear.x);
}

void handleHorizontalGate(geometry_msgs::Twist &vel, double backup_yaw){
    // koreksi yawu, suprisingly not pakek hgate, cause, dari mana kita tau itu bener?, 4 side bruh
    // instead pakek garis
    // vel.angular.z = 0;

    if(horizontalGateHelper.is_exist){
        soero::Log::Info("Ada helper hgate");
        
        vel.angular.z = soero::CalcYawVelPID(backup_yaw, soero::getYawFromQuaternion(current_pose.orientation), mypid_yaw);
        soero::Log::Info("Ada helper hgate| angle:=%f, so vel ang.z: %f", horizontalGateHelper.angle, vel.angular.z);
    }
    else{
        soero::Log::Info("ga ada helper gate");
        vel.angular.z = soero::CalcYawVelPID(backup_yaw, soero::getYawFromQuaternion(current_pose.orientation), mypid_yaw);
    }
    // sebenernya ga pas cause gatenya ga dilantai, but we'll try anyway
    double max_meter = get_max_width_meter();
    // mungkin kebalik arahnya?, but weird, i mean frikin landing bisa... oh shite mate, cause landing is 0 yaw ah maku sense
    vel.linear.y = calc_vel_based_vision_z(horizontalGate.x, 640.0, max_meter,mypid_y_hor_gate, 'y');
    vel.linear.x = calc_vel_based_vision_z(horizontalGate.y, 360.0, max_meter*0.5625, mypid_x_hor_gate, 'x');
}


// maju lewatin gate for a certain duration
void GASLUR(geometry_msgs::Twist &vel, double turbo_speed, double duration, ros::Publisher &vel_pub,ros::Rate &rate){
    // nullified all unecessary vel 
    zerofied_main_vel(vel);
    vel.linear.x = turbo_speed;
    ros::Time start_time = ros::Time::now();
    // mini loop for n duration we publish majuu 
    while(ros::ok() && ros::Time::now() - start_time < ros::Duration(duration)){
        soero::Log::Info("GAS LUR!!!!!");
        vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    } 
}

// turun lewat gate for a certain duration
void TURUNLUR(geometry_msgs::Twist &vel, geometry_msgs::Pose &current_pose, double duration, double lock_yaw,ros::Publisher &vel_pub,ros::Rate &rate){
    zerofied_main_vel(vel);// gausah sambil cenering ae turun --> nope ga bisa, nanti datanya ga ada
    vel.linear.z = mypid_z_hrz.calculate(0.9, current_pose.position.z);

    ros::Time start_time = ros::Time::now();
    while(ros::ok() && current_pose.position.z > 1.0){
        vel.angular.z = soero::CalcYawVelPID(lock_yaw, soero::getYawFromQuaternion(current_pose.orientation), mypid_yaw);
        vel.linear.z = mypid_z_hrz.calculate(0.9, current_pose.position.z);
        soero::Log::Info("TURUN LUR!!!!! %f", current_pose.position.z);
        vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    } 
}

// calc vel x,y,z NOT ang.z to go from point A from point B, this need a lil rotation cause drone vel ngikut heading drone, and the position are absolute 
void go_to_wp(geometry_msgs::Twist &vel, geometry_msgs::Pose &wp){
    double dx = wp.position.x - current_pose.position.x;
    double dy = wp.position.y - current_pose.position.y;
    double true_theta = std::atan2(dy, dx);
    double true_distance = std::sqrt(dx*dx + dy*dy);

    // rotate it to drone's theta (many trial and error)
    double dtheta = true_theta - soero::getYawFromQuaternion(current_pose.orientation);
    double vel_x = mypid_x.calculate(true_distance, 0);
    vel.linear.x = vel_x*std::cos(dtheta);
    vel.linear.y = vel_x*std::sin(dtheta);
    vel.linear.z = mypid_z.calculate(wp.position.z, current_pose.position.z);
    soero::Log::Info("output: %fx | %fy | %fz", vel.linear.x, vel.linear.y, vel.linear.z);
}

//=================================================MAIN-MAIN-MAIN===================================================================//
int main(int argc, char **argv)
{
    // cek banyak argument dulu bosku
    if(argc < 3){
        soero::Log::Error("Please input 3 parameter ;)");
        exit(0);
    }
    std::string temp = argv[2];
    int TOTAL_HGATE = convert_string_to_integer(temp);
    if(TOTAL_HGATE+3 != argc){
        soero::Log::Error("Input total ;) didnt match for all");
        exit(0);
    }
    // save all id of hgate inputted
    std::vector<int> list_hgate;
    int hgate_iter = 0;
    for(int i = 0;i<TOTAL_HGATE;i++){
        temp = argv[i+3];
        int tmp = convert_string_to_integer(temp);
        list_hgate.push_back(tmp);
    }
    soero::Log::Info("Received: %d argument!", argc);

    // total gate (vgate & hgate)
    std::string temp2 = argv[1];
    int TOTAL_GATE = convert_string_to_integer(temp2);

    // confirm flight
    soero::Log::Info("You sure want to fly with currrent setting?");
    std::string input_str;
    std::cin >> input_str;
    if(input_str!="Y" && input_str!="y"){
        soero::Log::Info("Canceling Flight");
        return 0;
    }

    ros::init(argc, argv, "bmctrl");
    ros::NodeHandle nh;
    soero::Log::EnableFileOutput();

    //takeoff api
    ros::Publisher takeoff_pub      = nh.advertise<std_msgs::Empty>      ("/drone/takeoff", 10, true);
    //movement api
    ros::Publisher vel_pub          = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
    ros::Subscriber pose_sub        = nh.subscribe<geometry_msgs::Pose>  ("/drone/gt_pose", 10, pose_cb);
    ros::Publisher land_pub         = nh.advertise<std_msgs::Empty>      ("/drone/land", 10);
    // vision datas
    ros::Subscriber landingpad_sub  = nh.subscribe<vision::RotatedRect>  ("/landingpad_data", 10, lpad_cb);
    ros::Subscriber upper_line_sub  = nh.subscribe<vision::RotatedRect>  ("/upper/line_data", 10, uline_cb);
    ros::Subscriber lower_line_sub  = nh.subscribe<vision::RotatedRect>  ("/lower/line_data", 10, lline_cb);
    ros::Subscriber vgate_sub       = nh.subscribe<vision::RotatedRect>  ("/gate/vertical/data", 10, vgate_cb);
    ros::Subscriber vgate_help1_sub = nh.subscribe<std_msgs::Float32MultiArray>  ("/gate/vertical/help/1", 10, vgate_help1_cb);
    // help2 ga kepake 
    ros::Subscriber vgate_help2_sub = nh.subscribe<std_msgs::UInt8>  ("/gate/vertical/help/2", 10, vgate_help2_cb);
    ros::Subscriber hgate_sub       = nh.subscribe<vision::RotatedRect>  ("/gate/horizontal/data", 10, hgate_cb);
    ros::Subscriber hgate_help1_sub = nh.subscribe<vision::RotatedRect>  ("/gate/horizontal/help", 10, hgate_help_cb);

    ros::Rate rate(1/dt);

    std_msgs::Empty empty;
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
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
    
    // Change drone mode to takeoff
    soero::Log::Info("Set Drone to takeoff");
    takeoff_pub.publish(empty);
    soero::Log::Info("Done set to takeoff");

    bool early_off = true;
    double hold_z = 1.35;//-> 0.5-1.5m kaki, 0.7 gate, so (0.5+1.5)/2 + 0.7/2
    double hold_yaw = -1.57079632679;
    geometry_msgs::Pose lastGatePos;
    ros::Time existGuideTime = ros::Time::now();
    ros::Time start_flight_time = ros::Time::now();     // self itung berapa lama terbang (not exact but close enough)
    
    // LOOP sampek visual guide ada cukup lama atau udah >4 detik
    while(ros::ok() && early_off){
        vel.linear.z = mypid_z.calculate(hold_z, current_pose.position.z);
        vel.linear.x = 0.25;
        vel.angular.z = soero::CalcYawVelPID(hold_yaw, soero::getYawFromQuaternion(current_pose.orientation), mypid_yaw);
        if(landingPad.is_exist){
            // udah agak naik,
            vel.linear.x = 0.4;
            soero::Log::Info("yo moite, ada pad, berarti dah cukup atas %f", current_pose.position.z);
        }
        if(upperLine.is_exist){
            soero::Log::Info("awal ada garis depan");
            handleLine(vel, upperLine);
            if(ros::Time::now()-existGuideTime > ros::Duration(1.0)){
                early_off=true;
                soero::Log::Info("ok, versi guide dah ada lama, keluar");
            }
        }else if(lowerLine.is_exist){
            handleLine(vel, upperLine);
            if(ros::Time::now()-existGuideTime > ros::Duration(1.0)){
                early_off=true;
                soero::Log::Info("ok, versi guide dah ada lama, keluar");
            }
        }
        else{
            existGuideTime= ros::Time::now();
        }
        vel_pub.publish(vel);

        if(ros::Time::now() - start_flight_time > ros::Duration(4.0)){
            early_off = false;
            soero::Log::Info("udah 4 detik maju awal");
        }
        // initialize lastGatePos here
        lastGatePos=current_pose;
        soero::Log::Info("maju and takeoff dur: %f| pr:%f | now: %f", (ros::Time::now() - start_flight_time).toSec(), start_flight_time.toSec(), ros::Time::now().toSec());
        ros::spinOnce();
        rate.sleep();
    }

    /*
    Movement priority from highest to lowest:
    Landing Pad -> Gate Horizontal -> Gate Vertical -> Line
    */

    // MAIN LOOP
    bool isDoingMission=true;
    double curYaw = 0;
    double isLanded = false;
    int gate_passed = 0;
    double threshold_vgate = 20.0;
    double threshold_vgateSide =10.0;
    bool doHold_Z = true;
    bool doUpdate_yaw = true;
    double lastHgateZ = 5.0;
    bool isLookingForHgate = false;
    int gatePhase = 0;
    double line_vel_x_multiplier[2] = {1.0, 0.75};// 0->for not islookingforhgate 1-> is for islookingforhgate

    ros::Time firstPassedLastGate;
    ros::Time firstAppereanceGate = ros::Time::now();
    while(ros::ok() && isDoingMission){
        // RESET ALL NECESSARY VALUE
        zerofied_main_vel(vel);
        doHold_Z = true;
        doUpdate_yaw = true;
        isLookingForHgate = false;

        // for time helper (hold_z to 2 when drone is going to landing)
        if(gate_passed == TOTAL_GATE - 1) firstPassedLastGate = ros::Time::now();
        
        // IS IT TIME TO CHECK HORIZONTAL GATE?
        if(gate_passed == list_hgate[hgate_iter]){
            hold_z = std::min(4.8, lastHgateZ);
            soero::Log::Info("time to detect hgate, hold to %fm", hold_z);
            isLookingForHgate=true;
        } else if(gate_passed == TOTAL_GATE){
            // if the last gate is horizontal gate
            if(list_hgate.back() == (TOTAL_GATE -1)){
                // we have to add 5 secs delay until the drone hold_z to 2
                if(ros::Time::now()-firstPassedLastGate > ros::Duration(5)){
                    hold_z = 2.0;
                    soero::Log::Info("time to detect landing pad, hold to %fm", hold_z);
                }
            }
            // if the last gate is vertical gate
            else{
                hold_z = 2.0;
                soero::Log::Info("time to detect landing pad, hold to %fm", hold_z);
            }
        } else{
            hold_z=1.0;
        }

        // Cek ada landing pad jika ga nyari hgate or udah at least 1 gate kelewat (nyegah false positive)
        if(!isLookingForHgate && gate_passed > 1 && landingPad.is_exist ){
            soero::Log::Info("ada pad PAD");
            doHold_Z=false;
            doUpdate_yaw = false;
            handleLandingPad(vel, curYaw, 2.0, false);
            // if(isCenteredStatic(landingPad.x, landingPad.y, 320.0, 180.0, current_pose.position.z*32.0)){
            //     soero::Log::Info("DEKET LANDING PAD CUY, TURUN SKUY");
            //     isDoingMission = false;
            // } 
            isDoingMission = false;
        }
        // Cek horizontal gate                                  // check for false positive
        else if(isLookingForHgate && horizontalGate.is_exist && isClose(horizontalGate.w/std::max(horizontalGate.h, 1.0F), 1.0, 0.5)){
            doUpdate_yaw = false;
            lastGatePos = current_pose; // update gate pos
            // doHold_Z = false;
            soero::Log::Info("ada horizontal gate cuy, passed: %d", gate_passed);
            handleHorizontalGate(vel, curYaw);
            // cek udah centered or no
            if(isCenteredRatio(320.0, 180.0,  horizontalGate, final_ratio_x, final_ratio_y)){
                double area = horizontalGate.h * horizontalGate.w;
                soero::Log::Info("ok, centerred, area: %f,sambil turun, but let see the line, with w,h: %f|%f", area, horizontalGate.w, horizontalGate.h);
                vel.linear.z = -1;
                doHold_Z=false;
                lastHgateZ = current_pose.position.z;
                // harusnya cek dulu dia ada apa engga, but di vision udah tak defaultin 90, so aman
                // if(isClose(horizontalGateHelper.angle, 90, 2.0)){
                if(area > 25000.0){
                    TURUNLUR(vel, current_pose, 2.0, soero::getYawFromQuaternion(current_pose.orientation),vel_pub, rate);
                    // vel.linear.z
                    soero::Log::Info("dah turun harusnya");
                    gate_passed++;
                    hold_z = 1.0;
                    hgate_iter++;
                    hgate_iter%=TOTAL_HGATE;// so that aint overflow
                    lastHgateZ = 5.0;
                    lastGatePos = current_pose; // update gate pos after lewat
                }
            }
        }
        // cek ada vertical gate ga? dan hanya jika ga lagi nyari hgate
        else if(!isLookingForHgate && verticalGate.is_exist){
            // dont hold z for this
            doHold_Z = false;
            // method / procedure for if the gate ilang but belum ngelewatin
            switch (gatePhase)
            {
            case 0:
                firstAppereanceGate = ros::Time::now();
                lastGatePos = current_pose;
                gatePhase+=1;
                soero::Log::Info("ok, wah ada gate, start timer");
                break;
            case 1:
                // ok timer start but we aint sure yet, lets wait till 1sec
                if(ros::Time::now() - firstAppereanceGate > ros::Duration(1.0)){
                    // ok we sure
                    soero::Log::Info("ok, udah 1 detikan muncul, yakin sih");
                    gatePhase+=1;
                    // only save the pos in this and after lewat, cause if tiap muncul, pas ilang balik ke terakhir, ga beda jauh
                    lastGatePos = current_pose;
                }
            
            default:
                break;
            }
            soero::Log::Info("ada VERTICAL GATE, passed: %d", gate_passed);
            handleVerticalGate(vel); 
            if(isCenteredRatio(320.0, 180.0, verticalGate) && isClose(verticalGateHelper.data[0], verticalGateHelper.data[1], threshold_vgateSide)){
                soero::Log::Info("udah deket centered with gate");
                GASLUR(vel, 1.1, calculate_needed_duration(verticalGate.w*verticalGate.h), vel_pub, rate);
                soero::Log::Info("harusnya this gate is passed");
                gate_passed++;
                hold_z = 1.0;
                gatePhase = 0;
                lastGatePos = current_pose; // update lastGatePos after lewat
                // hold_z = 1.35;
                
            }

        }
        // Before we choose line as a guide, check if we are in the phase 2 of gate
        else if(gatePhase==2){
            // huh doing gate, but ga ada
            soero::Log::Info("lololo, kok ilang gatenya: %d passed, sek, ke last pos", gate_passed);
            go_to_wp(vel, lastGatePos);
            hold_z = lastGatePos.position.z;
            //gatePhase=0;// reset, biar gak keterusan --> ngetest tanpa reset --> ok ternyata perlu direset--> no! dont reset it!
            // ok paling bener reset kalo dah deket
            if(calc_distance_to_wp(lastGatePos) < 0.25){
                gatePhase=0;//reset dah deket, lets hope ada gate (or ada gate before this is even better)
            }
        }
        // cek if ada garis atas
        else if(upperLine.is_exist){
            soero::Log::Info("ada garis atas");
            handleLine(vel, upperLine, line_vel_x_multiplier[isLookingForHgate]);
        }
        // cek ada garis bawah
        else if(lowerLine.is_exist){
            soero::Log::Info("ada garis bawah");
            handleLine(vel, lowerLine, line_vel_x_multiplier[isLookingForHgate]);
        }
        // hm, after all the visual guide, we found nothing, good thing we have memory
        else{
            soero::Log::Info("ga ada guide, lemme go to last gate pos...");// in hope something a bit slightly ke offset and now bisa dapet visual guide
            go_to_wp(vel, lastGatePos);
            if(!isLookingForHgate)
                hold_z = lastGatePos.position.z;
        }
        // soero::Log::Info("ngikutin garis");
        if(doHold_Z){
            soero::Log::Info("holding z to: %f from: %f", hold_z, current_pose.position.z);
            vel.linear.z = mypid_z.calculate(hold_z, current_pose.position.z);
        }
        if(doUpdate_yaw){
            curYaw = soero::getYawFromQuaternion(current_pose.orientation);
            soero::Log::Debug("update curyaw now: %f", curYaw);
        }
        vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    // loop landing
    curYaw = soero::getYawFromQuaternion(current_pose.orientation);
    bool notLanded = true;
    while(ros::ok() && notLanded){
        if(landingPad.is_exist){
            soero::Log::Info("ada pad: z=%f", current_pose.position.z);
            handleLandingPad(vel, curYaw, 0,true);
        }
        else{
            // uh turun ae
            vel.linear.z = 0.15;
            soero::Log::Info("hm, gak keliatan landingnya, naik ae biar muncul: %f", current_pose.position.z);
        }
        
        if(current_pose.position.z <= 1){
            notLanded = false;
            land_pub.publish(empty);
            soero::Log::Info("publish landing, udah 20 centian");
        }

        vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }
    soero::Log::Info("Misi selesai");
    soero::Log::Info("Total waktu: %f", (ros::Time::now() - start_flight_time).toSec());
    soero::Log::CloseFileOutput();
    return 0;
}