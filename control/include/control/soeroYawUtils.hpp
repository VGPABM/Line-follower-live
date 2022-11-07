#pragma once

#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include "pid.hpp"

#define S_TWO_PI 6.283185307179586
#define S_THREEHALF_PI 4.71238898038

namespace soero{
    double getYawFromQuaternion(double w, double x, double y, double z){
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp); 
        return yaw;
    }

    double getYawFromQuaternion(geometry_msgs::Quaternion &gq){
        double yaw = getYawFromQuaternion(gq.w, gq.x, gq.y, gq.z);
        return yaw;
    }

    //Convert radian into 0 - 2pi range
    double NormalizeRad(double rad){
        while(rad >= S_TWO_PI){
            rad -= S_TWO_PI;
        }
        while(rad < 0){
            rad += S_TWO_PI;
        }
        return rad;
    }
    //Calculate angular z velocity from current theta to target theta using input PID
    double CalcYawVelPID(double targetTheta, double currentTheta, PID &PID_YAW){
        targetTheta = NormalizeRad(targetTheta);
        currentTheta = NormalizeRad(currentTheta);

        double shortestDiff;
        double diff1 = targetTheta - currentTheta;
        double diff2 = diff1 - S_TWO_PI;
        double diff3 = diff1 + S_TWO_PI;



        if(std::abs(diff1) <= std::abs(diff2)){
            if(std::abs(diff1) <= std::abs(diff3)){
                shortestDiff = diff1;
            }else{
                shortestDiff = diff3;
            }
        }else{
            if(std::abs(diff2) <= std::abs(diff3)){
                shortestDiff = diff2;
            }else{
                shortestDiff = diff3;
            }
        }
        double vel = PID_YAW.calculate(shortestDiff, 0);
        return vel; 
    }

    double calcYawVelCompass(double targetTheta, double currentTheta, PID &PID_YAW){
        // targetTheta = NormalizeRad(targetTheta);
        // currentTheta = NormalizeRad(currentTheta);

        // double shortestDiff;
        // double diff1 = targetTheta - currentTheta;
        // double diff2 = diff1 - S_TWO_PI;
        // double diff3 = diff1 + S_TWO_PI;

        // double raw_diff = first > second ?

        // if(std::abs(diff1) <= std::abs(diff2)){
        //     if(std::abs(diff1) <= std::abs(diff3)){
        //         shortestDiff = diff1;
        //     }else{
        //         shortestDiff = diff3;
        //     }
        // }else{
        //     if(std::abs(diff2) <= std::abs(diff3)){
        //         shortestDiff = diff2;
        //     }else{
        //         shortestDiff = diff3;
        //     }
        // }
        // double vel = PID_YAW.calculate(shortestDiff, 0);
        // return vel;

        double signedDiff = 0;
        double raw_diff = currentTheta > targetTheta ? currentTheta - targetTheta : targetTheta - currentTheta;
        double mod_diff = std::fmod(raw_diff, S_TWO_PI);

        if(mod_diff > M_PI){
            signedDiff = S_TWO_PI - mod_diff;
            if(targetTheta > currentTheta){
                signedDiff *= -1;
            }
        }
        else{
            signedDiff = mod_diff;
            if(currentTheta > targetTheta){
                signedDiff *= -1;
            }
        }

        double vel = -PID_YAW.calculate(signedDiff, 0);
        return vel;
    }

}