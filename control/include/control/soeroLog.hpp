#pragma once
#include <ros/ros.h>
#include <ctime>
#include <pthread.h>
#include <vector>

namespace soero{
/*
    Verbosity Levels (from ros logging)
DEBUG
    Information that you never need to see if the system is working properly. Examples:
        "Received a message on topic X from caller Y"
        "Sent 20 bytes on socket 9". 
INFO
    Small amounts of information that may be useful to a user. Examples:
        "Node initialized"
        "Advertised on topic X with message type Y"
        "New subscriber to topic X: Y" 
WARN
    Information that the user may find alarming, and may affect the output of the application, but is part of the expected working of the system. Examples:

        "Could not load configuration file from <path>. Using defaults." 
ERROR
    Something serious (but recoverable) has gone wrong. Examples:
        "Haven't received an update on topic X for 10 seconds. Stopping robot until X continues broadcasting."
        "Received unexpected NaN value in transform X. Skipping..." 
FATAL
    Something unrecoverable has happened. Examples:
        "Motors have caught fire!" 
*/
enum LogPriority{
    DebugPriority, InfoPriority, WarnPriority, ErrorPriority, FatalPriority
};

// Custom log built on top of ros log (simplified with custom logfile)
class Log{

private:
    static LogPriority priority;
    static const char* filepath;
    static FILE* file;
    static pthread_mutex_t log_mutex;
public:
    // Set the logger priority to filter which level should we save/show
    static void SetPriority(LogPriority newPriority){
        priority = newPriority;
    }
    /* call this function to enable saving the log to a default file
    Don't forget to call CloseFileOutput to close the file!
    */
    static void EnableFileOutput(){
        std::string tmp = "log_" + get_date_time("%F_%T")+ ".txt";
        filepath = tmp.c_str();
        enable_file_output();
    }
    // call this function to save the log to specified path
    static void EnableFileOutput(const char* new_filepath){
        filepath = new_filepath;
        enable_file_output();
    }
    // close the opened log file
    static void CloseFileOutput(){
        free_file();
    }

    template<typename... Args>
    static void Debug(const char * msg, Args... args){
        if(priority <=  DebugPriority){
            ROS_DEBUG(msg, args...);
            filelog("[Debug]:\t", msg, args...);
        }
    }

    template<typename... Args>
    static void Info(const char * msg, Args... args){
        if(priority <=  InfoPriority){
            ROS_INFO(msg, args...);
            filelog("[Info]:\t", msg, args...);
        }
    }

    template<typename... Args>
    static void Warn(const char * msg, Args... args){
        if(priority <=  WarnPriority){
            ROS_WARN(msg, args...);
            filelog("[Warn]:\t", msg, args...);
        }
    }

    template<typename... Args>
    static void Error(const char * msg, Args... args){
        if(priority <=  ErrorPriority){
            ROS_ERROR(msg, args...);
            filelog("[Error]:\t", msg, args...);
        }
    }

    template<typename... Args>
    static void Fatal(const char * msg, Args... args){
        if(priority <=  FatalPriority){
            ROS_FATAL(msg, args...);
            filelog("[Fatal]:\t", msg, args...);
        }
    }

private:
    template<typename... Args>
    // save message log to a file
    static void filelog(const char * priority_str, const char * msg, Args...args){
        if(file){
            std::string datetime = get_date_time("%T");
            std::vector<char> timestamp(datetime.begin(), datetime.end());
            timestamp.push_back('\0');
            pthread_mutex_lock(&log_mutex);
            fprintf(file, "%s\t", &timestamp[0]);
            fprintf(file, priority_str);
            fprintf(file, msg, args...);
            fprintf(file, "\n");
            pthread_mutex_unlock(&log_mutex);
        }
    }

    // return a string of datetime formatted in YYYY-MM-DD_hh:mm:ss
    static std::string get_date_time(const char *format){
        std::time_t current_time = std::time(0);
        std::tm *timestamp = std::localtime(&current_time);
        char buffer[80];
        strftime(buffer, 80, format, timestamp);
        std::string datetime(buffer);
        return datetime;
    };

    static void enable_file_output(){
        if(file!=0){
            //close the previous openned file
            fclose(file);
        }
        
        // we are just gonna append to it, so that the previous data (if exist) is still there
        file = fopen(filepath, "a");

        if(file==0){
            printf("Log: Failed to open file at: %s\n", filepath);
        }
    }

    static void free_file(){
        fclose(file);
        file=0;
    }

};
// initialize class private static artributes
LogPriority Log::priority = InfoPriority;
const char* Log::filepath = 0;
FILE* Log::file = 0;
pthread_mutex_t Log::log_mutex;
}