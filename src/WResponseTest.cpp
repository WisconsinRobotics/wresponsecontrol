#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <fstream>
#include <csignal>
#include <chrono>

double delta = 0;
void deltaCallback(const std_msgs::Float64::ConstPtr& msg){
    delta = msg->data;
    // ROS_INFO("PID_OUTPUT: %0.6f", delta);
}

std::fstream logFile;
void closeFileHandleOnExit(int signal){
    logFile.close();
    exit(1);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "WResponseTest");
    ros::NodeHandle n;
    ros::Publisher setpointPub = n.advertise<std_msgs::Float64>("/testSetpointTopic0", 1000);
    ros::Publisher feedbackPub = n.advertise<std_msgs::Float64>("/testFeedbackTopic0", 1000);
    ros::Subscriber deltaSub = n.subscribe("/testOutputTopic0", 1000, &deltaCallback);

    // For whatever reason, this goes to ~/.ros/log.csv
    logFile.open("log.csv", std::ios::out | std::ios::trunc);
    ROS_INFO("FILE_OPEN: %s", logFile.is_open() ? "true" : "false");
    ROS_INFO("FILE_OPEN2: %s", logFile ? "true" : "false");
    if(logFile.is_open()) logFile << "setpoint,feedback,output" << std::endl;
    signal(SIGINT, &closeFileHandleOnExit);

    int hz = 50;
    ros::Rate loop(hz);
    bool setHigh = false;
    int setCounter = 0;
    double currOutput = 0;
    double lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    while(ros::ok()){
        if(setCounter >= 100){
            setCounter = 0;
            setHigh = !setHigh;
        }else setCounter++;

        std_msgs::Float64 setpoint;
        setpoint.data = setHigh ? 20 : 0;
        setpointPub.publish(setpoint);

        currOutput+=delta*(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()-lastTime)/1000.f;
        // ROS_INFO("CURR_OUTPUT: %0.6f", currOutput);
        currOutput -= (currOutput*0.15 + 2)*(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()-lastTime)/1000.f;
        lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // ROS_INFO("CURR_OUTPUT_W_FORCE: %0.6f", currOutput);

        std_msgs::Float64 feedback;
        feedback.data = currOutput;
        feedbackPub.publish(feedback);

        if(logFile.is_open()){
            std::string log;
            char buffer[300];
            sprintf(buffer, "%0.6f,%0.6f,%0.6f", setpoint.data, feedback.data, delta);
            log = buffer;
            logFile << log << std::endl;
        }

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}