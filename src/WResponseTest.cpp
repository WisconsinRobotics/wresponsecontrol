#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>

double delta = 0;
void deltaCallback(const std_msgs::Float64::ConstPtr& msg){
    delta = msg->data;
    ROS_INFO("PID_OUTPUT: %0.6f", delta);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "WResponseTest");
    ros::NodeHandle n;
    ros::Publisher setpointPub = n.advertise<std_msgs::Float64>("/testSetpointTopic0", 1000);
    ros::Publisher feedbackPub = n.advertise<std_msgs::Float64>("/testFeedbackTopic0", 1000);
    ros::Subscriber deltaSub = n.subscribe("/testOutputTopic0", 1000, &deltaCallback);

    int hz = 100;
    ros::Rate loop(hz);
    bool setHigh = false;
    int setCounter = 0;
    double currOutput = 0;
    while(ros::ok()){
        if(setCounter >= hz/2){
            setCounter = 0;
            setHigh = !setHigh;
        }else setCounter++;

        std_msgs::Float64 setpoint;
        setpoint.data = setHigh ? 20 : 0;
        setpointPub.publish(setpoint);

        currOutput+=delta;
        ROS_INFO("CURR_OUTPUT: %0.6f", currOutput);
        currOutput = currOutput*0.85 - 2;
        ROS_INFO("CURR_OUTPUT_W_FORCE: %0.6f", currOutput);

        std_msgs::Float64 feedback;
        feedback.data = currOutput;
        feedbackPub.publish(feedback);

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}