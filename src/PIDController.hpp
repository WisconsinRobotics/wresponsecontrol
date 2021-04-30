#ifndef PIDControllerH
#define PIDControllerH

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>

#define MsgPtr std_msgs::Float64::ConstPtr& 

class PIDController{

    private:

        double P;
        double I;
        double D;
        double err;
        double lastErr;
        double sumErr;
        double setpoint;
        double feedback;
        double lastOutput;
        ros::Subscriber setPointReader;
        ros::Subscriber feedbackReader;
        ros::Publisher outputController;
        void setPointCallback(const MsgPtr msg);
        void feedbackCallback(const MsgPtr msg);
        double computeNextOutput();
        void computeAndSendNextOutput();

    public:
        PIDController(std::string setPointTopic, std::string feedbackTopic, std::string outputTopic, ros::NodeHandle& node);
        void setP(double P);
        double getP();
        void setI(double I);
        double getI();
        void setD(double D);
        double getD();
        void setPID(double P, double I, double D);
        double getError();
        void executeNextControlCycle();

};

#endif