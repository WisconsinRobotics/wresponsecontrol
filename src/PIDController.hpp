#ifndef PIDControllerH
#define PIDControllerH

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <chrono>

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
        double outputCap[2];
        bool outputCapSet[2];
        double iCap;
        bool iCapSet;
        short initState;
        
        ros::Subscriber setPointReader;
        ros::Subscriber feedbackReader;
        ros::Publisher outputController;

        std::chrono::milliseconds lastCalculationTime;

        void setPointCallback(const MsgPtr msg);
        void feedbackCallback(const MsgPtr msg);
        void updateLastCalculationTime();
        static std::chrono::milliseconds getCurrentTime();
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
        void setMaxOutput(double max);
        double getMaxOutput();
        void setMinOutput(double min);
        double getMinOutput();
        void setMinMaxOutput(double min, double max);
        bool getMaxOutputSet();
        bool getMinOutputSet();
        void setICap(double iCap);
        double getICap();
        bool getICapSet();
        void executeNextControlCycle();

};

#endif