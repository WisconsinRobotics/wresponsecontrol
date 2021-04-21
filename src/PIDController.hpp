#ifndef PIDControllerH
#define PIDControllerH

#include <ros/ros.h>
#include <iostream>

template <typename InputType, typename FeedbackType, typename OutputType>
class PIDController{
    private:
        double P;
        double I;
        double D;
        double pCurr;
        double iCurr;
        double dCurr;
        InputType setpoint;
        FeedbackType lastFeedback;
        OutputType lastOutput;
        ros::Subscriber* setPointReader;
        ros::Subscriber* feedbackReader;
        ros::Publisher* outputController;
        void setPointCallback(InputType::ConstPtr&);
        void feedbackCallback(FeedbackType::ConstPtr&);

    public:
        PIDController(std::string setPointTopic, std::string feedbackTopic, std::string outputTopic);
        void setP(double P);
        double getP();
        void setI(double I);
        double getI();
        void setD(double D);
        double getD();
        // TODO: figure out method signature for passing in a function to convert from 
        // template <typename func>
        // void set


};

#endif