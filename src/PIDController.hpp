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
        double err;
        double lastErr;
        InputType setpoint;
        FeedbackType lastFeedback;
        OutputType lastOutput;
        ros::Subscriber* setPointReader;
        ros::Subscriber* feedbackReader;
        ros::Publisher* outputController;
        std::function<InputType(FeedbackType)>* feedbackConversion;
        void setPointCallback(InputType::ConstPtr&);
        void feedbackCallback(FeedbackType::ConstPtr&);
        void computeNextOutput();
        void computeAndSendNextOutput();

    public:
        PIDController(std::string setPointTopic, std::string feedbackTopic, std::string outputTopic);
        void setP(double P);
        double getP();
        void setI(double I);
        double getI();
        void setD(double D);
        double getD();
        void setPID(double P, double I, double D);
        double[] getPID();
        double getError();
        OutputType getLastOutput();

        void setFeedbackConversion(std::function<InputType(FeedbackType)>& func);


};

#endif