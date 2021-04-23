#ifndef PIDControllerH
#define PIDControllerH

#include <ros/ros.h>
#include <iostream>

template <typename InputType, typename FeedbackType, typename OutputType>
class PIDController{
    static_assert(ros::message_traits::IsMessage<InputType>, "InputType must derive from Base");
    static_assert(ros::message_traits::IsMessage<FeedbackType>, "FeedbackType must derive from Base");
    static_assert(ros::message_traits::IsMessage<OutputType>, "OutputType must derive from Base");

    private:

        typedef InputType::ConstPtr& inpMsg;
        typedef FeedbackType::ConstPtr& fdbMsg;

        double P;
        double I;
        double D;
        double err;
        double lastErr;
        inpMsg setpoint;
        fdbMsg feedback;
        OutputType lastOutput;
        ros::Subscriber* setPointReader;
        ros::Subscriber* feedbackReader;
        ros::Publisher* outputController;
        std::function<double(InputType)>* inputConversion;
        std::function<double(FeedbackType)>* feedbackConversion;
        std::function<OutputType(double)>* outputConversion;
        void setPointCallback(inpMsg msg);
        void feedbackCallback(fdbMsg msg);
        void computeNextOutput();
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
        double[] getPID();
        double getError();
        OutputType getLastOutput();

        void setInputConversion(std::function<double(inpMsg)>& func);
        void setFeedbackConversion(std::function<double(fdbMsg)>& func);
        void setOutputConversion(std::function<OutputType(double)& func);


};

#endif