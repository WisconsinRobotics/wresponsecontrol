#include "PIDController.hpp"

template<typename InputType, typename FeedbackType, typename OutputType>
PIDController<InputType, FeedbackType, OutputType>::PIDController(std::string setPointTopic, std::string feedbackTopic, std::string outputTopic, ros::NodeHandle& node){
    this->setPointReader = node.subscribe(setPointTopic, PIDController::setPointCallback, this);
    this->feedbackReader = node.subscribe(feedbackTopic, PIDController::feedbackCallback, this);
    this->outputController = node.advertise<OutputType>(outputTopic, 1000);

    this->P = 0;
    this->I = 0;
    this->D = 0;
    this->err = 0;
    this->lastErr = 0;
    this->inputConversion = NULL;
    this->feedbackConversion = NULL;
    this->outputConversion = NULL;
}