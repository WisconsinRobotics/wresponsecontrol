#include "PIDController.hpp"

PIDController::PIDController(std::string setPointTopic, std::string feedbackTopic, std::string outputTopic, ros::NodeHandle& node){
    this->setPointReader = &node.subscribe(setPointTopic, 1000, PIDController::setPointCallback, this);
    this->feedbackReader = &node.subscribe(feedbackTopic, 1000, PIDController::feedbackCallback, this);
    this->outputController = &node.advertise<std_msgs::Float64>(outputTopic, 1000);

    this->P = 0;
    this->I = 0;
    this->D = 0;
    this->err = 0;
    this->lastErr = 0;
}

void PIDController::setPointCallback(MsgPtr msg){
    this->setpoint = msg->data;
}

void PIDController::feedbackCallback(MsgPtr msg){
    this->feedback = msg->data;
}

double PIDController::computeNextOutput(){
    this->lastErr = this->err;
    this->err = this->setpoint - this->feedback;
    this->sumErr += this->err;
    return this->lastOutput = this->getP()*this->err + this->getI()*this->sumErr + this->getD()*(this->err - this->lastErr);
}

void PIDController::computeAndSendNextOutput(){
    std_msgs::Float64 msgNext;
    msgNext.data = this->computeNextOutput();
    this->outputController->publish(msgNext);
}

void PIDController::executeNextControlCycle(){
    this->computeAndSendNextOutput();
}

void PIDController::setP(double P){
    this->P = P;
}

double PIDController::getP(){
    return this->P;
}

void PIDController::setI(double I){
    this->I = I;
}

double PIDController::getI(){
    return this->I;
}

void PIDController::setD(double D){
    this->D = D;
}

double PIDController::getD(){
    return this->D;
}

void PIDController::setPID(double P, double I, double D){
    this->setP(P);
    this->setI(I);
    this->setD(D);
}