#include "PIDController.hpp"

void PIDController::setPointCallback(const MsgPtr msg){
    this->setpoint = msg->data;
    this->initState |= 1;
}

void PIDController::feedbackCallback(const MsgPtr msg){
    this->feedback = msg->data;
    this->initState |= 2;
}

std::chrono::milliseconds PIDController::getCurrentTime(){
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
}

void PIDController::updateLastCalculationTime(){
    this->lastCalculationTime = PIDController::getCurrentTime();
}

PIDController::PIDController(std::string setPointTopic, std::string feedbackTopic, std::string outputTopic, ros::NodeHandle& node){
    this->setPointReader = node.subscribe(setPointTopic, 1000, &PIDController::setPointCallback, this);
    this->feedbackReader = node.subscribe(feedbackTopic, 1000, &PIDController::feedbackCallback, this);
    this->outputController = node.advertise<std_msgs::Float64>(outputTopic, 1000);

    this->P = 0;
    this->I = 0;
    this->D = 0;
    this->err = 0;
    this->lastErr = 0;
    this->initState = 0;
    this->outputCapSet[0] = false;
    this->outputCapSet[1] = false;
    this->updateLastCalculationTime();
}

double PIDController::computeNextOutput(){
    this->lastErr = this->err;
    this->err = this->setpoint - this->feedback;
    this->sumErr += this->err;
    double timeElapsed = (PIDController::getCurrentTime().count() - this->lastCalculationTime.count())/1000.f;
    // ROS_INFO("TIME ELAPSED: %0.6f", timeElapsed*1000);
    this->lastOutput = this->getP()*this->err + this->getI()*this->sumErr*timeElapsed + this->getD()*(this->err - this->lastErr)/timeElapsed;
    this->lastOutput = this->getMaxOutputSet() && this->lastOutput > this->getMaxOutput() ? this->getMaxOutput() : this->getMinOutputSet() && this->lastOutput < this->getMinOutput() ? this->getMinOutput() : this->lastOutput;
    return this->lastOutput;
}

void PIDController::computeAndSendNextOutput(){
    if(this->initState == 3){
        std_msgs::Float64 msgNext;
        msgNext.data = this->computeNextOutput();
        // ROS_INFO("PID_CONTROLLER_CHECK: %0.6f", msgNext.data);
        this->outputController.publish(msgNext);
    }
    this->updateLastCalculationTime();
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

void PIDController::setMaxOutput(double max){
    this->outputCap[1] = max;
    this->outputCapSet[1] = true;
}

double PIDController::getMaxOutput(){
    if(this->getMaxOutputSet()) return this->outputCap[1];
    throw "The max value of this controller is not set";
}

void PIDController::setMinOutput(double min){
    this->outputCap[0] = min;
    this->outputCapSet[0] = true;
}
        
double PIDController::getMinOutput(){
    if(this->getMinOutputSet()) return this->outputCap[0];
    throw "The min value of this controller is not set";
}

void PIDController::setMinMaxOutput(double min, double max){
    this->setMinOutput(min);
    this->setMaxOutput(max);
}

bool PIDController::getMaxOutputSet(){
    return this->outputCapSet[1];
}

bool PIDController::getMinOutputSet(){
    return this->outputCapSet[0];
}