#include "PIDController.hpp"

void PIDController::setPointCallback(const MsgPtr msg){
    // Capture the setpoint message data
    this->setpoint = msg->data;
    // Change the init state to reflect that a setpoint has been received
    this->initState |= 1;
}

void PIDController::feedbackCallback(const MsgPtr msg){
    // Capture the feedback message data
    this->feedback = msg->data;
    // Change the init state to reflect that a feedback message has been received
    this->initState |= 2;
}

std::chrono::milliseconds PIDController::getCurrentTime(){
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
}

void PIDController::updateLastCalculationTime(){
    this->lastCalculationTime = PIDController::getCurrentTime();
}

PIDController::PIDController(std::string setPointTopic, std::string feedbackTopic, std::string outputTopic, ros::NodeHandle& node){
    // Initialize the ROS topics for this controller
    this->setPointReader = node.subscribe(setPointTopic, 1000, &PIDController::setPointCallback, this);
    this->feedbackReader = node.subscribe(feedbackTopic, 1000, &PIDController::feedbackCallback, this);
    this->outputController = node.advertise<std_msgs::Float64>(outputTopic, 1000);

    // Initialize all variables to 0 or false
    this->P = 0;
    this->I = 0;
    this->D = 0;
    this->err = 0;
    this->lastErr = 0;
    this->initState = 0;
    this->outputCapSet[0] = false;
    this->outputCapSet[1] = false;
    this->iCapSet = false;
    // Set the last calculation time for the next calculation
    this->updateLastCalculationTime();
}

double PIDController::computeNextOutput(){
    // Store the current error as the last error
    this->lastErr = this->err;
    // Compute the new current error
    this->err = this->setpoint - this->feedback;
    // Add the new current error to the summed error
    this->sumErr += this->err;
    // Cap the summed error if the summed error cap is set
    this->sumErr = this->getICapSet() && abs(this->sumErr) > abs(this->getICap()) ? this->getICap() * this->sumErr/abs(this->sumErr) : this->sumErr;
    // Calculate the time elapsed since the last calculation in seconds
    double timeElapsed = (PIDController::getCurrentTime().count() - this->lastCalculationTime.count())/1000.f;
    // Use the standard PID formula to compute the next output
    this->lastOutput = this->getP()*this->err + this->getI()*this->sumErr*timeElapsed + this->getD()*(this->err - this->lastErr)/timeElapsed;
    // Cap the next output if applicable
    this->lastOutput = this->getMaxOutputSet() && this->lastOutput > this->getMaxOutput() ? this->getMaxOutput() : this->getMinOutputSet() && this->lastOutput < this->getMinOutput() ? this->getMinOutput() : this->lastOutput;
    return this->lastOutput;
}

void PIDController::computeAndSendNextOutput(){
    // If both a setpoint and feedback message have been received...
    if(this->initState == 3){
        // Compute and send the next output to the output ROS topic
        std_msgs::Float64 msgNext;
        msgNext.data = this->computeNextOutput();
        this->outputController.publish(msgNext);
    }
    // Set the last calculation time to provide for the correct next output calculation
    this->updateLastCalculationTime();
}

void PIDController::executeNextControlCycle(){
    // This method does the control cycle work
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
    if(this->getMinOutputSet() && this->getMinOutput() >= max) throw "Max must be greater than min.";
    this->outputCap[1] = max;
    // Set that the max ouput value has been set
    this->outputCapSet[1] = true;
}

double PIDController::getMaxOutput(){
    // If the max output is not set, throw an error
    if(this->getMaxOutputSet()) return this->outputCap[1];
    throw "The max value of this controller is not set";
}

void PIDController::setMinOutput(double min){
    if(this->getMaxOutputSet() && this->getMaxOutput() <= min) throw "Min must be less than max.";
    this->outputCap[0] = min;
    // Set that the min output value has been set
    this->outputCapSet[0] = true;
}
        
double PIDController::getMinOutput(){
    // If the min output is not set, throw an error
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

void PIDController::setICap(double iCap){
    this->iCap = iCap;
    // Set that the summed error cap is set
    this->iCapSet = true;
}

double PIDController::getICap(){
    // If the summed error cap is not set, throw an error
    if(this->getICapSet()) return this->iCap;
    throw "The I Cap of this controller is not set";
}
        
bool PIDController::getICapSet(){
    return this->iCapSet;
}