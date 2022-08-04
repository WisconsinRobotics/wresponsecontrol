#include "PIDController.hpp"
#include <chrono>
#include <stdexcept>

void PIDController::setPointCallback(const MsgPtr &msg) {
    // Capture the setpoint message data
    this->setpoint = msg->data;
    // Change the init state to reflect that a setpoint has been received
    this->initState |= 1;
}

void PIDController::feedbackCallback(const MsgPtr &msg) {
    // Capture the feedback message data
    this->feedback = msg->data;
    // Change the init state to reflect that a feedback message has been
    // received
    this->initState |= 2;
}

auto PIDController::getCurrentTime()
    -> std::chrono::time_point<std::chrono::system_clock> {
    return std::chrono::system_clock::now();
}

void PIDController::updateLastCalculationTime() {
    this->lastCalculationTime = PIDController::getCurrentTime();
}

PIDController::PIDController(std::string setPointTopic,
                             std::string feedbackTopic, std::string outputTopic,
                             ros::NodeHandle &node)
    : P{0}, I{0}, D{0}, err{0}, lastErr{0}, initState{0} {
    // Initialize the ROS topics for this controller
    this->setPointReader = node.subscribe(
        setPointTopic, 1000, &PIDController::setPointCallback, this);
    this->feedbackReader = node.subscribe(
        feedbackTopic, 1000, &PIDController::feedbackCallback, this);
    this->outputController =
        node.advertise<std_msgs::Float64>(outputTopic, 1000);

    // Set the last calculation time for the next calculation
    this->updateLastCalculationTime();
}

double PIDController::computeNextOutput() {
    // Store the current error as the last error
    this->lastErr = this->err;
    // Compute the new current error
    this->err = this->setpoint - this->feedback;
    // Add the new current error to the summed error
    this->sumErr += this->err;
    // Cap the summed error if the summed error cap is set
    this->sumErr =
        this->getICapSet() && abs(this->sumErr) > abs(this->getICap())
            ? this->getICap() * this->sumErr / abs(this->sumErr)
            : this->sumErr;
    // Calculate the time elapsed since the last calculation in seconds
    double timeElapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            PIDController::getCurrentTime() - this->lastCalculationTime)
            .count();
    // Use the standard PID formula to compute the next output
    this->lastOutput = this->getP() * this->err +
                       this->getI() * this->sumErr * timeElapsed +
                       this->getD() * (this->err - this->lastErr) / timeElapsed;
    // Cap the next output if applicable
    this->lastOutput =
        this->getMaxOutputSet() && this->lastOutput > this->getMaxOutput()
            ? this->getMaxOutput()
        : this->getMinOutputSet() && this->lastOutput < this->getMinOutput()
            ? this->getMinOutput()
            : this->lastOutput;
    return this->lastOutput;
}

void PIDController::computeAndSendNextOutput() {
    // If both a setpoint and feedback message have been received...
    if (this->initState == 3) {
        // Compute and send the next output to the output ROS topic
        std_msgs::Float64 msgNext;
        msgNext.data = this->computeNextOutput();
        this->outputController.publish(msgNext);
    }
    // Set the last calculation time to provide for the correct next output
    // calculation
    this->updateLastCalculationTime();
}

void PIDController::executeNextControlCycle() {
    // This method does the control cycle work
    this->computeAndSendNextOutput();
}

void PIDController::setP(double P) { this->P = P; }

auto PIDController::getP() const -> double { return this->P; }

void PIDController::setI(double I) { this->I = I; }

auto PIDController::getI() const -> double { return this->I; }

void PIDController::setD(double D) { this->D = D; }

auto PIDController::getD() const -> double { return this->D; }

void PIDController::setPID(double P, double I, double D) {
    this->setP(P);
    this->setI(I);
    this->setD(D);
}

void PIDController::setMaxOutput(double max) {
    if (this->getMinOutputSet() && this->getMinOutput() >= max)
        throw std::invalid_argument{"Max must be greater than min"};
    this->outputCap.upper = max;
}

auto PIDController::getMaxOutput() const -> double {
    // If the max output is not set, throw an error
    if (this->getMaxOutputSet())
        return *this->outputCap.upper;
    throw std::runtime_error{"The max value of this controller is not set"};
}

void PIDController::setMinOutput(double min) {
    if (this->getMaxOutputSet() && this->getMaxOutput() <= min)
        throw std::invalid_argument{"Min must be less than max"};
    this->outputCap.lower = min;
}

auto PIDController::getMinOutput() const -> double {
    // If the min output is not set, throw an error
    if (this->getMinOutputSet())
        return *this->outputCap.lower;
    throw std::runtime_error{"The min value of this controller is not set"};
}

void PIDController::setMinMaxOutput(OutputBounds<double> bounds) {
    this->setMinOutput(bounds.lower);
    this->setMaxOutput(bounds.upper);
}

auto PIDController::getMaxOutputSet() const -> bool {
    return static_cast<bool>(this->outputCap.upper);
}

auto PIDController::getMinOutputSet() const -> bool {
    return static_cast<bool>(this->outputCap.lower);
}

void PIDController::setICap(double iCap) { this->iCap = iCap; }

auto PIDController::getICap() const -> double {
    // If the summed error cap is not set, throw an error
    if (this->getICapSet())
        return *this->iCap;
    throw std::runtime_error{"The I Cap of this controller is not set"};
}

auto PIDController::getICapSet() const -> bool {
    return static_cast<bool>(this->iCap);
}