/**
 * @file PIDController.hpp
 * @author Ben Nowotny
 * @brief The implementation requirements of a PID Controller
 * @date 2021-05-10
 */

// Header Guard
#ifndef PIDControllerH
#define PIDControllerH

#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

// Simplify reference to the double message
using MsgPtr = std_msgs::Float64::ConstPtr;

/**
 * @brief A PID-model controller
 *
 */
class PIDController {

private:
    template <typename T> struct OptionalBounds {
        std::optional<T> upper;
        std::optional<T> lower;
    };

    /// This controller's P value
    double P;
    /// This controller's I value
    double I;
    /// This controller's D value
    double D;
    /// The error on the last update cycle
    double err;
    /// The error on the update cycle before the err cycle
    double lastErr;
    /// The summed error over all cycles
    double sumErr;
    /// The current target value of the controller
    std::optional<double> setpoint;
    /// The current external feedback value to the controller
    std::optional<double> feedback;
    /// The last output sent
    double lastOutput;
    /// The set of min/max bounds for the output (may or may not be set)
    OptionalBounds<double> outputCap;
    /// The maximum magnitude of the summed error (may or may not be set)
    std::optional<double> iCap;

    /// The ROS subscriber to capture setpoint changes
    ros::Subscriber setPointReader;
    /// The ROS subscriber to capture the external controller feedback
    ros::Subscriber feedbackReader;
    /// The ROS publisher that publishes the output of this PID Controller
    ros::Publisher outputController;

    /// The timestamp of the last time the output was calculated
    std::chrono::time_point<std::chrono::system_clock> lastCalculationTime;

    /**
     * @brief The callback when a new setpoint is delivered
     *
     * @param msg The message containing the new setpoint
     */
    void setPointCallback(const MsgPtr &msg);

    /**
     * @brief The callback when new feedback data is delivered
     *
     * @param msg The message containing feedback data
     */
    void feedbackCallback(const MsgPtr &msg);

    /**
     * @brief Updates the stored time for the most recent calculation
     */
    void updateLastCalculationTime();

    /**
     * @brief Get the current time
     *
     * @return std::chrono::milliseconds The current time
     */
    static auto getCurrentTime()
        -> std::chrono::time_point<std::chrono::system_clock>;

    /**
     * @brief Compute the next output.  This alters the state of the controller,
     * and should not be called unless the output is used.
     *
     * @return double The next output of the controller
     */
    auto computeNextOutput() -> double;

    /**
     * @brief Compute the next controller output and send it to the output ROS
     * topic
     */
    void computeAndSendNextOutput();

public:
    /**
     * @brief Construct a new PIDController object
     *
     * @param setPointTopic The string containing the path towards the setpoint
     * ROS topic
     * @param feedbackTopic The string containing the path towards the feedback
     * ROS topic
     * @param outputTopic The string containing the path towards the output ROS
     * topic
     * @param node The ROS node that contains this PID Controller
     */
    PIDController(const std::string &setPointTopic,
                  const std::string &feedbackTopic,
                  const std::string &outputTopic, ros::NodeHandle &node);

    /**
     * @brief Sets this controller's P value
     *
     * @param P The new P value for this controller
     */
    void setP(double P);

    /**
     * @brief Gets this controller's P value
     *
     * @return double The P value for this controller
     */
    [[nodiscard]] auto getP() const -> double;

    /**
     * @brief Sets this controller's I value
     *
     * @param I The new I value for this controller
     */
    void setI(double I);

    /**
     * @brief Gets this controller's I value
     *
     * @return double The I value for this controller
     */
    [[nodiscard]] auto getI() const -> double;

    /**
     * @brief Sets this controller's D value
     *
     * @param D The new D value for this controller
     */
    void setD(double D);

    /**
     * @brief Gets this controller's D value
     *
     * @return double The D value for this controller
     */
    [[nodiscard]] auto getD() const -> double;

    /**
     * @brief Sets this controller's P, I, and D values
     *
     * @param P The new P value for this controller
     * @param I The new I value for this controller
     * @param D The new D value for this controller
     */
    void setPID(double P, double I, double D);

    /**
     * @brief Get the most recent error of the controller
     *
     * @return double the most recent error of this controller
     */
    [[nodiscard]] auto getError() const -> double;

    /**
     * @brief Set the maximum value of the output of the controller.  It must be
     * strictly greater than the min, if it is set.
     *
     * @param max The new maximum value of the controller's ouput
     */
    void setMaxOutput(double max);

    /**
     * @brief Get the maximum value of the output of the controller
     *
     * @return double The maximum value of the controller's output
     */
    [[nodiscard]] auto getMaxOutput() const -> double;

    /**
     * @brief Set the minimum value of the output of the controller.  It must be
     * strictly less than the max, if it is set.
     *
     * @param min The new minimum value of the controller's output
     */
    void setMinOutput(double min);

    /**
     * @brief Get the minimum value of the output of the controller
     *
     * @return double The minimum value of the controller's output
     */
    [[nodiscard]] auto getMinOutput() const -> double;

    template <typename T> struct OutputBounds {
        T upper;
        T lower;
    };

    /**
     * @brief Set the minimum and maximum of the controller's output.  The max
     * must be strictly greater than the min.
     *
     * @param min The new minimum value of the controller's output
     * @param max The new maximum value of the controller's output
     */
    void setMinMaxOutput(OutputBounds<double> bounds);

    /**
     * @brief Get whether or not the max output value is set
     *
     * @return true The max output value is set
     * @return false The max output value is not set (default)
     */
    [[nodiscard]] auto getMaxOutputSet() const -> bool;

    /**
     * @brief Get whether or not the min output value is set
     *
     * @return true The min ouptput value is set
     * @return false The min output value is not set (default)
     */
    [[nodiscard]] auto getMinOutputSet() const -> bool;

    /**
     * @brief Sets the maximum magnitude of the summed error
     *
     * @param iCap The new summed error cap
     */
    void setICap(double iCap);

    /**
     * @brief Gets the maximum magnitude of the summed error
     *
     * @return double The summed error cap
     */
    [[nodiscard]] auto getICap() const -> double;

    /**
     * @brief Get whether or not the summed error cap is set
     *
     * @return true The summed error cap is set
     * @return false The summed error cap is not set (default)
     */
    [[nodiscard]] auto getICapSet() const -> bool;

    /**
     * @brief Compute the next output and dispatch it to the output ROS topic
     */
    void executeNextControlCycle();
};

#endif