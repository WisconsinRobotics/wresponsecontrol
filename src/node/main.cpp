/**
 * @file WResponseControl.cpp
 * @author Ben Nowotny
 * @brief The driver node that creates and executes PID Control
 * @date 2021-05-12
 */

#include "PIDController.hpp"
#include "XmlRpcValue.h"
#include <iostream>
#include <optional>
#include <ros/ros.h>
#include <string>

using XmlRpc::XmlRpcValue;

template <typename T>
auto getRequiredParameter(const std::string &name,
                          const XmlRpcValue &containingLibrary,
                          const XmlRpcValue::Type &parameterType) -> T {
    ROS_ASSERT(containingLibrary.hasMember(name) &&
               containingLibrary[name].getType() == parameterType);
    return static_cast<T>(containingLibrary[name]);
}

template <typename T>
auto getOptionalParameter(const std::string &name,
                          const XmlRpcValue &containingLibrary,
                          const XmlRpcValue::Type &parameterType)
    -> std::optional<T> {
    if (containingLibrary.hasMember(name) &&
        containingLibrary[name].getType() == parameterType) {
        return static_cast<T>(containingLibrary[name]);
    }
    return {};
}

auto getRequiredNumericParameter(const std::string &name,
                                 const XmlRpcValue &containingLibrary)
    -> double {
    auto intVal{getOptionalParameter<int>(name, containingLibrary,
                                          XmlRpcValue::TypeInt)};
    auto doubleVal{getOptionalParameter<double>(name, containingLibrary,
                                                XmlRpcValue::TypeDouble)};
    ROS_ASSERT(intVal || doubleVal);
    return doubleVal ? *doubleVal : *intVal;
}

auto getOptionalNumericParameter(const std::string &name,
                                 const XmlRpcValue &containingLibrary)
    -> std::optional<double> {
    auto intVal{getOptionalParameter<int>(name, containingLibrary,
                                          XmlRpcValue::TypeInt)};
    auto doubleVal{getOptionalParameter<double>(name, containingLibrary,
                                                XmlRpcValue::TypeDouble)};
    if (intVal || doubleVal) {
        return doubleVal ? doubleVal : std::optional<double>{*intVal};
    }
    return {};
}

/**
 * @brief The driver method for the main class
 *
 * @param argc The number of command line arguments
 * @param argv The set of command line arguments
 * @return int The status code on the program's exit
 */
auto main(int argc, char **argv) -> int {

    // Create the new ROS node (Anonymous node for multiple instantiation)
    ros::init(argc, argv, "WResponseControl", ros::InitOption::AnonymousName);

    // Create the node handle and private node handle
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    // Capture the controller list ROS params
    XmlRpc::XmlRpcValue controllerParams;
    np.getParam("controllers", controllerParams);
    // Ensure that the caputered controllers object is a list
    ROS_ASSERT(controllerParams.getType() == XmlRpc::XmlRpcValue::TypeArray);

    // Create the PID Controllers list
    std::vector<std::unique_ptr<PIDController>> controllers{};
    controllers.reserve(static_cast<std::size_t>(controllerParams.size()));

    // For each controller...
    for (int i = 0; i < controllerParams.size(); i++) {
        // Ensure that the controller entry is a collection of values (struct)
        ROS_ASSERT(controllerParams[i].getType() ==
                   XmlRpc::XmlRpcValue::TypeStruct);

        // Set up capturing variables for the required controller parameters

        // Ensure that this controller has a setpointTopic parameter and that it
        // is a string; capture the parameter
        auto setpoint{getRequiredParameter<std::string>(
            "setpointTopic", controllerParams[i], XmlRpcValue::TypeString)};

        // Ensure that this controller has a feedbackTopic parameter and that it
        // is a string; capture the parameter
        auto feedback{getRequiredParameter<std::string>(
            "feedbackTopic", controllerParams[i], XmlRpcValue::TypeString)};

        // Ensure that this controller has an outputTopic parameter and that it
        // is a string; capture the parameter
        auto output{getRequiredParameter<std::string>(
            "outputTopic", controllerParams[i], XmlRpcValue::TypeString)};

        // Ensure that this controller has a P parameter and that it is a
        // number; capture the parameter
        auto P{getRequiredNumericParameter("P", controllerParams[i])};

        // Ensure that this controller has an I parameter and that it is a
        // number; capture the parameter
        auto I{getRequiredNumericParameter("I", controllerParams[i])};

        // Ensure that this controller has a D parameter and that it is a
        // number; capture the parameter
        auto D{getRequiredNumericParameter("D", controllerParams[i])};

        // Initialize the new controller from the required information
        controllers.push_back(
            std::make_unique<PIDController>(setpoint, feedback, output, n));
        controllers.at(i)->setPID(P, I, D);

        // Check for the optional max parameter; if it exists and is a number,
        // capture it in the controller
        auto maxValue{getOptionalNumericParameter("max", controllerParams[i])};
        if (maxValue)
            controllers.at(i)->setMaxOutput(*maxValue);

        // Check for the optional min parameter; if it exists and is a number,
        // capture it in the controller
        auto minValue{getOptionalNumericParameter("min", controllerParams[i])};
        if (minValue)
            controllers.at(i)->setMinOutput(*minValue);

        // Check for the optional ICap parameter; if it exists and is a number,
        // capture it in the controller
        auto iCap{getOptionalNumericParameter("ICap", controllerParams[i])};
        if (iCap)
            controllers.at(i)->setICap(*iCap);
    }

    // Get the rate of the controllers from the config file
    XmlRpc::XmlRpcValue rate;
    np.getParam("rate", rate);
    // Check that the rate is a number; set the appropriate rate
    ROS_ASSERT(rate.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
               rate.getType() == XmlRpc::XmlRpcValue::TypeInt);
    // Define the loop rate (Hz)
    ros::Rate loop(rate.getType() == XmlRpc::XmlRpcValue::TypeDouble
                       ? static_cast<double>(rate)
                       : static_cast<int>(rate));

    // While ROS is active...
    while (ros::ok()) {

        // For each controller, execute its control cycle
        for (const auto &controller : controllers)
            controller->executeNextControlCycle();

        // ROS spin to dispatch and receive messages
        ros::spinOnce();
        // Sleep until the next loop cycle
        loop.sleep();
    }
}