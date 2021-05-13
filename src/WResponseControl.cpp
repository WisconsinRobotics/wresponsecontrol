/**
 * @file WResponseControl.cpp
 * @author Ben Nowotny
 * @brief The driver node that creates and executes PID Control
 * @date 2021-05-12 
 */

#include <ros/ros.h>
#include <iostream>
#include "PIDController.hpp"

/**
 * @brief The driver method for the main class
 * 
 * @param argc The number of command line arguments
 * @param argv The set of command line arguments
 * @return int The status code on the program's exit
 */
int main(int argc, char** argv){

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
    PIDController* controllers[controllerParams.size()];
    
    // For each controller...
    for(int i = 0; i < controllerParams.size(); i++){
        // Ensure that the controller entry is a collection of values (struct)
        ROS_ASSERT(controllerParams[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        
        // Set up capturing variables for the required controller parameters
        std::string setpoint;
        std::string feedback;
        std::string output;
        double P;
        double I;
        double D;

        // Ensure that this controller has a setpointTopic parameter and that it is a string; capture the parameter
        ROS_ASSERT(controllerParams[i].hasMember("setpointTopic") && controllerParams[i]["setpointTopic"].getType() == XmlRpc::XmlRpcValue::TypeString);
        setpoint = static_cast<std::string>(controllerParams[i]["setpointTopic"]);

        // Ensure that this controller has a feedbackTopic parameter and that it is a string; capture the parameter
        ROS_ASSERT(controllerParams[i].hasMember("feedbackTopic") && controllerParams[i]["feedbackTopic"].getType() == XmlRpc::XmlRpcValue::TypeString);
        feedback = static_cast<std::string>(controllerParams[i]["feedbackTopic"]);

        // Ensure that this controller has an outputTopic parameter and that it is a string; capture the parameter
        ROS_ASSERT(controllerParams[i].hasMember("outputTopic") && controllerParams[i]["outputTopic"].getType() == XmlRpc::XmlRpcValue::TypeString);
        output = static_cast<std::string>(controllerParams[i]["outputTopic"]);

        // Ensure that this controller has a P parameter and that it is a number; capture the parameter
        ROS_ASSERT(controllerParams[i].hasMember("P") && controllerParams[i]["P"].getType() == XmlRpc::XmlRpcValue::TypeDouble || controllerParams[i]["P"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        P = controllerParams[i]["P"].getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<double>(controllerParams[i]["P"]) : static_cast<int>(controllerParams[i]["P"]);

        // Ensure that this controller has an I parameter and that it is a number; capture the parameter
        ROS_ASSERT(controllerParams[i].hasMember("I") && controllerParams[i]["I"].getType() == XmlRpc::XmlRpcValue::TypeDouble || controllerParams[i]["I"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        I = controllerParams[i]["I"].getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<double>(controllerParams[i]["I"]) : static_cast<int>(controllerParams[i]["I"]);

        // Ensure that this controller has a D parameter and that it is a number; capture the parameter
        ROS_ASSERT(controllerParams[i].hasMember("D") && controllerParams[i]["D"].getType() == XmlRpc::XmlRpcValue::TypeDouble || controllerParams[i]["D"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        D = controllerParams[i]["D"].getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<double>(controllerParams[i]["D"]) : static_cast<int>(controllerParams[i]["D"]);

        // Initialize the new controller from the required information
        controllers[i] = new PIDController(setpoint, feedback, output, n);
        controllers[i]->setPID(P, I, D);

        // TODO:  Should max > min be enforced in the controller?  Is there a point in min <= max?
        // Check for the optional max parameter; if it exists and is a number, capture it in the controller
        if(controllerParams[i].hasMember("max")){
            ROS_ASSERT(controllerParams[i]["max"].getType() == XmlRpc::XmlRpcValue::TypeDouble || controllerParams[i]["max"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            controllers[i]->setMaxOutput(controllerParams[i]["max"].getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<double>(controllerParams[i]["max"]) : static_cast<int>(controllerParams[i]["max"]));
        }

        // Check for the optional min parameter; if it exists and is a number, capture it in the controller
        if(controllerParams[i].hasMember("min")){
            ROS_ASSERT(controllerParams[i]["min"].getType() == XmlRpc::XmlRpcValue::TypeDouble || controllerParams[i]["min"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            controllers[i]->setMinOutput(controllerParams[i]["min"].getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<double>(controllerParams[i]["min"]) : static_cast<int>(controllerParams[i]["min"]));
        }

        // Check for the optional ICap parameter; if it exists and is a number, capture it in the controller
        if(controllerParams[i].hasMember("ICap")){
            ROS_ASSERT(controllerParams[i]["ICap"].getType() == XmlRpc::XmlRpcValue::TypeDouble || controllerParams[i]["ICap"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            controllers[i]->setICap(controllerParams[i]["ICap"].getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<double>(controllerParams[i]["ICap"]) : static_cast<int>(controllerParams[i]["ICap"]));
        }
        
    }

    // Define the loop rate (Hz)
    // TODO:  Should this be configurable?  I mean, I already have a config file.
    ros::Rate loop(50);

    // While ROS is active...
    while(ros::ok()){

        // For each controller, execute its control cycle
        for(int i = 0; i < sizeof(controllers)/sizeof(PIDController*); i++) controllers[i]->executeNextControlCycle();

        // ROS spin to dispatch and receive messages
        ros::spinOnce();
        // Sleep until the next loop cycle
        loop.sleep();
    }
}