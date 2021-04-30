#include <ros/ros.h>
#include <iostream>
#include "PIDController.hpp"

int main(int argc, char** argv){

    ros::init(argc, argv, "WResponseControl", ros::InitOption::AnonymousName);

    ros::NodeHandle n;
    XmlRpc::XmlRpcValue controllerParams;
    n.getParam("controllers", controllerParams);
    ROS_ASSERT(controllerParams.getType() == XmlRpc::XmlRpcValue::TypeArray);
    PIDController* controllers[controllerParams.size()]; 
    for(int i = 0; i < controllerParams.size(); i++){
        ROS_ASSERT(controllerParams[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        std::string setpoint;
        std::string feedback;
        std::string output;
        double P;
        double I;
        double D;

        ROS_ASSERT(controllerParams[i].hasMember("setpointTopic") && controllerParams[i]["setpointTopic"].getType() == XmlRpc::XmlRpcValue::TypeString);
        setpoint = static_cast<std::string>(controllerParams[i]["setpointTopic"]);

        ROS_ASSERT(controllerParams[i].hasMember("feedbackTopic") && controllerParams[i]["feedbackTopic"].getType() == XmlRpc::XmlRpcValue::TypeString);
        feedback = static_cast<std::string>(controllerParams[i]["feedbackTopic"]);

        ROS_ASSERT(controllerParams[i].hasMember("outputTopic") && controllerParams[i]["outputTopic"].getType() == XmlRpc::XmlRpcValue::TypeString);
        output = static_cast<std::string>(controllerParams[i]["outputTopic"]);

        ROS_ASSERT(controllerParams[i].hasMember("P") && controllerParams[i]["P"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        P = static_cast<double>(controllerParams[i]["P"]);

        ROS_ASSERT(controllerParams[i].hasMember("I") && controllerParams[i]["I"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        I = static_cast<double>(controllerParams[i]["I"]);

        ROS_ASSERT(controllerParams[i].hasMember("D") && controllerParams[i]["D"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        D = static_cast<double>(controllerParams[i]["D"]);

        controllers[i] = new PIDController(setpoint, feedback, output, n);
        controllers[i]->setPID(P, I, D);
        
    }

    ros::Rate loop(50);

    while(ros::ok()){

        for(int i = 0; i < sizeof(controllers)/sizeof(PIDController*); i++) controllers[i]->executeNextControlCycle();

        ros::spinOnce();
        loop.sleep();
    }
}