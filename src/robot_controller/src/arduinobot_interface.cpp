#include "robot_controller/arduinobot_interface.h"
#include <std_msgs/Float32MultiArray.h>
#include "robot_controller/AnglesConverter.h"

ArduinobotInterface::ArduinobotInterface(ros::NodeHandle& nh) : nh_(nh), 
            pnh_("~"),
            pos_(6, 0),
            vel_(6, 0),
            eff_(6, 0),
            cmd_(6, 0),
            names_{"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}
{
    // Read from the param server
    pnh_.param("joint_names", names_, names_);

    // Init the publisher with the hardware
    hardware_pub_ = pnh_.advertise<std_msgs::Float32MultiArray>("/arduino/arm_actuate", 1000);
    hardware_srv_ = pnh_.serviceClient<robot_controller::AnglesConverter>("/radians_to_degrees");
    // hardware_server = pnh_.serviceClient<rospy_tutorials::AddTwoInts>("/encoder_feed");
    
    ROS_INFO("Starting Arduinobot Hardware Interface...");

    // connect and register joint state interface
    hardware_interface::JointStateHandle state_handle1(names_.at(0), &pos_.at(0), &vel_.at(0), &eff_.at(0));
    joint_state_interface_.registerHandle(state_handle1);
    hardware_interface::JointStateHandle state_handle2(names_.at(1), &pos_.at(1), &vel_.at(1), &eff_.at(1));
    joint_state_interface_.registerHandle(state_handle2);
    hardware_interface::JointStateHandle state_handle3(names_.at(2), &pos_.at(2), &vel_.at(2), &eff_.at(2));
    joint_state_interface_.registerHandle(state_handle3);
    hardware_interface::JointStateHandle state_handle4(names_.at(3), &pos_.at(3), &vel_.at(3), &eff_.at(3));
    joint_state_interface_.registerHandle(state_handle4);
    hardware_interface::JointStateHandle state_handle5(names_.at(4), &pos_.at(4), &vel_.at(4), &eff_.at(4));
    joint_state_interface_.registerHandle(state_handle5);
    hardware_interface::JointStateHandle state_handle6(names_.at(5), &pos_.at(5), &vel_.at(5), &eff_.at(5));
    joint_state_interface_.registerHandle(state_handle6);

    registerInterface(&joint_state_interface_);

    // connect and register joint position interface
    // the motors accept position inputs
    hardware_interface::JointHandle position_handle1(joint_state_interface_.getHandle(names_.at(0)), &cmd_.at(0));
    joint_position_interface_.registerHandle(position_handle1);
    hardware_interface::JointHandle position_handle2(joint_state_interface_.getHandle(names_.at(1)), &cmd_.at(1));
    joint_position_interface_.registerHandle(position_handle2);
    hardware_interface::JointHandle position_handle3(joint_state_interface_.getHandle(names_.at(2)), &cmd_.at(2));
    joint_position_interface_.registerHandle(position_handle3);
    hardware_interface::JointHandle position_handle4(joint_state_interface_.getHandle(names_.at(3)), &cmd_.at(3));
    joint_position_interface_.registerHandle(position_handle4);
    hardware_interface::JointHandle position_handle5(joint_state_interface_.getHandle(names_.at(4)), &cmd_.at(4));
    joint_position_interface_.registerHandle(position_handle5);
    hardware_interface::JointHandle position_handle6(joint_state_interface_.getHandle(names_.at(5)), &cmd_.at(5));
    joint_position_interface_.registerHandle(position_handle6);

    registerInterface(&joint_position_interface_);

    ROS_INFO("Interfaces registered.");


    ROS_INFO("Preparing the Controller Manager");

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    update_freq_ = ros::Duration(0.1);
    looper_ = nh_.createTimer(update_freq_, &ArduinobotInterface::update, this);
    
    ROS_INFO("Ready to execute the control loop");
}

void ArduinobotInterface::update(const ros::TimerEvent& e)
{
    // This function is called periodically in order to update the controller
    // manager about the progress in the execution of the goal of the hardware
    ROS_INFO("Update Event");
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ArduinobotInterface::read()
{
    // Reads the current status of the Hardware (Arduino)
    // Open Loop Control - no sensor available on the robot taht detects the effective
    // angle of totation of each joint. Suppose that the motors are always able to follow
    // the position command
    // encoder_read.request.a=1.0;
    // encoder_read.request.b=1.0;
    // if(hardware_server.call(encoder_read))
    // {
    // pos_.at[0]=angles::from_degrees(encoder_read.response.sum);
	// pos_.at[1]=angles::from_degrees(encoder_read.response.sum[1]);
	// pos_.at[2]=angles::from_degrees(encoder_read.response.sum[2]);
    // }
    // else
    // {
    pos_.at(0) = double(cmd_.at(0));
    pos_.at(1) = double(cmd_.at(1));
    pos_.at(2) = double(cmd_.at(2));
    pos_.at(3) = double(cmd_.at(3));
    pos_.at(4) = double(cmd_.at(4));
    pos_.at(5) = double(cmd_.at(5));
    // }
}

void ArduinobotInterface::write(ros::Duration elapsed_time)
{    
    // Send the command to the Hardware (Arduino)
    // First converts the angle from the moveit/urdf convention 
    // to the Arduino convention and then publishes the converted angles
    robot_controller::AnglesConverter srv;
    srv.request.base = double(cmd_.at(0));
    srv.request.shoulder = double(cmd_.at(1));
    srv.request.elbow = double(cmd_.at(2));
    srv.request.forearm = double(cmd_.at(3));
    srv.request.wrist = double(cmd_.at(4));
    srv.request.finger = double(cmd_.at(5));

    // Call the service and show the response of the service
    if (hardware_srv_.call(srv))
    {
        // compose the array message
        // std::vector<double> angles_deg;
        
        // std::round(srv.response.base * 100.0) / 100.0;
        // std::round(srv.response.shoulder * 100.0) / 100.0;
        // std::round(srv.response.elbow * 100.0) / 100.0;
        // std::round(srv.response.forearm * 100.0) / 100.0;
        // std::round(srv.response.wrist * 100.0) / 100.0;
        // std::round(srv.response.finger * 100.0) / 100.0;
        
        // angles_deg.push_back(srv.response.base);
        // angles_deg.push_back(srv.response.shoulder);
        // angles_deg.push_back(srv.response.elbow);
        // angles_deg.push_back(srv.response.forearm);
        // angles_deg.push_back(srv.response.wrist);
        // angles_deg.push_back(srv.response.finger);

        std_msgs::Float32MultiArray msg;

        // set up dimensions
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = 6;
        msg.layout.dim[0].stride = 1;

        // copy in the data
        msg.data = {
            round(srv.response.base * 100) / 100,
            round(srv.response.shoulder * 100) / 100,
            round(srv.response.elbow * 100) / 100,
            round(srv.response.forearm * 100) / 100,
            round(srv.response.wrist * 100) / 100,
            round(srv.response.finger * 100) / 100};
        //    msg.data = {
        //     srv.response.base,
        //     srv.response.shoulder,
        //     srv.response.elbow,
        //     srv.response.forearm,
        //     srv.response.wrist,
        //     srv.response.finger};
        // msg.data.insert(msg.data.end(), angles_deg.begin(), angles_deg.end());

        // publish the array message to the defined topic
        hardware_pub_.publish(msg);
    }
    else
    {
        ROS_ERROR("Failed to call service radians_to_degrees");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arduinobot_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    ArduinobotInterface robot(nh);

    // Keep ROS up and running
    spinner.spin();
    return 0;
}