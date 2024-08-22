/*
# Copyright (c) 2016-2022 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver_denso.
#
#    sas_denso_communcation is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_denso is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_denso.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Hung-ching Lin email: qlin1806@g.ecc.u-tokyo.ac.jp
#   Adapted from sas_robot_driver_denso_node.cpp
#
# ################################################################*/
#include <exception>
#include <dqrobotics/utils/DQ_Math.h>
#include <sas_common/sas_common.h>
#include <sas_conversions/eigen3_std_conversions.h>
#include <sas_robot_driver_denso/sas_robot_driver_cobotta_hand.h>


/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
    }

    ros::init(argc, argv, "sas_robot_driver_denso_hand_node", ros::init_options::NoSigintHandler);

    ROS_INFO_STREAM(ros::this_node::getName()+"::Loading parameters from parameter server.");
    ros::NodeHandle nh;
    sas::RobotDriverDensoHandConfiguration robot_driver_denso_hand_configuration;
    sas::get_ros_param(nh,"/robot_ip_address",robot_driver_denso_hand_configuration.ip_address);
    sas::get_ros_param(nh,"/robot_port",robot_driver_denso_hand_configuration.port);
    sas::get_ros_param(nh,"/default_speed",robot_driver_denso_hand_configuration.default_speed);
    sas::get_ros_param(nh,"/lock_name",robot_driver_denso_hand_configuration.lock_name);
    sas::get_ros_param(nh,"/gripper_range_max",robot_driver_denso_hand_configuration.gripper_range_max);
    sas::get_ros_param(nh,"/gripper_range_min",robot_driver_denso_hand_configuration.gripper_range_min);

    robot_driver_denso_hand_configuration.prefix = ros::this_node::getName();
    sas::RobotDriverDensoHand robot_driver_denso_hand(nh, robot_driver_denso_hand_configuration,&kill_this_process);
    try
    {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating DENSO robot hand interface node.");
        ROS_INFO_STREAM(ros::this_node::getName()+"::Connecting to DENSO robot hand interface.");
        robot_driver_denso_hand.connect();
        ROS_INFO_STREAM(ros::this_node::getName()+"::Initializing DENSO robot hand interface node.");
        robot_driver_denso_hand.initialize();
        ROS_INFO_STREAM(ros::this_node::getName()+"::Starting control loop.");
        robot_driver_denso_hand.control_loop();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }
    ROS_INFO_STREAM(ros::this_node::getName()+"::Deinitializing DENSO robot hand interface node.");
    robot_driver_denso_hand.deinitialize();
    ROS_INFO_STREAM(ros::this_node::getName()+"::Disconnecting from DENSO robot hand interface.");
    robot_driver_denso_hand.disconnect();

    ROS_INFO_STREAM(ros::this_node::getName()+"::Exiting DENSO robot hand interface node.");

    return 0;
}
