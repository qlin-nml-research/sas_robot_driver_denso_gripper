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
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/
#include <exception>
#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <sas_common/sas_common.h>
#include <sas_conversions/eigen3_std_conversions.h>
#include <sas_robot_driver/sas_robot_driver_ros.h>
#include <sas_robot_driver_denso/sas_robot_driver_denso.h>


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

    ros::init(argc, argv, "sas_robot_driver_denso_node", ros::init_options::NoSigintHandler);

    ROS_INFO_STREAM(ros::this_node::getName()+"::Loading parameters from parameter server.");
    ros::NodeHandle nh;
    sas::RobotDriverDensoConfiguration robot_driver_denso_configuration;
    sas::get_ros_param(nh,"/robot_ip_address",robot_driver_denso_configuration.ip_address);
    sas::get_ros_param(nh,"/robot_port",robot_driver_denso_configuration.port);
    sas::get_ros_param(nh,"/robot_speed",robot_driver_denso_configuration.speed);
    try {
        sas::get_ros_param(nh,"/using_hand",robot_driver_denso_configuration.using_hand);
        if(robot_driver_denso_configuration.using_hand) {
            robot_driver_denso_configuration.lock_name = ros::this_node::getName();
            ROS_WARN_STREAM(ros::this_node::getName() + "::Using hand is enabled. We will instantiate a mutex to lock movement is.");
            auto temp_str = robot_driver_denso_configuration.lock_name;
            if(temp_str.front()=='/'){ temp_str = temp_str.substr(1);}
            ROS_WARN_STREAM(ros::this_node::getName() + "::Please ensure that hand node is created with lock at [" + temp_str + "].");
        }
    }catch(...){}
    sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
    sas::get_ros_param(nh,"/thread_sampling_time_nsec",robot_driver_ros_configuration.thread_sampling_time_nsec);
    sas::get_ros_param(nh,"/q_min",robot_driver_ros_configuration.q_min);
    sas::get_ros_param(nh,"/q_max",robot_driver_ros_configuration.q_max);
    robot_driver_ros_configuration.robot_driver_provider_prefix = ros::this_node::getName();

    try
    {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating DENSO robot (bCap).");
        sas::RobotDriverDenso robot_driver_denso(robot_driver_denso_configuration,&kill_this_process);
        robot_driver_denso.set_joint_limits({sas::std_vector_double_to_vectorxd(robot_driver_ros_configuration.q_min),
                                            sas::std_vector_double_to_vectorxd(robot_driver_ros_configuration.q_max)});
        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(nh,
                                             &robot_driver_denso,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        robot_driver_ros.control_loop();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }

    return 0;
}
