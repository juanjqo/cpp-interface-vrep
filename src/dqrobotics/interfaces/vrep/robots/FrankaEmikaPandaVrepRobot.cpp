/**
(C) Copyright 2023 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Juan Jose Quiroz Omana   (juanjqo@g.ecc.u-tokyo.ac.jp)
        - Responsible for the original implementation.
          (Adapted from LBR4pVrepRobot.h)
*/
#include<dqrobotics/interfaces/vrep/robots/FrankaEmikaPandaVrepRobot.h>
#include<dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>

namespace DQ_Robotics
{

FrankaEmikaPandaVrepRobot::FrankaEmikaPandaVrepRobot(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface):DQ_VrepRobot(robot_name, vrep_interface)
{
    _set_names(robot_name);
}

void FrankaEmikaPandaVrepRobot::_set_names(const std::string& robot_name)
{
    std::vector<std::string> splited_name = strsplit(robot_name,'#');
    std::string robot_label = splited_name[0];

    if(robot_label.compare(std::string("Franka")) != 0)
    {
        throw std::runtime_error("Expected Franka ");
    }

    std::string robot_index("");
    if(splited_name.size() > 1)
        robot_index = "#"+splited_name[1];

    for(int i=1;i<8;i++)
    {
        std::string current_joint_name = robot_label + std::string("_joint") + std::to_string(i) + robot_index;
        joint_names_.push_back(current_joint_name);

        std::string current_link_name = robot_label + std::string("_link") + std::to_string(i+1)+
                                        std::string("_resp") + robot_index;
        link_names_.push_back(current_link_name);
    }
}

void FrankaEmikaPandaVrepRobot::send_q_to_vrep(const VectorXd &q)
{
    _get_interface_sptr()->set_joint_positions(joint_names_, q);
}

void FrankaEmikaPandaVrepRobot::send_q_target_to_vrep(const VectorXd& q_target)
{
    _get_interface_sptr()->set_joint_target_positions(joint_names_, q_target);
}

void FrankaEmikaPandaVrepRobot::send_q_dot_target_to_vrep(const VectorXd& q_dot_target)
{
   _get_interface_sptr()->set_joint_target_velocities(joint_names_, q_dot_target);
}

void FrankaEmikaPandaVrepRobot::send_torques_target_to_vrep(const VectorXd& torques_target)
{
   throw std::runtime_error("Not implemented yet ");
}


VectorXd FrankaEmikaPandaVrepRobot::get_q_from_vrep()
{
    return _get_interface_sptr()->get_joint_positions(joint_names_);
}

VectorXd FrankaEmikaPandaVrepRobot::get_q_dot_from_vrep()
{
   return _get_interface_sptr()->get_joint_velocities(joint_names_);
}

DQ_SerialManipulatorMDH FrankaEmikaPandaVrepRobot::kinematics()
{
    return FrankaEmikaPandaRobot::kinematics();
}

DQ FrankaEmikaPandaVrepRobot::get_robot_base_from_vrep(const DQ& current_base)
{
    return current_base*_get_interface_sptr()->get_object_pose(robot_name_)*(offset_);
}




}