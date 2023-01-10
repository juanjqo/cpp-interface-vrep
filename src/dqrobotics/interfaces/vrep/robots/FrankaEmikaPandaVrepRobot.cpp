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
#include<dqrobotics/robots/FrankaEmikaPandaRobot.h>

namespace DQ_robotics
{

/**
 * @brief Constructor of the FrankaEmikaPandaVrepRobot class
 * 
 * @param robot_name The name of robot used on the vrep scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 *  
 *               Example:
 *               auto vi = std::make_shared<DQ_VrepInterface>(DQ_VrepInterface());
 *               vi->connect(19997,100,5);
 *               vi->start_simulation();
 *               FrankaEmikaPandaVrepRobot franka_vreprobot("LBR4p", vi);
 *               auto q = franka_vreprobot.get_q_from_vrep();
 *               vi->stop_simulation();
 *               vi->disconnect();
 */
FrankaEmikaPandaVrepRobot::FrankaEmikaPandaVrepRobot(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface):DQ_VrepRobot(robot_name, vrep_interface)
{
   _set_names(robot_name);
}


/**
 * @brief  sets the joint_names_ and the link_names_ attributes.
 * 
 * @param robot_name The name of robot used on the vrep scene.
 */
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

    for(int i=1;i<dim_configuration_space_+1;i++)
    {
        std::string current_joint_name = robot_label + std::string("_joint") + std::to_string(i) + robot_index;
        joint_names_.push_back(current_joint_name);

        std::string current_link_name = robot_label + std::string("_link") + std::to_string(i+1)+
                                        std::string("_resp") + robot_index;
        link_names_.push_back(current_link_name);
    }
}

/**
 * @brief Protected method to check if the size of the vector of joint values is valid.
 * @param msg Message to be shown.
 * @param q_vec Vector of joint values.
 * @throws std::runtime_error when the size of  q_vec is invalid.
 */
void FrankaEmikaPandaVrepRobot::_check_q_vec(const std::string& msg, const VectorXd &q_vec) const
{
    if(q_vec.size() != dim_configuration_space_)
    {
        throw std::runtime_error(msg + std::string("Input vector must have size ") + std::to_string(dim_configuration_space_));
    }
}

/**
 * @brief Protected method to check if the index to a link is valid.
 * 
 * @param msg Message to be shown.
 * @param to_ith_link to_ith_link The index to a link.
 */
void FrankaEmikaPandaVrepRobot::_check_to_ith_link(const std::string& msg, const int &to_ith_link) const
{
    if(to_ith_link >= dim_configuration_space_ || to_ith_link < 0)
    {
        throw std::runtime_error(msg + std::string("Tried to access link index ") + std::to_string(to_ith_link) + std::string(" which is unnavailable."));
    }
}


/**
 * @brief 
 * 
 * @param q 
 */
void FrankaEmikaPandaVrepRobot::send_q_to_vrep(const VectorXd &q)
{
    _check_q_vec(std::string("Error in send_q_to_vrep. "), q);
    _get_interface_sptr()->set_joint_positions(joint_names_, q);
}


/**
 * @brief 
 * 
 * @param q_target 
 */
void FrankaEmikaPandaVrepRobot::send_q_target_to_vrep(const VectorXd& q_target)
{
    _check_q_vec(std::string("Error in send_q_target_to_vrep. "), q_target);
    _get_interface_sptr()->set_joint_target_positions(joint_names_, q_target);
}


/**
 * @brief 
 * 
 * @param q_dot_target 
 */
void FrankaEmikaPandaVrepRobot::send_q_dot_target_to_vrep(const VectorXd& q_dot_target)
{
   _check_q_vec(std::string("Error in send_q_dot_target_to_vrep. "), q_dot_target);
   _get_interface_sptr()->set_joint_target_velocities(joint_names_, q_dot_target);
}


/**
 * @brief 
 * 
 * @param torques_target 
 */
void FrankaEmikaPandaVrepRobot::send_torques_target_to_vrep(const VectorXd& torques_target)
{
   _check_q_vec(std::string("Error in send_torques_target_to_vrep. "), torques_target);
   _get_interface_sptr()->set_joint_torques(joint_names_, torques_target);
}


/**
 * @brief 
 * 
 * @param ith_joint 
 * @returns std::string 
 */
std::string FrankaEmikaPandaVrepRobot::get_joint_name(const int& ith_joint)
{
    _check_to_ith_link(std::string("Error in get_joint_name. "), ith_joint);
    return joint_names_[ith_joint];
}


/**
 * @brief 
 * 
 * @returns std::vector<std::string> 
 */
std::vector<std::string> FrankaEmikaPandaVrepRobot::get_joint_names()
{
    return joint_names_;
}


/**
 * @brief 
 * 
 * @param ith_joint 
 * @returns std::string 
 */
std::string FrankaEmikaPandaVrepRobot::get_link_name(const int& ith_joint)
{
    _check_to_ith_link(std::string("Error in get_link_name. "), ith_joint);
    return link_names_[ith_joint];
}


/**
 * @brief 
 * 
 * @returns std::vector<std::string> 
 */
std::vector<std::string> FrankaEmikaPandaVrepRobot::get_link_names()
{
    return link_names_;
}

/**
 * @brief 
 * 
 * @returns VectorXd 
 */
VectorXd FrankaEmikaPandaVrepRobot::get_q_from_vrep()
{
    return _get_interface_sptr()->get_joint_positions(joint_names_);
}


/**
 * @brief 
 * 
 * @returns VectorXd 
 */
VectorXd FrankaEmikaPandaVrepRobot::get_q_dot_from_vrep()
{
   return _get_interface_sptr()->get_joint_velocities(joint_names_);
}


/**
 * @brief 
 * 
 * @returns VectorXd 
 */
VectorXd FrankaEmikaPandaVrepRobot::get_torques_from_vrep()
{
    return _get_interface_sptr()->get_joint_torques(joint_names_);
}


/**
 * @brief 
 * 
 * @returns DQ_SerialManipulatorMDH 
 */
DQ_SerialManipulatorMDH FrankaEmikaPandaVrepRobot::kinematics()
{
    auto franka = FrankaEmikaPandaRobot::kinematics();
    franka.set_base_frame(franka.get_reference_frame()*_get_interface_sptr()->get_object_pose(robot_name_)*offset_);
    return franka;
}

}