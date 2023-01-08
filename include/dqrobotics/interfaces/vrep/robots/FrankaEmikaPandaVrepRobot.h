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

#pragma once
#include <vector>
#include <dqrobotics/interfaces/vrep/DQ_VrepRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{
class FrankaEmikaPandaVrepRobot: public DQ_VrepRobot
{
  private:
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::string base_frame_name_;
    DQ offset_ = 1+0.5*E_*(-0.07*k_);
    void _set_names(const std::string& robot_name);
  public:
    FrankaEmikaPandaVrepRobot(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr);
    void send_q_to_vrep(const VectorXd &q) override;
    void send_q_target_to_vrep(const VectorXd& q_target);
    void send_q_dot_target_to_vrep(const VectorXd& q_dot_target);
    void send_torques_target_to_vrep(const VectorXd& torques_target);
    VectorXd get_q_from_vrep() override;
    VectorXd get_q_dot_from_vrep();
    DQ get_robot_base_from_vrep(const DQ& base);

    DQ_SerialManipulatorMDH kinematics();
};
}