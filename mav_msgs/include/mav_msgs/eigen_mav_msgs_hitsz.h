/*
 * Copyright 2016 Yegui Du, NRSL, HITSZ, Shenzhen, China
 * Copyright 2015 Xuancong Li, HITSZ, Shenzhen, China
 * Copyright 2015 Haoyao Chen, HITSZ, Shenzhen/ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAV_MSGS_EIGEN_MAV_MSGS_HITSZ_H
#define MAV_MSGS_EIGEN_MAV_MSGS_HITSZ_H

#include "mav_msgs/eigen_mav_msgs.h"

namespace mav_msgs {

struct EigenGripperAngle{
    EigenGripperAngle()
        :gripper_angle(Eigen::VectorXd::Zero(5,1)),
         grasp_or_not(false){};
    EigenGripperAngle(const Eigen::Vector4d& _gripper_angle, bool _grasp_or_not){
        gripper_angle = _gripper_angle;
        grasp_or_not = _grasp_or_not;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::VectorXd gripper_angle;
    bool grasp_or_not;
};

struct EigenImageFeatureV {
  EigenImageFeatureV()
      : position(Eigen::Vector2d::Zero()),
      velocity(Eigen::Vector2d::Zero()),
      acceleration(Eigen::Vector2d::Zero()),
      jerk(Eigen::Vector2d::Zero()),
      snap(Eigen::Vector2d::Zero()) {};

  EigenImageFeatureV(
                const Eigen::Vector2d& _position,
                const Eigen::Vector2d& _velocity,
                const Eigen::Vector2d& _acceleration,
                const Eigen::Vector2d& _jerk,
                const Eigen::Vector2d& _snap)
      : position(_position),
      velocity(_velocity),
      acceleration(_acceleration),
      jerk(_jerk),
      snap(_snap) {}

  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  Eigen::Vector2d acceleration;
  Eigen::Vector2d jerk;
  Eigen::Vector2d snap;
};

MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenImageFeatureV)
MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenGripperAngle)

}

#endif // MAV_MSGS_EIGEN_MAV_MSGS_HITSZ_H
