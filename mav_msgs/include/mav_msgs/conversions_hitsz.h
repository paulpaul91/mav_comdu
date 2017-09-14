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

// Extended Conversion functions between Eigen types and MAV ROS message types.

#ifndef MAV_MSGS_CONVERSIONS_HITSZ_H
#define MAV_MSGS_CONVERSIONS_HITSZ_H

#include "mav_msgs/conversions.h"
#include "mav_msgs/eigen_mav_msgs_hitsz.h"
#include "mav_msgs/CommandMavAndGripper.h"
#include "mav_msgs/ImageFeatureArmServoAngleTrajectory.h"

namespace mav_msgs {

// Convenience method to quickly create a trajectory from a single waypoint.
inline void msgMultiDofJointTrajectoryFromPositionVelocityYaw(
  const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, double yaw,
  trajectory_msgs::MultiDOFJointTrajectory* msg) {
  assert(msg != NULL);

  EigenTrajectoryPoint point;
  point.position_W = position;
  point.velocity_W = velocity;
  point.setFromYaw(yaw);

  msgMultiDofJointTrajectoryFromEigen(point, msg);
}

inline void eigenGripperAngleFromMsg(const CommandArmServoAngle& msg,
                                     EigenGripperAngle* gripper_angle){
    assert(gripper_angle != NULL);
    gripper_angle->gripper_angle << msg.beta,
            msg.dbeta,
            msg.d2beta,
            msg.d3beta,
            msg.d4beta;
    gripper_angle->grasp_or_not = msg.grasp_or_not;

}

inline void msgCommandArmAngleFromEigen(const EigenGripperAngle &gripper_point,
                                        CommandArmServoAngle *msg){
    assert(msg != NULL);
    msg->beta = gripper_point.gripper_angle(0);
    msg->dbeta = gripper_point.gripper_angle(1);
    msg->d2beta = gripper_point.gripper_angle(2);
    msg->d3beta = gripper_point.gripper_angle(3);
    msg->d4beta = gripper_point.gripper_angle(4);
    msg->grasp_or_not = gripper_point.grasp_or_not;
}

inline void msgCommandArmAngleTrajectoryFromEigen(const EigenGripperAngle& gripper_point,
                                                  mav_msgs::CommandArmServoAngleTrajectory* msg){
    assert(msg != NULL);
    mav_msgs::CommandArmServoAngle point_msg;
    msgCommandArmAngleFromEigen(gripper_point, &point_msg);
    msg->trajectory.clear();
    msg->trajectory.push_back(point_msg);
}

inline void msgVFromEigen(const Eigen::Vector2d& v, mav_msgs::ImageFeatureV* msg)
{
    msg->v1 = v[0];
    msg->v2 = v[1];
}

inline void eigenVFromMsg(const mav_msgs::ImageFeatureV& msg, Eigen::Vector2d* v)
{
    *v << msg.v1, msg.v2;
}

inline void msgImageTrajectoryPointFromEigen(const EigenImageFeatureV& v, 
                            mav_msgs::ImageFeatureArmServoAngle* msg)
{
    msgVFromEigen(v.position, &msg->desired_v_position);
    msgVFromEigen(v.velocity, &msg->desired_v_velocity);
    msgVFromEigen(v.acceleration, &msg->desired_v_acceleration);
    msgVFromEigen(v.jerk, &msg->desired_v_jerk);
    msgVFromEigen(v.snap, &msg->desired_v_snap);
}

inline void eigenImageFeatureVFromMsg(const mav_msgs::ImageFeatureArmServoAngle& msg,
                            EigenImageFeatureV* v)
{
    assert(v != NULL);

    eigenVFromMsg(msg.desired_v_position, &v->position);
    eigenVFromMsg(msg.desired_v_velocity, &v->velocity);
    eigenVFromMsg(msg.desired_v_acceleration, &v->acceleration);
    v->jerk.setZero();
    v->snap.setZero();
}

inline void msgImageTrajectoryFromEigen(const EigenImageFeatureVDeque& trajectory, 
                            mav_msgs::ImageFeatureArmServoAngleTrajectory* msg)
{
    msg->desired_trajectory.clear();

    for (const auto& trajectory_point : trajectory) 
    {
        mav_msgs::ImageFeatureArmServoAngle point_msg;
        msgImageTrajectoryPointFromEigen(trajectory_point, &point_msg);
        msg->desired_trajectory.push_back(point_msg);
    }
}

} // end namespace mav_msgs

#endif // MAV_MSGS_CONVERSIONS_HITSZ_H

