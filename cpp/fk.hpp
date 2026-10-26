#pragma once
#include <Eigen/Dense>
Eigen::Matrix3d Rz(double theta);
Eigen::Matrix3d Rx(double theta);
Eigen::Matrix3d Ry(double theta);
Eigen::Matrix3d rpy_R(double roll, double pitch, double yaw);
Eigen::Matrix4d origin_to_T(double x, double y, double z, double roll, double pitch, double yaw);
Eigen::Matrix4d Rz4(double theta);
Eigen::Matrix4d joint_T(double x, double y, double z, double roll, double pitch, double yaw, double theta);
Eigen::Vector3d fk_so101(double q1, double q2, double q3, double q4, double q5);