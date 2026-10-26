#include"fk.hpp"
#include <cmath>

//##################rpy函数####################
//#工具函数
Eigen::Matrix3d Rz(double theta){
    double c = std::cos(theta);
    double s = std::sin(theta);
    Eigen::Matrix3d Rz ;
    Rz <<c,-s,0,
        s,c,0,
        0,0,1;
    return  Rz;

}
Eigen::Matrix3d Rx(double theta){
    double c = std::cos(theta);
    double s = std::sin(theta);
    Eigen::Matrix3d Rx ;
    Rx <<1,0,0,
        0,c,-s,
        0,s,c;
    return  Rx;
}

Eigen::Matrix3d Ry(double theta){
    double c = std::cos(theta);
    double s = std::sin(theta);
    Eigen::Matrix3d Ry ;
    Ry <<c,0,s,
        0,1,0,
        -s,0,c;
    return  Ry;
}

Eigen::Matrix3d rpy_R(double roll,double pitch,double yaw){
    return Rz(yaw)*Ry(pitch)*Rx(roll);
}
//构建齐次方程最后一行0 0 0 1是用来占位

Eigen::Matrix4d origin_to_T(double x,double y,double z,double roll,double pitch,double yaw){
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = rpy_R(roll,pitch,yaw);
    T.block<3,1>(0,3) << x,y,z;
    return T;
}
//构建joint_T包括rpy,xyz 最后绕z轴变换
Eigen::Matrix4d Rz4(double theta){
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = Rz(theta);
    return T;
}


Eigen::Matrix4d joint_T(double x,double y,double z,double roll,double pitch,double yaw,double theta){
    Eigen::Matrix4d T = origin_to_T(x,y,z,roll,pitch, yaw)*Rz4(theta);
    return T;


}

//写FK链
/*
1. shoulder_pan：xyz (0.038835, 0, 0.0624)，rpy (π, 0, −π)
2. shoulder_lift：xyz (−0.030399, −0.018278, −0.0542)，rpy (−π/2, −π/2, 0)
3. elbow_flex：xyz (−0.11257, −0.028, 0)，rpy (0, 0, π/2)
4. wrist_flex：xyz (−0.1349, 0.0052, 0)，rpy (0, 0, −π/2)
5. wrist_roll：xyz (0, −0.0611, 0.0181)，rpy (π/2, 0.04868, π)
6. gripper_frame_joint（固定）：xyz (−0.0079, −0.000218, −0.098127)，rpy (0, π, 0)
*/
Eigen::Vector3d fk_so101(double q1, double q2, double q3, double q4, double q5){
    const double PI = std::acos(-1.0);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(); 
    T = T*joint_T(0.038835, 0, 0.0624,PI, 0, -PI,q1);
    T = T*joint_T(-0.030399, -0.018278, -0.0542,-PI/2, -PI/2, 0,q2);
    T = T*joint_T(-0.11257, -0.028, 0,0,0, PI/2, q3);
    T = T*joint_T(-0.1349, 0.0052, 0,0,0, -PI/2, q4);
    T = T*joint_T(0, -0.0611, 0.0181,PI/2, 0.04868, PI, q5);
    T = T * origin_to_T(-0.0079, -0.000218, -0.098127,0, PI, 0);
    return T.block<3,1>(0,3);
}