// Z-axis pointing down, X-axis pointing forward
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <limits.h>
#include <cstdint>
#include <utility>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

#include "wrist_IK_FK.hpp"
using namespace Eigen;

// IK for left hand
Eigen::Vector2f solveIK_LH(Eigen::Vector2f roll_pitch) {

    float roll = roll_pitch(0);
    float pitch = roll_pitch(1);

    Matrix4f roll_trans;
    roll_trans << 1, 0, 0, 0,
                  0, 1, 0, 14,
                  0, 0, 1, 176.5,
                  0, 0, 0, 1;

    Matrix4f roll_rot;
    roll_rot << 1, 0,          0,          0,
                0, cos(roll), -sin(roll),  0,
                0, sin(roll),  cos(roll),  0,
                0, 0,          0,          1;

    Matrix4f pitch_trans;
    pitch_trans << 1, 0, 0, 0,
                  0, 1, 0, -18.1,
                  0, 0, 1, 11.5,
                  0, 0, 0, 1;

    Matrix4f pitch_rot;
    pitch_rot << cos(pitch),  0, sin(pitch), 0,
                 0,           1, 0,          0,
                 -sin(pitch), 0, cos(pitch),  0,
                 0,           0, 0,          1;
                
    Vector4f Sf_local;
    Sf_local << 17.95,
                0,
                -11.5,
                1;
          
    Vector4f Sb_local;
    Sb_local << -17.95,
                0,
                -11.5,
                1;

    Vector4f Sf_wrist = roll_trans * roll_rot * pitch_trans * pitch_rot * Sf_local;
    Vector4f Sb_wrist = roll_trans * roll_rot * pitch_trans * pitch_rot * Sb_local;

    Vector4f Pf;
    Pf << 24,
          0,
          14,
          1;

    Vector4f Pb;
    Pb << -24,
          0,
          14,
          1;

    float Lf = (Pf.head<3>() - Sf_wrist.head<3>()).norm() - 148.7;
    float Lb = (Pb.head<3>() - Sb_wrist.head<3>()).norm() - 148.7;

    // std::cout << "Lf: " << Lf << " | Lb: "<< Lb << "\n";
    Vector2f joint_length;
    joint_length << Lf,
                    Lb;
    return joint_length;
}

// Compute the Jacobian
Eigen::Matrix2f computeJacobian_LH(Eigen::Vector2f theta) {
    Eigen::Matrix2f J;

    float roll = theta(0);
    float pitch = theta(1);

    // Populate the J matrix
    J(0, 0) = ((1.0/2.0)*(-359.0/10.0*sin(pitch)*sin(roll) - 23*sin(roll)*cos(pitch) + 23*sin(roll) + (181.0/5.0)*cos(roll))*((359.0/20.0)*sin(pitch)*cos(roll) + (181.0/10.0)*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0) + (1.0/2.0)*(-359.0/10.0*sin(pitch)*cos(roll) - 181.0/5.0*sin(roll) - 23*cos(pitch)*cos(roll) + 23*cos(roll))*(-359.0/20.0*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) + (181.0/10.0)*cos(roll) - 14))/sqrt(pow((23.0/2.0)*sin(pitch) - 359.0/20.0*cos(pitch) + 24, 2) + pow(-359.0/20.0*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) + (181.0/10.0)*cos(roll) - 14, 2) + pow((359.0/20.0)*sin(pitch)*cos(roll) + (181.0/10.0)*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2));

    J(0, 1) = ((1.0/2.0)*(23*sin(pitch)*sin(roll) - 359.0/10.0*sin(roll)*cos(pitch))*(-359.0/20.0*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) + (181.0/10.0)*cos(roll) - 14) + (1.0/2.0)*(-23*sin(pitch)*cos(roll) + (359.0/10.0)*cos(pitch)*cos(roll))*((359.0/20.0)*sin(pitch)*cos(roll) + (181.0/10.0)*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0) + (1.0/2.0)*((359.0/10.0)*sin(pitch) + 23*cos(pitch))*((23.0/2.0)*sin(pitch) - 359.0/20.0*cos(pitch) + 24))/sqrt(pow((23.0/2.0)*sin(pitch) - 359.0/20.0*cos(pitch) + 24, 2) + pow(-359.0/20.0*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) + (181.0/10.0)*cos(roll) - 14, 2) + pow((359.0/20.0)*sin(pitch)*cos(roll) + (181.0/10.0)*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2));
    
    J(1, 0) = ((1.0/2.0)*((359.0/10.0)*sin(pitch)*sin(roll) - 23*sin(roll)*cos(pitch) + 23*sin(roll) + (181.0/5.0)*cos(roll))*(-359.0/20.0*sin(pitch)*cos(roll) + (181.0/10.0)*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0) + (1.0/2.0)*((359.0/10.0)*sin(pitch)*cos(roll) - 181.0/5.0*sin(roll) - 23*cos(pitch)*cos(roll) + 23*cos(roll))*((359.0/20.0)*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) + (181.0/10.0)*cos(roll) - 14))/sqrt(pow((23.0/2.0)*sin(pitch) + (359.0/20.0)*cos(pitch) - 24, 2) + pow((359.0/20.0)*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) + (181.0/10.0)*cos(roll) - 14, 2) + pow(-359.0/20.0*sin(pitch)*cos(roll) + (181.0/10.0)*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2));
    
    J(1, 1) = ((1.0/2.0)*(23*sin(pitch)*sin(roll) + (359.0/10.0)*sin(roll)*cos(pitch))*((359.0/20.0)*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) + (181.0/10.0)*cos(roll) - 14) + (1.0/2.0)*(-23*sin(pitch)*cos(roll) - 359.0/10.0*cos(pitch)*cos(roll))*(-359.0/20.0*sin(pitch)*cos(roll) + (181.0/10.0)*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0) + (1.0/2.0)*(-359.0/10.0*sin(pitch) + 23*cos(pitch))*((23.0/2.0)*sin(pitch) + (359.0/20.0)*cos(pitch) - 24))/sqrt(pow((23.0/2.0)*sin(pitch) + (359.0/20.0)*cos(pitch) - 24, 2) + pow((359.0/20.0)*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) + (181.0/10.0)*cos(roll) - 14, 2) + pow(-359.0/20.0*sin(pitch)*cos(roll) + (181.0/10.0)*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2));

    return J;
}

Eigen::Vector2f solveFK_LH(Eigen::Vector2f theta_init, Eigen::Vector2f q_meas) {
    const int max_iters = 20;
    const float tol = 1e-2;

    Eigen::Vector2f theta;
    theta << theta_init(0),
             theta_init(1);

    Eigen::Vector2f residual;
    Eigen::Matrix2f J;
    Eigen::Vector2f delta_theta;
    for (int iter = 0; iter < max_iters; ++iter){
        float roll = theta(0);
        float pitch = theta(1);

        double Lf_model = sqrt(pow((23.0/2.0)*sin(pitch) - 359.0/20.0*cos(pitch) + 24, 2) + pow(-359.0/20.0*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) + (181.0/10.0)*cos(roll) - 14, 2) + pow((359.0/20.0)*sin(pitch)*cos(roll) + (181.0/10.0)*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2)) - 1487.0/10.0;
        double Lb_model = sqrt(pow((23.0/2.0)*sin(pitch) + (359.0/20.0)*cos(pitch) - 24, 2) + pow((359.0/20.0)*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) + (181.0/10.0)*cos(roll) - 14, 2) + pow(-359.0/20.0*sin(pitch)*cos(roll) + (181.0/10.0)*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2)) - 1487.0/10.0;

        residual(0) = Lf_model - q_meas(0);
        residual(1) = Lb_model - q_meas(1);

        J = computeJacobian_LH(theta);

        delta_theta = -(J.transpose() * J).ldlt().solve(J.transpose() * residual);

        theta += delta_theta;

        if (delta_theta.norm() < tol) break;
    }
    
    return theta;
}

// IK for right hand
Eigen::Vector2f solveIK_RH(Eigen::Vector2f roll_pitch) {

    float roll = roll_pitch(0);
    float pitch = roll_pitch(1);

    Matrix4f roll_trans;
    roll_trans << 1, 0, 0, 0,
                  0, 1, 0, -14,
                  0, 0, 1, 176.5,
                  0, 0, 0, 1;

    Matrix4f roll_rot;
    roll_rot << 1, 0,          0,          0,
                0, cos(roll), -sin(roll),  0,
                0, sin(roll),  cos(roll),  0,
                0, 0,          0,          1;

    Matrix4f pitch_trans;
    pitch_trans << 1, 0, 0, 0,
                  0, 1, 0, 18.1,
                  0, 0, 1, 11.5,
                  0, 0, 0, 1;

    Matrix4f pitch_rot;
    pitch_rot << cos(pitch),  0, sin(pitch), 0,
                 0,           1, 0,          0,
                 -sin(pitch), 0, cos(pitch),  0,
                 0,           0, 0,          1;
                
    Vector4f Sf_local;
    Sf_local << 17.95,
                0,
                -11.5,
                1;
          
    Vector4f Sb_local;
    Sb_local << -17.95,
                0,
                -11.5,
                1;

    Vector4f Sf_wrist = roll_trans * roll_rot * pitch_trans * pitch_rot * Sf_local;
    Vector4f Sb_wrist = roll_trans * roll_rot * pitch_trans * pitch_rot * Sb_local;

    Vector4f Pf;
    Pf << 24,
          0,
          14,
          1;

    Vector4f Pb;
    Pb << -24,
          0,
          14,
          1;

    float Lf = (Pf.head<3>() - Sf_wrist.head<3>()).norm() - 148.7;
    float Lb = (Pb.head<3>() - Sb_wrist.head<3>()).norm() - 148.7;

    // std::cout << "Lf: " << Lf << " | Lb: "<< Lb << "\n";
    Vector2f joint_length;
    joint_length << Lf,
                    Lb;
    return joint_length;
}

// Compute the Jacobian
Eigen::Matrix2f computeJacobian_RH(Eigen::Vector2f theta) {
    Eigen::Matrix2f J;

    float roll = theta(0);
    float pitch = theta(1);

    // Populate the J matrix
    J(0, 0) = ((1.0/2.0)*(-359.0/10.0*sin(pitch)*sin(roll) - 23*sin(roll)*cos(pitch) + 23*sin(roll) - 181.0/5.0*cos(roll))*((359.0/20.0)*sin(pitch)*cos(roll) - 181.0/10.0*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0) + (1.0/2.0)*(-359.0/10.0*sin(pitch)*cos(roll) + (181.0/5.0)*sin(roll) - 23*cos(pitch)*cos(roll) + 23*cos(roll))*(-359.0/20.0*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) - 181.0/10.0*cos(roll) + 14))/sqrt(pow((23.0/2.0)*sin(pitch) - 359.0/20.0*cos(pitch) + 24, 2) + pow(-359.0/20.0*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) - 181.0/10.0*cos(roll) + 14, 2) + pow((359.0/20.0)*sin(pitch)*cos(roll) - 181.0/10.0*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2));
    
    J(0, 1) = ((1.0/2.0)*(23*sin(pitch)*sin(roll) - 359.0/10.0*sin(roll)*cos(pitch))*(-359.0/20.0*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) - 181.0/10.0*cos(roll) + 14) + (1.0/2.0)*(-23*sin(pitch)*cos(roll) + (359.0/10.0)*cos(pitch)*cos(roll))*((359.0/20.0)*sin(pitch)*cos(roll) - 181.0/10.0*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0) + (1.0/2.0)*((359.0/10.0)*sin(pitch) + 23*cos(pitch))*((23.0/2.0)*sin(pitch) - 359.0/20.0*cos(pitch) + 24))/sqrt(pow((23.0/2.0)*sin(pitch) - 359.0/20.0*cos(pitch) + 24, 2) + pow(-359.0/20.0*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) - 181.0/10.0*cos(roll) + 14, 2) + pow((359.0/20.0)*sin(pitch)*cos(roll) - 181.0/10.0*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2));
    
    J(1, 0) = ((1.0/2.0)*((359.0/10.0)*sin(pitch)*sin(roll) - 23*sin(roll)*cos(pitch) + 23*sin(roll) - 181.0/5.0*cos(roll))*(-359.0/20.0*sin(pitch)*cos(roll) - 181.0/10.0*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0) + (1.0/2.0)*((359.0/10.0)*sin(pitch)*cos(roll) + (181.0/5.0)*sin(roll) - 23*cos(pitch)*cos(roll) + 23*cos(roll))*((359.0/20.0)*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) - 181.0/10.0*cos(roll) + 14))/sqrt(pow((23.0/2.0)*sin(pitch) + (359.0/20.0)*cos(pitch) - 24, 2) + pow((359.0/20.0)*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) - 181.0/10.0*cos(roll) + 14, 2) + pow(-359.0/20.0*sin(pitch)*cos(roll) - 181.0/10.0*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2));
    
    J(1, 1) = ((1.0/2.0)*(23*sin(pitch)*sin(roll) + (359.0/10.0)*sin(roll)*cos(pitch))*((359.0/20.0)*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) - 181.0/10.0*cos(roll) + 14) + (1.0/2.0)*(-23*sin(pitch)*cos(roll) - 359.0/10.0*cos(pitch)*cos(roll))*(-359.0/20.0*sin(pitch)*cos(roll) - 181.0/10.0*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0) + (1.0/2.0)*(-359.0/10.0*sin(pitch) + 23*cos(pitch))*((23.0/2.0)*sin(pitch) + (359.0/20.0)*cos(pitch) - 24))/sqrt(pow((23.0/2.0)*sin(pitch) + (359.0/20.0)*cos(pitch) - 24, 2) + pow((359.0/20.0)*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) - 181.0/10.0*cos(roll) + 14, 2) + pow(-359.0/20.0*sin(pitch)*cos(roll) - 181.0/10.0*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2));
    
    return J;
}

// Solve FK
Eigen::Vector2f solveFK_RH(Eigen::Vector2f theta_init, Eigen::Vector2f q_meas) {
    const int max_iters = 20;
    const float tol = 1e-2;

    Eigen::Vector2f theta;
    theta << theta_init(0),
             theta_init(1);

    Eigen::Vector2f residual;
    Eigen::Matrix2f J;
    Eigen::Vector2f delta_theta;
    for (int iter = 0; iter < max_iters; ++iter){
        float roll = theta(0);
        float pitch = theta(1);

        double Lf_model = sqrt(pow((23.0/2.0)*sin(pitch) - 359.0/20.0*cos(pitch) + 24, 2) + pow(-359.0/20.0*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) - 181.0/10.0*cos(roll) + 14, 2) + pow((359.0/20.0)*sin(pitch)*cos(roll) - 181.0/10.0*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2)) - 1487.0/10.0;
        double Lb_model = sqrt(pow((23.0/2.0)*sin(pitch) + (359.0/20.0)*cos(pitch) - 24, 2) + pow((359.0/20.0)*sin(pitch)*sin(roll) - 23.0/2.0*sin(roll)*cos(pitch) + (23.0/2.0)*sin(roll) - 181.0/10.0*cos(roll) + 14, 2) + pow(-359.0/20.0*sin(pitch)*cos(roll) - 181.0/10.0*sin(roll) + (23.0/2.0)*cos(pitch)*cos(roll) - 23.0/2.0*cos(roll) - 325.0/2.0, 2)) - 1487.0/10.0;
        
        residual(0) = Lf_model - q_meas(0);
        residual(1) = Lb_model - q_meas(1);

        J = computeJacobian_RH(theta);

        delta_theta = -(J.transpose() * J).ldlt().solve(J.transpose() * residual);

        theta += delta_theta;

        if (delta_theta.norm() < tol) break;
    }
    
    return theta;
}

std::pair<float, float> solveFK(float roll_old, float pitch_old, float f_len, float b_len, Side side){
    Eigen::Vector2f wrist_length;
    Eigen::Vector2f wrist_angle;

    wrist_angle(0) = roll_old;
    wrist_angle(1) = pitch_old;

    wrist_length(0) = f_len;
    wrist_length(1) = b_len;

    if (side == Side::Left) wrist_angle = solveFK_LH(wrist_angle, wrist_length);
    else if (side == Side::Right) wrist_angle = solveFK_RH(wrist_angle, wrist_length);

    return { wrist_angle(0), wrist_angle(1) };
}

std::pair<float, float> solveIK(float roll, float pitch, Side side){
    Eigen::Vector2f wrist_angle;
    Eigen::Vector2f wrist_length;

    wrist_angle(0) = roll;
    wrist_angle(1) = pitch;

    if (side == Side::Left) wrist_length = solveIK_LH(wrist_angle);
    else if (side == Side::Right) wrist_length = solveIK_RH(wrist_angle);

    return { wrist_length(0), wrist_length(1) };
}