#ifndef KDLROBOT
#define KDLROBOT

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/solveri.hpp>

#include <stdio.h>
#include <iostream>
#include <sstream>

#include "utils.h"

class KDLRobot
{

public:

    // robot
    KDLRobot();
    KDLRobot(KDL::Tree &robot_tree);
    void update(std::vector<double> _jnt_values,std::vector<double> _jnt_vel);
    void addEE(const KDL::Frame &_f_tip);
    void setJntLimits(KDL::JntArray &q_low, KDL::JntArray &q_high);

    // Chain getter functions
    unsigned int getNrJnts() const {return n_;}
    unsigned int getNrSgmts() const {return chain_.getNrOfSegments();}

    // Joints getter functions
    Eigen::VectorXd getJntPositions() const {return jntPos_.data;}
    Eigen::VectorXd getJntVelocities() const {return jntVel_.data;}
    Eigen::MatrixXd getJntLimits();

    // Dynamics getter functions
    Eigen::MatrixXd getJsim() const {return jsim_.data;}
    Eigen::VectorXd getCoriolis() const {return coriol_.data;}
    Eigen::VectorXd getGravity() const {return grav_.data;}
    Eigen::VectorXd getID(const KDL::JntArray &q,
                          const KDL::JntArray &q_dot,
                          const KDL::JntArray &q_dotdot,
                          const KDL::Wrenches &f_ext);

    // End-effector getter functions
    KDL::Frame getEEFrame() const {return s_F_ee_;}
    KDL::Twist getEEVelocity() const {return s_V_ee_;}

    // Jacobians getter functions
    KDL::Jacobian getEEJacobian() const {return s_J_ee_;}
    KDL::Jacobian getEEBodyJacobian() const {return b_J_ee_;}
    Eigen::MatrixXd getEEJacDotqDot()const { return s_J_dot_ee_.data;}

    // inverse kinematics
    void getInverseKinematics(KDL::Frame &f, KDL::JntArray &q);
    void getInverseKinematicsVel(KDL::Frame &f, KDL::Twist &twist, KDL::JntArray &q, KDL::JntArray &q_dot);

private:

    // chain
    unsigned int n_;
    void createChain(KDL::Tree &robot_tree);
    KDL::Chain chain_;

    KDL::ChainDynParam* dynParam_;
    KDL::ChainJntToJacSolver* jacSol_;
    KDL::ChainFkSolverPos_recursive* fkSol_;
    KDL::ChainFkSolverVel_recursive* fkVelSol_;
    KDL::ChainIkSolverPos_NR_JL* ikSol_;
    KDL::ChainIkSolverVel_pinv* ikVelSol_;
 
    KDL::ChainJntToJacDotSolver* jntJacDotSol_;
    KDL::ChainIdSolver_RNE* idSolver_;

    // joints
    void updateJnts(std::vector<double> _jnt_values, std::vector<double> _jnt_vel);

    KDL::JntArray jntPos_;          // joint positions
    KDL::JntArray jntVel_;          // joint velocities
    KDL::JntArray q_min_;           // joint positions lower limits
    KDL::JntArray q_max_;           // joint positions upper limits

    KDL::JntSpaceInertiaMatrix jsim_;   // inertia matrix
    KDL::JntArray coriol_;              // coriolis matrix
    KDL::JntArray grav_;                // gravity vector

    // end-effector
    KDL::Frame f_F_ee_;             // end-effector frame in flange frame
    KDL::Frame s_F_ee_;             // end-effector frame in spatial frame
    KDL::Twist s_V_ee_;             // end-effector twist in spatial frame
    KDL::Jacobian s_J_ee_;          // end-effector Jacobian in spatial frame
    KDL::Jacobian b_J_ee_;          // end-effector Jacobian in body frame
    KDL::Jacobian s_J_dot_ee_;      // end-effector Jacobian dot in spatial frame
    KDL::Jacobian b_J_dot_ee_;      // end-effector Jacobian dot in body frame
    KDL::Twist s_J_dot_q_dot_ee_;   // end-effector Jdot*qdot in spatial frame

    std::string strError(const int error);

};

#endif