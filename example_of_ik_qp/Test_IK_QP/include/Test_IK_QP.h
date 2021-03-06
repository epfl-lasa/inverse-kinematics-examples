/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef Test_IK_QP_H_
#define Test_IK_QP_H_

#include "RobotLib/RobotInterface.h"
#include "MathLib/MathLib.h"
#include "MathLib/IKGroupSolver.h"
#include "RobotLib/ForwardDynamics.h"
#include "RobotLib/InverseDynamics.h"

#include "RobotLib/KinematicChain.h"
#include "sensor_msgs/JointState.h"

#include "std_msgs/Float64.h"

#include "ros/ros.h"

#include <sstream>
#include <vector>

#include "sKinematics.h"

#define KUKA_DOF 7
#define FINGER_DOF 0
#define IK_CONSTRAINTS 9
#define _dt (1.0/500.0)
double cJob[]  = {0.0, -PI/4.0, 0.0, -PI/2.0, 0.0, -PI/4.0, 0.0};
enum ENUM_COMMAND{COMMAND_JOB,COMMAND_TEST, NOCOMMAND};
enum ENUM_PLANNER{PLANNER_CARTESIAN, PLANNER_JOINT,NONE};



class Test_IK_QP : public RobotInterface
{
public:
	Test_IK_QP();
	virtual ~Test_IK_QP();

	virtual Status              RobotInit();
	virtual Status              RobotFree();

	virtual Status              RobotStart();
	virtual Status              RobotStop();

	virtual Status              RobotUpdate();
	virtual Status              RobotUpdateCore();

	virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
protected:
	sKinematics                 *mSKinematicChain;

	RevoluteJointSensorGroup    mSensorsGroup;
	RevoluteJointActuatorGroup  mActuatorsGroup;
	KinematicChain              mKinematicChain;

	//Declaration fo the solver
	MKESolver					mSolver;

	Vector 						mTargetPos;
	Vector 						mTargetVel;


	MathLib::Vector 			mInitPos;

	MathLib::Vector 			mThetaMinus;
	MathLib::Vector				mThetaPlus;


	int                  		mEndEffectorId;
	MathLib::Vector             mInitialJointPos;
	MathLib::Vector             mJointKinematics;
	MathLib::Vector             mJointPos;
	MathLib::Vector             mJointPosAll;
	MathLib::Vector             mJointDesPos;
	MathLib::Vector             mJointTargetPos;

	MathLib::Vector             mJointDesVel;
	MathLib::Vector             mJointDesResidualVel;

	MathLib::Vector             mJointTorque;
	MathLib::Vector             mJointTorqueAll;

	MathLib::Vector             mJointVelLimitsUp;
	MathLib::Vector             mJointVelLimitsDn;
	MathLib::Vector       		mJointVelocityLimitsDT;
	MathLib::Vector 			mTargetVelocity; // for inverse kinematics input
	MathLib::Vector 			lJointWeight;
	ENUM_COMMAND 				mCommand;
	ENUM_PLANNER 				mPlanner;


	MathLib::Vector 			lPos;
	MathLib::Vector 			lJoints;

	// Jacobian Variables
	MathLib::Vector 					lTargetPos;
/*	MathLib::Matrix 					mJacobian3;
	MathLib::Matrix 					mJacobian6;
	MathLib::Matrix 					mJacobian9;
	MathLib::Vector 					mJobJoints;
	MathLib::Vector						mreleaseJoints;
	MathLib::Vector 					mJointWeights;
	MathLib::Vector					    mreleaseJoints2;

	MathLib::Vector						lTDirection;
	MathLib::Vector3 					lTargetDirX;
	MathLib::Vector3					lTargetDirY;
	MathLib::Vector3 					lTargetDirZ;*/

	double 						mTIME;
	unsigned int 				mStep;
};



#endif 
