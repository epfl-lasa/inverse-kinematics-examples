/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Seyed Sina Mirrazavi Salehian
 * email:   sina.mirrazavi@a3.epf.ch
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

#ifndef Test_1_IK_H_
#define Test_1_IK_H_

#include "RobotLib/RobotInterface.h"
#include "MathLib/MathLib.h"
#include "MathLib/IKGroupSolver.h"
#include "RobotLib/ForwardDynamics.h"
#include "RobotLib/InverseDynamics.h"
#include "RobotLib/KinematicChain.h"

#include "sKinematics.h"

#define KUKA_DOF 7
#define FINGER_DOF 0
#define IK_CONSTRAINTS 9
#define _dt (1.0/500.)
double cJob[]  = {0.0, -PI/4.0, 0.0, -PI/2.0, 0.0, -PI/4.0, 0.0};
enum ENUM_COMMAND{COMMAND_JOB,COMMAND_TEST};
enum ENUM_PLANNER{PLANNER_CARTESIAN, PLANNER_JOINT,NONE};
enum ENUM_AXIS{AXIS_X=0, AXIS_Y, AXIS_Z};




class Test_1_IK : public RobotInterface
{
public:
	Test_1_IK();
	virtual ~Test_1_IK();

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

	IKGroupSolver               mIKSolver;

	int                         mEndEffectorId;
	Vector                      mInitialJointPos;
	Vector                      mJointKinematics;
	Vector                      mJointPos;
	Vector                      mJointPosAll;
	Vector                      mJointDesPos;
	Vector                      mJointTargetPos;

	Vector                      mJointDesVel;
	Vector                      mJointDesResidualVel;

	Vector                      mJointTorque;
	Vector                      mJointTorqueAll;

	Vector                      mJointVelLimitsUp;
	Vector                      mJointVelLimitsDn;
	Vector       				mJointVelocityLimitsDT;
	Vector 						mTargetVelocity; // for inverse kinematics input
	Vector 						lJointWeight;
	ENUM_COMMAND 				mCommand;
	ENUM_PLANNER 				mPlanner;
	Vector 						lPos;
	Vector3 					lDirX, lDirY, lDirZ;
	Vector 						lJoints;
	Matrix 						lJacobianDirY;
	Matrix 						lJacobianDirZ;
	// Jacobian Variables
	Matrix 						mJacobian3;
	Matrix 						mJacobian6;
	Matrix 						mJacobian9;
	Vector 						mJobJoints;
	Vector						mreleaseJoints;
	Vector 						mJointWeights;
	Vector					    mreleaseJoints2;
	Vector 						lTargetPos;
	Vector						lTDirection;
	Vector3 					lTargetDirX;
	Vector3						lTargetDirY;
	Vector3 					lTargetDirZ;
};



#endif 
