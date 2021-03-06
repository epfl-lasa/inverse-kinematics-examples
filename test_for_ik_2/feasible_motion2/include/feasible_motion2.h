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

#ifndef feasible_motion_H_
#define feasible_motion_H_


#include "RobotLib/RobotInterface.h"
#include "MathLib/MathLib.h"
#include "MathLib/IKGroupSolver.h"
#include "RobotLib/ForwardDynamics.h"
#include "RobotLib/InverseDynamics.h"
#include "RobotLib/KinematicChain.h"
#include <time.h>

#include "sKinematics.h"

#define KUKA_DOF 7
#define FINGER_DOF 0
#define IK_CONSTRAINTS 9
#define _dt (1.0/500.)
enum ENUM_COMMAND{COMMAND_NON,COMMAND_JOB,COMMAND_TEST};
enum ENUM_PLANNER{PLANNER_CARTESIAN, PLANNER_JOINT,NONE};
enum ENUM_AXIS{AXIS_X=0, AXIS_Y, AXIS_Z};


class feasible_motion2 : public RobotInterface
{
public:
	feasible_motion2();
	virtual ~feasible_motion2();

	virtual Status              RobotInit();
	virtual Status              RobotFree();

	virtual Status              RobotStart();
	virtual Status              RobotStop();

	virtual Status              RobotUpdate();
	virtual Status              RobotUpdateCore();

	virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
private:
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
	Vector						mJointDesPos_old;

	Vector                      mJointDesVel;
	Vector                      mJointDesResidualVel;

	Vector                      mJointTorque;
	Vector                      mJointTorqueAll;

	Vector 						lJointWeight;
	ENUM_COMMAND 				mCommand;
	ENUM_PLANNER 				mPlanner;
	Vector 						lPos;
	Vector3 					lDirX, lDirY, lDirZ;
	Vector 						lJoints;
	// Jacobian Variables
	Vector 						lTargetPos;
	Vector						lTDirection;
	Vector3 					lTargetDirX;
	Vector3						lTargetDirY;
	Vector3 					lTargetDirZ;

	int 						Motion_counter;
	std::ofstream 				Positionofdynamicalsystem;


	Matrix                      DesiredFrame;

	bool						IK;
	bool						solve_inverse_kinematic(sKinematics *Chain,IKGroupSolver  Solver, Vector q_in, Vector &q_out, Matrix Final_transformation);
};



#endif 
