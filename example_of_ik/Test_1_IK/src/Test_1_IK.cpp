/*
 * Copyright (C) 2015 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
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

#include "Test_1_IK.h"


Test_1_IK::Test_1_IK()
:RobotInterface(){
}
Test_1_IK::~Test_1_IK(){
}

RobotInterface::Status Test_1_IK::RobotInit(){

	// resize the global variables
	mInitialJointPos.Resize(KUKA_DOF);
	mJointPos.Resize(KUKA_DOF);
	mJointKinematics.Resize(KUKA_DOF);
	mJointDesPos.Resize(KUKA_DOF);
	mJointTargetPos.Resize(KUKA_DOF);
	mJointDesVel.Resize(KUKA_DOF);
	mJointDesResidualVel.Resize(KUKA_DOF);
	mJointTorque.Resize(KUKA_DOF);
	lJointWeight.Resize(KUKA_DOF);
	lPos.Resize(3);
	lJoints.Resize(KUKA_DOF);
	lJacobianDirY.Resize(3,KUKA_DOF);
	lJacobianDirZ.Resize(3,KUKA_DOF);
	lTargetPos.Resize(3);
	lTDirection.Resize(3);

	mJointPosAll.Resize(KUKA_DOF+FINGER_DOF);
	mJointTorqueAll.Resize(KUKA_DOF+FINGER_DOF);

	// for inverse kinematics
	mJointVelLimitsUp.Resize(KUKA_DOF);
	mJointVelLimitsDn.Resize(KUKA_DOF);
	mJointVelocityLimitsDT.Resize(KUKA_DOF);
	mTargetVelocity.Resize(IK_CONSTRAINTS);

	// initialize sensor group
	mSensorsGroup.SetSensorsList(mRobot->GetSensors());
	mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());
	mSensorsGroup.ReadSensors();

	// Kinematic Chain for real robot
	mEndEffectorId = mRobot->GetLinksCount()-1;
	mKinematicChain.SetRobot(mRobot);
	mKinematicChain.Create(0,0,mEndEffectorId);

	// kinematic chain for virtual robot
	mSKinematicChain = new sKinematics(KUKA_DOF, _dt);

	// you should call sKin->setHD function lDofRobot times
	// @index               : starting from 0 to (num-1)
	// @a, d, alpha, theta0 : D-H Parameters
	// @min, max            : joint limits
	// @maxVel              : maximum joint speed

	mSKinematicChain->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD( -85.), DEG2RAD( 85.), DEG2RAD(98.0)*0.90);
	mSKinematicChain->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(98.0)*0.90);
	mSKinematicChain->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-100.), DEG2RAD(100.), DEG2RAD(100.0)*0.90);
	mSKinematicChain->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-110.), DEG2RAD(110.), DEG2RAD(130.0)*0.90);
	mSKinematicChain->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-140.), DEG2RAD(140.), DEG2RAD(140.0)*0.90);
	mSKinematicChain->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -90.), DEG2RAD( 90.), DEG2RAD(180.0)*0.90); // reduced joint angle to save the fingers
	//	mSKinematicChain->setDH(6, 0.0, 0.1260,    0.0, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)*0.90); // for sim lab
	mSKinematicChain->setDH(6, -0.05,   0.2290,    0.0, 0.0, 1,  DEG2RAD(-120.), DEG2RAD(120.), DEG2RAD(180.0)*0.90); // with Alegro Hand
	mSKinematicChain->readyForKinematics();
	// T0 is a transformation matrix from global basement to base coordinate of 0th links
	// T0 are allocated by Identity matrix default. (if you not call this function T0 = I )
	double T0[4][4];
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			T0[i][j] = 0.0;

	T0[0][0] = 1;
	T0[1][1] = 1;
	T0[2][2] = 1;
	T0[3][3] = 1;
	mSKinematicChain->setT0(T0);


	MathLib::Matrix3 mTF;
	double TF[4][4];
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			TF[i][j] = 0.0;
	TF[3][3] = 1;
	mTF = Matrix3::SRotationY(M_PI/4.0);

	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			TF[i][j] = mTF(i,j);

	//TF[1][3] = 0.03;

	// ready for kinematics
	mSKinematicChain->readyForKinematics();

	mSKinematicChain->setTF(TF);

	// variable for ik
	mJacobian3.Resize(3,KUKA_DOF);
	mJacobian6.Resize(6,KUKA_DOF);
	mJacobian9.Resize(9,KUKA_DOF);

	// Inverse kinematics
	mIKSolver.SetSizes(KUKA_DOF);  // Dof counts
	mIKSolver.AddSolverItem(IK_CONSTRAINTS);
	mIKSolver.SetVerbose(false);                // No comments
	mIKSolver.SetThresholds(0.0001,0.00001);    // Singularities thresholds
	mIKSolver.Enable(true,0);                   // Enable first solver
	mIKSolver.SetDofsIndices(mKinematicChain.GetJointMapping(),0); // Joint maps for first solver

	lJointWeight(0) = 0.35;
	lJointWeight(1) = 0.35;
	lJointWeight(2) = 0.35;
	lJointWeight(3) = 0.35;
	lJointWeight(4) = 1.0;
	lJointWeight(5) = 1.0;
	lJointWeight(6) = 1.0;

	mIKSolver.SetDofsWeights(lJointWeight);

	Vector lMaxVel(KUKA_DOF);
	mSKinematicChain->getMaxVel(lMaxVel.Array());
	mJointVelocityLimitsDT = lMaxVel*_dt;

	mJobJoints.Resize(KUKA_DOF);
	mJobJoints.Set(cJob, KUKA_DOF);
	mJointWeights.Resize(KUKA_DOF);



	AddConsoleCommand("test");
	AddConsoleCommand("job");
	mPlanner =NONE;

	return STATUS_OK;
}
RobotInterface::Status Test_1_IK::RobotFree(){
	return STATUS_OK;
}
RobotInterface::Status Test_1_IK::RobotStart(){

	return STATUS_OK;
}    
RobotInterface::Status Test_1_IK::RobotStop(){
	return STATUS_OK;
}
RobotInterface::Status Test_1_IK::RobotUpdate(){


	mSKinematicChain->setJoints(mJointKinematics.Array());
	mSKinematicChain->getEndPos(lPos.Array());
	mSKinematicChain->getEndDirAxis(AXIS_X, lDirX.Array());
	mSKinematicChain->getEndDirAxis(AXIS_Y, lDirY.Array());
	mSKinematicChain->getEndDirAxis(AXIS_Z, lDirZ.Array());
	mSKinematicChain->getJoints(mJointDesPos.Array());

	switch(mCommand){
	case COMMAND_TEST :
		mPlanner = PLANNER_CARTESIAN;
		break;
	case COMMAND_JOB:
		mPlanner = PLANNER_JOINT;
	}

	switch(mPlanner){
	case PLANNER_CARTESIAN:
		//lPos.Print("lPos");
		lTDirection(0)=1;
		lTDirection(1)=0.2;
		lTDirection(2)=-1;
		lTDirection.Mult(0.05*_dt,lTDirection);
		lTargetPos=lPos+lTDirection;
		lTargetDirY.Zero(); lTargetDirY(1)=1;
		lTargetDirZ.Zero(); lTargetDirZ(2)=1;
/*		lTargetDirY=lDirY;
		lTargetDirZ=lDirZ;*/
		//lTargetPos.Print("lTargetPos");
		// Set Jacobian
		mSKinematicChain->getJacobianPos(mJacobian3);
		mSKinematicChain->getJacobianDirection(AXIS_Y, lJacobianDirY);
		mSKinematicChain->getJacobianDirection(AXIS_Z, lJacobianDirZ);
		// Set maximum joint velocity
		for(int i=0;i<KUKA_DOF;i++){
			mJointVelLimitsDn(i) = -mSKinematicChain->getMaxVel(i);
			mJointVelLimitsUp(i) =  mSKinematicChain->getMaxVel(i);
		}
		mIKSolver.SetLimits(mJointVelLimitsDn,mJointVelLimitsUp);
		mJacobian3.Mult(1.0,mJacobian3);
		double lDirWeight=0.0;
		lJacobianDirY.Mult(lDirWeight,lJacobianDirY);
		lJacobianDirZ.Mult(lDirWeight,lJacobianDirZ);
		for(int i=0; i<3; i++){
			mJacobian9.SetRow(mJacobian3.GetRow(i)   , i  );
			mJacobian9.SetRow(lJacobianDirY.GetRow(i), i+3);
			mJacobian9.SetRow(lJacobianDirZ.GetRow(i), i+6);
		}
		mTargetVelocity.SetSubVector(0, (lTargetPos -lPos ) / _dt);
		mTargetVelocity.SetSubVector(3, (lTargetDirY-lDirY) / _dt );
		mTargetVelocity.SetSubVector(6, (lTargetDirZ-lDirZ) / _dt );
		mIKSolver.SetJacobian(mJacobian9);
		mIKSolver.SetTarget(mTargetVelocity, 0);
		//mIKSolver.SetNullTarget(mJobJoints);
		mIKSolver.Solve();
		mJointDesVel = mIKSolver.GetOutput();
		mSKinematicChain->getJoints(lJoints.Array());
		mJointDesPos = lJoints + mJointDesVel*_dt;
		mSKinematicChain->setJoints(mJointDesPos.Array());
		mSKinematicChain->getEndPos(lPos.Array());
		lTargetPos=lTargetPos-lPos;
		lTargetPos.Print("lTargetPos");
	}
	mSKinematicChain->setJoints(mJointDesPos.Array());
	mJointKinematics.Set(mJointDesPos);


	return STATUS_OK;
}
RobotInterface::Status Test_1_IK::RobotUpdateCore(){

	mSensorsGroup.ReadSensors();
	mJointPosAll    = mSensorsGroup.GetJointAngles();
	//mJointPosAll.Print("mJointPosAll");
	for(int i=0; i<KUKA_DOF; i++) mJointTargetPos(i)= mJointPosAll(i);
//	mJointTorqueAll = mSensorsGroup.GetJointTorques();
//	for(int i=0; i<KUKA_DOF; i++) mJointTorque(i)= mJointTorqueAll(i);

	if(mRobot->GetControlMode()!=Robot::CTRLMODE_POSITION)
			mRobot->SetControlMode(Robot::CTRLMODE_POSITION);
	mActuatorsGroup.SetJointAngles(mJointDesPos);
	mActuatorsGroup.WriteActuators();
	mKinematicChain.Update();


	return STATUS_OK;
}
int Test_1_IK::RespondToConsoleCommand(const string cmd, const vector<string> &args){
	cout<<"Write your command"<<endl;

	if(cmd=="job"){
		mCommand = COMMAND_JOB;
	}
	else if(cmd=="test"){
		mCommand = COMMAND_TEST;
	}

	return 0;
}


extern "C"{
// These two "C" functions manage the creation and destruction of the class
Test_1_IK* create(){return new Test_1_IK();}
void destroy(Test_1_IK* module){delete module;}
}

