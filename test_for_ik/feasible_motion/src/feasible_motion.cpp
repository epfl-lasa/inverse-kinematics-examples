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

#include "feasible_motion.h"


feasible_motion::feasible_motion()
:RobotInterface(){
}
feasible_motion::~feasible_motion(){
}

RobotInterface::Status feasible_motion::RobotInit(){
	// resize the global variables
	mInitialJointPos.Resize(KUKA_DOF);
	mJointPos.Resize(KUKA_DOF);
	mJointKinematics.Resize(KUKA_DOF);
	mJointDesPos.Resize(KUKA_DOF);
	mJointDesPos_old.Resize(KUKA_DOF);
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
	double Rgain=0.90;
	mSKinematicChain->setDH(0,  0.0,  0.34, M_PI_2, 0.0, 1,  DEG2RAD( -170.)*Rgain, DEG2RAD( 170.)*Rgain, DEG2RAD(110.0)*Rgain);
	mSKinematicChain->setDH(1,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -120.)*Rgain, DEG2RAD( 120.)*Rgain, DEG2RAD(110.0)*Rgain);
	mSKinematicChain->setDH(2,  0.0,  0.40,-M_PI_2, 0.0, 1,  DEG2RAD(-170.)*Rgain, DEG2RAD(170.)*Rgain, DEG2RAD(128.0)*Rgain);
	mSKinematicChain->setDH(3,  0.0,  0.00, M_PI_2, 0.0, 1,  DEG2RAD(-120.)*Rgain, DEG2RAD(120.)*Rgain, DEG2RAD(128.0)*Rgain);
	mSKinematicChain->setDH(4,  0.0,  0.40, M_PI_2, 0.0, 1,  DEG2RAD(-170.)*Rgain, DEG2RAD(170.)*Rgain, DEG2RAD(204.0)*Rgain);
	mSKinematicChain->setDH(5,  0.0,  0.00,-M_PI_2, 0.0, 1,  DEG2RAD( -120.)*Rgain, DEG2RAD( 120.)*Rgain, DEG2RAD(184.0)*Rgain); // reduced joint angle to save the fingers
	mSKinematicChain->setDH(6, -0.05,   0.2290,    0.0, 0.0, 1,  DEG2RAD(-170.)*Rgain, DEG2RAD(170.)*Rgain, DEG2RAD(184.0)*Rgain); // with Alegro Hand
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

	lJointWeight(0) = 1.0;
	lJointWeight(1) = 1.0;
	lJointWeight(2) = 1.0;
	lJointWeight(3) = 1.0;
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
	AddConsoleCommand("New_Point");
	mPlanner =NONE;

	Motion_counter=0;
	return STATUS_OK;
}
RobotInterface::Status feasible_motion::RobotFree(){
	return STATUS_OK;
}
RobotInterface::Status feasible_motion::RobotStart(){
	return STATUS_OK;
}    
RobotInterface::Status feasible_motion::RobotStop(){
	return STATUS_OK;
}
RobotInterface::Status feasible_motion::RobotUpdate(){

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
	}
	mSKinematicChain->setJoints(mJointDesPos.Array());
	mJointKinematics.Set(mJointDesPos);
	if ((mJointDesPos_old-mJointDesPos).Norm2()>0.000001)
	{
		Positionofdynamicalsystem<<mJointDesPos(0)<<" "<<mJointDesPos(1)<<" "<<mJointDesPos(2)<<" "<<mJointDesPos(3)<<" "
				<<mJointDesPos(4)<<" "<<mJointDesPos(5)<<" "<<mJointDesPos(6)<<" "
				<<lPos(0)<<" "<<lPos(1)<<" "<<lPos(2)<<" "
				<<lDirX(0)<<" "<<lDirX(1)<<" "<<lDirX(2)<<" "
				<<lDirY(0)<<" "<<lDirY(1)<<" "<<lDirY(2)<<" "
				<<lDirZ(0)<<" "<<lDirZ(1)<<" "<<lDirZ(2)<<" "
				<<endl;
		mJointDesPos_old=mJointDesPos;
	}

	return STATUS_OK;
}
RobotInterface::Status feasible_motion::RobotUpdateCore(){


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
int feasible_motion::RespondToConsoleCommand(const string cmd, const vector<string> &args){
	cout<<"Write your command"<<endl;

	if(cmd=="job"){
		mJointDesPos.Zero();
		mSKinematicChain->setJoints(mJointDesPos.Array());
		mSKinematicChain->getEndDirAxis(AXIS_Y, lTargetDirY.Array());
		mSKinematicChain->getEndDirAxis(AXIS_Z, lTargetDirZ.Array());
		mJointKinematics.Set(mJointDesPos);
		mActuatorsGroup.SetJointAngles(mJointDesPos);
		mActuatorsGroup.WriteActuators();
		mKinematicChain.Update();
		if (Positionofdynamicalsystem.is_open())
		{
			Positionofdynamicalsystem.close();
		}
		mCommand = COMMAND_JOB;
	}
	else if(cmd=="test"){
		mCommand = COMMAND_TEST;
	}
	else if(cmd=="New_Point"){
		srand(time(NULL));
		Motion_counter=Motion_counter+1;
		char Outputfile[256];
		if (Positionofdynamicalsystem.is_open())
		{
			Positionofdynamicalsystem.close();
		}
		sprintf(Outputfile,"TheobjectPosition%d.txt",Motion_counter);
		Positionofdynamicalsystem.open(Outputfile);
		double A=rand() % 100;
		lTargetPos(0)= (A/100-0.5);
		A=rand() % 100;
		lTargetPos(1)= (A/100-0.5);
		A=rand() % 100;
		lTargetPos(2)= A/400+0.5;
		lTargetPos.Print("lTargetPos");
		lTargetDirY.Print("lTargetDirY");
		lTargetDirZ.Print("lTargetDirZ");
	}

	return 0;
}


extern "C"{
// These two "C" functions manage the creation and destruction of the class
feasible_motion* create(){return new feasible_motion();}
void destroy(feasible_motion* module){delete module;}
}

