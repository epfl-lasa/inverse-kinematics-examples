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

#include "Test_IK_QP.h"


Test_IK_QP::Test_IK_QP()
:RobotInterface(){
}
Test_IK_QP::~Test_IK_QP(){
}

RobotInterface::Status Test_IK_QP::RobotInit(){

	// resize the global variables
		mTargetVelocity.Resize(IK_CONSTRAINTS);
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
		lTargetPos.Resize(3);

		mJointPosAll.Resize(KUKA_DOF+FINGER_DOF);
		mJointTorqueAll.Resize(KUKA_DOF+FINGER_DOF);


	// for inverse kinematics
		mJointVelLimitsUp.Resize(KUKA_DOF);
		mJointVelLimitsDn.Resize(KUKA_DOF);
		mJointVelocityLimitsDT.Resize(KUKA_DOF);

	// initialize sensor group
		mSensorsGroup.SetSensorsList(mRobot->GetSensors());
		mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());
		mSensorsGroup.ReadSensors();
		mEndEffectorId = mRobot->GetLinksCount()-1;
		mKinematicChain.SetRobot(mRobot);
		mKinematicChain.Create(0,0,mEndEffectorId);

		// kinematic chain for virtual robot
		mSKinematicChain = new sKinematics(KUKA_DOF, _dt);

		//Constraints for the joints angles
		mThetaMinus.Resize(KUKA_DOF);
		mThetaPlus.Resize(KUKA_DOF);
		//constraint for the joint velocities
		Vector tmpMaxVel(KUKA_DOF);

		double SafetyRange = 0.8;

		mThetaMinus[0] = SafetyRange* DEG2RAD( -85.); mThetaPlus[0] = SafetyRange* DEG2RAD(85.); tmpMaxVel[0] = SafetyRange* DEG2RAD(110.0);
		mThetaMinus[1] = SafetyRange* DEG2RAD( -90.); mThetaPlus[1] = SafetyRange* DEG2RAD(90.); tmpMaxVel[1] = SafetyRange*DEG2RAD(110.0);
		mThetaMinus[2] = SafetyRange* DEG2RAD( -100.); mThetaPlus[2] = SafetyRange* DEG2RAD(100.); tmpMaxVel[2] = SafetyRange*DEG2RAD(128.0);
		mThetaMinus[3] = SafetyRange* DEG2RAD( -110.); mThetaPlus[3] = SafetyRange* DEG2RAD(110.); tmpMaxVel[3] = SafetyRange*DEG2RAD(128.0);
		mThetaMinus[4] = SafetyRange* DEG2RAD( -140.); mThetaPlus[4] = SafetyRange* DEG2RAD(140.); tmpMaxVel[4] = SafetyRange*DEG2RAD(204.0);
		mThetaMinus[5] = SafetyRange* DEG2RAD( -90.); mThetaPlus[5] = SafetyRange* DEG2RAD(90.); tmpMaxVel[5] = SafetyRange*DEG2RAD(184.0);
		mThetaMinus[6] = SafetyRange* DEG2RAD( -120.); mThetaPlus[6] = SafetyRange* DEG2RAD(120.); tmpMaxVel[6] = SafetyRange*DEG2RAD(184.0);

		//initial joints position of the robot
		Vector tmpInitPos(7);
		tmpInitPos(0) = 0.2460;
		tmpInitPos(1) = -1.4137;
		tmpInitPos(2) = -1.0231;
		tmpInitPos(3) = -1.7218;
		tmpInitPos(4) = -0.2202;
		tmpInitPos(5) = 0.5573;
		tmpInitPos(6) = -0.2399;

		//sets the DH parameters
		mSKinematicChain->setDH(0,  0.0,  0.34, M_PI_2, 0, 1,  mThetaMinus[0], mThetaPlus[0], tmpMaxVel[0]);
		mSKinematicChain->setDH(1,  0.0,  0.00,-M_PI_2, 0, 1,  mThetaMinus[1], mThetaPlus[1], tmpMaxVel[1]);
		mSKinematicChain->setDH(2,  0.0,  0.40,-M_PI_2, 0, 1,  mThetaMinus[2], mThetaPlus[2], tmpMaxVel[2]);
		mSKinematicChain->setDH(3,  0.0,  0.00, M_PI_2, 0, 1,  mThetaMinus[3], mThetaPlus[3], tmpMaxVel[3]);
		mSKinematicChain->setDH(4,  0.0,  0.40, M_PI_2, 0, 1,  mThetaMinus[4], mThetaPlus[4], tmpMaxVel[4]);
		mSKinematicChain->setDH(5,  0.0,  0.00,-M_PI_2, 0 , 1,  mThetaMinus[5], mThetaPlus[5], tmpMaxVel[5]);
		mSKinematicChain->setDH(6, -0.05,   0.2290,    0.0, 0, 1, mThetaMinus[6],mThetaPlus[6], tmpMaxVel[6]);
		mSKinematicChain->readyForKinematics();

		//For the kinematic chain
		double T0[4][4];
		for(int i=0; i<4; i++)
			for(int j=0; j<4; j++)
				T0[i][j] = 0.0;

		T0[0][0] = 1;
		T0[1][1] = 1;
		T0[2][2] = 1;
		T0[3][3] = 1;
		mSKinematicChain->setT0(T0);

		// ready for kinematics
		mSKinematicChain->readyForKinematics();
		mSKinematicChain->setJoints(tmpInitPos.Array());
		mJointKinematics.Set(tmpInitPos);

		// Sets the number of DOF of the joints and of the end effector (here 6)
		mSolver.setDimensions(KUKA_DOF, 6);

		//Sets the constraints for the angles and the angles velocities
		mSolver.setConstraints(mThetaMinus, mThetaPlus, tmpMaxVel*(-1), tmpMaxVel);




		//Sets the weights for the inertia matrix H
		lJointWeight(0) = 1.0;
		lJointWeight(1) = 1.0;
		lJointWeight(2) = 1.0;
		lJointWeight(3) = 1.0;
		lJointWeight(4) = 0.1;
		lJointWeight(5) = 0.1;
		lJointWeight(6) = 0.1;
		//
		Matrix tmpH(KUKA_DOF,KUKA_DOF);
		for(unsigned int i(0); i<KUKA_DOF;++i){
			tmpH(i,i) = lJointWeight[i];
		}
		//sets the inertia matrix
		mSolver.setH(tmpH);
		//sets the timestep
		mSolver.setDeltaT(_dt);
		//MuP is the  hyperparameter that tunes the avoidance of the joints limits
		mSolver.setMuP(25);
		//Gamma sets the size of the steps for the convergence (if gamma is larger than 200 it will be unstable for the KUKA)
		mSolver.setGamma(50);

		//variables used for the trajectory
		mTIME = 0;
		mStep = 0;


		AddConsoleCommand("test");
		mPlanner =NONE;

	return STATUS_OK;
}

RobotInterface::Status Test_IK_QP::RobotFree(){
	return STATUS_OK;
}
RobotInterface::Status Test_IK_QP::RobotStart(){

	return STATUS_OK;
}
RobotInterface::Status Test_IK_QP::RobotStop(){
	return STATUS_OK;
}
RobotInterface::Status Test_IK_QP::RobotUpdate(){

	//updates the kinematic chain
	mSKinematicChain->setJoints(mJointKinematics.Array());
	mSKinematicChain->getEndPos(lPos.Array());
	mSKinematicChain->getJoints(mJointDesPos.Array());

	//gets the jacobian from the kinematic chain and gives it to the solver
	Matrix tmpJ(6,KUKA_DOF);
	mSKinematicChain->getJacobian(tmpJ);
	mSolver.setJacobian(tmpJ);

	Vector TargetPos(6);


	if(mStep==0){
		//initial position of the end effector
		mInitPos = lPos;
		mStep = 1;
		mTIME=0;
	}
	if(mStep == 1){
		//The end effector goes to the desired position to follow the trajectory
		double TimeToInitialPosition(10);

		//Target position of the end effector
		TargetPos[0] = (1-mTIME/TimeToInitialPosition)* mInitPos[0] ;
		TargetPos[1] = (1-mTIME/TimeToInitialPosition)* mInitPos[1] + 0.5* mTIME/TimeToInitialPosition ;
		TargetPos[2] = (1-mTIME/TimeToInitialPosition)* mInitPos[2] + 0.6* mTIME/TimeToInitialPosition ;

		//Velocity of the end effector if the trajectory is perfectly followed, this is the (if possible analytical) time derivative of TargetPos
		mTargetVelocity[0] = (- mInitPos[0] )/TimeToInitialPosition ;
		mTargetVelocity[1] = (- mInitPos[1] + 0.5)/TimeToInitialPosition ;
		mTargetVelocity[2] = (- mInitPos[2] + 0.6)/TimeToInitialPosition ;

		if(mTIME>TimeToInitialPosition){
			mTIME=0;
			mStep = 2;
		}
	}
	if(mStep ==2){
		//"infinity motion

		//Proportional to the frequency of the loop
		double k = 0.4;

		TargetPos[0] = 0.5 *sin(sin(k*mTIME) *  sin(k*mTIME));
		TargetPos[1] = 0.5* cos(sin(k*mTIME) * sin(k*mTIME));
		TargetPos[2] = 0.6 + 0.2 * sin(4*k*mTIME);

		//the target velocity has been found with wolfram, numerical differentiation of TargetPos would also work
		mTargetVelocity[0] = 0.5*k*sin(2*k*mTIME) * cos(sin(k*mTIME)*sin(k*mTIME));
		mTargetVelocity[1] = -0.5*2*k*sin(k*mTIME) * sin(sin(k*mTIME)*sin(k*mTIME))*cos(k* mTIME);
		mTargetVelocity[2] = 4* 0.2 * k* cos(4*k*mTIME);
	}

	Vector tmpRdot(6);
	//Velocity of the end effector that will be given to the solver, it includes a gain to do positional tracking in addition to velocity tracking
	double gain (0.2);
	for(unsigned int i(0); i<3; ++i){
		tmpRdot[i] =  mTargetVelocity[i]+ gain*(TargetPos[i]- lPos[i])/_dt;
	}

	//Solves the inverse kinematics
	mSolver.Solve(tmpRdot, mSensorsGroup.GetJointAngles());

	//gets the joints velocity
	mJointDesVel = mSolver.getOutput();

	//update the position of the joints by forward Euler numerical integration
	mSKinematicChain->getJoints(lJoints.Array());
	mJointDesPos = lJoints + mJointDesVel*_dt;

	//sets the updated joints positions
	mSKinematicChain->setJoints(mJointDesPos.Array());
	mJointKinematics.Set(mJointDesPos);

	mTIME += _dt;

	return STATUS_OK;
}


RobotInterface::Status Test_IK_QP::RobotUpdateCore(){

	mSensorsGroup.ReadSensors();
	mJointPosAll    = mSensorsGroup.GetJointAngles();
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
int Test_IK_QP::RespondToConsoleCommand(const string cmd, const vector<string> &args){
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
Test_IK_QP* create(){return new Test_IK_QP();}
void destroy(Test_IK_QP* module){delete module;}
}

