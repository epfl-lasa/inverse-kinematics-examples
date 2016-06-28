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

#include "feasible_motion2.h"



bool feasible_motion2::solve_inverse_kinematic(sKinematics *_Chain,IKGroupSolver _Solver, Vector _q_in, Vector &_q_out, Matrix _Final_transformation)
{
	if ((_Final_transformation.RowSize()!=4)&&(_Final_transformation.ColumnSize()!=4))
	{
		return false;
		cout<<"The size of the target is not correct"<<endl;
	}

	if ((_q_in.Size()!=7))
	{
		return false;
		cout<<"The size of the target is not correct"<<endl;
	}

	if ((_Final_transformation.RowSize()!=4)&&(_Final_transformation.ColumnSize()!=4))
	{
		return false;
		cout<<"The size of the target is not correct"<<endl;
	}

	int _AXIS_X=0; int _AXIS_Y=1; int _AXIS_Z=2; int _KUKA_DOF=7;
	Vector _Pos;				_Pos.Resize(3);
	Vector _DirY;				_DirY.Resize(3);
	Vector _DirZ;				_DirZ.Resize(3);
	Vector _TargetPos;			_TargetPos.Resize(3);
	Vector _TargetDirY;			_TargetDirY.Resize(3);
	Vector _TargetDirZ;			_TargetDirZ.Resize(3);
	Vector _FinalPos;			_FinalPos.Resize(3);
	Vector _FinalDirY;			_FinalDirY.Resize(3);
	Vector _FinalDirZ;			_FinalDirZ.Resize(3);
	Vector  _JointDesPos;		_JointDesPos.Resize(7);
	Vector  _Joints;			_Joints.Resize(7);
	Vector  _JointDesVel;		_JointDesVel.Resize(7);

	Vector 	_TargetVelocity;	_TargetVelocity.Resize(9);
	Vector	_error;				_error.Resize(9);

	Vector _JointVelLimitsDn;	_JointVelLimitsDn.Resize(_KUKA_DOF);
	Vector _JointVelLimitsUp;	_JointVelLimitsUp.Resize(_KUKA_DOF);

	double _Dt=0.001;



	Matrix 	_Jacobian3; 	_Jacobian3.Resize(3,_KUKA_DOF);
	Matrix 	_Jacobian9; 	_Jacobian9.Resize(9,KUKA_DOF);
	Matrix 	_JacobianDirY;	_JacobianDirY.Resize(3,_KUKA_DOF);
	Matrix 	_JacobianDirZ;	_JacobianDirZ.Resize(3,_KUKA_DOF);

	for(int i=0;i<_KUKA_DOF;i++){
		_JointVelLimitsDn(i) = _Chain->getMin(i);
		_JointVelLimitsUp(i) = _Chain->getMax(i);
	}

	_Solver.SetLimits(_JointVelLimitsDn,_JointVelLimitsUp);
	_Joints=_q_in;
	_Chain->setJoints(_Joints.Array());
	_Chain->getEndPos(_Pos.Array());

	_Chain->getEndDirAxis(_AXIS_Y, _DirY.Array());
	_Chain->getEndDirAxis(_AXIS_Z, _DirZ.Array());

	_FinalPos(0)=_Final_transformation(0,3);
	_FinalPos(1)=_Final_transformation(1,3);
	_FinalPos(2)=_Final_transformation(2,3);

	_FinalDirY(0)=_Final_transformation(0,1);
	_FinalDirY(1)=_Final_transformation(1,1);
	_FinalDirY(2)=_Final_transformation(2,1);

	_FinalDirZ(0)=_Final_transformation(0,2);
	_FinalDirZ(1)=_Final_transformation(1,2);
	_FinalDirZ(2)=_Final_transformation(2,2);

	_error(0)=_Pos(0)-_FinalPos(0);
	_error(1)=_Pos(1)-_FinalPos(1);
	_error(2)=_Pos(2)-_FinalPos(2);

	_error(3)=_DirY(0)-_FinalDirY(0);
	_error(4)=_DirY(1)-_FinalDirY(1);
	_error(5)=_DirY(2)-_FinalDirY(2);

	_error(6)=_DirZ(0)-_FinalDirZ(0);
	_error(7)=_DirZ(1)-_FinalDirZ(1);
	_error(8)=_DirZ(2)-_FinalDirZ(2);

	double _old_error=1000;
	while (_error.Norm()<_old_error)
	{
		_Chain->setJoints(_Joints.Array());
		_Chain->getEndPos(_Pos.Array());
		_Chain->getEndDirAxis(_AXIS_Y, _DirY.Array());
		_Chain->getEndDirAxis(_AXIS_Z, _DirZ.Array());

		_Chain->getJacobianPos(_Jacobian3);
		_Chain->getJacobianDirection(_AXIS_Y, _JacobianDirY);
		_Chain->getJacobianDirection(_AXIS_Z, _JacobianDirZ);

		for(int i=0; i<3; i++){
			_Jacobian9.SetRow(_Jacobian3.GetRow(i)   , i  );
			_Jacobian9.SetRow(_JacobianDirY.GetRow(i), i+3);
			_Jacobian9.SetRow(_JacobianDirZ.GetRow(i), i+6);
		}
		_TargetVelocity.SetSubVector(0, (_FinalPos -_Pos ) / _Dt );
		_TargetVelocity.SetSubVector(3, (_FinalDirY-_DirY) / _Dt );
		_TargetVelocity.SetSubVector(6, (_FinalDirZ-_DirZ) / _Dt );
		_Solver.SetJacobian(_Jacobian9);
		_Solver.SetTarget(_TargetVelocity, 0);
		_Solver.Solve();
		_JointDesVel = _Solver.GetOutput();
		_JointDesPos = _Joints + _JointDesVel*_Dt;
		_Joints=_JointDesPos;
		mJointDesPos=_JointDesPos;

		_old_error=_error.Norm();

		_Chain->setJoints(_Joints.Array());
		_Chain->getEndPos(_Pos.Array());
		_Chain->getEndDirAxis(_AXIS_Y, _DirY.Array());
		_Chain->getEndDirAxis(_AXIS_Z, _DirZ.Array());

		_error(0)=_Pos(0)-_FinalPos(0);
		_error(1)=_Pos(1)-_FinalPos(1);
		_error(2)=_Pos(2)-_FinalPos(2);

		_error(3)=_DirY(0)-_FinalDirY(0);
		_error(4)=_DirY(1)-_FinalDirY(1);
		_error(5)=_DirY(2)-_FinalDirY(2);

		_error(6)=_DirZ(0)-_FinalDirZ(0);
		_error(7)=_DirZ(1)-_FinalDirZ(1);
		_error(8)=_DirZ(2)-_FinalDirZ(2);

		cout<<"Pos "<<(_Pos-_FinalPos).Norm2()<<" "<<(_DirY-_FinalDirY).Norm2()
													  <<" "<<(_DirZ-_FinalDirZ).Norm2()<<endl;
	}
	_q_out=_JointDesPos;
	return true;
}

feasible_motion2::feasible_motion2()
:RobotInterface(){
}
feasible_motion2::~feasible_motion2(){
}

RobotInterface::Status feasible_motion2::RobotInit(){
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
	lTargetPos.Resize(3);
	lTDirection.Resize(3);

	mJointPosAll.Resize(KUKA_DOF+FINGER_DOF);
	mJointTorqueAll.Resize(KUKA_DOF+FINGER_DOF);


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




	AddConsoleCommand("test");
	AddConsoleCommand("job");
	AddConsoleCommand("New_Point");

	DesiredFrame.Resize(4,4);
	mPlanner =NONE;

	Motion_counter=0;
	return STATUS_OK;
}
RobotInterface::Status feasible_motion2::RobotFree(){
	return STATUS_OK;
}
RobotInterface::Status feasible_motion2::RobotStart(){
	return STATUS_OK;
}    
RobotInterface::Status feasible_motion2::RobotStop(){
	return STATUS_OK;
}
RobotInterface::Status feasible_motion2::RobotUpdate(){

	switch(mCommand){
	case COMMAND_TEST :
		mPlanner = PLANNER_CARTESIAN;
		IK=false;
		mCommand=COMMAND_NON;
		break;
	case COMMAND_JOB:
		mPlanner = PLANNER_JOINT;
		IK=false;
		mCommand=COMMAND_NON;
	}

	switch(mPlanner){
	case PLANNER_CARTESIAN:

		if (IK==false)
		{
			bool state=solve_inverse_kinematic(mSKinematicChain,mIKSolver,mJointKinematics,mJointDesPos,DesiredFrame);
			IK=true;
		}

	}
	mSKinematicChain->setJoints(mJointDesPos.Array());

	return STATUS_OK;
}
RobotInterface::Status feasible_motion2::RobotUpdateCore(){


	mSensorsGroup.ReadSensors();
	mJointPosAll    = mSensorsGroup.GetJointAngles();

	for(int i=0; i<KUKA_DOF; i++) mJointKinematics(i)= mJointPosAll(i);


	mActuatorsGroup.SetJointAngles(mJointDesPos);
	mActuatorsGroup.WriteActuators();
	mKinematicChain.Update();


	return STATUS_OK;
}
int feasible_motion2::RespondToConsoleCommand(const string cmd, const vector<string> &args){
	cout<<"Write your command"<<endl;

	if(cmd=="job"){
		mJointDesPos.Zero();
		mJointKinematics.Set(mJointDesPos);
		mSKinematicChain->setJoints(mJointDesPos.Array());
		mSKinematicChain->getEndDirAxis(AXIS_Y, lDirY.Array());
		mSKinematicChain->getEndDirAxis(AXIS_Z, lDirZ.Array());
		mActuatorsGroup.SetJointAngles(mJointKinematics);
		mActuatorsGroup.WriteActuators();
		mKinematicChain.Update();

		mCommand = COMMAND_JOB;
	}
	else if(cmd=="test"){
		mCommand = COMMAND_TEST;
	}
	else if(cmd=="New_Point"){
		lTargetDirY=lDirY;
		lTargetDirZ=lDirZ;

		double A=rand() % 100;
		lTargetPos(0)= (A/100-0.5);
		A=rand() % 100;
		lTargetPos(1)= (A/100-0.5);
		A=rand() % 100;
		lTargetPos(2)= A/400+0.5;
		DesiredFrame.Zero();
		DesiredFrame.SetColumn(lTargetPos,3);
		DesiredFrame.SetColumn(lTargetDirZ,2);
		DesiredFrame.SetColumn(lTargetDirY,1);
		lTargetPos.Print("lTargetPos");
		lTargetDirY.Print("lTargetDirY");
		lTargetDirZ.Print("lTargetDirZ");
		DesiredFrame.Print("DesiredFrame");
	}

	return 0;
}


extern "C"{
// These two "C" functions manage the creation and destruction of the class
feasible_motion2* create(){return new feasible_motion2();}
void destroy(feasible_motion2* module){delete module;}
}

