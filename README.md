# inverse-kinematics-examples


# Example 2: QP solver

This is a short tutorial about how to use the quadratic programming based inverse kinematics solver. The algorithm presented here is done at the velocity level and minimizes the kinetic energy. Another algorithm that minimizes the acceleration norm is also implemented but it doesn't work as well and a good range of hyper parameters has not been found yet.


Initialization:
First a instance of the solver has to be declared in the class (in the .h file):
	MKESolver mSolver;

Set the parameters of the solver in RobotInit():
	mSolver.setConstraints(tmpThetaMinus, tmpThetaPlus, tmpMaxVel*(-1), tmpMaxVel);
	mSolver.setDimensions(KUKA_DOF, 6);
	mSolver.setH(tmpH);		
	mSolver.setDeltaT(_dt);
	mSolver.setMuP(25);
	mSolver.setGamma(50);

tmpThetaMinus and tmpThetaPlus are vectors containing the joints limits and tmpMaxVel contains the joints velocity limits.
The second parameter in setDimensions is the number of DOF of the end-effector.
tmpH being the inertia matrix, it is square and has the dimension of the number of DOF of the robot. _dt is the timestep. 
setMuP and setGamma will set the hyper parameters of the solver. The parameters given here yield good results. mu_p could be set to a lower value in order to have a smoothe joint limits avoidance to the price of a loss of precision. Gamma should stay around 50. This is optimized for the KUKA LWR robot so optimal parameters could be different for other robots.


Update:
In RobotUpdate():
	mSolver.setJacobian(tmpJ); 
tmpJ being the jacobian found by a mSKinematicChain

	mSolver.Solve(tmpRdot, mSensorsGroup.GetJointAngles());
The solver requires the desired end effector velocity (tmpRdot) and the current joints angles

	mJointDesVel = mSolver.getOutput();
Gets the joints velocities from the solver

	mJointDesPos = lJoints + mJointDesVel*_dt;
The new position of the joints is deduced from the current one (lJoints) and the velocity

The mSKinematicChain and the mSensorsGroup are set in the same way as with the other solver of the robot toolkit.

(if you have any question: benjamin.benhattar@epfl.ch)
