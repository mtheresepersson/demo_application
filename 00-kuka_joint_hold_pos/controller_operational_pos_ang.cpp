// This example application runs a controller for the IIWA

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <tinyxml2.h>

// Makes sure that program exits nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "../resources/00-kuka_joint_hold_pos/iiwa7.urdf";
const std::string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;

const bool simulation = true;
// const bool simulation = false;

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_KEY;

int main() {
	if(simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
		EE_FORCE_SENSOR_KEY = "sai2::optoforceSensor::6Dsensor::force_moment";
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::KUKA_IIWA::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::KUKA_IIWA::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::KUKA_IIWA::sensors::dq";
		EE_FORCE_SENSOR_KEY = "sai2::optoforceSensor::6Dsensor::force";
	}

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = RedisClient();
	redis_client.serverIs(info);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);

	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q); // Read from redis and write to robot
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the joint controller        /////
	////////////////////////////////////////////////

	//cout<<"hello"<<endl;

	robot->updateModel();

	int dof = robot->dof();
	

	/*double kp = 25.0;
	double kv = 5.0;
	double kvq = 5.0; // Joints*/
	double kp = 100.0;
	double kv = 30.0;
	double kvq = 20.0;

	VectorXd coriolis = VectorXd::Zero(dof);

	//Position
	Vector3d endeffector_pos = Vector3d(0.0,0.0,0.025); // Endeffector's position in frame 7
	//Vector3d initial_xpos;
	//robot->position(initial_xpos, "link7", endeffector_pos); // Get initial position of link7 (endeffector)
	//Vector3d desired_xpos = initial_xpos;
	Vector3d desired_xpos = Vector3d(0.0,0.3,0.2);


	//Rotation
	//Matrix3d initial_xrot;
	//robot->rotation(initial_xrot, "link7");
	Matrix3d desired_xrot = Matrix3d::Identity(3,3);
	desired_xrot << 1,0,0,
					0,1,0,
					0,0,-1;


	// Operational space matrices
	MatrixXd lambda = MatrixXd::Zero(6,6);
	MatrixXd Jbar = MatrixXd::Zero(dof,6);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	// Full Jacobian
	MatrixXd J = MatrixXd::Zero(6,dof);

	// Initialize forces and torques
	VectorXd F = VectorXd::Zero(6); 
	VectorXd command_torques = VectorXd::Zero(dof);


	// Position and orientation
	Vector3d x_pos = Vector3d::Zero(3);
	Matrix3d x_rot = Matrix3d::Zero(3,3);

	// Position and Orientation error
	Vector3d pos_error = Vector3d::Zero(3);
	Vector3d rot_error = Vector3d::Zero(3);
	VectorXd error = VectorXd::Zero(6);

	// Velocity
	Vector3d linVel = Vector3d::Zero(3);
    Vector3d angVel = Vector3d::Zero(3);
    VectorXd dx = VectorXd::Zero(6);

    VectorXd gamma0 = VectorXd::Zero(dof);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop
	//cout << "Hello"<<endl;

	// KEVEN TRAJECTORY
	/*double max_x = 0.6;
	double min_x = -0.6;
	double center_y = 0.3;
	double move_step = 1.0 / 10000;
	int direction = 1; // 1 is positive, -1 is negative*/

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);
		
		// update the model 20 times slower (hacky, should use a separate thread)
		if(controller_counter%20 == 0)
		{
			robot->updateModel();

			robot->coriolisForce(coriolis);
		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		//----------------TRAJECTORIES-----------------------------------

		// Trajectory Half circle in yz-direction
		/*desired_x(1) = 0.5*cos(M_PI/4*time);
		desired_x(2) = abs(0.5*sin(M_PI/4*time));*/

		// Trajectory Circle (Position)
		desired_xpos(0) = 0.5*cos(M_PI/4*time);
		desired_xpos(1) = 0.5*sin(M_PI/4*time);

		// KEVEN TRAJECTORY
		// set desired end effector position
		/*auto x = desired_xpos[0];
		if (x > max_x) {
			direction = -1;

		} else if (x < min_x) {
			direction = 1;
		}
		desired_xpos[0] += direction * move_step;
		x = desired_xpos[0];
		desired_xpos[2] = center_y + 0.3 * sin(x * 20.);*/

		//----------------------------------------------------------------


		// Get current position
		robot->position(x_pos, "link7", endeffector_pos);

		// Get current rotation
		robot->rotation(x_rot, "link7");

		// Current velocity
		robot->linearVelocity(linVel, "link7", endeffector_pos);
		robot->angularVelocity(angVel, "link7");
		dx << linVel, angVel;
		//cout<<"dx"<<dx<<endl;

		// Get current error
		pos_error << x_pos - desired_xpos;
		robot->orientationError(rot_error, desired_xrot, x_rot);
		error << pos_error, rot_error;
		//cout<<"error"<<error<<endl;
		

		robot->J_0(J, "link7", endeffector_pos); // Populate Jacobian
		lambda = (J*robot->_M_inv*J.transpose()).inverse();
		Jbar = (robot->_M_inv)*J.transpose()*lambda;
		N << MatrixXd::Identity(dof,dof)-Jbar*J; // Nullspace

		F = lambda*(-kp*error - kv*dx);
		gamma0 << -kvq*robot->_M*robot->_dq; // Torques to be projected in nullspace
		command_torques = J.transpose()*F + N.transpose()*gamma0;

		//------ Final torques
		// command_torques.setZero();
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}
