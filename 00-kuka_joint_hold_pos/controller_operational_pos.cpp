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

	robot->updateModel();

	int dof = robot->dof();
	VectorXd F = VectorXd::Zero(3);
	VectorXd command_torques = VectorXd::Zero(dof);

	double kp = 25.0;
	double kv = 5.0;
	double kvq = 5.0; // Joints

	VectorXd coriolis = VectorXd::Zero(dof);
	Vector3d endeffector_pos = Vector3d(0,0,0.025);

	// Set desired position to initial position
	Vector3d initial_x;
	robot->position(initial_x, "link7", endeffector_pos); // Get initial position of link7 (endeffector)
	Vector3d desired_x = initial_x;

	// Current position and velocity
	Vector3d x = Vector3d::Zero(3);
	VectorXd dx = VectorXd::Zero(3);

	// Jacobian
	MatrixXd Jv = MatrixXd::Zero(3, dof);
	
	// Operational space matrices
	MatrixXd lambda = MatrixXd::Zero(3,3);
	MatrixXd Jbar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	VectorXd gamma0 = VectorXd::Zero(dof);


	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

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

		// Trajectory Half circle in yz-direction
		/*desired_x(1) = 0.5*cos(M_PI/4*time);
		desired_x(2) = abs(0.5*sin(M_PI/4*time));*/

		// Trajectory Circle
		desired_x(0) = 0.8*cos(M_PI/2*time);
		desired_x(1) = 0.8*sin(M_PI/2*time);

		

		// Get current position and velocity
		robot->position(x, "link7", endeffector_pos);
		robot->Jv(Jv, "link7", Vector3d::Zero(3)); // Populate linear Jacobian
		dx = Jv*robot->_dq; 

		// Update operational matrices
		lambda = (Jv*robot->_M_inv*Jv.transpose()).inverse();
		Jbar = (robot->_M_inv)*Jv.transpose()*lambda;
		N = MatrixXd::Identity(dof,dof)-Jbar*Jv; // Nullspace

		// Get current F
		F = lambda*(-kp*(x - desired_x) - kv*(dx));
		gamma0 = -kvq*robot->_M*robot->_dq; // Torques to be projected in nullspace
		command_torques = Jv.transpose()*F + (N.transpose())*gamma0+coriolis;


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
