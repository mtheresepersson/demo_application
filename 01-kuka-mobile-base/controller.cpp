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

/* STATES
0: Go to initial position
1: Go to desired position
2: Open gripper
3: Surround object
4: Close gripper
5; Lift Gripper
6: Stop
*/

// TODO: Only use one state to go to a desired position instead of one state for each position
// Define states
#define START_POS 0
#define GOAL_POS 1
#define OPEN_GRIPPER 2
#define SURROUND_OBJECT 3
#define CLOSE_GRIPPER 4
#define LIFT_GRIPPER 5
#define STOP 6

// Set how long time it should stay in open and close states
const double openGripperTime = 2;
const double closeGripperTime = 2;

const string robot_file = "../resources/01-kuka-mobile-base/iiwa7.urdf";
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
	//auto robotObject = new Sai2Model::Sai2Model(robotObject_file, false);

	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q); // Read from redis and write to robot
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the joint controller        /////
	////////////////////////////////////////////////


	robot->updateModel();

	int dof = robot->dof();
	

	double kp = 100.0;
	double kv = 30.0;
	double kvq = 20.0;

	VectorXd coriolis = VectorXd::Zero(dof);

	//Position
	Vector3d endeffector_pos = Vector3d(0.0,0.0,0.025); // Endeffector's position in frame 7
	Vector3d initial_xpos = Vector3d(0.0,0.0,0.0);
	robot->position(initial_xpos, "link7", endeffector_pos); // Get initial position of link7 (endeffector)
	Vector3d desired_xpos = initial_xpos; // Set desired start position to initial position
	Vector3d desired_start_pos = initial_xpos; // Set desired start position
	Vector3d desired_final_pos = Vector3d(0.0,-0.8,0.7); // Set desired final position
	Vector3d pick_pos = Vector3d(0.0,-1.02,0.52); // Position of endeffector when surrounding object
	Vector3d lift_pos = Vector3d(0.0,-0.8,0.8); // Position of endeffector when lifting object
	//Vector3d place_pos = Vector3d(-0.8, 0.0,0.52); // Position of endeffector when placing object

	//TODO: Make a list of all positions, set next_pos and prev_pos, keep track of position index

	// For position controlled vector
	double desired_gripper_size = 0;
	bool force_controlled = false; 
	//VectorXd desired_joint_position = VectorXd::Zero(dof);

	// For force controlled gripper
	double desired_gripper_force = 0;

	// Velocity of movement
	double velocity = 0.05;

	//Rotation
	//robot->rotation(initial_xrot, "link7");
	Matrix3d desired_xrot = Matrix3d::Identity(3,3);
	robot->rotation(desired_xrot, "link7");
	/*desired_xrot << 1,0,0,
					0,1,0,
					0,0,-1;	*/				

	//Desired velocity
	Vector3d desired_linvel = Vector3d(0.0,0.0,0.0);
	Vector3d desired_angvel = Vector3d(0.0,0.0,0.0);


	// Linear operational space matrices
	MatrixXd lambda_pos = MatrixXd::Zero(3,3);
	MatrixXd Jbar_pos = MatrixXd::Zero(dof,3);
	MatrixXd N_pos = MatrixXd::Zero(dof,dof);

	// Angular operational space matrices
	MatrixXd lambda_ang = MatrixXd::Zero(3,3);
	MatrixXd Jbar_ang = MatrixXd::Zero(dof,3);
	MatrixXd N_ang = MatrixXd::Zero(dof,dof);

	// Augmented
	MatrixXd J_ang_aug = MatrixXd::Zero(3, dof);
	MatrixXd lambda_ang_aug = MatrixXd::Zero(3,3);
	MatrixXd Jbar_ang_aug = MatrixXd::Zero(dof,3);
	MatrixXd N_ang_aug = MatrixXd::Zero(dof,dof);

	// Full Jacobian
	//MatrixXd J = MatrixXd::Zero(6,dof);

	// Linear and angular jacobian
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Jw = MatrixXd::Zero(3,dof);

	// Initialize forces and torques
	VectorXd F_pos = VectorXd::Zero(3); 
	VectorXd F_ang = VectorXd::Zero(3); 
	VectorXd command_torques = VectorXd::Zero(dof);

	// Position and orientation of endeffector
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

	Vector3d direction = (desired_final_pos - desired_start_pos)/(desired_final_pos - desired_start_pos).norm();
	Vector3d step = (velocity*direction)/control_freq; // Set how much the desired position should change for each iteration in loop
	int state = START_POS;

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

		//---------------- SET STATE ------------------------------------

		// Go to state 1 if distance to start position is small enough
		if(state == START_POS && (x_pos - desired_start_pos).norm() < 0.01){
			cout<<"distance to start: "<< (x_pos - desired_start_pos).norm() <<endl;
			state = GOAL_POS;
			cout<<"state: GOAL_POS, "<<state<<endl;
			controller_counter = 0;
			time = 0;
			//TODO: Change to velocity instead (step=velocity/control_freq)
			//step = (desired_final_pos - desired_start_pos)/(timeToGoal*control_freq);
			direction = (desired_final_pos - desired_start_pos)/(desired_final_pos - desired_start_pos).norm();
			step = (velocity*direction)/control_freq;
		} 

		// Go to state 2 if distance to final position is small enough
		if(state == GOAL_POS && (x_pos - desired_final_pos).norm() < 0.01) {
			cout<<"distance to goal: "<< (x_pos - desired_final_pos).norm() <<endl;
			state = OPEN_GRIPPER;
			cout<<"state: OPEN_GRIPPER, "<<state<<endl;
			controller_counter = 0;
			time = 0;
			force_controlled = false;
			desired_gripper_size = 0.12;
			step = Vector3d::Zero(3);
		}

		// Go to state 3 if the distance to desired joint position is small enough
		if(state == OPEN_GRIPPER && time==openGripperTime) {
			state = SURROUND_OBJECT;
			cout<<"state: SURROUND_OBJECT, "<<state<<endl;
			controller_counter = 0;
			time = 0;
			direction = (pick_pos - desired_final_pos)/(pick_pos - desired_final_pos).norm();
			step = (velocity*direction)/control_freq;
		}

		// Go to state 4 if distance to the box is small enough
		if(state == SURROUND_OBJECT && (x_pos - pick_pos).norm() < 0.04) {
			cout<<"distance to object: "<< (x_pos - pick_pos).norm() <<endl;
			state = CLOSE_GRIPPER;
			cout<<"state: CLOSE_GRIPPER, "<<state<<endl;
			controller_counter = 0;
			time = 0;

			// Control force on gripper
			force_controlled = true;
			desired_gripper_force = -10;
			step = Vector3d::Zero(3);
		}

		// Go to state 5 when the gripper is closed
		if(state == CLOSE_GRIPPER && time==closeGripperTime) {
			state = LIFT_GRIPPER;
			cout<<"state: LIFT_GRIPPER, "<<state<<endl;
			controller_counter = 0;
			time = 0;
			direction = (lift_pos - pick_pos)/(lift_pos - pick_pos).norm();
			step = (velocity*direction)/control_freq;
		}

		if(state == LIFT_GRIPPER && (x_pos - lift_pos).norm() < 0.04) {
			state = STOP;
			cout<<"state: STOP, "<<state<<endl;
			controller_counter = 0;
			time = 0;
			step = Vector3d::Zero(3);
		}


		//---------------- CONTROLLER ------------------------------------
	
		robot->position(x_pos, "link7", endeffector_pos); // Get current position
		robot->rotation(x_rot, "link7"); // Get current rotation
		desired_xpos += step; // Increase desired position with stepsize

		// Current velocity
		robot->linearVelocity(linVel, "link7", endeffector_pos);
		angVel = Jw*robot->_dq;

		// Get current error
		pos_error << x_pos - desired_xpos;
		Sai2Model::orientationError(rot_error, desired_xrot, x_rot);
		
		// Get Jacobians
		robot->Jv(Jv, "link7", endeffector_pos);
		robot->Jw(Jw, "link7");

		// Mass
		lambda_pos = (Jv*robot->_M_inv*Jv.transpose()).inverse();
		//lambda_ang = (Jw*robot->_M_inv*Jw.transpose()).inverse();

		// Inverse Jacobians
		Jbar_pos = (robot->_M_inv)*Jv.transpose()*lambda_pos;
		Jbar_ang = (robot->_M_inv)*Jw.transpose()*lambda_ang;


		// Nullspace
		N_pos << MatrixXd::Identity(dof,dof)-Jbar_pos*Jv;
		N_ang << MatrixXd::Identity(dof,dof)-Jbar_ang*Jw;

		// Augmented
		J_ang_aug = Jw*N_pos;
		lambda_ang = (J_ang_aug*robot->_M_inv*J_ang_aug.transpose()).inverse();
		Jbar_ang_aug = (robot->_M_inv)*J_ang_aug.transpose()*lambda_ang_aug;

		N_ang_aug = N_pos.transpose()*(MatrixXd::Identity(dof,dof) - J_ang_aug.transpose()*Jbar_ang_aug.transpose());
		F_pos = lambda_pos*(-kp*pos_error - kv*(linVel-desired_linvel));
		F_ang = lambda_ang*(-kp*rot_error - kv*(angVel-desired_angvel));
		gamma0 = -kvq*robot->_M*robot->_dq; // Damp joint motions

		command_torques = Jv.transpose()*F_pos + J_ang_aug.transpose()*F_ang + N_ang_aug*gamma0;

		// Control gripper
		if(force_controlled){
			command_torques(10) = desired_gripper_force;
			command_torques(11) = desired_gripper_force;
		}
		else{
			command_torques(10) = -kp * ((robot->_q)(10) - desired_gripper_size/2) - kv * (robot->_dq)(10);
			command_torques(11) = command_torques(10);
		}
		

		//------ Final torques
		// command_torques.setZero();

		//command_torques << 1,0,0,0,0,0,0,0,0,0,0,0;
		//cout<<"Command Torques: "<<command_torques<<endl;

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}
 