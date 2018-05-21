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

const string robot_file = "../resources/02-kuka-dual-arm/iiwa7.urdf";
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

	//Position right
	Vector3d endeffector_pos = Vector3d(0.0,0.0,0.025); // Endeffector's position in frame 7
	Vector3d initial_pos_r = Vector3d(0.0,0.0,0.0);
	robot->position(initial_pos_r, "link7", endeffector_pos); // Get initial position of link7 (endeffector)
	Vector3d desired_xpos_r = initial_pos_r; // Set desired start position to initial position
	Vector3d desired_start_pos_r = initial_pos_r; // Set desired start position
	Vector3d desired_final_pos_r = Vector3d(-0.3,-0.98,0.53); // Set desired final position
	Vector3d pick_pos_r = Vector3d(-0.08,-0.98,0.53); // Position of endeffector when surrounding object
	Vector3d lift_pos_r = Vector3d(-0.08,-0.98,0.78); // Position of endeffector when lifting object

	//Position left
	Vector3d initial_pos_l = Vector3d(0.0,0.0,0.0);
	robot->position(initial_pos_l, "arm2_link7", endeffector_pos); // Get initial position of link7 (endeffector)
	Vector3d desired_xpos_l = initial_pos_l; // Set desired start position to initial position
	Vector3d desired_start_pos_l = initial_pos_l; // Set desired start position
	Vector3d desired_final_pos_l = Vector3d(0.3,-0.98,0.48); // Set desired final position
	Vector3d pick_pos_l = Vector3d(0.08,-0.98,0.48); // Position of endeffector when surrounding object
	Vector3d lift_pos_l = Vector3d(0.08,-0.98,0.83); // Position of endeffector when lifting object

	//TODO: Make a list of all positions, set next_pos and prev_pos, keep track of position index

	// For position controlled vector
	double desired_gripper_size = 0;
	bool force_controlled = false; 

	// For force controlled gripper
	double desired_gripper_force = 0;

	// Velocity of movement
	double velocity = 0.05;

	//Rotation right endeffector
	Matrix3d desired_rot_r = Matrix3d::Identity(3,3);
	robot->rotation(desired_rot_r, "link7");
	// TODO: Interpolate rotation	
	desired_rot_r << 0,0,1,
					0,1,0,
					-1,0,0;		

	//Rotation left endeffector
	Matrix3d desired_rot_l = Matrix3d::Identity(3,3);
	robot->rotation(desired_rot_l, "arm2_link7");
	// TODO: Interpolate rotation	
	desired_rot_l << 0,0,-1,
					0,1,0,
					1,0,0;	

	//Desired velocity
	Vector3d desired_v_r = Vector3d(0.0,0.0,0.0);
	Vector3d desired_w_r = Vector3d(0.0,0.0,0.0);
	Vector3d desired_v_l = Vector3d(0.0,0.0,0.0);
	Vector3d desired_w_l = Vector3d(0.0,0.0,0.0);


	// Linear operational space matrices
	MatrixXd lambda_pos_r = MatrixXd::Zero(3,3);
	MatrixXd Jv_bar_r = MatrixXd::Zero(dof,3);
	MatrixXd Nv_r = MatrixXd::Zero(dof,dof);
	MatrixXd lambda_pos_l = MatrixXd::Zero(3,3);
	MatrixXd Jv_bar_l = MatrixXd::Zero(dof,3);
	MatrixXd Nv_l = MatrixXd::Zero(dof,dof);

	// Angular operational space matrices
	MatrixXd lambda_ang_r = MatrixXd::Zero(3,3);
	MatrixXd Jw_bar_r = MatrixXd::Zero(dof,3);
	MatrixXd Nw_r = MatrixXd::Zero(dof,dof);
	MatrixXd lambda_ang_l = MatrixXd::Zero(3,3);
	MatrixXd Jw_bar_l = MatrixXd::Zero(dof,3);
	MatrixXd Nw_l = MatrixXd::Zero(dof,dof);

	// Constrained
	MatrixXd Jw_r_c = MatrixXd::Zero(3, dof);
	MatrixXd lambda_ang_r_c = MatrixXd::Zero(3,3);
	MatrixXd Jw_bar_r_c = MatrixXd::Zero(dof,3);
	MatrixXd Nw_r_c = MatrixXd::Zero(dof,dof);
	MatrixXd Jw_l_c = MatrixXd::Zero(3, dof);
	MatrixXd lambda_ang_l_c = MatrixXd::Zero(3,3);
	MatrixXd Jw_bar_l_c = MatrixXd::Zero(dof,3);
	MatrixXd Nw_l_c = MatrixXd::Zero(dof,dof);

	// Linear and angular jacobian
	MatrixXd Jv_r = MatrixXd::Zero(3,dof);
	MatrixXd Jw_r = MatrixXd::Zero(3,dof);
	MatrixXd Jv_l = MatrixXd::Zero(3,dof);
	MatrixXd Jw_l = MatrixXd::Zero(3,dof);


	// Initialize forces and torques
	VectorXd Fv_r = VectorXd::Zero(3); 
	VectorXd Fw_r = VectorXd::Zero(3); 
	VectorXd Fv_l = VectorXd::Zero(3); 
	VectorXd Fw_l = VectorXd::Zero(3); 
	VectorXd command_torques = VectorXd::Zero(dof);

	// Position and orientation of right endeffector
	Vector3d x_pos_r = Vector3d::Zero(3);
	Matrix3d x_rot_r = Matrix3d::Zero(3,3);

	// Position and orientation of left endeffector
	Vector3d x_pos_l = Vector3d::Zero(3);
	Matrix3d x_rot_l = Matrix3d::Zero(3,3);

	// Position and Orientation error of right endeffector
	Vector3d pos_error_r = Vector3d::Zero(3);
	Vector3d rot_error_r = Vector3d::Zero(3);
	//VectorXd error = VectorXd::Zero(6);

	// Position and Orientation error of left endeffector
	Vector3d pos_error_l = Vector3d::Zero(3);
	Vector3d rot_error_l = Vector3d::Zero(3);

	// Velocity
	Vector3d linVel_r = Vector3d::Zero(3);
    Vector3d angVel_r = Vector3d::Zero(3);
    Vector3d linVel_l = Vector3d::Zero(3);
    Vector3d angVel_l = Vector3d::Zero(3);

    VectorXd gamma0 = VectorXd::Zero(dof);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	Vector3d direction_r = (desired_final_pos_r - desired_start_pos_r)/(desired_final_pos_r - desired_start_pos_r).norm();
	Vector3d step_r = (velocity*direction_r)/control_freq; // Set how much the desired position should change for each iteration in loop
	Vector3d direction_l = (desired_final_pos_l - desired_start_pos_l)/(desired_final_pos_l - desired_start_pos_l).norm();
	Vector3d step_l = (velocity*direction_l)/control_freq; // Set how much the desired position should change for each iteration in loop
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
		if(state == START_POS && (x_pos_r - desired_start_pos_r).norm() < 0.01){
			cout<<"distance to start: "<< (x_pos_r - desired_start_pos_r).norm() <<endl;
			state = GOAL_POS;
			cout<<"state: GOAL_POS, "<<state<<endl;
			controller_counter = 0;
			time = 0;
			//TODO: Change to velocity instead (step_r=velocity/control_freq)
			//step_r = (desired_final_pos_r - desired_start_pos_r)/(timeToGoal*control_freq);
			direction_r = (desired_final_pos_r - desired_start_pos_r)/(desired_final_pos_r - desired_start_pos_r).norm();
			direction_l = (desired_final_pos_l - desired_start_pos_l)/(desired_final_pos_l - desired_start_pos_l).norm();
			step_r = (velocity*direction_r)/control_freq;
			step_l = (velocity*direction_l)/control_freq;
		} 

		// Go to state 2 if distance to final position is small enough
		if(state == GOAL_POS && (x_pos_r - desired_final_pos_r).norm() < 0.01) {
			cout<<"distance to goal: "<< (x_pos_r - desired_final_pos_r).norm() <<endl;
			sleep(1);
			state = OPEN_GRIPPER;
			cout<<"state: OPEN_GRIPPER, "<<state<<endl;
			controller_counter = 0;
			time = 0;
			force_controlled = false;
			desired_gripper_size = 0.15;
			step_r = Vector3d::Zero(3);
			step_l = Vector3d::Zero(3);
		}

		// Go to state 3 if the distance to desired joint position is small enough
		if(state == OPEN_GRIPPER && time==openGripperTime) {
			state = SURROUND_OBJECT;
			sleep(1);
			cout<<"state: SURROUND_OBJECT, "<<state<<endl;
			controller_counter = 0;
			time = 0;
			direction_r = (pick_pos_r - desired_final_pos_r)/(pick_pos_r - desired_final_pos_r).norm();
			direction_l = (pick_pos_l - desired_final_pos_l)/(pick_pos_l - desired_final_pos_l).norm();
			step_r = (velocity*direction_r)/control_freq;
			step_l = (velocity*direction_l)/control_freq;
		}

		// Go to state 4 if distance to the box is small enough
		if(state == SURROUND_OBJECT && (x_pos_r - pick_pos_r).norm() < 0.06) {
			sleep(1);
			cout<<"distance to object: "<< (x_pos_r - pick_pos_r).norm() <<endl;
			state = CLOSE_GRIPPER;
			cout<<"state: CLOSE_GRIPPER, "<<state<<endl;
			controller_counter = 0;
			time = 0;

			// Control force on gripper
			force_controlled = true;
			desired_gripper_force = -10;
			step_r = Vector3d::Zero(3);
			step_l = Vector3d::Zero(3);
		}

		// Go to state 5 when the gripper is closed
		if(state == CLOSE_GRIPPER && time==closeGripperTime) {
			sleep(1);
			state = LIFT_GRIPPER;
			cout<<"state: LIFT_GRIPPER, "<<state<<endl;
			controller_counter = 0;
			time = 0;
			direction_r = (lift_pos_r - pick_pos_r)/(lift_pos_r - pick_pos_r).norm();
			direction_l = (lift_pos_l - pick_pos_l)/(lift_pos_l - pick_pos_l).norm();
			step_r = (velocity*direction_r)/control_freq;
			step_l = (velocity*direction_l)/control_freq;
		}

		if(state == LIFT_GRIPPER && (x_pos_r - lift_pos_r).norm() < 0.04) {
			sleep(1);
			state = STOP;
			cout<<"state: STOP, "<<state<<endl;
			controller_counter = 0;
			time = 0;
			step_r = Vector3d::Zero(3);
			step_l = Vector3d::Zero(3);
		}


		//---------------- CONTROLLER ------------------------------------
	
		robot->position(x_pos_r, "link7", endeffector_pos); // Get current position
		robot->rotation(x_rot_r, "link7"); // Get current rotation
		desired_xpos_r += step_r; // Increase desired position with step_rsize

		robot->position(x_pos_l, "arm2_link7", endeffector_pos); // Get current position
		robot->rotation(x_rot_l, "arm2_link7"); // Get current rotation
		desired_xpos_l += step_l; // Increase desired position with step_rsize

		// Current velocity
		robot->linearVelocity(linVel_r, "link7", endeffector_pos);
		angVel_r = Jw_r*robot->_dq;

		robot->linearVelocity(linVel_l, "arm2_link7", endeffector_pos);
		angVel_l = Jw_l*robot->_dq;

		// Get current right error
		pos_error_r << x_pos_r - desired_xpos_r;
		Sai2Model::orientationError(rot_error_r, desired_rot_r, x_rot_r);

		// Get current left error
		pos_error_l << x_pos_l - desired_xpos_l;
		Sai2Model::orientationError(rot_error_l, desired_rot_l, x_rot_l);
		
		// Get Jacobians
		robot->Jv(Jv_r, "link7", endeffector_pos);
		robot->Jw(Jw_r, "link7");
		robot->Jv(Jv_l, "arm2_link7", endeffector_pos);
		robot->Jw(Jw_l, "arm2_link7");

		// Mass
		lambda_pos_r = (Jv_r*robot->_M_inv*Jv_r.transpose()).inverse();
		lambda_pos_l = (Jv_l*robot->_M_inv*Jv_l.transpose()).inverse();
		//lambda_ang_r = (Jw_r*robot->_M_inv*Jw_r.transpose()).inverse();

		// Inverse Jacobians
		Jv_bar_r = (robot->_M_inv)*Jv_r.transpose()*lambda_pos_r;
		Jw_bar_r = (robot->_M_inv)*Jw_r.transpose()*lambda_ang_r;
		Jv_bar_l = (robot->_M_inv)*Jv_l.transpose()*lambda_pos_l;
		Jw_bar_l = (robot->_M_inv)*Jw_l.transpose()*lambda_ang_l;


		// Nullspace
		Nv_r << MatrixXd::Identity(dof,dof)-Jv_bar_r*Jv_r;
		Nw_r << MatrixXd::Identity(dof,dof)-Jw_bar_r*Jw_r;
		Nv_l << MatrixXd::Identity(dof,dof)-Jv_bar_l*Jv_l;
		Nw_l << MatrixXd::Identity(dof,dof)-Jw_bar_l*Jw_l;

		// Constrained
		Jw_r_c = Jw_r*Nv_r;
		lambda_ang_r = (Jw_r_c*robot->_M_inv*Jw_r_c.transpose()).inverse();
		Jw_bar_r_c = (robot->_M_inv)*Jw_r_c.transpose()*lambda_ang_r_c;
		Jw_l_c = Jw_l*Nv_l;
		lambda_ang_l = (Jw_l_c*robot->_M_inv*Jw_l_c.transpose()).inverse();
		Jw_bar_l_c = (robot->_M_inv)*Jw_l_c.transpose()*lambda_ang_l_c;

		Nw_r_c = Nv_r.transpose()*(MatrixXd::Identity(dof,dof) - Jw_r_c.transpose()*Jw_bar_r_c.transpose());
		Fv_r = lambda_pos_r*(-kp*pos_error_r - kv*(linVel_r-desired_v_r));
		Fw_r = lambda_ang_r*(-kp*rot_error_r - kv*(angVel_r-desired_w_r));
		Nw_l_c = Nv_l.transpose()*(MatrixXd::Identity(dof,dof) - Jw_l_c.transpose()*Jw_bar_l_c.transpose());
		Fv_l = lambda_pos_l*(-kp*pos_error_l - kv*(linVel_l-desired_v_l));
		Fw_l = lambda_ang_l*(-kp*rot_error_l - kv*(angVel_l-desired_w_l));
		gamma0 = -kvq*robot->_M*robot->_dq; // Damp joint motions

		command_torques = Jv_r.transpose()*Fv_r + Jv_l.transpose()*Fv_l + Jw_r_c.transpose()*Fw_r + Jw_l_c.transpose()*Fw_l + Nw_r_c*gamma0 + Nw_l_c*gamma0;

		// Control gripper
		if(force_controlled){
			command_torques(robot->jointId("jgripper1")) = desired_gripper_force;
			command_torques(robot->jointId("jgripper2")) = desired_gripper_force;
			command_torques(robot->jointId("arm2_jgripper1")) = desired_gripper_force;
			command_torques(robot->jointId("arm2_jgripper2")) = desired_gripper_force;
		}
		else{
			command_torques(robot->jointId("jgripper1")) = -kp * ((robot->_q)(robot->jointId("jgripper1")) - desired_gripper_size/2) - kv * (robot->_dq)(robot->jointId("jgripper1"));
			command_torques(robot->jointId("jgripper2")) = command_torques(robot->jointId("jgripper1"));
			command_torques(robot->jointId("arm2_jgripper1")) = -kp * ((robot->_q)(robot->jointId("arm2_jgripper1")) - desired_gripper_size/2) - kv * (robot->_dq)(robot->jointId("arm2_jgripper1"));
			command_torques(robot->jointId("arm2_jgripper2")) = command_torques(robot->jointId("arm2_jgripper1"));
		}
		

		//------ Final torques
		// command_torques.setZero();

		//command_torques << 1,0,0,0,0,0,0,0,0,0,0,0;
		//cout<<"Command Torques: "<<command_torques<<endl;

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    command_torques.setZero(dof);
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}
 