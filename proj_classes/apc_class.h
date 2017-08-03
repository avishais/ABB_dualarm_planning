#pragma once
/* This code defines the kinematics (IK and FK) of two ABB IRB120 robotic arms */

#include <iostream>
#include <vector>
#include <fstream>
#include <ctime>
#include <math.h>

#define PI 3.1416
#define NUM_IK_SOLUTIONS 12

using namespace std;

typedef vector<double> State;
typedef vector< double > Vector;
typedef vector<vector< double >> Matrix;

class two_robots
{
private:
	double b, l1, l2, l3a, l3b, l4, l5, l6, lee; // ABB link lengths
	double L; // Length of grasped rod
	double q1minmax, q2minmax, q3min, q3max, q4minmax, q5minmax, q6minmax; // Joint limits
	Vector V_pose_rob_1_o; // The vector [x,y,theta] that describes the position and orientation of robot 1 relative to origin (assume on the same x-y plane)
	Vector V_pose_rob_2_o; // The vector [x,y,theta] that describes the position and orientation of robot 2 relative to origin (assume on the same x-y plane)
	// FK parameters
	Matrix T_fk_solution_1; // FK solution of robot 1
	Vector p_fk_solution_1; // FK solution of robot 1
	Matrix T_fk_solution_2; // FK solution of robot 2
	Vector p_fk_solution_2; // FK solution of robot 2
	// IK parameters
	Vector q_IK_solution_1; // IK solution of robot 1
	Vector q_IK_solution_2; // IK solution of robot 2
	Matrix Q_IK_solutions_1; // All IK solutions of robot 1
	Matrix Q_IK_solutions_2; // All IK solutions of robot 2
	Vector valid_IK_solutions_indices_1; // Indices of the IK solutions to use in the IK_solve function for robot 1
	Vector valid_IK_solutions_indices_2; // Indices of the IK solutions to use in the IK_solve function for robot 2
	int IK_number;
	int countSolutions;

	// Temp variable for time efficiency
	Matrix T_fk_temp;
	Vector p_fk_temp;
	Vector V_pose;
	Matrix T1, T2, T_mult_temp;

public:
	/** Constructor */
	two_robots(Vector, Vector, double);

	/** Forward kinematics */
	void FKsolve_rob(Vector, int);
	Matrix get_FK_solution_T1();
	Vector get_FK_solution_p1();
	Matrix get_FK_solution_T2();
	Vector get_FK_solution_p2();

	/** Inverse kinematics */
	bool IKsolve_rob(Matrix, int, int);
	Vector get_IK_solution_q1();
	Vector get_IK_solution_q2();
	int get_countSolutions();
	bool calc_specific_IK_solution_R1(Matrix, Vector, int);
	bool calc_specific_IK_solution_R2(Matrix, Vector, int);

	/** Is robots configurations feasible? */
	bool IsRobotsFeasible_R1(Matrix, Vector);
	bool IsRobotsFeasible_R2(Matrix, Vector);
	int calc_all_IK_solutions_1(Matrix);
	int calc_all_IK_solutions_2(Matrix);

	/** Check the angles limits of the ABB */
	bool check_angle_limits(Vector q);

	// Getters
	int get_IK_number();
	Matrix get_T2();
	Vector get_all_IK_solutions_1(int);
	Vector get_all_IK_solutions_2(int);
	int get_valid_IK_solutions_indices_1(int);
	int get_valid_IK_solutions_indices_2(int);

	// Misc
	void initVector(Vector &, int);
	void initMatrix(Matrix &, int, int);
	double deg2rad(double);
	void printMatrix(Matrix);
	void printVector(Vector);
	Matrix MatricesMult(Matrix, Matrix);
	bool InvertMatrix(Matrix M, Matrix &Minv); // Inverse of a 4x4 matrix
	void clearMatrix(Matrix &);
	double normDistance(Vector, Vector);

	Matrix Q;
	Matrix P;
	Matrix getPMatrix() {
		return P;
	}
	Matrix getQ() {
		return Q;
	}

	Vector get_robots_properties() {
		Vector E = {b, l1, l2, l3a, l3b, l4, l5, lee};
		return E;
	}

	/** Performance parameters */
	int IK_counter;
	double IK_time;
	int get_IK_counter() {
		return IK_counter;
	}
	double get_IK_time() {
		return IK_time;
	}

	bool include_joint_limits = true;

};
