/*
 * kdl_class.h
 *
 *  Created on: Feb 27, 2017
 *      Author: avishai
 */

// This projection approach is the implementation of the Newton-Raphson method in Lynch and Park (http://hades.mech.northwestern.edu/images/7/7f/MR.pdf)
// pages 230-231.

#ifndef GD_CLASS_H_
#define GD_CLASS_H_

#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#define ARMA_DONT_PRINT_ERRORS
#include <armadillo>

#define PI 3.1416

using namespace std;
using namespace arma;

typedef vector<double> State;
typedef vector<vector< double >> Matrix;

class GDproject
{
private:
	double b,l1, l2, l3a, l3b, l4, l5, l6, lee; // Links lengths
	double q1minmax, q2minmax, q3min, q3max, q4minmax, q5minmax, q6minmax; // Joint limits
	double L; // Rod length

	double rot1, rot2, r1x, r1y, r2x, r2y;

	State GD_result;
	
	Matrix Tsd; // Closure constraint - desired matrix of the 12D arm. 

public:
	// Constructor
	GDproject(State, State, double);

	Matrix Tsb_matrix(State);
	Matrix jacobian(State);
	State get_twist(Matrix Tsb);
	

	double linesearch(State q, State dFq);
	bool GD(State q0);

	State get_GD_result();

	bool check_angle_limits(State);

	// Misc
	void initVector(State &, int);
	void initMatrix(Matrix &, int, int);
	double deg2rad(double);
	void printMatrix(Matrix);
	void printVector(State);
	void clearMatrix(Matrix &);
	double normVector(State v);
	State VectorSum(State v1, State v2, double);
	State MATxVEC(Matrix M, State v);
	void log_q(State q);
	Matrix MatrixMult(Matrix M1, Matrix M2);
	Matrix MatrixInv(Matrix M);
	mat MATvector2armadilo(Matrix M);
	Matrix pseudo_inv(Matrix J);
	bool checkTwistSize(State Vb, double ep_v, double ep_w);
	
	Matrix get_Tsd() {
		return Tsd;
	}

	// Performance parameters
	int iterations;
	int IK_counter;
	double IK_time;
	int get_IK_counter() {
		return IK_counter;
	}
	double get_IK_time() {
		return IK_time;
	}
	int get_num_iterations() {
		return iterations;
	}

	bool include_joint_limits = true;
};



#endif /* KDL_CLASS_H_ */
