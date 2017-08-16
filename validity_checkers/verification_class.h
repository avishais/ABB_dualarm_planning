/*
 * veification_class.cpp
 *
 *  Created on: Aug 1, 2017
 *      Author: avishai
 */

#include "StateValidityCheckerPCS.h"
//#include "StateValidityCheckerGD.h"

//#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

typedef vector<double> State;
typedef vector<vector< double >> Matrix;

class verification_class : public StateValidityChecker
{
public:
	verification_class() {
		cout << "Initiated verification module." << endl;
		continuity_tol = get_RBS_tol() * 1.1;;

		initMatrix(Tsd, 4, 4);
		Tsd = {{1, 0, 0, r2x-r1x}, {0, 1, 0, r2y-r1y}, {0, 0, 1, 0}, {0, 0, 0, 1}};

		State Pr = get_robots_properties();
		b = Pr[0]; l1 = Pr[1]; l2 = Pr[2]; l3a = Pr[3]; l3b = Pr[4]; l4 = Pr[5]; l5 = Pr[6]; lee = Pr[7];
	}

	bool verify_path();
	bool verify_path(Matrix M);
	void log_path_file(Matrix M);

	bool test_constraint(State q);
	Matrix Tsb_matrix(State q);

private:
	double b, l1, l2, l3a, l3b, l4, l5, lee;
	double L = ROD_LENGTH;
	double r1x = -ROBOTS_DISTANCE/2;
	double r1y = 0;
	double r2x = ROBOTS_DISTANCE/2;
	double r2y = 0;

	double continuity_tol;
	
	string path_file = "../paths/path.txt";

	Matrix Tsd;
};


