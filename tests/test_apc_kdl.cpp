#include "apc_class.h"
#include "kdl_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#define ROBOTS_DISTANCE 900
#define ROD_LENGTH 300

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

double dist(State p1, State p2) {
	double sum = 0;
	for (int i = 0; i < p1.size(); i++)
		sum += (p1[i]-p2[i])*(p1[i]-p2[i]);

	return sqrt(sum);
}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// APC
	two_robots A({-ROBOTS_DISTANCE/2, 0, 0 }, {ROBOTS_DISTANCE/2, 0, PI}, ROD_LENGTH);

	// GD
	kdl K(ROBOTS_DISTANCE, ROD_LENGTH);

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/apc_kdl_proj.txt", ios::app);

	int N = 0.5e6;
	State q(12), q1(6), q2(6);
	double aph_time = 0, kdl_time = 0;

	int i = 0;
	while (i < N) {

		for (int j = 0; j < 12; j++)
			q[j] = fRand(-3.14, 3.14);

		State q_rand = q;

		// APC ---------------------------------------
		bool Asuc;

		int ik_sol = rand() % 8;
		for (int j = 0; j < 6; j++)
			q1[j] = q[j];

		clock_t begin = clock();
		Matrix Q = A.getQ();
		if (A.calc_specific_IK_solution_R1(Q, q1, ik_sol)) {
			q2 = A.get_IK_solution_q2();
			Asuc = true;
		}
		else
			Asuc = false;
		aph_time = double(clock() - begin) / CLOCKS_PER_SEC;

		for (int j = 0; j < 6; j++)
				q[j+6] = q2[j];

		double d_aph = A.normDistance(q_rand, q);

		// kdl ----------------------------------------

		begin = clock();
		bool Ksuc = K.GD(q_rand);
		q = K.get_GD_result();
		kdl_time = double(clock() - begin) / CLOCKS_PER_SEC;
		double d_kdl = A.normDistance(q_rand, q);


		f << Asuc << " " << d_aph << " " << aph_time << " " << Ksuc << " " << d_kdl << " " << kdl_time << endl;

		i++;


	}

	f.close();


}

