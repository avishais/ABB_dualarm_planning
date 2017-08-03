#include "apc_class.h"
#include "gd_class.h"

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
	GDproject G({-ROBOTS_DISTANCE/2, 0, 0 }, {ROBOTS_DISTANCE/2, 0, PI}, ROD_LENGTH);

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/apc_verification.txt", ios::app);

	int N = 0.5e6;
	State q(12), qp_gd(12), qp_apc(12);
	double aph_time = 0;

	int i = 0;
	while (i < N) {

		for (int j = 0; j < 12; j++)
			q[j] = fRand(-3.14, 3.14);

		State q_rand = q;

		// APC
		int tries;
		bool Asuc;
		for (tries = 0; tries < 16; tries++) {
			int active_chain = 1;//rand()/RAND_MAX * 2;
			clock_t begin = clock();
			Asuc = A.project(q, active_chain); // q is updated
			aph_time = double(clock() - begin) / CLOCKS_PER_SEC;
			if (Asuc)
				break;
		}
		if (tries==16)
			continue;

		Matrix Tsb = G.Tsb_matrix(q);
		Matrix Tsd = G.get_Tsd();

		bool valid = true;
		for (int j = 0; j < 3 && valid; j++)
			for (int k = 0; k < 4; k++) {
				if ((k < 3 && fabs(Tsb[j][k]-Tsd[j][k]) > 0.1) || (k==3 && fabs(Tsb[j][k]-Tsd[j][k]) > 0.5)) {
					valid = false;
					break;
				}

			}

		f << Asuc << " " << valid << " " << aph_time << endl;

		i++;


	}

	f.close();


}

