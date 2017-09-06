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

	// KDL
	kdl K(ROBOTS_DISTANCE, ROD_LENGTH);

	State q(12);

	int N = 10000;
	int c = 0, c_jl = 0, c_jl_ver = 0;
	double t = 0, t_jl = 0;
	for (int i = 0; i < N; i++) {

		for (int j = 0; j < 12; j++)
			q[j] = fRand(-3.14, 3.14);

		//bool Ksuc = K.GD(q);
		//qp_kdl = K.get_GD_result();
		clock_t s = clock();
		if (K.GD(q)) {
			c++;
		}
		t += double(clock() - s) / CLOCKS_PER_SEC;

		s = clock();
		if (K.GD_JL(q)) {
			c_jl++;

			q = K.get_GD_result();
			if (K.check_angle_limits(q))
				c_jl_ver++;
		}
		t_jl += double(clock() - s) / CLOCKS_PER_SEC;
	}

	cout << "Success without JL: " << c << " out of " << N << ", avg. projection time: " << (double)t/N*1e3 << " msec." << endl;
	cout << "Success with JL: " << c_jl << " out of " << N << ", avg. projection time: " << (double)t_jl/N*1e3 << " msec." << endl;
	cout << "Success with JL verified: " << c_jl_ver << " out of " << N << "." << endl;

}
