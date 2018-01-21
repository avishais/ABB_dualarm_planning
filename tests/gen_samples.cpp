#include "../proj_classes/apc_class.h"
#include "../proj_classes/kdl_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#define ROBOTS_DISTANCE_ENV_I 900.
#define ROD_LENGTH_ENV_I 300.

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

int main() {
	
	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;
	
	// KDL
	kdl K(ROBOTS_DISTANCE_ENV_I, ROD_LENGTH_ENV_I);
	two_robots A({-ROBOTS_DISTANCE_ENV_I/2, 0, 0 }, {ROBOTS_DISTANCE_ENV_I/2, 0, PI_}, ROD_LENGTH_ENV_I);
	
	
	int n = 12;
	State q(n), q1(n/2), q2(n/2);
	
	std::ofstream fa;
	fa.open("./results/samples_apc_ABB.txt", ios::app);
	std::ofstream fk;
	fk.open("./results/samples_kdl_ABB.txt", ios::app);
	
	int N = 0.5e5, i = 0;
	while (i < N) {
		
		while (1) {
			q = A.rand_q_ambient();
			
			// APC
			int tries;
			bool Asuc = false;
			for (tries = 0; tries < 16; tries++) {
				
				int active_chain = rand()/RAND_MAX * 2;
				int ik_sol = rand() % 8;
				
				if (!active_chain) {
					for (int j = 0; j < 6; j++)
						q1[j] = q[j];
					if (A.calc_specific_IK_solution_R1(A.getQ(), q1, ik_sol)) {
						q2 = A.get_IK_solution_q2();
						Asuc = true;
					}
				}
				else {
					for (int j = 0; j < 6; j++)
					q2[j] = q[j+6];
					if (A.calc_specific_IK_solution_R2(A.getQ(), q2, ik_sol)) {
						q1 = A.get_IK_solution_q1();
						Asuc = true;
					}
				}
				
				if (Asuc)
					break;
			}
			if (tries==16)
				continue;
			break;
		}

		for (int j = 0; j < q1.size(); j++)
			fa << q1[j] << " ";
		for (int j = 0; j < q2.size(); j++)
			fa << q2[j] << " ";
		fa << endl;
		// A.log_q(q1, q2);
		
		// KDL
		while (1) {
			for (int j = 0; j < 12; j++)
				q[j] = fRand(-3.14, 3.14);
			if (K.GD(q))
				break;
		}
		q = K.get_GD_result();
		for (int j = 0; j < q.size(); j++)
			fk << q[j] << " ";
		fk << endl;
		// K.log_q(q);
		
		// cin.ignore();
		

		
		i++;
		if (!(i % 100))
			cout << (double)i/N*100 << "%\n";
	}
	
	fa.close();
	fk.close();
	
	return 0;
}

