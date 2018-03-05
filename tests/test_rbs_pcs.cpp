#include "../validity_checkers/StateValidityCheckerPCS.h"
//#include "../validity_checkers/verification_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

int main() {

	int Seed = time(NULL);//1501629079;//
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
	StateValidityChecker svc(1);
	svc.initiate_log_parameters();
	//Verify
	//verification_class vfc;

	int n = 12;
	State q1(n), q2(n), q1a(n/2), q1b(n/2), q2a(n/2), q2b(n/2);

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/pcs_rbs_verification_withObs4.txt", ios::app); //_env2_distMix

	int N = 1e5, i = 0;
	while (i < N) {

		q1 = svc.sample_q();

		int active_chain;
		if (1){//rand()%2==0) {
			double s = fRand(0.01, 1);
			for (int j = 0; j < n; j++)
				q2[j] = q1[j] + s * (fRand(-PI_, PI_)-q1[j]);

			svc.seperate_Vector(q2, q2a, q2b);
			int active_chain = rand() % 2;
			int ik_sol = rand() % 8;

			if (!svc.IKproject(q2a, q2b, active_chain, ik_sol))
				continue;
		}
		else {
			q2 = svc.sample_q();
			svc.seperate_Vector(q2, q2a, q2b);
		}
		State ik2 = svc.identify_state_ik(q2a, q2b);

		svc.seperate_Vector(q1, q1a, q1b);
		State ik1 = svc.identify_state_ik(q1a, q1b);

		bool vsuc = false;
		double rbs_time;
		clock_t begin = clock();
		if (ik1[0]==ik2[0]) {
			vsuc = svc.checkMotionRBS(q1a, q1b, q2a, q2b, 0, ik1[0], 0, 0);
			active_chain = 0;
		}
		if (!vsuc && ik1[1]==ik2[1]) {
			vsuc = svc.checkMotionRBS(q1a, q1b, q2a, q2b, 1, ik1[1], 0, 0);
			active_chain = 1;
		}
		rbs_time = double(clock() - begin) / CLOCKS_PER_SEC;

		if (vsuc) {
			//cout << "Found LC." << endl;
			/*Matrix path;
			path.push_back(svc.join_Vectors(q1a, q1b));
			path.push_back(svc.join_Vectors(q2a, q2b));
			svc.reconstructRBS(q1a, q1b, q2a, q2b, active_chain, ik1[active_chain], path, 0, 1, 1);*/

			bool path_valid = true;//vfc.verify_path(path);

			/*if (!path_valid) {
				vfc.log_path_file(path);
				cout << "Verified to be: " << vfc.verify_path(path) << endl;
				cout << "Press...\n";
				//cin.ignore();
			}*/

			f << vsuc << " " << path_valid << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
		}
		else
			f << 0 << " " << 0 << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
		i++;
	}

	f.close();

	return 0;
}

