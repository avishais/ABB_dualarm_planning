#include "../validity_checkers/StateValidityCheckerGD.h"
#include "../validity_checkers/verification_class.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

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
	StateValidityChecker svc;
	//Verify
	verification_class vfc;

	int n = 12;
	State q1(n), q2(n);

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/gd_rbs_verification.txt", ios::app);

	int N = 0.5e6, i = 0;
	while (i < N) {
		q1 = svc.sample_q();
		if (q1[0]==-1000)
			continue;

		double s = fRand(0.01, 1);
		for (int j = 0; j < n; j++)
			q2[j] = q1[j] + s * (fRand(-PI, PI)-q1[j]);

		if (!svc.IKproject(q2))
			continue;

		clock_t begin = clock();
		bool vsuc = svc.checkMotionRBS(q1, q2, 0, 0);
		double rbs_time = double(clock() - begin) / CLOCKS_PER_SEC;

		if (vsuc) {
			Matrix path;
			path.push_back(q1);
			path.push_back(q2);
			svc.reconstructRBS(q1, q2, path, 1, 1);

			bool path_valid = vfc.verify_path(path);

			if (!path_valid) {
				vfc.log_path_file(path);
				cout << "Verified to be: " << vfc.verify_path(path) << endl;
				cout << "Press...\n";
				cin.ignore();
			}

			f << vsuc << " " << path_valid << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
			i++;
		}
		else
			f << 0 << " " << 0 << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
	}

	f.close();

	return 0;
}

