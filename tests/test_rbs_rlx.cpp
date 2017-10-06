#include "../validity_checkers/StateValidityCheckerRX.h"

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

double fRand(double fMin, double fMax)
{
	double f = (double)rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

void gen_data(StateValidityChecker svc) {

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/data/rlx_rand_confs_eps0.5_env2.txt", ios::app);

	State q(12);
	int N = 1e5;
	for (int k = 0; k < N; k++) {
		q = svc.sample_q();

		for (int i = 0; i < 12; i++)
			f << q[i] << " ";
		f << endl;
	}
	f.close();
}

Matrix load_ranf_confs(double eps) {
	Matrix C;
	State c_temp(12);
	double t;

	ifstream fq;
	//fq.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/data/rlx_rand_confs_eps" + std::to_string(eps) + ".txt");
	fq.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/data/rlx_rand_confs_eps0.5_env2.txt");
	int i = 0;
	while(!fq.eof()) {// || i > 1e5) {
		for (int j=0; j < 12; j++) {
			fq >> c_temp[j];
		}
		C.push_back(c_temp);
		i++;
	}
	fq.close();

	return C;
}

int main() {

	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;

	// KDL
	StateValidityChecker svc(2);
	svc.initiate_log_parameters();
	svc.set_epsilon(0.5);

	//gen_data(svc);
	Matrix C = load_ranf_confs(0.5);//svc.get_epsilon()
	//exit(1);

	int n = 12;
	State q1(n), q2(n);

	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/rlx_rbs_verification_eps0.5_withObs_env2.txt", ios::app);

	int N = 5.5e5, i = 0;

	while (i < N) {
		int k = rand() % C.size();
		q1 = C[k];
		//q1 = svc.sample_q();

		if (rand()%2==0) {
			bool flag = false;
			for (int a = 0; a < 20; a++) {
				double s = fRand(0.01, 1);
				for (int j = 0; j < n; j++)
					q2[j] = q1[j] + s * (fRand(-PI, PI) - q1[j]);

				if (!svc.isValidRBS(q2))
					continue;
				else {
					flag = true;
					break;
				}
			}
			if (!flag)
				continue;
		}
		else {
			int j;
			do {
				j = rand() % C.size();
			} while (j==k);

			q2 = C[j];
		}

		clock_t begin = clock();
		bool vsuc = svc.checkMotionRBS(q1, q2, 0, 0);
		double rbs_time = double(clock() - begin) / CLOCKS_PER_SEC;

		if (vsuc) {
			Matrix path;
			path.push_back(q1);
			path.push_back(q2);
			svc.reconstructRBS(q1, q2, path, 0, 1, 1);

			bool path_valid = 1;

			f << vsuc << " " << path_valid << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
		}
		else
			f << 0 << " " << 0 << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
		i++;
	}
/*

	for (int j = 0; j < C.size()-1; j++) {
		for (int k = j+1; k < C.size(); k++) {
			q1 = C[j];
			q2 = C[k];

			clock_t begin = clock();
			bool vsuc = svc.checkMotionRBS(q1, q2, 0, 0);
			double rbs_time = double(clock() - begin) / CLOCKS_PER_SEC;

			if (vsuc) {
				bool path_valid = 1;

				f << vsuc << " " << path_valid << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;
				i++;
			}
			else
				f << 0 << " " << 0 << " " << svc.normDistance(q1, q2) << " " << rbs_time << endl;

		}
	}*/

	f.close();


	return 0;
}

