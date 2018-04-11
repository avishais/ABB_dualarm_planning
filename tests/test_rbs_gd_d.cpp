#include "../validity_checkers/StateValidityCheckerGD.h"
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
	
	int Seed = time(NULL);
	srand( Seed );
	cout << "Seed in testing: " << Seed << endl;
	
	// KDL
	StateValidityChecker svc(2);
	svc.initiate_log_parameters();
	//Verify
	//verification_class vfc;
	
	int n = 12;
	State q1(n), q2(n);
	
	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/gd_rbs_verification_withObs_d.txt", ios::app);
	
	int N = 1e5, i = 0;
	while (i < N) {
		q1 = svc.sample_q();
		
		if (q1[0]==-1000)
		continue;
		
		if (1){//rand()%2==0) {
			double s = fRand(0.01, 1);
			for (int j = 0; j < n; j++)
			q2[j] = q1[j] + s * (fRand(-PI, PI)-q1[j]);
		}
		else {
			for (int i = 0; i < 12; i++)
			q2[i] = fRand(-PI, PI);
		}
		
		double d_before = svc.normDistance(q1, q2);
		
		if (!svc.IKproject(q2))
			continue;
		
		double d_after = svc.normDistance(q1, q2);
		
		clock_t begin = clock();
		bool vsuc = svc.checkMotionRBS(q1, q2, 0, 0);
		double rbs_time = double(clock() - begin) / CLOCKS_PER_SEC;
		
		f << vsuc << " " << d_before << " " << d_after << " " << rbs_time << endl;
		
		i++;
	}
	
	f.close();
	
	return 0;
}

