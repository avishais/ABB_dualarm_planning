#include "kdl_class.h"
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

	// GD
	GDproject G({-ROBOTS_DISTANCE/2, 0, 0 }, {ROBOTS_DISTANCE/2, 0, PI}, ROD_LENGTH);

	// KDL
	kdl K(ROBOTS_DISTANCE, ROD_LENGTH);

	/*std::ofstream f_gd;
	f_gd.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/gd_random_conf.txt", ios::app);
	std::ofstream f_kdl;
	f_kdl.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/kdl_random_conf.txt", ios::app);*/
	std::ofstream f;
	f.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/tests/results/kdl_verification.txt", ios::app);

	int N = 0.5e6;
	State q(12), qp_gd(12), qp_kdl(12);
	double kdl_time = 0, gd_time = 0;

	for (int i = 0; i < N; i++) {

		for (int j = 0; j < 12; j++)
			q[j] = fRand(-3.14, 3.14);

		//q = {1.97746724216702,	2.54966593680547,	-2.34371095524892,	2.59731710563547,	0.831637671152858,	-2.52872821404116,	-1.39173673672867,	0.294565272647012,	2.87460022633501,	2.92098081384033	,-2.15128045457413,	2.95682165202307};

		// KDL
		clock_t begin = clock();
		bool Ksuc = K.GD(q);
		qp_kdl = K.get_GD_result();
		kdl_time = double(clock() - begin) / CLOCKS_PER_SEC;

		Matrix Tsb = G.Tsb_matrix(qp_kdl);
		Matrix Tsd = G.get_Tsd();

		//G.printMatrix(Tsb);
		//G.printMatrix(Tsd);

		bool valid = true;
		for (int j = 0; j < 3 && valid; j++)
			for (int k = 0; k < 4; k++) {
				if ((k < 3 && fabs(Tsb[j][k]-Tsd[j][k]) > 0.1) || (k==3 && fabs(Tsb[j][k]-Tsd[j][k]) > 0.5)) {
					valid = false;
					break;
				}

			}

		f << Ksuc << " " << valid << " " << kdl_time << endl;


		// GD
		/*begin = clock();
		bool Gsuc = G.GD(q);
		qp_gd = G.get_GD_result();
		gd_time = double(clock() - begin) / CLOCKS_PER_SEC;

		f << Ksuc << " " << kdl_time << " " << dist(q, qp_kdl) << " " << Gsuc << " " << gd_time << " " << dist(q, qp_gd) << endl;*/

		/*if (Ksuc && Gsuc) {
			for (int i = 0; i < qp_gd.size(); i++)
				f_gd << qp_gd[i] << " ";
			f_gd << endl;
			for (int i = 0; i < qp_kdl.size(); i++)
				f_kdl << qp_kdl[i] << " ";
			f_kdl << endl;
		}*/

	}

	f.close();


}

/*
int main() {

	srand( time(NULL));

	// KDL
	kdl K(ROBOTS_DISTANCE, ROD_LENGTH);

	// GD
	GDproject G({-ROBOTS_DISTANCE/2, 0, 0 }, {ROBOTS_DISTANCE/2, 0, PI}, ROD_LENGTH);

	std::ofstream mf;
	mf.open("/home/avishai/Downloads/omplapp/ompl/Workspace/dualarm/matlab/KG_6.txt", ios::app);

	int N = 10000;
	State J(12), Jp(12);

	double GD_time = 0, GD_time_sum = 0;
	double KDL_time = 0, KDL_time_sum = 0;
	double GD_dist, KDL_dist;
	int checks = 0;
	clock_t startT;

	for (int i = 0; i < N; i++) {

		checks++;

		for (int j = 0; j < 12; j++)
			J[j] = fRand(-3.14, 3.14);

		// KDL
		startT = clock();
		bool Ksuc = K.GD(J);
		KDL_time = double(clock() - startT) / CLOCKS_PER_SEC;
		Jp = K.get_GD_result();
		KDL_dist = dist(J,Jp);
		//K.log_q(J);

		// GD
		startT = clock();
		bool Gsuc = G.GD(J, 0.01);
		GD_time = double(clock() - startT) / CLOCKS_PER_SEC;
		Jp = G.get_GD_result();
		GD_dist = dist(J,Jp);

		//G.printVector(J);
		//G.log_q(J);

		KDL_time_sum += KDL_time;
		GD_time_sum += GD_time;

		mf << Ksuc << " " << KDL_time  << " " << Gsuc << " " << GD_time << " " << KDL_dist << " " << GD_dist << " " << G.get_num_iterations() << endl;
	}

	cout << "KDL average time: " << KDL_time_sum/N * 1000 << " msec." << endl;
	cout << "GD average time: " << GD_time_sum/N * 1000 << " msec." << endl;

	mf.close();

}
 */
