/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Avishai Sintov */

#include "plan_GD.h"

bool isStateValid(const ob::State *state)
{
	return true;
}

void plan_C::plan(State c_start, State c_goal, double runtime, double maxStep) {

	// construct the state space we are planning inz
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(12)); // Angles of Robot 1 & 2 - R^12

	// set the bounds for the Q=R^12 part of 'Cspace'
	ob::RealVectorBounds Qbounds(12);
	Qbounds.setLow(0, -2.88); // Robot 1
	Qbounds.setHigh(0, 2.88);
	Qbounds.setLow(1, -1.919);
	Qbounds.setHigh(1, 1.919);
	Qbounds.setLow(2, -1.919);
	Qbounds.setHigh(2, 1.22);
	Qbounds.setLow(3, -2.79);
	Qbounds.setHigh(3, 2.79);
	Qbounds.setLow(4, -2.09);
	Qbounds.setHigh(4, 2.09);
	Qbounds.setLow(5, -PI);// -6.98); // Should be -6.98 but currently the IK won't allow it - this impacts the sampler
	Qbounds.setHigh(5, PI);// 6.98); // Should be 6.98 but currently the IK won't allow it
	Qbounds.setLow(6, -2.88); // Robot 2
	Qbounds.setHigh(6, 2.88);
	Qbounds.setLow(7, -1.919);
	Qbounds.setHigh(7, 1.919);
	Qbounds.setLow(8, -1.919);
	Qbounds.setHigh(8, 1.22);
	Qbounds.setLow(9, -2.79);
	Qbounds.setHigh(9, 2.79);
	Qbounds.setLow(10, -2.09);
	Qbounds.setHigh(10, 2.09);
	Qbounds.setLow(11, -PI);// -6.98); // Should be -6.98 but currently the IK won't allow it
	Qbounds.setHigh(11, PI);// 6.98); // Should be 6.98 but currently the IK won't allow it

	// set the bound for the compound space
	Q->as<ob::RealVectorStateSpace>()->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Cspace(Q);

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Cspace));

	// set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.1); // 3% ???

	// create start state
	ob::ScopedState<ob::RealVectorStateSpace> start(Cspace);
	for (int i = 0; i < 12; i++) {
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = c_start[i]; // Access the first component of the start a-state
	}

	// create goal state
	ob::ScopedState<ob::RealVectorStateSpace> goal(Cspace);
	for (int i = 0; i < 12; i++) {
		goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = c_goal[i]; // Access the first component of the goal a-state
	}

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();

	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner(new og::RRTConnect(si, maxStep));
	//ob::PlannerPtr planner(new og::RRT(si));

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();

	//planner->printSettings(std::cout); // Prints some parameters such as range
	//planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	// print the settings for this space
	//si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	//si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	// print the problem settings
	//pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	// attempt to solve the problem within one second of planning time
	clock_t begin = clock();
	ob::PlannerStatus solved = planner->solve(runtime);
	clock_t end = clock();
	cout << "Runtime: " << double(end - begin) / CLOCKS_PER_SEC << endl;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		//ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution:" << std::endl;

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		//std::ofstream myfile;
		//myfile.open("pathGD.txt");
		//og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		//pog.printAsMatrix(myfile); // Print as matrix to file
		//myfile.close();
		solved_bool = true;
	}
	else {
		std::cout << "No solutions found" << std::endl;
		solved_bool = false;
	}
}

void extract_from_perf_file(ofstream &ToFile) {
	ifstream FromFile;
	FromFile.open("./paths/perf_log.txt");

	string line;
	while (getline(FromFile, line))
		ToFile << line << "\t";

	FromFile.close();
}


int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime;

	if (argn == 1)
		runtime = 1; // sec
	else {
		runtime = atof(args[1]);
	}

	plan_C Plan;

	srand (time(NULL));

	int mode = 3;
	switch (mode) {
	case 1: {
		State c_start = {0.5236, 1.7453, -1.8326, -1.4835,	1.5708,	0, 1.004278, 0.2729, 0.9486, -1.15011, 1.81001, -1.97739};
		//State c_goal = {0.5236, 0.34907, 0.69813, -1.3963, 1.5708, 0, -2.432, -1.4148, -1.7061, -1.6701, -1.905, 1.0015}; // Robot 2 backfilp - Elbow down
		State c_goal = {0.5236, 0.34907, 0.69813, -1.3963, 1.5708, 0, 0.7096, 1.8032, -1.7061, -1.6286, 1.9143, -2.0155}; // Robot 2 no backflip - Elbow down

		Plan.plan(c_start, c_goal, runtime);

		Plan.vfc.verify_path();
		break;
	}
	case 2 : { // Benchmark planning time with constant maximum step size
		State c_start = {0.5236, 1.7453, -1.8326, -1.4835,	1.5708,	0, 1.004278, 0.2729, 0.9486, -1.15011, 1.81001, -1.97739};
		State c_goal = {0.5236, 0.34907, 0.69813, -1.3963, 1.5708, 0, 0.7096, 1.8032, -1.7061, -1.6286, 1.9143, -2.0155}; // Robot 2 no backflip - Elbow down

		ofstream GD;
		GD.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/matlab/benchmark_GD_3poles_range2.txt", ios::app);

		for (int k = 0; k < 500; k++) {
			Plan.plan(c_start, c_goal, runtime);

			GD << Plan.vfc.verify_path() << "\t";

			// Extract from perf file
			ifstream FromFile;
			FromFile.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/paths/perf_log.txt");
			string line;
			while (getline(FromFile, line))
				GD << line << "\t";
			FromFile.close();
			GD << endl;
		}
		GD.close();
		break;
	}
	case 3 : { // Benchmark maximum step size
		State c_start = {0.5236, 1.7453, -1.8326, -1.4835,	1.5708,	0, 1.004278, 0.2729, 0.9486, -1.15011, 1.81001, -1.97739};
		State c_goal = {0.5236, 0.34907, 0.69813, -1.3963, 1.5708, 0, 0.7096, 1.8032, -1.7061, -1.6286, 1.9143, -2.0155}; // Robot 2 no backflip - Elbow down

		ofstream GD;
		GD.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/matlab/benchmark_GD_3poles_rangeB.txt", ios::app);


		for (int k = 0; k < 1800; k++) {

			for (int j = 0; j < 24; j++) {
				double maxStep = 0.05 + 0.25*j;

				Plan.plan(c_start, c_goal, runtime, maxStep);

				bool verf = Plan.vfc.verify_path();

				GD << maxStep << " " << verf << "\t";

				// Extract from perf file
				ifstream FromFile;
				FromFile.open("/home/avishai/Downloads/omplapp/ompl/Workspace/ckc3d/paths/perf_log.txt");
				string line;
				while (getline(FromFile, line))
					GD << line << "\t";
				FromFile.close();
				GD << endl;
			}
		}
		GD.close();
		break;
	}
	}

	std::cout << std::endl << std::endl;

	return 0;
}

