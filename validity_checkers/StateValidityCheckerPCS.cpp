/*
 * Checker.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

/*
myStateValidityCheckerClass::myStateValidityCheckerClass(const ob::SpaceInformationPtr &si) {

}*/

#include "StateValidityCheckerPCS.h"
#include <queue>

void StateValidityChecker::defaultSettings()
{
	stateSpace_ = mysi_->getStateSpace().get();
	if (!stateSpace_)
		OMPL_ERROR("No state space for motion validator");
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, Vector &q1, Vector &q2) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	for (unsigned i = 0; i < 6; i++) {
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+6]; // Set state of robot1
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State q1, State q2) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	for (unsigned i = 0; i < 6; i++) {
		Q->values[i] = q1[i];
		Q->values[i+6]= q2[i];
	}
}

void StateValidityChecker::printStateVector(const ob::State *state) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	Vector q1(6), q2(6);

	for (unsigned i = 0; i < 6; i++) {
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+6]; // Set state of robot1
	}

	cout << "q1: "; printVector(q1);
	cout << "q2: "; printVector(q2);
}


bool StateValidityChecker::close_chain(const ob::State *state, int q1_active_ik_sol) {
	// c is a 14 dimensional vector composed of [q1 q2 ik]

	Vector q1(6), q2(6), ik(2), q1_temp;
	retrieveStateVector(state, q1, q2);

	FKsolve_rob(q1, 1);
	Matrix T2 = MatricesMult(get_FK_solution_T1(), getQ()); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	if (calc_all_IK_solutions_2(T2) == 0)
		return false;
	q2 = get_all_IK_solutions_2(q1_active_ik_sol);
	ik[0] = get_valid_IK_solutions_indices_2(q1_active_ik_sol);

	Matrix Tinv = getQ();
	InvertMatrix(getQ(), Tinv); // Invert matrix
	FKsolve_rob(q2, 2);
	Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
	int i;
	for (i=0; i<8; i++) {
		if (!IKsolve_rob(T1, 1, i))
			continue;
		q1_temp = get_IK_solution_q1();
		if (normDistance(q1_temp,q1)<1e-1) {
			ik[1] = i;
			break;
		}
	}
	if (i==8)
		return false;

	return true;
}

bool StateValidityChecker::IKproject(Vector &q1, Vector &q2, int active_chain, int ik_sol) {

	bool valid = true;
	IK_counter++;
	clock_t sT = clock();

	if (!active_chain) {
		if (calc_specific_IK_solution_R1(Q, q1, ik_sol))
			q2 = get_IK_solution_q2();
		else
			valid = false;
	}
	else {
		if (calc_specific_IK_solution_R2(Q, q2, ik_sol))
			q1 = get_IK_solution_q1();
		else
			valid = false;
	}

	IK_time += double(clock() - sT) / CLOCKS_PER_SEC;


	return valid;
}


bool StateValidityChecker::IKproject(Vector &q1, Vector &q2, int active_chain) {

	// Try to project to all IK solutions
	int i;
	vector<int> IKsol = {0,1,2,3,4,5,6,7};
	std::random_shuffle( IKsol.begin(), IKsol.end() );

	for (i = 0; i < IKsol.size(); i++) {
		if (!active_chain) {
			if (calc_specific_IK_solution_R1(Q, q1, IKsol[i]))
				q2 = get_IK_solution_q2();
			else
				continue;
		}
		else {
			if (calc_specific_IK_solution_R2(Q, q2, IKsol[i]))
				q1 = get_IK_solution_q1();
			else
				continue;
		}
		Vector ik = identify_state_ik(q1, q2);
		if (ik[0]==-1 || ik[1]==-1)
			continue;
	}
	if (i==8) // Failed
			return false;
	else
		return true;

}

bool StateValidityChecker::sample_q(ob::State *st) {

	Vector q(12), q1(6), q2(6), ik(2);

	while (1) {
		// Random active chain
		for (int i = 0; i < 6; i++)
			q1[i] = ((double) rand() / (RAND_MAX)) * 2 * PI - PI;

		int ik_sol = rand() % 8;

		if (calc_specific_IK_solution_R1(Q, q1, ik_sol))
			q2 = get_IK_solution_q2();

		ik = identify_state_ik(q1, q2);
		if (ik[0]==-1 && ik[1]==-1)
			continue;

		q = join_Vectors(q1, q2);
		if (withObs && collision_state(P, q1, q2) && !check_angle_limits(q))
			continue;

		break;
	}

	updateStateVector(st, q1, q2);
	return true;

}

Vector StateValidityChecker::sample_q() {

	Vector q(12), q1(6), q2(6), ik(2);

	while (1) {
		// Random active chain
		for (int i = 0; i < 6; i++)
			q1[i] = ((double) rand() / (RAND_MAX)) * 2 * PI - PI;

		int ik_sol = rand() % 8;

		if (calc_specific_IK_solution_R1(Q, q1, ik_sol))
			q2 = get_IK_solution_q2();

		ik = identify_state_ik(q1, q2);
		if (ik[0]==-1 && ik[1]==-1)
			continue;

		q = join_Vectors(q1, q2);
		if (withObs && collision_state(P, q1, q2) && !check_angle_limits(q))
			continue;

		break;
	}

	return q;
}

Vector StateValidityChecker::identify_state_ik(const ob::State *state, Vector ik) {

	clock_t sT = clock();
	Vector q1(6), q2(6), q_temp(6);
	retrieveStateVector(state, q1, q2);

	ik = identify_state_ik(q1, q2, ik);

	clock_t eT = clock();
	iden +=  double(eT - sT) / CLOCKS_PER_SEC;

	return ik;
}

Vector StateValidityChecker::identify_state_ik(Vector q1, Vector q2, Vector ik) {



	if (ik[0] == -1) { // Compute only if the ik index for the active chain 0 is unknown
		// q1 is the active chain
		FKsolve_rob(q1, 1);
		Matrix T2 = MatricesMult(get_FK_solution_T1(), getQ()); // Returns the opposing required matrix of the rods tip at robot 2
		T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
		int n = calc_all_IK_solutions_2(T2);
		if (n == 0)
			return ik;

		for (int i = 0; i < n; i++) {
			q_temp = get_all_IK_solutions_2(i);
			if (normDistance(q_temp,q2)<1e-1) {
				ik[0] = get_valid_IK_solutions_indices_2(i);
				break;
			}
		}
	}

	if (ik[1] == -1) { // Compute only if the ik index for the active chain 1 is unknown
		// q2 is the active chain
		Matrix Tinv = getQ();
		InvertMatrix(getQ(), Tinv); // Invert matrix
		FKsolve_rob(q2, 2);
		Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
		T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
		int n = calc_all_IK_solutions_1(T1);
		if (n == 0)
			return ik;

		for (int i = 0; i < n; i++) {
			q_temp = get_all_IK_solutions_1(i);
			if (normDistance(q_temp,q1)<1e-1) {
				ik[1] = get_valid_IK_solutions_indices_1(i);
				break;
			}
		}
	}

	return ik;
}


// -------------------------------------------------------------
// ------------------- Validity check --------------------------

// Validates a state by switching between the two possible active chains and computing the specific IK solution (input) and checking collision
bool StateValidityChecker::isValid(const ob::State *state) {

	isValid_counter++;

	State q1(6), q2(6), ik(2);
	retrieveStateVector(state, q1, q2);
	ik = identify_state_ik(state);

	// q1 is the active chain
	if (calc_specific_IK_solution_R1(Q, q1, ik[0])) {
		Vector q_IK = get_IK_solution_q2();
		if (normDistance(q2, q_IK) > 0.5e-1)
			return false;
		if (withObs && collision_state(P, q1, q_IK))
			return false;
	}
	else
		return false;

	// q2 is the active chain
	if (calc_specific_IK_solution_R2(Q, q2, ik[1])) {
		Vector q_IK = get_IK_solution_q1();
		if (normDistance(q1, q_IK) > 0.12)
			return false;
		if (withObs && collision_state(P, q_IK, q2))
			return false;
	}
	else {
		Vector q_IK = get_IK_solution_q1();
		printVector(q_IK);
		return false;
	}

	return true;
}

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool StateValidityChecker::isValid(const ob::State *state, int active_chain, int IK_sol) {

	isValid_counter++;

	State q1(6), q2(6);
	retrieveStateVector(state, q1, q2);

	switch (active_chain) {
	case 0:
		if (calc_specific_IK_solution_R1(Q, q1, IK_sol)) {
			Vector q_IK = get_IK_solution_q2();
			if (!withObs || !collision_state(P, q1, q_IK))
				return true;
		}
		else
			return false;
		break;
	case 1:
		if (calc_specific_IK_solution_R2(Q, q2, IK_sol)) {
			Vector q_IK = get_IK_solution_q1();
			if (!withObs || !collision_state(P, q_IK, q2))
				return true;
		}
		else
			return false;
	}
	return false;
}

bool StateValidityChecker::checkMotion(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol)
{
	// We assume motion starts and ends in a valid configuration - due to projection

	bool result = true;
	//int nd = stateSpace_->validSegmentCount(s1, s2);

	double d = stateDistance(s1, s2);
	int nd = d / 0.1;

	// initialize the queue of test positions
	std::queue< std::pair<int, int> > pos;
	if (nd >= 2)
	{
		pos.push(std::make_pair(1, nd - 1));

		// temporary storage for the checked state
		ob::State *test = mysi_->allocState();

		// repeatedly subdivide the path segment in the middle (and check the middle)
		while (!pos.empty())
		{
			std::pair<int, int> x = pos.front();

			int mid = (x.first + x.second) / 2;
			stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

			if (!isValid(test, active_chain, ik_sol))
			{
				result = false;
				break;
			}
			pos.pop();

			if (x.first < mid)
				pos.push(std::make_pair(x.first, mid - 1));
			if (x.second > mid)
				pos.push(std::make_pair(mid + 1, x.second));
		}

		mysi_->freeState(test);
	}
	return result;
}

double StateValidityChecker::normDistance(Vector a1, Vector a2) {
	double sum = 0;
	for (int i=0; i < a1.size(); i++)
		sum += pow(a1[i]-a2[i], 2);
	return sqrt(sum);
}

double StateValidityChecker::stateDistance(const ob::State * s1, const ob::State * s2) {
	Vector qa1(6), qa2(6), qb1(6), qb2(6);
	retrieveStateVector(s1, qa1, qa2);
	retrieveStateVector(s2, qb1, qb2);

	double sum = 0;
	for (int i=0; i < qa1.size(); i++)
		sum += pow(qa1[i]-qb1[i], 2) + pow(qa2[i]-qb2[i], 2);
	return sqrt(sum);
}

// ------------------------------------ RBS -------------------------------------------

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool StateValidityChecker::isValidRBS(Vector &q1, Vector &q2, int active_chain, int IK_sol) {

	isValid_counter++;

	if (!IKproject(q1, q2, active_chain, IK_sol))
		return false;

	if (!withObs || !collision_state(P, q1, q2))
		return true;

	return false;
}


// Calls the Recursive Bi-Section algorithm (Hauser)
bool StateValidityChecker::checkMotionRBS(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol)
{
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;

	Vector qa1(6), qa2(6), qb1(6), qb2(6);
	retrieveStateVector(s1, qa1, qa2);
	retrieveStateVector(s2, qb1, qb2);

	result = checkMotionRBS(qa1, qa2, qb1, qb2, active_chain, ik_sol, 0, 0);

	return result;
}


// Implements local-connection using Recursive Bi-Section Technique (Hauser)
bool StateValidityChecker::checkMotionRBS(Vector qa1, Vector qa2, Vector qb1, Vector qb2, int active_chain, int ik_sol, int recursion_depth, int non_decrease_count) {

	Vector q1(6), q2(6);

	// Check if reached the required resolution
	double d = normDistanceDuo(qa1, qa2, qb1, qb2);
	if (d < RBS_tol)
		return true;

	if (recursion_depth > RBS_max_depth)// || non_decrease_count > 10)
		return false;

	midpoint(qa1, qa2, qb1, qb2, q1, q2);

	//if ( (qa2[5]>3.1415 && qb2[5]<-3.1415) || (qa2[5]<-3.1415 && qb2[5]>3.1415) )
	//	return false;

	// Check obstacles collisions and joint limits
	if (!isValidRBS(q1, q2, active_chain, ik_sol)) // Also updates s_mid with the projected value
		return false;

	//if ( normDistanceDuo(qa1, qa2, q1, q2) > d || normDistanceDuo(q1, q2, qb1, qb2) > d )
	//		non_decrease_count++;

	if ( checkMotionRBS(qa1, qa2, q1, q2, active_chain, ik_sol, recursion_depth+1, non_decrease_count) && checkMotionRBS(q1, q2, qb1, qb2, active_chain, ik_sol, recursion_depth+1, non_decrease_count) )
		return true;
	else
		return false;
}

double StateValidityChecker::normDistanceDuo(Vector qa1, Vector qa2, Vector qb1, Vector qb2) {
	double sum = 0;
	for (int i=0; i < qa1.size(); i++)
		sum += pow(qa1[i]-qb1[i], 2) + pow(qa2[i]-qb2[i], 2);
	return sqrt(sum);
}

void StateValidityChecker::midpoint(Vector qa1, Vector qa2, Vector qb1, Vector qb2, Vector &q1, Vector &q2) {

	for (int i = 0; i < 6; i++) {
		q1[i] = (qa1[i]+qb1[i])/2;
		q2[i] = (qa2[i]+qb2[i])/2;
	}
}

// *************** Reconstruct the RBS - for post-processing and validation

// Reconstruct local connection with the Recursive Bi-Section algorithm (Hauser)
bool StateValidityChecker::reconstructRBS(const ob::State *s1, const ob::State *s2, Matrix &Confs, int active_chain, int ik_sol)
{
	Vector qa1(6), qa2(6), qb1(6), qb2(6);
	retrieveStateVector(s1, qa1, qa2);
	retrieveStateVector(s2, qb1, qb2);

	Confs.push_back(join_Vectors(qa1, qa2));
	Confs.push_back(join_Vectors(qb1, qb2));

	return reconstructRBS(qa1, qa2, qb1, qb2, active_chain, ik_sol, Confs, 0, 1, 1);
}

bool StateValidityChecker::reconstructRBS(Vector qa1, Vector qa2, Vector qb1, Vector qb2, int active_chain, int ik_sol, Matrix &M, int iteration, int last_index, int firstORsecond) {
	// firstORsecond - tells if the iteration is from the first or second call for the recursion (in the last iteration).
	// last_index - the last index that was added to M.

	Vector q1(6), q2(6);
	iteration++;

	// Check if reached the required resolution
	double d = normDistanceDuo(qa1, qa2, qb1, qb2);
	if (d < RBS_tol)
		return true;

	if (iteration > RBS_max_depth)
		return false;

	midpoint(qa1, qa2, qb1, qb2, q1, q2);

	// Check obstacles collisions and joint limits
	if (!isValidRBS(q1, q2, active_chain, ik_sol)) // Also updates s_mid with the projected value
		return false; // Not suppose to happen since we run this function only when local connection feasibility is known

	if (firstORsecond==1)
		M.insert(M.begin()+last_index, join_Vectors(q1, q2)); // Inefficient operation, but this is only for post-processing and validation
	else
		M.insert(M.begin()+(++last_index), join_Vectors(q1, q2)); // Inefficient operation, but this is only for post-processing and validation

	int prev_size = M.size();
	if (!reconstructRBS(qa1, qa2, q1, q2, active_chain, ik_sol, M, iteration, last_index, 1))
		return false;
	last_index += M.size()-prev_size;
	if (!reconstructRBS(q1, q2, qb1, qb2, active_chain, ik_sol, M, iteration, last_index, 2))
		return false;

	return true;
}


// ----------------------------------------------------------------------------------

Vector StateValidityChecker::join_Vectors(Vector q1, Vector q2) {

	Vector q(q1.size()+q2.size());

	for (int i = 0; i < q1.size(); i++) {
		q[i] = q1[i];
		q[i+q1.size()] = q2[i];
	}

	return q;
}

void StateValidityChecker::seperate_Vector(Vector q, Vector &q1, Vector &q2) {

	for (int i = 0; i < q.size()/2; i++) {
		q1[i] = q[i];
		q2[i] = q[i+q.size()/2];
	}
}

void StateValidityChecker::log_q(ob::State *s) {

	Vector q1(6), q2(6);
	retrieveStateVector(s, q1, q2);

	// Open a_path file
	std::ofstream myfile;
	myfile.open("./paths/path.txt");

	myfile << 1 << endl;

	for (int j = 0; j < 6; j++)
		myfile << q1[j] << " ";
	for (int j = 0; j < 6; j++)
		myfile << q2[j] << " ";
	myfile << endl;

myfile.close();

}
