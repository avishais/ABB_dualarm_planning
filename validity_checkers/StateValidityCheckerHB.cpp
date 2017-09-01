/*
 * Checker.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

/*
myStateValidityCheckerClass::myStateValidityCheckerClass(const ob::SpaceInformationPtr &si) {

}*/

#include "StateValidityCheckerGD.h"
#include <queue>

void StateValidityChecker::defaultSettings()
{
	stateSpace_ = mysi_->getStateSpace().get();
	if (!stateSpace_)
		OMPL_ERROR("No state space for motion validator");
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, State &q) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	for (unsigned i = 0; i < 12; i++) {
		q[i] = Q->values[i]; // Set state of robot1
	}
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, Vector &q1, Vector &q2) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	for (unsigned i = 0; i < 6; i++) {
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+6]; // Set state of robot1
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, State q) {
	// cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();

	for (unsigned i = 0; i < 12; i++) {
		Q->values[i] = q[i];
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

	State q(12);

	for (unsigned i = 0; i < 12; i++) {
		q[i] = Q->values[i]; // Set state of robot1
	}
	cout << "q: "; printVector(q);
}

State StateValidityChecker::sample_q() {
	// c is a 12 dimensional vector composed of [q1 q2]

	State q(12), q1(6), q2(6);

	while (1) {
		for (int i = 0; i < q.size(); i++)
			q[i] = -PI + (double)rand()/RAND_MAX * 2*PI;

		if (!GD(q))
			continue;

		q = get_GD_result();

		seperate_Vector(q, q1, q2);
		if (withObs && collision_state(P, q1, q2) && !check_angle_limits(q))
			continue;

		return q;
	}
}

bool StateValidityChecker::IKproject(const ob::State *state, bool includeObs) {

	State q(12);
	retrieveStateVector(state, q);

	if (!IKproject(q, includeObs))
		return false;

	updateStateVector(state, q);

	return true;
}


bool StateValidityChecker::IKproject(State &q, bool includeObs) {

	if (!GD(q))
		return false;

	q = get_GD_result();

	State q1(6), q2(6);
	seperate_Vector(q, q1, q2);
	if (includeObs && withObs && collision_state(P, q1, q2))
		return false;

	return true;
}

// ---------------------------- Identify state IK --------------------------------




// ------------------- Check motion by projecting points on the connecting straight line ------------------------------------

// Validates a state by switching between the two possible active chains and computing the specific IK solution (input) and checking collision
bool StateValidityChecker::isValid(const ob::State *state) {

	isValid_counter++;

	State q(12), q1(6), q2(6);
	retrieveStateVector(state, q);

	if (!GD(q))
		return false;

	q = get_GD_result();

	seperate_Vector(q, q1, q2);
	if (withObs && collision_state(P, q1, q2))
		return false;

	updateStateVector(state, q);
	q_prev = q;
	return true;
}

bool StateValidityChecker::checkMotion(const ob::State *s1, const ob::State *s2)
{
	State q(12), q1(12), q2(12);
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;
	//int nd = stateSpace_->validSegmentCount(s1, s2);
	//cout << nd << endl;
	retrieveStateVector(s1, q1);
	retrieveStateVector(s2, q2);
	int nd = normDistance(q1,q2)/dq;


	if (nd > 2)
	{
		retrieveStateVector(s1, q);
		q_prev = q;

		// temporary storage for the checked state
		ob::State *test = mysi_->allocState();

		for (int i = 1; i < nd; i++) {
			stateSpace_->interpolate(s1, s2, (double)i / (double)(nd-1), test);

			if (!isValid(test))
			{
				result = false;
				break;
			}
			//printStateVector(test);
		}

		retrieveStateVector(s2, q);
		//if (MaxAngleDistance(q, q_prev) > 4*0.174533) //4*10deg
		//result = false;

		mysi_->freeState(test);
	}

	return result;
}

// ------------------------------------ v Check motion with RBS v -------------------------------------------

// Validates a state by switching between the two possible active chains and computing the specific IK solution (input) and checking collision
bool StateValidityChecker::isValidRBS(State &q) {

	isValid_counter++;

	State q1(6), q2(6);

	if (!GD(q))
		return false;

	q = get_GD_result();

	seperate_Vector(q, q1, q2);
	if (withObs && collision_state(P, q1, q2))
		return false;

	return true;
}

// Calls the Recursive Bi-Section algorithm (Hauser)
bool StateValidityChecker::checkMotionRBS(const ob::State *s1, const ob::State *s2)
{
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;

	State q1(n), q2(n);
	retrieveStateVector(s1,q1);
	retrieveStateVector(s2,q2);

	result = checkMotionRBS(q1, q2, 0, 0);

	return result;
}

// Implements local-connection using Recursive Bi-Section Technique (Hauser)
bool StateValidityChecker::checkMotionRBS(State s1, State s2, int recursion_depth, int non_decrease_count) {

	State s_mid(n);

	// Check if reached the required resolution
	double d = normDistance(s1, s2);
	if (d < RBS_tol)
		return true;

	if (recursion_depth > RBS_max_depth)// || non_decrease_count > 10)
		return false;

	// Interpolate
	for (int i = 0; i < n; i++)
		s_mid[i] = (s1[i]+s2[i])/2;

	// Check obstacles collisions and joint limits
	if (!isValidRBS(s_mid)) // Also updates s_mid with the projected value
		return false;

	//if ( normDistance(s1, s_mid) > d || normDistance(s_mid, s2) > d )
	//	non_decrease_count++;

	if ( checkMotionRBS(s1, s_mid, recursion_depth+1, non_decrease_count) && checkMotionRBS(s_mid, s2, recursion_depth+1, non_decrease_count) )
		return true;
	else
		return false;
}

// *************** Reconstruct the RBS - for post-processing and validation

// Calls the Recursive Bi-Section algorithm (Hauser)
bool StateValidityChecker::reconstructRBS(const ob::State *s1, const ob::State *s2, Matrix &Confs)
{
	State q1(n), q2(n);
	retrieveStateVector(s1,q1);
	retrieveStateVector(s2,q2);

	Confs.push_back(q1);
	Confs.push_back(q2);

	return reconstructRBS(q1, q2, Confs, 0, 1, 1);
}

bool StateValidityChecker::reconstructRBS(State q1, State q2, Matrix &M, int iteration, int last_index, int firstORsecond) {
	// firstORsecond - tells if the iteration is from the first or second call for the recursion (in the previous iteration).
	// last_index - the last index that was added to M.

	State q_mid(n);
	iteration++;

	// Check if reached the required resolution
	double d = normDistance(q1, q2);
	if (d < RBS_tol)
		return true;

	if (iteration > RBS_max_depth)
		return false;

	for (int i = 0; i < n; i++)
		q_mid[i] = (q1[i]+q2[i])/2;

	// Check obstacles collisions and joint limits
	if (!isValidRBS(q_mid)) // Also updates s_mid with the projected value
		return false; // Not suppose to happen since we run this function only when local connection feasibility is known

	if (firstORsecond==1)
		M.insert(M.begin()+last_index, q_mid); // Inefficient operation, but this is only for post-processing and validation
	else
		M.insert(M.begin()+(++last_index), q_mid); // Inefficient operation, but this is only for post-processing and validation

	int prev_size = M.size();
	if (!reconstructRBS(q1, q_mid, M, iteration, last_index, 1))
		return false;
	last_index += M.size()-prev_size;
	if (!reconstructRBS(q_mid, q2, M, iteration, last_index, 2))
		return false;

	return true;
}

// ------------------------------------ v Sewing v -------------------------------------------

// Validates a state by switching between the two possible active chains and computing the specific IK solution (input) and checking collision
bool StateValidityChecker::isValidSew(State& q) {

	isValid_counter++;

	State q1(6), q2(6);

	if (!GD(q))
		return false;

	q = get_GD_result();

	seperate_Vector(q, q1, q2);
	if (withObs && collision_state(P, q1, q2))
		return false;

	return true;
}

bool StateValidityChecker::checkMotionSew(const ob::State *s1, const ob::State *s2)
{
	State q(12), q1(12), q2(12);
	// We assume motion starts and ends in a valid configuration - due to projection
	retrieveStateVector(s1, q1);
	retrieveStateVector(s2, q2);

	double d = normDistance(q1,q2);
	while (d > dq) {
		// interpolate
		for (int i = 0; i < q1.size(); i++)
			q[i] = q1[i] + dq/d * (q2[i]-q1[i]);

		if (!isValidSew(q) || (d = normDistance(q1, q)) > RBS_tol)
			return false;

		q1 = q;
	}

}

// *************** Reconstruct the Sewing Local-Connection - for post-processing and validation

bool StateValidityChecker::reconstructSew(const ob::State *s1, const ob::State *s2, Matrix &Confs)
{
	State q1(n), q2(n);
	retrieveStateVector(s1,q1);
	retrieveStateVector(s2,q2);

	return reconstructSew(q1, q2, Confs);
}

bool StateValidityChecker::reconstructSew(State q1, State q2, Matrix &M) {

	State q(n);

	M.push_back(q1);

	double d = normDistance(q1,q2);

	while (d > RBS_tol) {
		// interpolate
		for (int i = 0; i < q1.size(); i++)
			q[i] = q1[i] + dq/d * (q2[i]-q1[i]);

		if (!isValidSew(q) || (d = normDistance(q1, q)) > RBS_tol)
			return false;

		M.push_back(q);
		q1 = q;
	}

	M.push_back(q2);

	return true;
}

// ------------------------------------ MISC functions ---------------------------------------------------

double StateValidityChecker::normDistance(State a1, State a2) {
	double sum = 0;
	for (int i=0; i < a1.size(); i++)
		sum += pow(a1[i]-a2[i], 2);
	return sqrt(sum);
}

double StateValidityChecker::stateDistance(const ob::State *s1, const ob::State *s2) {
	State q1(12), q2(12);
	retrieveStateVector(s1, q1);
	retrieveStateVector(s2, q2);

	return normDistance(q1, q2);
}

double StateValidityChecker::maxDistance(State a1, State a2) {
	double Max = 0;
	for (int i=0; i < a1.size(); i++)
		Max = fabs(a1[i]-a2[i]) > Max ? fabs(a1[i]-a2[i]) : Max ;
	return Max;
}

double StateValidityChecker::MaxAngleDistance(State a1, State a2) {
	double Max = 0;
	for (int i=0; i < a1.size(); i++)
		if (fabs(a1[i]-a2[i]) > Max)
			Max = fabs(a1[i]-a2[i]);
	return Max;
}

void StateValidityChecker::Join_States(State &q, State q1, State q2) {

	for (int i = 0; i < q1.size(); i++)
		q[i] = q1[i];

	int j = 0;
	for (int i = q1.size(); i < q1.size()+q1.size(); i++) {
		q[i] = q2[j];
		j++;
	}
}

void StateValidityChecker::seperate_Vector(State q, State &q1, State &q2) {

	for (int i = 0; i < q1.size(); i++)
		q1[i] = q[i];

	int j = 0;
	for (int i = q1.size(); i < q1.size()+q1.size(); i++) {
		q2[j] = q[i];
		j++;
	}
}


void StateValidityChecker::log_q(State q, bool New) {
	std::ofstream myfile;

	if (New) {
		myfile.open("./paths/path.txt");
		myfile << 1 << endl;
	}
	else
		myfile.open("./paths/path.txt", ios::app);

	for (int j = 0; j<12; j++)
		myfile << q[j] << " ";
	myfile << endl;

	myfile.close();
}



