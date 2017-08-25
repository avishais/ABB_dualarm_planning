/*
 * Checker.h
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

#ifndef CHECKER_H_
#define CHECKER_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/base/State.h"
#include <ompl/config.h>

#include "kdl_class.h"
#include "collisionDetection.h"

#include <iostream>

#define ROBOTS_DISTANCE 900
#define ROD_LENGTH 300

namespace ob = ompl::base;
using namespace std;

class StateValidityChecker : public collisionDetection, public kdl
{
public:
	/** Constructors */
	StateValidityChecker(const ob::SpaceInformationPtr &si) : mysi_(si.get()), kdl(ROBOTS_DISTANCE, ROD_LENGTH), collisionDetection(ROBOTS_DISTANCE,0,0,0) {q_prev.resize(12);setQ();setP();}; //Constructor // Avishai
	StateValidityChecker() : kdl(ROBOTS_DISTANCE, ROD_LENGTH), collisionDetection(ROBOTS_DISTANCE,0,0,0) {q_prev.resize(12);setQ();setP();}; //Constructor // Avishai

	/** Validity check using standard OMPL */
	bool isValid(const ob::State *);

	/** Validity check for a vector<double> type  */
	bool isValidRBS(State&);

	/** Serial local connection check  */
	bool checkMotion(const ob::State *, const ob::State *);

	/** Recursive Bi-Section local connection check  */
	bool checkMotionRBS(const ob::State *, const ob::State *);
	bool checkMotionRBS(State, State, int, int);

	/** Reconstruct a local connection using RBS for post-processing  */
	bool reconstructRBS(const ob::State *, const ob::State *, Matrix &);
	bool reconstructRBS(State, State, Matrix &, int, int, int);

	/** Sewing local connection check */
	bool isValidSew(State&);
	bool checkMotionSew(const ob::State *, const ob::State *);

	/** Reconstruct a local connection using Sewing strategy for post-processing  */
	bool reconstructSew(const ob::State *, const ob::State *, Matrix &);
	bool reconstructSew(State, State, Matrix &);

	/** Retrieve state from ob::State to vector<double> */
	void retrieveStateVector(const ob::State *, State &);

	/** Update state to ob::State from vector<double> */
	void updateStateVector(const ob::State *, State);

	/** Print ob::State ro console */
	void printStateVector(const ob::State *state);

	/** Set default OMPL setting */
	void defaultSettings();

	/** Calculate norm distance between two vectors */
	double normDistance(State, State);
	double stateDistance(const ob::State*, const ob::State*);

	/** Max distance between two vectors */
	double maxDistance(State, State);
	double MaxAngleDistance(State, State);

	/** Project a configuration in the ambient space to the constraint surface (and check collisions and joint limits) */
	bool IKproject(const ob::State *, bool = true);
	bool IKproject(State &, bool = true);

	/** Sample a random configuration */
	State sample_q();

	/** Join the two robots joint vectors */
	void Join_States(State&, State, State);

	/** Decouple the two robots joint vectors */
	void seperate_Vector(State, State &, State &);

	int get_valid_solution_index() {
		return valid_solution_index;
	}

	/** Return matrix of coordinated along the rod (in rod coordinate frame) */
	Matrix getPMatrix() {
		return P;
	}

	/** Return transformation matrix of rod end-tip in rod coordinate frame (at the other end-point) */
	Matrix getQ() {
		return Q;
	}

	/** Set transformation matrix of rod end-tip in rod coordinate frame (at the other end-point) */
	void setQ() {
		State v(4);

		v = {1,0,0,L};
		Q.push_back(v);
		v = {0,1,0,0};
		Q.push_back(v);
		v = {0,0,1,0};
		Q.push_back(v);
		v = {0,0,0,1};
		Q.push_back(v);
	}

	/** Set matrix of coordinated along the rod (in rod coordinate frame) */
	void setP() {
		State v(3);
		int dl = 20;
		int n = L / dl;
		for (int i = 0; i <= n; i++) {
			v = {(double)i*dl,0,0};
			P.push_back(v);
		}
	}

	/** Performance parameters measured during the planning */
	int isValid_counter;
	int get_isValid_counter() {
		return isValid_counter;
	}

	double iden = 0;
	double get_iden() {
		return iden;
	}

	int get_n() {
		return n;
	}
	double get_RBS_tol() {
		return RBS_tol;
	}

	void set_q_prev(State q) {
		for (int i = 0; i < q.size(); i++)
			q_prev[i] = q[i];
	}

	void log_q(State q, bool New);

private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation    *mysi_;
	int valid_solution_index;
	State q_prev;

	double L = ROD_LENGTH;
	Matrix Q;
	Matrix P;

	double dq = 0.05; // Serial local connection resolution
	bool withObs = true; // Include obstacles?
	double RBS_tol = 0.05; // RBS local connection resolution
	int RBS_max_depth = 150; // Maximum RBS recursion depth
	int n = 12; // Dimension of system
};





#endif /* CHECKER_H_ */
