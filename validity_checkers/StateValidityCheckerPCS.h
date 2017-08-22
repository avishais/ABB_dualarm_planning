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

#include "apc_class.h"
#include "collisionDetection.h"

#include <iostream>

#define ROBOTS_DISTANCE 900
#define ROD_LENGTH 300

namespace ob = ompl::base;
using namespace std;

class StateValidityChecker : public two_robots, public collisionDetection
{
public:
	/** Constructors */
	StateValidityChecker(const ob::SpaceInformationPtr &si) : mysi_(si.get()), two_robots({-ROBOTS_DISTANCE/2, 0, 0 }, {ROBOTS_DISTANCE/2, 0, PI}, ROD_LENGTH), collisionDetection(ROBOTS_DISTANCE,0,0,0) {q_temp.resize(6);setQ();setP();}; //Constructor // Avishai
	StateValidityChecker() : two_robots({-ROBOTS_DISTANCE/2, 0, 0 }, {ROBOTS_DISTANCE/2, 0, PI}, ROD_LENGTH), collisionDetection(ROBOTS_DISTANCE,0,0,0) {q_temp.resize(6);setQ();setP();}; //Constructor // Avishai

	/** Validity check using standard OMPL */
	bool isValid(const ob::State *);
	bool isValid(const ob::State *, int, int);

	/** Validity check for a vector<double> type  */
	bool isValidRBS(Vector&, Vector&, int, int);

	/** Serial local connection check  */
	bool checkMotion(const ob::State *, const ob::State *, int, int);

	/** Recursive Bi-Section local connection check  */
	bool checkMotionRBS(const ob::State *, const ob::State *, int, int);
	bool checkMotionRBS(Vector, Vector, Vector, Vector, int, int, int, int);

	/** Reconstruct a local connection using RBS for post-processing  */
	bool reconstructRBS(const ob::State *, const ob::State *, Matrix &, int, int);
	bool reconstructRBS(Vector, Vector, Vector, Vector, int, int, Matrix &, int, int, int);

	/** Norm distance of 2 vectors while each is separated */
	double normDistanceDuo(Vector, Vector, Vector, Vector);

	/** Return mid-point of two vectors for the RBS */
	void midpoint(Vector, Vector, Vector, Vector, Vector &, Vector&);

	/** Retrieve state from ob::State to vector<double> */
	void retrieveStateVector(const ob::State *, Vector &, Vector &);
	void retrieveStateVector(const ob::State *, Vector &, Vector &, Vector &);
	void retrieveStateVector(const ob::State *, Vector &);

	/** Update state to ob::State from vector<double> */
	void updateStateVector(const ob::State *, Vector, Vector);
	void updateStateVector(const ob::State *, Vector, Vector, Vector);
	void updateStateVector(const ob::State *, Vector);

	/** Print ob::State to console */
	void printStateVector(const ob::State *);

	/** Set default OMPL setting */
	void defaultSettings();

	/** Calculate norm distance between two vectors */
	double normDistance(Vector, Vector);

	/** Calculate norm distance between two ob::State's */
	double stateDistance(const ob::State *, const ob::State *);

	/** Close chain (project) */
	bool close_chain(const ob::State *, int);

	/** Sample a random configuration */
	Vector sample_q();

	/** Project a configuration in the ambient space to the constraint surface (and check collisions and joint limits) */
	bool IKproject(Vector &, Vector &, int, int = -1);

	/** Identify the IK solutions of a configuration using two passive chains */
	Vector identify_state_ik(const ob::State *, Vector = {-1, -1});
	Vector identify_state_ik(Vector, Vector, Vector = {-1, -1});

	/** Join the two robots joint vectors */
	Vector join_Vectors(Vector, Vector);

	/** Decouple the two robots joint vectors */
	void seperate_Vector(Vector, Vector &, Vector &);

	/** Log configuration to path file */
	void log_q(ob::State *);

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
		Vector v(4);

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
		Vector v(3);
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

	double get_RBS_tol(){
		return RBS_tol;
	}
private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation    *mysi_;
	Vector q_temp;
	int valid_solution_index;

	double L = ROD_LENGTH;
	Matrix Q;
	Matrix P;

	bool withObs = true; // Include obstacles?
	double RBS_tol = 0.05; // RBS local connection resolution
	int RBS_max_depth = 150; // Maximum RBS recursion depth


};





#endif /* CHECKER_H_ */
