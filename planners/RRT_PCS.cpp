/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

//#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

#include "RRT_PCS.h"

ompl::geometric::RRT::RRT(const base::SpaceInformationPtr &si) : base::Planner(si, "RRT"), StateValidityChecker(si)
{
	specs_.approximateSolutions = true;
	specs_.directed = true;

	defaultSettings(); // Avishai

	goalBias_ = 0.05;
	maxDistance_ = 0.0;
	lastGoalMotion_ = nullptr;

	Planner::declareParam<double>("range", this, &RRT::setRange, &RRT::getRange, "0.:1.:10000.");
	Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1.");

	Range = 2;
}

ompl::geometric::RRT::~RRT()
{
	freeMemory();
}

void ompl::geometric::RRT::clear()
{
	Planner::clear();
	sampler_.reset();
	freeMemory();
	if (nn_)
		nn_->clear();
	lastGoalMotion_ = nullptr;
}

void ompl::geometric::RRT::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);

	if (!nn_)
		nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	nn_->setDistanceFunction(std::bind(&RRT::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
}

void ompl::geometric::RRT::freeMemory()
{
	if (nn_)
	{
		std::vector<Motion*> motions;
		nn_->list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}
}

ompl::base::PlannerStatus ompl::geometric::RRT::solve(const base::PlannerTerminationCondition &ptc)
{
	State q1(6), q2(6), ik(2);
	initiate_log_parameters();
	setRange(Range); // Maximum local connection distance *** will need to profile this value

	base::State *start_node = si_->allocState();

	checkValidity();
	startTime = clock();
	base::Goal                 *goal   = pdef_->getGoal().get();
	base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

	while (const base::State *st = pis_.nextStart())
	{
		ik = identify_state_ik(st);
		Motion *motion = new Motion(si_);
		si_->copyState(motion->state, st);
		motion->ik_q1_active = ik[0];
		motion->ik_q2_active = ik[1];
		motion->a_chain = 0;
		nn_->add(motion);

		si_->copyState(start_node,st);
	}

	if (nn_->size() == 0)
	{
		OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

	Motion *solution  = nullptr;
	Motion *approxsol = nullptr;
	double  approxdif = std::numeric_limits<double>::infinity();
	Motion *rmotion   = new Motion(si_);
	base::State *rstate = rmotion->state;
	base::State *xstate = si_->allocState();

	base::State *gstate = si_->allocState();
	goal_s->sampleGoal(gstate);
	PlanDistance = si_->distance(start_node, gstate);
	Vector ik_goal = identify_state_ik(gstate);

	int active_chain;

	while (ptc == false)
	{
		/* sample random state (with goal biasing) */
		bool gg = false;
		if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
			goal_s->sampleGoal(rstate);
			gg = true;
		}
		else
			sampler_->sampleUniform(rstate);

		// Choose active chain
		active_chain = rand() % 2; // 0 - (q1,a) is the active chain, 1 - (q2,a) is the active chain

		/* find closest state in the tree */
		Motion *nmotion = nn_->nearest(rmotion);
		base::State *dstate = rstate;

		/* find state to add */
		bool reach = true;
		double d = si_->distance(nmotion->state, rstate);
		if (d > maxDistance_)
		{
			si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
			dstate = xstate;
			reach = false;
		}

		// If not goal, then must project
		if (!(gg && reach)) {
			retrieveStateVector(dstate, q1, q2);
			Matrix T = getQ();

			ik = {-1, -1};

			// Project dstate (currently not on the manifold)
			if (!active_chain) {
				if (!calc_specific_IK_solution_R1(T, q1, nmotion->ik_q1_active)) {
					if (!calc_specific_IK_solution_R2(T, q2, nmotion->ik_q2_active))
						continue;
					active_chain = !active_chain;
					q1 = get_IK_solution_q1();
					ik[1] =  nmotion->ik_q2_active;
				}
				else {
					q2 = get_IK_solution_q2();
					ik[0] =  nmotion->ik_q1_active;
				}
			}
			else {
				if (!calc_specific_IK_solution_R2(T, q2, nmotion->ik_q2_active)) {
					if (!calc_specific_IK_solution_R1(T, q1, nmotion->ik_q1_active))
						continue;
					active_chain = !active_chain;
					q2 = get_IK_solution_q2();
					ik[0] =  nmotion->ik_q1_active;
				}
				else {
					q1 = get_IK_solution_q1();
					ik[1] =  nmotion->ik_q2_active;
				}
			}

			ik = identify_state_ik(q1, q2, ik);
			updateStateVector(xstate, q1, q2);
			dstate = xstate;
		}
		else  // check if can connect to the goal
			ik = ik_goal;

		// Local connection using the Recursive Bi-Section (RBS)
		clock_t sT = clock();
		local_connection_count++;
		bool validMotion = false;
		if (nmotion->ik_q1_active == ik[0] && ik[0]!=-1)
			validMotion = checkMotionRBS(nmotion->state, dstate, 0, nmotion->ik_q1_active);
		if (!validMotion && nmotion->ik_q2_active == ik[1] && ik[1]!=-1)
			validMotion = checkMotionRBS(nmotion->state, dstate, 1, nmotion->ik_q2_active);
		local_connection_time += double(clock() - sT) / CLOCKS_PER_SEC;

		if (validMotion)
		{
			/* create a motion */
			Motion *motion = new Motion(si_);
			motion->ik_q1_active = ik[0];
			motion->ik_q2_active = ik[1];
			si_->copyState(motion->state, dstate);
			motion->a_chain = active_chain;
			motion->parent = nmotion;

			nn_->add(motion);
			double dist = 0.0;
			bool sat = goal->isSatisfied(motion->state, &dist);
			if (sat)
			{
				approxdif = dist;
				solution = motion;
				break;
			}
			if (dist < approxdif)
			{
				approxdif = dist;
				approxsol = motion;
			}
		}
	}

	bool solved = false;
	bool approximate = false;
	if (solution == nullptr)
	{
		solution = approxsol;
		approximate = true;
	}

	if (solution != nullptr)
	{
		total_runtime = double(clock() - startTime) / CLOCKS_PER_SEC;
		cout << "Solved in " << total_runtime << "s." << endl;

		lastGoalMotion_ = solution;

		/* construct the solution path */
		std::vector<Motion*> mpath;
		while (solution != nullptr)
		{
			mpath.push_back(solution);
			solution = solution->parent;
		}

		save2file(mpath);

		nodes_in_path = mpath.size();
		nodes_in_trees = nn_->size();

		/* set the solution path */
		PathGeometric *path = new PathGeometric(si_);
		for (int i = mpath.size() - 1 ; i >= 0 ; --i)
			path->append(mpath[i]->state);
		pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
		solved = true;
	}

	if (!solved)
	{
		// Report computation time
		total_runtime = double(clock() - startTime) / CLOCKS_PER_SEC;

		nodes_in_trees = nn_->size();
	}

	si_->freeState(xstate);
	if (rmotion->state)
		si_->freeState(rmotion->state);
	delete rmotion;

	final_solved = solved;
	LogPerf2file(); // Log planning parameters

	OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

	return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::RRT::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	std::vector<Motion*> motions;
	if (nn_)
		nn_->list(motions);

	if (lastGoalMotion_)
		data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

	for (unsigned int i = 0 ; i < motions.size() ; ++i)
	{
		if (motions[i]->parent == nullptr)
			data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
		else
			data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
					base::PlannerDataVertex(motions[i]->state));
	}
}

void ompl::geometric::RRT::save2file(vector<Motion*> mpath) {

	cout << "Logging path to files..." << endl;

	Vector q1(6), q2(6), ik(2);
	int active_chain, ik_sol;
	vector<Motion*> path;

	{
		// Open a_path file
		std::ofstream myfile;
		myfile.open("./paths/path_milestones.txt");

		myfile << mpath.size() << endl;

		for (int i = mpath.size()-1 ; i >= 0; i--) {
			retrieveStateVector(mpath[i]->state, q1, q2);
			for (int j = 0; j < 6; j++) {
				myfile << q1[j] << " ";
			}
			for (int j = 0; j < 6; j++) {
				myfile << q2[j] << " ";
			}
			myfile << endl;
			path.push_back(mpath[i]);
		}
		myfile.close();
	}

	{ // Reconstruct RBS

		// Open a_path file
		std::ofstream fp, myfile;
		std::ifstream myfile1;
		myfile.open("./paths/temp.txt",ios::out);

		retrieveStateVector(path[0]->state, q1, q2);
		for (int j = 0; j < 6; j++) {
			myfile << q1[j] << " ";
		}
		for (int j = 0; j < 6; j++) {
			myfile << q2[j] << " ";
		}
		myfile << endl;

		int count = 1;
		for (int i = 1; i < path.size(); i++) {

			Matrix M;
			bool valid = false;
			if (path[i]->ik_q1_active == path[i-1]->ik_q1_active)
				valid = reconstructRBS(path[i-1]->state, path[i]->state, M, 0, path[i-1]->ik_q1_active);
			if (!valid && path[i]->ik_q2_active == path[i-1]->ik_q2_active) {
				M.clear();
				valid = reconstructRBS(path[i-1]->state, path[i]->state, M, 1, path[i-1]->ik_q2_active);
			}

			if (!valid) {
				cout << "Error in reconstructing...\n";
				return;
			}

			for (int k = 1; k < M.size(); k++) {
				for (int j = 0; j<M[k].size(); j++) {
					myfile << M[k][j] << " ";
				}
				myfile << endl;
				count++;
			}
		}

		// Update file with number of conf.
		myfile.close();
		myfile1.open("./paths/temp.txt",ios::in);
		fp.open("./paths/path.txt",ios::out);
		fp << count << endl;
		std::string line;
		while(myfile1.good()) {
			std::getline(myfile1, line ,'\n');
			fp << line << endl;
		}
		myfile1.close();
		fp.close();
		std::remove("./paths/temp.txt");
	}
}

void ompl::geometric::RRT::LogPerf2file() {

	std::ofstream myfile;
	myfile.open("./paths/perf_log.txt");

	myfile << final_solved << endl;
	myfile << PlanDistance << endl; // Distance between nodes 1
	myfile << total_runtime << endl; // Overall planning runtime 2
	myfile << get_IK_counter() << endl; // How many IK checks? 5
	myfile << get_IK_time() << endl; // IK computation time 6
	myfile << get_collisionCheck_counter() << endl; // How many collision checks? 7
	myfile << get_collisionCheck_time() << endl; // Collision check computation time 8
	myfile << get_isValid_counter() << endl; // How many nodes checked 9
	myfile << nodes_in_path << endl; // Nodes in path 10
	myfile << nodes_in_trees << endl; // 11
	myfile << local_connection_time/local_connection_count << endl;

	myfile.close();
}
