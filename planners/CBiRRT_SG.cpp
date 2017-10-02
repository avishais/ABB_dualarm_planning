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

/* Author: Ioan Sucan, Avishai Sintov */

//#include "ompl/geometric/planners/rrt/CBiRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include "CBiRRT_SG.h"

// Debugging tool
template <class T>
void o(T a) {
	cout << a << endl;
}

ompl::geometric::CBiRRT::CBiRRT(const base::SpaceInformationPtr &si, double maxStep, int env) : base::Planner(si, "CBiRRT"), StateValidityChecker(si, env)
{
	specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
	specs_.directed = true;

	maxDistance_ = 0.0;

	Planner::declareParam<double>("range", this, &CBiRRT::setRange, &CBiRRT::getRange, "0.:1.:10000.");
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);

	defaultSettings();

	Range = maxStep;
}

ompl::geometric::CBiRRT::~CBiRRT()
{
	freeMemory();
}

void ompl::geometric::CBiRRT::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);

	if (!tStart_)
		tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	if (!tGoal_)
		tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	tStart_->setDistanceFunction(std::bind(&CBiRRT::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
	tGoal_->setDistanceFunction(std::bind(&CBiRRT::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
}

void ompl::geometric::CBiRRT::freeMemory()
{
	std::vector<Motion*> motions;

	if (tStart_)
	{
		tStart_->list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}

	if (tGoal_)
	{
		tGoal_->list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}
}

void ompl::geometric::CBiRRT::clear()
{
	Planner::clear();
	sampler_.reset();
	freeMemory();
	if (tStart_)
		tStart_->clear();
	if (tGoal_)
		tGoal_->clear();
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);
}

double ompl::geometric::CBiRRT::activeDistance(const Motion *a, const Motion *b) {

	State qa(6), qa_dummy(6);
	State qb(6), qb_dummy(6);

	retrieveStateVector(a->state, qa, qa_dummy);
	retrieveStateVector(b->state, qb, qb_dummy);

	double sum = 0;
	for (int i=0; i < qa.size(); i++)
		sum += pow(qa[i]-qb[i], 2);
	return sqrt(sum);
}

double ompl::geometric::CBiRRT::distanceBetweenTrees(TreeData &tree1, TreeData &tree2) {

	std::vector<Motion*> motions;
	tree1->list(motions);

	Motion *nmotion;
	double minD = 1e10, curD;
	for (unsigned int i = 0 ; i < motions.size() ; ++i)
	{
		nmotion = tree2->nearest(motions[i]);
		curD = distanceFunction(nmotion, motions[i]);
		if (curD < minD) {
			minD = curD;
		}
	}
	return minD;
}

ompl::geometric::CBiRRT::Motion* ompl::geometric::CBiRRT::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *nmotion, Motion *tmotion, int mode, int count_iterations)
// tmotion - target
// nmotion - nearest
// mode = 1 -> extend, mode = 2 -> connect.
{
	State q1(6), q2(6), ik(2);
	int ik_sol;

	bool reach = false;
	growTree_reached = false;

	//int count_iterations = 500;
	while (count_iterations) {
		count_iterations--;

		// find state to add
		base::State *dstate = tmotion->state;
		double d = activeDistance(nmotion, tmotion);

		if (d > maxDistance_)
		{
			si_->getStateSpace()->interpolate(nmotion->state, tmotion->state, maxDistance_ / d, tgi.xstate);
			dstate = tgi.xstate;
			reach = false;
		}
		else
			reach = true;

		if (mode==1 || !reach) { // equivalent to (!(mode==2 && reach))
			retrieveStateVector(dstate, q1, q2);
			Matrix T = getQ();

			if (nmotion->ik_q1_active == -2)
				ik_sol = rand() % 8;//rng_.uniformInt(0,7); // The nearest neighbor has a singularity, connect to it with a random IK solution
			else
				ik_sol = nmotion->ik_q1_active;

			IK_counter++;
			clock_t sT = clock();
			// Project dstate (currently not on the manifold)
			if (!calc_specific_IK_solution_R1(T, q1, ik_sol)) {
				sampling_time += double(clock() - sT) / CLOCKS_PER_SEC;
				sampling_counter[1]++;
				return nmotion;
			}

			q2 = get_IK_solution_q2();
			IK_time += double(clock() - sT) / CLOCKS_PER_SEC;

			if (collision_state(getPMatrix(), q1, q2)) {
				sampling_time += double(clock() - sT) / CLOCKS_PER_SEC;
				sampling_counter[1]++;
				return nmotion;
			}
			sampling_time += double(clock() - sT) / CLOCKS_PER_SEC;
			sampling_counter[0]++;

			updateStateVector(tgi.xstate, q1, q2);
			dstate = tgi.xstate;
		}
		else if (mode == 2) { // Target in tree
			ik_sol = tmotion->ik_q1_active;
			if (nmotion->ik_q1_active == -2 && ik_sol == -2) // Both singular
				return nmotion;

			if (nmotion->ik_q1_active != -2 && ik_sol != -2) {
				if (nmotion->ik_q1_active != ik_sol)
					return nmotion;
			}
			else
				if (ik_sol == -2)
					ik_sol = nmotion->ik_q1_active;
		}

		if (mode==3 && reach) {
			if (nmotion->ik_q1_active == -2)
				return nmotion;
			ik_sol = nmotion->ik_q1_active;
		}

		if (nmotion->ik_q1_active == -2 && ik_sol == -2) // Not necessary, but...
			return nmotion;

		// Check motion
		bool validMotion;
		clock_t sT = clock();
		local_connection_count++;
		if (mode==3 && reach) // dstate is singular
			validMotion = checkMotionRBS(nmotion->state, dstate, 2, 0, ik_sol);
		else if (nmotion->ik_q1_active == -2) // nmotion is singular
			validMotion = checkMotionRBS(nmotion->state, dstate, 1, 0, ik_sol);
		else
			validMotion = checkMotionRBS(nmotion->state, dstate, 0, ik_sol);
		//bool validMotion = checkMotion(nmotion->state, dstate, 0, ik_sol);
		local_connection_time += double(clock() - sT) / CLOCKS_PER_SEC;

		if (validMotion)
		{
			local_connection_success_count++;
			/* Update advanced motion */
			Motion *motion = new Motion(si_);
			motion->ik_q1_active = (mode == 3 && reach) ? -2 : ik_sol;
			si_->copyState(motion->state, dstate);
			motion->parent = nmotion;
			motion->root = nmotion->root;
			tgi.xmotion = motion;
			tree->add(motion);

			nmotion = motion;

			if (nmotion->ik_q1_active == -2)
				return nmotion;

			if (reach) {
				growTree_reached = true;
				return nmotion;
			}
		}
		else
			return nmotion;
	}
	return nmotion;
}

ompl::base::PlannerStatus ompl::geometric::CBiRRT::solve(const base::PlannerTerminationCondition &ptc)
{
	initiate_log_parameters();
	setRange(Range);
	base::State *start_node = si_->allocState();

	State q1(6), q2(6), ik(2);

	checkValidity();
	startTime = clock();
	base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

	if (!goal)
	{
		OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
		return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
	}

	while (const base::State *st = pis_.nextStart())
	{
		ik = identify_state_ik(st);
		retrieveStateVector(st, q1, q2);
		if (ik[0]==-1 || collision_state(getPMatrix(), q1, q2)) {
			OMPL_ERROR("%s: Start state not feasible!", getName().c_str());
			return base::PlannerStatus::INVALID_START;
		}
		Motion *motion = new Motion(si_);
		si_->copyState(motion->state, st);
		motion->root = motion->state;
		motion->ik_q1_active = ik[0];
		tStart_->add(motion);

		si_->copyState(start_node,st);
	}

	if (tStart_->size() == 0)
	{
		OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	if (!goal->couldSample())
	{
		OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
		return base::PlannerStatus::INVALID_GOAL;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(), (int)(tStart_->size() + tGoal_->size()));

	TreeGrowingInfo tgi;
	tgi.xstate = si_->allocState();

	Motion   *rmotion   = new Motion(si_);
	base::State *rstate = rmotion->state;
	bool startTree      = true;
	bool solved         = false;
	
	// ==============================
	
	/*cout << sampleSingular(rstate) << endl;
	ik = identify_state_ik(rstate);
	printStateVector(rstate);
	printVector(ik);*/
	
	/*base::State *s1 = si_->allocState();
	State q1a = {1.12043, -0.752519, -0.73328, 2.25027 ,-1.91537, 1.64312   };
	State q2a = {0.203096, 1.17365, -1.5708, -0.928081, 0, 2.49823};
	updateStateVector(s1, q1a, q2a);
	ik = identify_state_ik(s1);
	printVector(ik);
		
	base::State *s2 = si_->allocState();
	State q1b = {1.12043, -0.752519, -0.73328, 2.25027 ,-1.91537, 1.64312   };
	State q2b = {0.203096, 0.7, -0.9, -0.928081, 0.2, 2.49823};
	cout << IKproject(q1b, q2b, 1, ik[1]) << endl;
	
	q1 = {0.970051, -1.00187, -0.252599, 2.28423, -1.58884, 1.7927};
	q2 = {0.203096, 0.936825, -1.2354, -0.928081, 0.1, 2.49823}; 
	cout << IKproject(q1, q2, 0, 3) << endl;
	
	updateStateVector(s2, q1, q2);
	ik = identify_state_ik(s2);
	printVector(ik);
	
	//bool validMotion = checkMotionRBS(s1, s2, 0, ik[0]);
	//cout << validMotion << endl;
	
	log_q(s2);
	exit(1);*/
	
	// ==============================
	
	while (ptc == false)
	{
		TreeData &tree      = startTree ? tStart_ : tGoal_;
		tgi.start = startTree;
		startTree = !startTree;
		TreeData &otherTree = startTree ? tStart_ : tGoal_;

		if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
		{
			const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
			if (st)
			{
				ik = identify_state_ik(st);
				retrieveStateVector(st, q1, q2);
				if (ik[0]==-1 || collision_state(getPMatrix(), q1, q2)) {
					OMPL_ERROR("%s: Goal state not feasible!", getName().c_str());
					return base::PlannerStatus::INVALID_START;
				}
				Motion *motion = new Motion(si_);
				si_->copyState(motion->state, st);
				motion->root = motion->state;
				motion->ik_q1_active = ik[0];
				tGoal_->add(motion);
				PlanDistance = si_->distance(start_node, st);
			}

			if (tGoal_->size() == 0)
			{
				OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
				break;
			}
		}

		//cout << "Trees size: " << tStart_->size() << ", " << tGoal_->size() << endl;
		//cout << "Current trees distance: " << distanceBetweenTrees(tree, otherTree) << endl << endl;

		//==========================================================================

		Motion* reached_motion;
		int mode = 1;
		if (rng_.uniform01() > 0.20) // 20% of the sampling, sample a singular point.
			// sample random state
			sampler_->sampleUniform(rstate);
		else {
			// Sample singular configuration
			sampleSingular(rstate);
			mode = 3;
			//cout << "Sampled singular.\n";
		}

		// Grow tree
		Motion *nmotion = tree->nearest(rmotion); // NN over the active distance
		reached_motion = growTree(tree, tgi, nmotion, rmotion, mode, 500);

		// remember which motion was just added
		Motion *addedMotion = reached_motion;

		// Grow otherTree toward reached_motion in tree <-connect
		tgi.xmotion = nullptr;

		nmotion = otherTree->nearest(reached_motion); // NN over the active distance
		reached_motion = growTree(otherTree, tgi, nmotion, reached_motion, 2, 500);

		Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
		Motion *goalMotion  = startTree ? addedMotion : tgi.xmotion;

		// if we connected the trees in a valid way (start and goal pair is valid)
		if (growTree_reached) {

			// Report computation time
			endTime = clock();
			total_runtime = double(endTime - startTime) / CLOCKS_PER_SEC;
			cout << "Solved in " << total_runtime << "s." << endl;

			// it must be the case that either the start tree or the goal tree has made some progress
			// so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
			// on the solution path
			if (startMotion->parent)
				startMotion = startMotion->parent;
			else
				goalMotion = goalMotion->parent;

			connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

			// construct the solution path
			Motion *solution = startMotion;
			std::vector<Motion*> mpath1;
			while (solution != nullptr)
			{
				mpath1.push_back(solution);
				solution = solution->parent;
			}

			solution = goalMotion;
			std::vector<Motion*> mpath2;
			while (solution != nullptr)
			{
				mpath2.push_back(solution);
				solution = solution->parent;
			}

			cout << "Path from tree 1 size: " << mpath1.size() << ", path from tree 2 size: " << mpath2.size() << endl;
			nodes_in_path = mpath1.size() + mpath2.size();
			nodes_in_trees = tStart_->size() + tGoal_->size();

			PathGeometric *path = new PathGeometric(si_);
			path->getStates().reserve(mpath1.size() + mpath2.size());
			for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
				path->append(mpath1[i]->state);
			for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
				path->append(mpath2[i]->state);

			final_solved = true;
			LogPerf2file();
			ompl::geometric::CBiRRT::save2file(mpath1, mpath2);

			pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
			solved = true;
			break;
		}

		//====================================================
	}
	if (!solved)
	{
		// Report computation time
		endTime = clock();
		total_runtime = double(endTime - startTime) / CLOCKS_PER_SEC;

		nodes_in_trees = tStart_->size() + tGoal_->size();
		final_solved = false;
		LogPerf2file();
	}

	si_->freeState(tgi.xstate);
	si_->freeState(rstate);
	delete rmotion;

	OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());

	return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::CBiRRT::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	std::vector<Motion*> motions;
	if (tStart_)
		tStart_->list(motions);

	for (unsigned int i = 0 ; i < motions.size() ; ++i)
	{
		if (motions[i]->parent == nullptr)
			data.addStartVertex(base::PlannerDataVertex(motions[i]->state, 1));
		else
		{
			data.addEdge(base::PlannerDataVertex(motions[i]->parent->state, 1),
					base::PlannerDataVertex(motions[i]->state, 1));
		}
	}

	motions.clear();
	if (tGoal_)
		tGoal_->list(motions);

	for (unsigned int i = 0 ; i < motions.size() ; ++i)
	{
		if (motions[i]->parent == nullptr)
			data.addGoalVertex(base::PlannerDataVertex(motions[i]->state, 2));
		else
		{
			// The edges in the goal tree are reversed to be consistent with start tree
			data.addEdge(base::PlannerDataVertex(motions[i]->state, 2),
					base::PlannerDataVertex(motions[i]->parent->state, 2));
		}
	}

	// Add the edge connecting the two trees
	data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}

void ompl::geometric::CBiRRT::smoothPath(vector<Motion*> &path) {

	int orgSize = path.size();
	int s, c = 30;
	do {
		s = path.size();
		int i1, i2;
		do {
			i1 = rand() % path.size();
			i2 = rand() % path.size();
		} while (abs(i1-i2) < 2);

		bool valid = false;
		if (path[i1]->ik_q1_active == path[i2]->ik_q1_active && path[i2]->ik_q1_active != -1)
			valid =  checkMotionRBS(path[i1]->state, path[i2]->state, 0, path[i1]->ik_q1_active);
		if (!valid && path[i1]->ik_q2_active == path[i2]->ik_q2_active && path[i2]->ik_q2_active != -1) {
			valid =  checkMotionRBS(path[i1]->state, path[i2]->state, 1, path[i1]->ik_q2_active);
		}

		if (valid)
			path.erase(path.begin()+min(i1,i2)+1, path.begin()+max(i1,i2));

		if (path.size() == s)
			c--;
		else
			c = 30;
	} while (c > 0);

	cout << "Reduced path size from " << orgSize << " milestones to " << path.size() << " milestones." << endl;

	nodes_in_path = path.size();
}

void ompl::geometric::CBiRRT::save2file(vector<Motion*> mpath1, vector<Motion*> mpath2) {

	cout << "Logging path to files..." << endl;

	State q1(6), q2(6), ik(2);
	int ik_sol;

	{ // Log only milestones

		// Open a_path file
		std::ofstream myfile, ikfile;
		myfile.open("./paths/path_milestones.txt");

		myfile << mpath1.size() + mpath2.size() << endl;

		State temp;
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i) {
			retrieveStateVector(mpath1[i]->state, q1, q2);
			for (int j = 0; j<6; j++) {
				myfile << q1[j] << " ";
			}
			for (int j = 0; j<6; j++) {
				myfile << q2[j] << " ";
			}
			myfile << endl;
		}
		for (unsigned int i = 0 ; i < mpath2.size() ; ++i) {
			retrieveStateVector(mpath2[i]->state, q1, q2);
			for (int j = 0; j < 6; j++) {
				myfile << q1[j] << " ";
			}
			for (int j = 0; j < 6; j++) {
				myfile << q2[j] << " ";
			}
			myfile << endl;
		}
		myfile.close();
	}

	{ // Reconstruct RBS
		// Open a_path file
		std::ofstream fp, myfile;
		std::ifstream myfile1;
		myfile.open("./paths/temp.txt",ios::out);

		std::vector<Motion*> path;

		// Bulid basic path
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
			path.push_back(mpath1[i]);
		for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
			path.push_back(mpath2[i]);

		//smoothPath(path);

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
			/*bool valid = false;
			if (path[i]->ik_q1_active != -2 && path[i-1]->ik_q1_active != -2 && path[i]->ik_q1_active == path[i-1]->ik_q1_active)
				valid =  reconstructRBS(path[i-1]->state, path[i]->state, M, 0, path[i-1]->ik_q1_active);
			else if (path[i]->ik_q1_active == -2 && path[i-1]->ik_q1_active != -2)
				valid =  reconstructRBS(path[i-1]->state, path[i]->state, M, 0, path[i-1]->ik_q1_active);
			else if (path[i]->ik_q1_active != -2 && path[i-1]->ik_q1_active == -2)
				valid =  reconstructRBS(path[i-1]->state, path[i]->state, M, 0, path[i]->ik_q1_active);*/
			bool valid = reconstruct(path[i-1], path[i], M);

			if (!valid) {
				cout << "Error in reconstructing...\n";
				return;
			}

			for (int k = 1; k < M.size(); k++) {
				//pathLength += normDistance(M[k], M[k-1]);
				for (int j = 0; j < M[k].size(); j++) {
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

bool ompl::geometric::CBiRRT::reconstruct(Motion *m1, Motion *m2, Matrix &M) {
	
	State q1(6), q2(6);
	double d = si_->distance(m1->state, m2->state);
	int n = d / 0.1;
	
	ob::StateSpace *stateSpace_ = si_->getStateSpace().get();
	
	ob::State *test = si_->allocState();
	for (int i = 0; i <= n; i++) {
		stateSpace_->interpolate(m1->state, m2->state, (double)i / (double)n, test);
		
		retrieveStateVector(test, q1, q2);
		
		//cout << m1->ik_q1_active << m2->ik_q1_active << endl;
		
		bool valid = false;
		int ik_sol;
		if (m1->ik_q1_active == m2->ik_q1_active && m1->ik_q1_active != -2)
			ik_sol = m1->ik_q1_active;
		else if (m1->ik_q1_active == -2 && m2->ik_q1_active != -2)
			ik_sol = m2->ik_q1_active;
		else if (m1->ik_q1_active != -2 && m2->ik_q1_active == -2)
			ik_sol = m1->ik_q1_active;
		
		if (calc_specific_IK_solution_R1(getQ(), q1, ik_sol))
			q2 = get_IK_solution_q2();
		else
			return false;

		State q = join_Vectors(q1, q2);
		M.push_back(q);
	}
	
	return true;	
}
