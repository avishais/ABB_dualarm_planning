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

//#include "ompl/geometric/planners/sbl/SBL.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

#include "SBL_SG.h"

ompl::geometric::SBL::SBL(const base::SpaceInformationPtr &si, double maxStep, int env) : base::Planner(si, "SBL"), StateValidityChecker(si, env)
{
	specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
	maxDistance_ = 0.0;
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);

	Planner::declareParam<double>("range", this, &SBL::setRange, &SBL::getRange, "0.:1.:10000.");

	defaultSettings();

	Range = maxStep;
}

ompl::geometric::SBL::~SBL()
{
	freeMemory();
}

void ompl::geometric::SBL::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configureProjectionEvaluator(projectionEvaluator_);
	sc.configurePlannerRange(maxDistance_);

	tStart_.grid.setDimension(projectionEvaluator_->getDimension());
	tGoal_.grid.setDimension(projectionEvaluator_->getDimension());
}

void ompl::geometric::SBL::freeGridMotions(Grid<MotionInfo> &grid)
{
	for (Grid<MotionInfo>::iterator it = grid.begin(); it != grid.end() ; ++it)
	{
		for (unsigned int i = 0 ; i < it->second->data.size() ; ++i)
		{
			if (it->second->data[i]->state)
				si_->freeState(it->second->data[i]->state);
			delete it->second->data[i];
		}
	}
}

ompl::base::PlannerStatus ompl::geometric::SBL::solve(const base::PlannerTerminationCondition &ptc)
{
	State q1(6), q2(6), ik(2);
	int ik_goal, ik_start;

	initiate_log_parameters();
	setRange(Range); // Maximum local connection distance *** will need to profile this value

	base::State *start_node = si_->allocState();

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
		if (ik[0]==-1 || ik[1] == -1 || collision_state(getPMatrix(), q1, q2)) {
			OMPL_ERROR("%s: Start state not feasible!", getName().c_str());
			return base::PlannerStatus::INVALID_START;
		}

		Motion *motion = new Motion(si_);
		si_->copyState(motion->state, st);
		motion->valid = true;
		motion->root = motion->state;
		motion->ik_q1_active = ik[0];
		addMotion(tStart_, motion);

		si_->copyState(start_node,st);

		ik_start = ik[0];
	}

	if (tStart_.size == 0)
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
		sampler_ = si_->allocValidStateSampler();

	OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(), (int)(tStart_.size + tGoal_.size));

	std::vector<Motion*> solution;
	base::State *xstate = si_->allocState();

	bool      startTree = true;
	bool         solved = false;

	int ik_sol;

	while (ptc == false)
	{
		TreeData &tree      = startTree ? tStart_ : tGoal_;
		startTree = !startTree;
		TreeData &otherTree = startTree ? tStart_ : tGoal_;

		// if we have not sampled too many goals already
		if (tGoal_.size == 0 || pis_.getSampledGoalsCount() < tGoal_.size / 2)
		{
			const base::State *st = tGoal_.size == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
			if (st)
			{
				ik = identify_state_ik(st);
				retrieveStateVector(st, q1, q2);
				if (ik[0]==-1 || ik[1] == -1 || collision_state(getPMatrix(), q1, q2)) {
					OMPL_ERROR("%s: Start state not feasible!", getName().c_str());
					return base::PlannerStatus::INVALID_START;
				}

				Motion *motion = new Motion(si_);
				si_->copyState(motion->state, st);
				motion->root = motion->state;
				motion->ik_q1_active = ik[0];
				motion->valid = true;
				addMotion(tGoal_, motion);

				ik_goal = ik[0];

				PlanDistance = si_->distance(start_node, motion->state);
			}
			if (tGoal_.size == 0)
			{
				OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
				break;
			}
		}
		Motion *existing;// = selectMotion(tree);
		//assert(existing);

		bool sg = false;
		if (rng_.uniform01() > 0.05) { // 20% of the sampling, sample a singular point.
			existing = selectMotion(tree);
			assert(existing);

			// sample random state
			if (!sampler_->sampleNear(xstate, existing->state, maxDistance_))
				continue;
		}
		else {
			// Sample singular configuration
			sampleSingular(xstate);
			sg = true;
			Grid<MotionInfo>::Coord coord;
			projectionEvaluator_->computeCoordinates(xstate, coord);
			Grid<MotionInfo>::Cell* cell = tree.grid.getCell(coord);
			if (cell && !cell->data.empty()) {
				existing = cell->data[rng_.uniformInt(0, cell->data.size() - 1)];
			}
			else
				continue;
		}

		if (!sg) {
			// Project xstate (currently not on the manifold)
			retrieveStateVector(xstate, q1, q2);
			Matrix T = getQ();

			if (existing->ik_q1_active == -2)
				ik_sol = startTree ? ik_goal : ik_start;
				//rand() % 8;//rng_.uniformInt(0,7); // The nearest neighbor has a singularity, connect to it with a random IK solution
			else
				ik_sol = existing->ik_q1_active;

			clock_t sT = clock();
			if (!calc_specific_IK_solution_R1(T, q1, ik_sol)) {
				sampling_time += double(clock() - sT) / CLOCKS_PER_SEC;
				sampling_counter[1]++;
				continue;
			}

			q2 = get_IK_solution_q2();

			if (collision_state(getPMatrix(), q1, q2)) {
				sampling_time += double(clock() - sT) / CLOCKS_PER_SEC;
				sampling_counter[1]++;
				continue;
			}
			sampling_time += double(clock() - sT) / CLOCKS_PER_SEC;
			sampling_counter[0]++;

			updateStateVector(xstate, q1, q2);
		}

		/* create a motion */
		Motion *motion = new Motion(si_);
		motion->ik_q1_active = sg ? -2 : ik_sol;
		si_->copyState(motion->state, xstate);
		motion->parent = existing;
		motion->root = existing->root;
		existing->children.push_back(motion);

		addMotion(tree, motion);

		if (checkSolution(!startTree, tree, otherTree, motion, solution))
		{
			PathGeometric *path = new PathGeometric(si_);
			for (unsigned int i = 0 ; i < solution.size() ; ++i)
				path->append(solution[i]->state);

			pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
			solved = true;
			final_solved = true;
			LogPerf2file(); // Log planning parameters
			break;
		}
	}

	if (!solved)
	{
		// Report computation time
		endTime = clock();
		total_runtime = double(endTime - startTime) / CLOCKS_PER_SEC;

		nodes_in_trees = tStart_.size + tGoal_.size;

		final_solved = false;
		LogPerf2file(); // Log planning parameters
	}


	si_->freeState(xstate);

	OMPL_INFORM("%s: Created %u (%u start + %u goal) states in %u cells (%u start + %u goal)",
			getName().c_str(), tStart_.size + tGoal_.size, tStart_.size, tGoal_.size,
			tStart_.grid.size() + tGoal_.grid.size(), tStart_.grid.size(), tGoal_.grid.size());

	return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

bool ompl::geometric::SBL::checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion *motion, std::vector<Motion*> &solution)
{
	Grid<MotionInfo>::Coord coord;
	projectionEvaluator_->computeCoordinates(motion->state, coord);
	Grid<MotionInfo>::Cell* cell = otherTree.grid.getCell(coord);

	if (cell && !cell->data.empty())
	{
		Motion *connectOther = cell->data[rng_.uniformInt(0, cell->data.size() - 1)];

		//cout << connectOther->ik_q1_active << " " << motion->ik_q1_active << endl;

		// Check if a connection is even possible considering connected components
		if ( connectOther->ik_q1_active == -2 && motion->ik_q1_active == -2)
			return false;
		if ( connectOther->ik_q1_active != motion->ik_q1_active)
			if (connectOther->ik_q1_active != -2 && motion->ik_q1_active != -2)
				return false;

		if (pdef_->getGoal()->isStartGoalPairValid(start ? motion->root : connectOther->root, start ? connectOther->root : motion->root))
		{
			Motion *connect = new Motion(si_);

			si_->copyState(connect->state, connectOther->state);
			connect->parent = motion;
			connect->root = motion->root;
			connect->ik_q1_active = connectOther->ik_q1_active;
			motion->children.push_back(connect);
			addMotion(tree, connect);

			if (isPathValid(tree, connect) && isPathValid(otherTree, connectOther))
			{
				// Solution found, report computation time
				endTime = clock();
				total_runtime = double(endTime - startTime) / CLOCKS_PER_SEC;
				cout << "Solved in " << total_runtime << "s." << endl;

				if (start)
					connectionPoint_ = std::make_pair(motion->state, connectOther->state);
				else
					connectionPoint_ = std::make_pair(connectOther->state, motion->state);

				/* extract the motions and put them in solution vector */

				std::vector<Motion*> mpath1;
				while (motion != nullptr)
				{
					mpath1.push_back(motion);
					motion = motion->parent;
				}

				std::vector<Motion*> mpath2;
				while (connectOther != nullptr)
				{
					mpath2.push_back(connectOther);
					connectOther = connectOther->parent;
				}

				if (!start)
					mpath1.swap(mpath2);

				save2file(mpath1, mpath2);
				cout << "Path from tree 1 size: " << mpath1.size() << ", path from tree 2 size: " << mpath2.size() << endl;
				nodes_in_path = mpath1.size() + mpath2.size();
				nodes_in_trees = tree.size + otherTree.size;

				for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
					solution.push_back(mpath1[i]);
				solution.insert(solution.end(), mpath2.begin(), mpath2.end());

				return true;
			}
		}
	}
	return false;
}

bool ompl::geometric::SBL::isPathValid(TreeData &tree, Motion *motion)
{
	std::vector<Motion*> mpath;

	/* construct the solution path */
	while (motion != nullptr)
	{
		mpath.push_back(motion);
		motion = motion->parent;
	}

	/* check the path */
	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
		if (!mpath[i]->valid)
		{
			clock_t sT = clock();
			local_connection_count++;
			bool validMotion = false;
			if (mpath[i]->parent->ik_q1_active == -2) {
				validMotion = checkMotionRBS(mpath[i]->parent->state, mpath[i]->state, 1, 0, mpath[i]->ik_q1_active);
				//cout << "sg1\n";
			}
			else if (mpath[i]->ik_q1_active == -2) {
				validMotion = checkMotionRBS(mpath[i]->parent->state, mpath[i]->state, 2, 0, mpath[i]->parent->ik_q1_active);
				//cout << "sg2\n";
			}
			else if (mpath[i]->parent->ik_q1_active == mpath[i]->ik_q1_active) {
				validMotion = checkMotionRBS(mpath[i]->parent->state, mpath[i]->state, 0, mpath[i]->ik_q1_active);
				//cout << "sg0\n";
			}
			local_connection_time += double(clock() - sT) / CLOCKS_PER_SEC;

			if (validMotion) {
				local_connection_success_count++;
				mpath[i]->valid = true;
			}
			else
			{
				removeMotion(tree, mpath[i]);
				return false;
			}
		}
	return true;
}

ompl::geometric::SBL::Motion* ompl::geometric::SBL::selectMotion(TreeData &tree)
{
	GridCell* cell = tree.pdf.sample(rng_.uniform01());
	return cell && !cell->data.empty() ? cell->data[rng_.uniformInt(0, cell->data.size() - 1)] : nullptr;
}

void ompl::geometric::SBL::removeMotion(TreeData &tree, Motion *motion)
{
	/* remove from grid */

	Grid<MotionInfo>::Coord coord;
	projectionEvaluator_->computeCoordinates(motion->state, coord);
	Grid<MotionInfo>::Cell* cell = tree.grid.getCell(coord);
	if (cell)
	{
		for (unsigned int i = 0 ; i < cell->data.size(); ++i)
		{
			if (cell->data[i] == motion)
			{
				cell->data.erase(cell->data.begin() + i);
				tree.size--;
				break;
			}
		}
		if (cell->data.empty())
		{
			tree.pdf.remove(cell->data.elem_);
			tree.grid.remove(cell);
			tree.grid.destroyCell(cell);
		}
		else
		{
			tree.pdf.update(cell->data.elem_, 1.0/cell->data.size());
		}
	}

	/* remove self from parent list */

	if (motion->parent)
	{
		for (unsigned int i = 0 ; i < motion->parent->children.size() ; ++i)
		{
			if (motion->parent->children[i] == motion)
			{
				motion->parent->children.erase(motion->parent->children.begin() + i);
				break;
			}
		}
	}

	/* remove children */
	for (unsigned int i = 0 ; i < motion->children.size() ; ++i)
	{
		motion->children[i]->parent = nullptr;
		removeMotion(tree, motion->children[i]);
	}

	if (motion->state)
		si_->freeState(motion->state);
	delete motion;
}

void ompl::geometric::SBL::addMotion(TreeData &tree, Motion *motion)
{
	Grid<MotionInfo>::Coord coord;
	projectionEvaluator_->computeCoordinates(motion->state, coord);
	Grid<MotionInfo>::Cell* cell = tree.grid.getCell(coord);
	if (cell)
	{
		cell->data.push_back(motion);
		tree.pdf.update(cell->data.elem_, 1.0/cell->data.size());
	}
	else
	{
		cell = tree.grid.createCell(coord);
		cell->data.push_back(motion);
		tree.grid.add(cell);
		cell->data.elem_ = tree.pdf.add(cell, 1.0);
	}
	tree.size++;
}

void ompl::geometric::SBL::clear()
{
	Planner::clear();

	sampler_.reset();

	freeMemory();

	tStart_.grid.clear();
	tStart_.size = 0;
	tStart_.pdf.clear();

	tGoal_.grid.clear();
	tGoal_.size = 0;
	tGoal_.pdf.clear();
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);
}

void ompl::geometric::SBL::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	std::vector<MotionInfo> motions;
	tStart_.grid.getContent(motions);

	for (unsigned int i = 0 ; i < motions.size() ; ++i)
		for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
			if (motions[i][j]->parent == nullptr)
				data.addStartVertex(base::PlannerDataVertex(motions[i][j]->state, 1));
			else
				data.addEdge(base::PlannerDataVertex(motions[i][j]->parent->state, 1),
						base::PlannerDataVertex(motions[i][j]->state, 1));

	motions.clear();
	tGoal_.grid.getContent(motions);
	for (unsigned int i = 0 ; i < motions.size() ; ++i)
		for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
			if (motions[i][j]->parent == nullptr)
				data.addGoalVertex(base::PlannerDataVertex(motions[i][j]->state, 2));
			else
				// The edges in the goal tree are reversed so that they are in the same direction as start tree
				data.addEdge(base::PlannerDataVertex(motions[i][j]->state, 2),
						base::PlannerDataVertex(motions[i][j]->parent->state, 2));

	data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}


void ompl::geometric::SBL::save2file(vector<Motion*> mpath1, vector<Motion*> mpath2) {

	cout << "Logging path to files..." << endl;

	State q1(6), q2(6), ik(2);
	int active_chain, ik_sol;

	{ // Log only milestones

		// Open a_path file
		std::ofstream myfile, ikfile;
		myfile.open("../paths/path_milestones.txt");
		ikfile.open("../paths/ik_path.txt");

		myfile << mpath1.size() + mpath2.size() << endl;

		State temp;
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i) {
			retrieveStateVector(mpath1[i]->state, q1, q2);
			ikfile << mpath1[i]->a_chain << " " << mpath1[i]->ik_q1_active << " " << mpath1[i]->ik_q2_active << endl;
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
			ikfile << mpath2[i]->a_chain << " " << mpath2[i]->ik_q1_active << " " << mpath2[i]->ik_q2_active << endl;
			for (int j = 0; j < 6; j++) {
				myfile << q1[j] << " ";
			}
			for (int j = 0; j < 6; j++) {
				myfile << q2[j] << " ";
			}
			myfile << endl;
		}
		myfile.close();
		ikfile.close();
	}

	{ // Reconstruct RBS
		// Open a_path file
		std::ofstream fp, myfile;
		std::ifstream myfile1;
		myfile.open("./paths/temp.txt",ios::out);

		//myfile << mpath1.size() + mpath2.size() << endl;

		std::vector<Motion*> path;

		// Bulid basic path
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
			path.push_back(mpath1[i]);
		for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
			path.push_back(mpath2[i]);

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
			if (path[i]->ik_q1_active == -2)
				valid = reconstructRBS(path[i-1]->state, path[i]->state, M, 2, 0, path[i-1]->ik_q1_active);
			else if (path[i-1]->ik_q1_active == -2)
				valid = reconstructRBS(path[i-1]->state, path[i]->state, M, 1, 0, path[i]->ik_q1_active);
			else if (path[i]->ik_q1_active == path[i-1]->ik_q1_active)
				valid = reconstructRBS(path[i-1]->state, path[i]->state, M, 0, path[i-1]->ik_q1_active);

			if (!valid) {
				cout << "Error in reconstructing...\n";
				return;
			}

			for (int k = 1; k<M.size(); k++) {
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

