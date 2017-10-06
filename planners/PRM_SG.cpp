/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

/* Author: Ioan Sucan, James D. Marble, Ryan Luna, Avishai Sintov */

//#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/datastructures/PDF.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <thread>

#include "GoalVisitor.hpp"
#include "PRM_SG.h"

#define foreach BOOST_FOREACH

namespace ompl
{
namespace magic
{

/** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of PRM. */
static const unsigned int MAX_RANDOM_BOUNCE_STEPS   = 5;

/** \brief The time in seconds for a single roadmap building operation (dt)*/
static const double ROADMAP_BUILD_TIME = 0.2;

/** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
}
}

ompl::geometric::PRM::PRM(const base::SpaceInformationPtr &si, int env, bool starStrategy) :
    		base::Planner(si, "PRM"),
			starStrategy_(starStrategy),
			stateProperty_(boost::get(vertex_state_t(), g_)),
			totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_)),
			successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_)),
			weightProperty_(boost::get(boost::edge_weight, g_)),
			disjointSets_(boost::get(boost::vertex_rank, g_),
			boost::get(boost::vertex_predecessor, g_)),
			userSetConnectionStrategy_(false),
			addedNewSolution_(false),
			iterations_(0),
			bestCost_(std::numeric_limits<double>::quiet_NaN()),
			StateValidityChecker(si, env)
{
	specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
	specs_.approximateSolutions = false;
	specs_.optimizingPaths = true;
	specs_.multithreaded = true;

	if (!starStrategy_)
		Planner::declareParam<unsigned int>("max_nearest_neighbors", this, &PRM::setMaxNearestNeighbors, std::string("8:1000"));

	addPlannerProgressProperty("iterations INTEGER",
			std::bind(&PRM::getIterationCount, this));
	addPlannerProgressProperty("best cost REAL",
			std::bind(&PRM::getBestCost, this));
	addPlannerProgressProperty("milestone count INTEGER",
			std::bind(&PRM::getMilestoneCountString, this));
	addPlannerProgressProperty("edge count INTEGER",
			std::bind(&PRM::getEdgeCountString, this));

	defaultSettings(); // Avishai
}

ompl::geometric::PRM::~PRM()
{
	freeMemory();
}

void ompl::geometric::PRM::setup()
{
	Planner::setup();
	if (!nn_)
	{
		specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
		nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
		specs_.multithreaded = true;
		nn_->setDistanceFunction(std::bind(&PRM::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
	}
	if (!connectionStrategy_)
	{
		if (starStrategy_)
			connectionStrategy_ = KStarStrategy<Vertex>(std::bind(&PRM::milestoneCount, this), nn_, si_->getStateDimension());
		else
			connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
	}
	if (!connectionFilter_)
		connectionFilter_ = [] (const Vertex&, const Vertex&) { return true; };

	// Setup optimization objective
	//
	// If no optimization objective was specified, then default to
	// optimizing path length as computed by the distance() function
	// in the state space.
	if (pdef_)
	{
		if (pdef_->hasOptimizationObjective())
			opt_ = pdef_->getOptimizationObjective();
		else
		{
			opt_.reset(new base::PathLengthOptimizationObjective(si_));
			if (!starStrategy_)
				opt_->setCostThreshold(opt_->infiniteCost());
		}
	}
	else
	{
		OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
		setup_ = false;
	}
}

void ompl::geometric::PRM::setMaxNearestNeighbors(unsigned int k)
{
	if (starStrategy_)
		throw Exception("Cannot set the maximum nearest neighbors for " + getName());
	if (!nn_)
	{
		specs_.multithreaded = false; // temporarily set to false since nn_ is used only in single thread
		nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
		specs_.multithreaded = true;
		nn_->setDistanceFunction(std::bind(&PRM::distanceFunction, this, std::placeholders::_1, std::placeholders::_2));
	}
	if (!userSetConnectionStrategy_)
		connectionStrategy_ = ConnectionStrategy();
	if (isSetup())
		setup();
}

void ompl::geometric::PRM::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
	Planner::setProblemDefinition(pdef);
	clearQuery();
}

void ompl::geometric::PRM::clearQuery()
{
	startM_.clear();
	goalM_.clear();
	pis_.restart();
}

void ompl::geometric::PRM::clear()
{
	Planner::clear();
	sampler_.reset();
	simpleSampler_.reset();
	freeMemory();
	if (nn_)
		nn_->clear();
	clearQuery();

	iterations_ = 0;
	bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::PRM::freeMemory()
{
	foreach (Vertex v, boost::vertices(g_))
        		si_->freeState(stateProperty_[v]);
	g_.clear();
}

void ompl::geometric::PRM::expandRoadmap(double expandTime)
{
	expandRoadmap(base::timedPlannerTerminationCondition(expandTime));
}

void ompl::geometric::PRM::expandRoadmap(const base::PlannerTerminationCondition &ptc)
{
	if (!simpleSampler_)
		simpleSampler_ = si_->allocStateSampler();

	std::vector<base::State*> states(magic::MAX_RANDOM_BOUNCE_STEPS);
	si_->allocStates(states);
	expandRoadmap(ptc, states);
	si_->freeStates(states);
}

void ompl::geometric::PRM::expandRoadmap(const base::PlannerTerminationCondition &ptc,
		std::vector<base::State*> &workStates)
{
	// construct a probability distribution over the vertices in the roadmap
	// as indicated in
	//  "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces"
	//        Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars

	PDF<Vertex> pdf;
	foreach (Vertex v, boost::vertices(g_))
	{
		const unsigned long int t = totalConnectionAttemptsProperty_[v];
		pdf.add(v, (double)(t - successfulConnectionAttemptsProperty_[v]) / (double)t);
	}

	if (pdf.empty())
		return;

	while (ptc == false)
	{
		iterations_++;
		Vertex v = pdf.sample(rng_.uniform01());
		unsigned int s = si_->randomBounceMotion(simpleSampler_, stateProperty_[v], workStates.size(), workStates, false);
		if (s > 0)
		{
			s--;
			Vertex last = addMilestone(si_->cloneState(workStates[s]));

			graphMutex_.lock();
			for (unsigned int i = 0 ; i < s ; ++i)
			{
				// add the vertex along the bouncing motion
				Vertex m = boost::add_vertex(g_);
				stateProperty_[m] = si_->cloneState(workStates[i]);
				totalConnectionAttemptsProperty_[m] = 1;
				successfulConnectionAttemptsProperty_[m] = 0;
				disjointSets_.make_set(m);

				// add the edge to the parent vertex
				const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[m]);
				const Graph::edge_property_type properties(weight);
				boost::add_edge(v, m, properties, g_);
				uniteComponents(v, m);

				// add the vertex to the nearest neighbors data structure
				nn_->add(m);
				v = m;
			}

			// if there are intermediary states or the milestone has not been connected to the initially sampled vertex,
			// we add an edge
			if (s > 0 || !sameComponent(v, last))
			{
				// add the edge to the parent vertex
				const base::Cost weight = opt_->motionCost(stateProperty_[v], stateProperty_[last]);
				const Graph::edge_property_type properties(weight);
				boost::add_edge(v, last, properties, g_);
				uniteComponents(v, last);
			}
			graphMutex_.unlock();
		}
	}
}

void ompl::geometric::PRM::growRoadmap(double growTime)
{
	growRoadmap(base::timedPlannerTerminationCondition(growTime));
}

void ompl::geometric::PRM::growRoadmap(const base::PlannerTerminationCondition &ptc)
{
	if (!isSetup())
		setup();
	if (!sampler_)
		sampler_ = si_->allocValidStateSampler();

	base::State *workState = si_->allocState();
	growRoadmap (ptc, workState);
	si_->freeState(workState);
}

void ompl::geometric::PRM::growRoadmap(const base::PlannerTerminationCondition &ptc,
		base::State *workState)
{
	State q_rand(12);

	/* grow roadmap in the regular fashion -- sample valid states, add them to the roadmap, add valid connections */
	while (ptc == false)
	{
		iterations_++;
		// search for a valid state
		bool found = false;
		while (!found && ptc == false)
		{
			unsigned int attempts = 0;
			do
			{
				if (rng_.uniform01() > 0.30)
					found = sample_q(workState, ConCom); // Use custom sampler
				else {
					found = sampleSingular(workState);
					updateStateVector(workState, {-2, -1});
				}
				//found = sampler_->sample(workState);

				attempts++;
			} while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
		}
		// add it as a milestone
		if (found) {
			addMilestone(si_->cloneState(workState));
		}
	}
}

void ompl::geometric::PRM::checkForSolution(const base::PlannerTerminationCondition &ptc,
		base::PathPtr &solution)
{
	base::GoalSampleableRegion *goal = static_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
	while (!ptc && !addedNewSolution_)
	{
		// Check for any new goal states
		if (goal->maxSampleCount() > goalM_.size())
		{
			const base::State *st = pis_.nextGoal();
			if (st)
				goalM_.push_back(addMilestone(si_->cloneState(st)));
		}

		// Check for a solution
		addedNewSolution_ = maybeConstructSolution(startM_, goalM_, solution);
		// Sleep for 1ms
		if (!addedNewSolution_)
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

bool ompl::geometric::PRM::maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution)
{
	base::Goal *g = pdef_->getGoal().get();
	base::Cost sol_cost(opt_->infiniteCost());
	foreach (Vertex start, starts)
	{
		foreach (Vertex goal, goals)
        		{
			// we lock because the connected components algorithm is incremental and may change disjointSets_
			graphMutex_.lock();
			bool same_component = sameComponent(start, goal);
			graphMutex_.unlock();

			if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
			{
				base::PathPtr p = constructSolution(start, goal);
				if (p)
				{
					base::Cost pathCost = p->cost(opt_);
					if (opt_->isCostBetterThan(pathCost, bestCost_))
						bestCost_ = pathCost;
					// Check if optimization objective is satisfied
					if (opt_->isSatisfied(pathCost))
					{
						solution = p;
						return true;
					}
					else if (opt_->isCostBetterThan(pathCost, sol_cost))
					{
						solution = p;
						sol_cost = pathCost;
					}
				}
			}
        		}
	}

	return false;
}

bool ompl::geometric::PRM::addedNewSolution() const
{
	return addedNewSolution_;
}

ompl::base::PlannerStatus ompl::geometric::PRM::solve(const base::PlannerTerminationCondition &ptc)
{
	/*const base::State *st1 = pis_.nextStart();
	printStateVector(st1);
	State ik = identify_state_ik(st1);
	printVector(ik);
	const base::State *st2 = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
	printStateVector(st1);
	ik = identify_state_ik(st2);
	printVector(ik);
	exit(1);*/

	initiate_log_parameters();

	checkValidity();
	startTime = clock();
	base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

	if (!goal)
	{
		OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
		return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
	}

	// Add the valid start states as milestones
	while (const base::State *st = pis_.nextStart()) {
		State ik = identify_state_ik(st);
		updateStateVector(st, ik);
		ConCom.push_back(ik[0]);
		startM_.push_back(addMilestone(si_->cloneState(st)));
	}

	if (startM_.size() == 0)
	{
		OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	if (!goal->couldSample())
	{
		OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
		return base::PlannerStatus::INVALID_GOAL;
	}

	// Ensure there is at least one valid goal state
	if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
	{
		const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
		State ik = identify_state_ik(st);
		updateStateVector(st, ik);
		ConCom.push_back(ik[0]);
		if (st)
			goalM_.push_back(addMilestone(si_->cloneState(st)));

		if (goalM_.empty())
		{
			OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
			return base::PlannerStatus::INVALID_GOAL;
		}
	}

	unsigned long int nrStartStates = boost::num_vertices(g_);
	OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

	// Reset addedNewSolution_ member and create solution checking thread
	addedNewSolution_ = false;
	base::PathPtr sol;
	std::thread slnThread(std::bind(&PRM::checkForSolution, this, ptc, boost::ref(sol)));

	// construct new planner termination condition that fires when the given ptc is true, or a solution is found
	base::PlannerTerminationCondition ptcOrSolutionFound =
			base::plannerOrTerminationCondition(ptc, base::PlannerTerminationCondition(std::bind(&PRM::addedNewSolution, this)));

	constructRoadmap(ptcOrSolutionFound);

	// Ensure slnThread is ceased before exiting solve
	slnThread.join();

	OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

	if (sol)
	{
		// Report computation time
		total_runtime = double(clock() - startTime) / CLOCKS_PER_SEC;
		cout << "Solved in " << total_runtime << "s." << endl;
		final_solved = true;

		base::PlannerSolution psol(sol);
		psol.setPlannerName(getName());
		// if the solution was optimized, we mark it as such
		psol.setOptimized(opt_, bestCost_, addedNewSolution());
		pdef_->addSolutionPath(psol);

		ompl::geometric::PathGeometric Path( dynamic_cast< const ompl::geometric::PathGeometric& >( *pdef_->getSolutionPath()));
		//const std::vector< ompl::base::State* > &states
		nodes_in_path = Path.getStates().size();

		nodes_in_trees = boost::num_vertices(g_);
		LogPerf2file();

		save2file(pdef_);
	}
	else {
		// Report computation time
		total_runtime = double(clock() - startTime) / CLOCKS_PER_SEC;
		final_solved = false;

		nodes_in_trees = boost::num_vertices(g_);
		LogPerf2file();
	}

	return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::PRM::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
	if (!isSetup())
		setup();
	if (!sampler_)
		sampler_ = si_->allocValidStateSampler();
	if (!simpleSampler_)
		simpleSampler_ = si_->allocStateSampler();

	std::vector<base::State*> xstates(magic::MAX_RANDOM_BOUNCE_STEPS);
	si_->allocStates(xstates);
	bool grow = true;

	bestCost_ = opt_->infiniteCost();
	while (ptc() == false)
	{
		// maintain a 2:1 ratio for growing/expansion of roadmap
		// call growRoadmap() twice as long for every call of expandRoadmap()
		if (grow)
			growRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(2.0 * magic::ROADMAP_BUILD_TIME)), xstates[0]);
		else
			//expandRoadmap(base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(magic::ROADMAP_BUILD_TIME)), xstates);
		grow = !grow;
	}

	si_->freeStates(xstates);
}

ompl::geometric::PRM::Vertex ompl::geometric::PRM::addMilestone(base::State *state)
{
	std::lock_guard<std::mutex> _(graphMutex_);

	Vertex m = boost::add_vertex(g_);
	stateProperty_[m] = state;
	totalConnectionAttemptsProperty_[m] = 1;
	successfulConnectionAttemptsProperty_[m] = 0;

	// Initialize to its own (dis)connected component.
	disjointSets_.make_set(m);

	// Which milestones will we attempt to connect to?
	const std::vector<Vertex>& neighbors = connectionStrategy_(m);

	foreach (Vertex n, neighbors)
	if (connectionFilter_(n, m))
	{
		totalConnectionAttemptsProperty_[m]++;
		totalConnectionAttemptsProperty_[n]++;

		State ik1(2), ik2(2);
		retrieveStateVector(stateProperty_[n], ik1);
		retrieveStateVector(stateProperty_[m], ik2);

		// Local connection using the Recursive Bi-Section (RBS)
		clock_t sT = clock();
		local_connection_count++;
		bool validMotion = false;
		if (!(ik1[0] == -2 && ik2[0] == -2)) {
			if (ik1[0] == -2)
				validMotion = checkMotionRBS(stateProperty_[n], stateProperty_[m], 1, 0, ik2[0]);
			else if (ik2[0] == -2)
				validMotion = checkMotionRBS(stateProperty_[n], stateProperty_[m], 2, 0, ik1[0]);
			else if (ik1[0] == ik2[0])
				validMotion = checkMotionRBS(stateProperty_[n], stateProperty_[m], 0, ik1[0]);
		}
		/*if (validMotion && (ik1[0] == 0 || ik2[0] == 0)) {
			cout << "--------------------------------\n";
			printStateVector(stateProperty_[n]);
			printStateVector(stateProperty_[m]);
			cout << "valid\n";
		}*/
		local_connection_time += double(clock() - sT) / CLOCKS_PER_SEC;

		if (validMotion)
		{
			local_connection_success_count++;
			successfulConnectionAttemptsProperty_[m]++;
			successfulConnectionAttemptsProperty_[n]++;
			const base::Cost weight = opt_->motionCost(stateProperty_[n], stateProperty_[m]);
			const Graph::edge_property_type properties(weight);
			boost::add_edge(n, m, properties, g_);
			uniteComponents(n, m);
		}
	}

	nn_->add(m);

	return m;
}

void ompl::geometric::PRM::uniteComponents(Vertex m1, Vertex m2)
{
	disjointSets_.union_set(m1, m2);
}

bool ompl::geometric::PRM::sameComponent(Vertex m1, Vertex m2)
{
	return boost::same_component(m1, m2, disjointSets_);
}

ompl::base::PathPtr ompl::geometric::PRM::constructSolution(const Vertex &start, const Vertex &goal)
{
	std::lock_guard<std::mutex> _(graphMutex_);
	boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

	try
	{
		// Consider using a persistent distance_map if it's slow
		boost::astar_search(g_, start,
				std::bind(&PRM::costHeuristic, this, std::placeholders::_1, goal),
				boost::predecessor_map(prev).
				distance_compare(std::bind(&base::OptimizationObjective::
						isCostBetterThan, opt_.get(), std::placeholders::_1, std::placeholders::_2)).
						distance_combine(std::bind(&base::OptimizationObjective::
								combineCosts, opt_.get(), std::placeholders::_1, std::placeholders::_2)).
								distance_inf(opt_->infiniteCost()).
								distance_zero(opt_->identityCost()).
								visitor(AStarGoalVisitor<Vertex>(goal)));
	}
	catch (AStarFoundGoal&)
	{
	}

	if (prev[goal] == goal)
		throw Exception(name_, "Could not find solution path");

	PathGeometric *p = new PathGeometric(si_);
	for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
		p->append(stateProperty_[pos]);
	p->append(stateProperty_[start]);
	p->reverse();

	return base::PathPtr(p);
}

void ompl::geometric::PRM::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	// Explicitly add start and goal states:
	for (size_t i = 0; i < startM_.size(); ++i)
		data.addStartVertex(base::PlannerDataVertex(stateProperty_[startM_[i]], const_cast<PRM*>(this)->disjointSets_.find_set(startM_[i])));

	for (size_t i = 0; i < goalM_.size(); ++i)
		data.addGoalVertex(base::PlannerDataVertex(stateProperty_[goalM_[i]], const_cast<PRM*>(this)->disjointSets_.find_set(goalM_[i])));

	// Adding edges and all other vertices simultaneously
	foreach(const Edge e, boost::edges(g_))
	{
		const Vertex v1 = boost::source(e, g_);
		const Vertex v2 = boost::target(e, g_);
		data.addEdge(base::PlannerDataVertex(stateProperty_[v1]),
				base::PlannerDataVertex(stateProperty_[v2]));

		// Add the reverse edge, since we're constructing an undirected roadmap
		data.addEdge(base::PlannerDataVertex(stateProperty_[v2]),
				base::PlannerDataVertex(stateProperty_[v1]));

		// Add tags for the newly added vertices
		data.tagState(stateProperty_[v1], const_cast<PRM*>(this)->disjointSets_.find_set(v1));
		data.tagState(stateProperty_[v2], const_cast<PRM*>(this)->disjointSets_.find_set(v2));
	}
}

ompl::base::Cost ompl::geometric::PRM::costHeuristic(Vertex u, Vertex v) const
{
	return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}


void ompl::geometric::PRM::save2file(ompl::base::ProblemDefinitionPtr pdef) {
	ompl::geometric::PathGeometric Path( dynamic_cast< const ompl::geometric::PathGeometric& >( *pdef->getSolutionPath()));

	const std::vector< ompl::base::State* > &states = Path.getStates();

	Matrix Q1, Q2, IK;
	ompl::base::State *state;

	State q1(6), q2(6), ik(2), ik1(2), ik2(2);
	for( size_t i = 0 ; i < states.size( ) ; ++i ) {
		state = states[i]->as< ob::State >();
		retrieveStateVector(state, q1, q2);
		Q1.push_back(q1);
		Q2.push_back(q2);
	}

	nodes_in_path = Q1.size();

	cout << "Logging path to files..." << endl;

	{ // Save only milestones
		// Open a_path file
		std::ofstream myfile;
		myfile.open("./paths/path_milestones.txt");

		myfile << Q1.size() << endl;

		for (int i = 0; i < Q1.size(); i++) {
			for (int j = 0; j < 6; j++) {
				myfile << Q1[i][j] << " ";
			}
			for (int j = 0; j < 6; j++) {
				myfile << Q2[i][j] << " ";
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

		for (int j = 0; j < 6; j++) {
			myfile << Q1[0][j] << " ";
		}
		for (int j = 0; j < 6; j++) {
			myfile << Q2[0][j] << " ";
		}
		myfile << endl;

		int count = 1;
		for (int i = 1; i < Q1.size(); i++) {

			ik1 = identify_state_ik(Q1[i-1], Q2[i-1]);
			ik2 = identify_state_ik(Q1[i], Q2[i]);

			Matrix M;
			M.push_back(join_Vectors(Q1[i-1], Q2[i-1]));
			M.push_back(join_Vectors(Q1[i], Q2[i]));

			bool valid = false;
			if (ik1[0] == ik2[0])
				valid = reconstructRBS(Q1[i-1], Q2[i-1], Q1[i], Q2[i], 0, ik1[0], M, 0, 1, 1);
			if (!valid && ik1[1] == ik2[1]) {
				M.clear();
				M.push_back(join_Vectors(Q1[i-1], Q2[i-1]));
				M.push_back(join_Vectors(Q1[i], Q2[i]));
				valid = reconstructRBS(Q1[i-1], Q2[i-1], Q1[i], Q2[i], 1, ik1[1], M, 0, 1, 1);
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
