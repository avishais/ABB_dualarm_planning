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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include "../validity_checkers/StateValidityCheckerHB.h"

namespace ompl
{

namespace geometric
{

/**
           @anchor gRRT
           @par Short description
           RRT is a tree-based motion planner that uses the following
           idea: RRT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           @par External documentation
           J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI: [10.1109/ROBOT.2000.844730](http://dx.doi.org/10.1109/ROBOT.2000.844730)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
 */

/** \brief Rapidly-exploring Random Trees */
class RRT : public base::Planner, StateValidityChecker
{
public:

	/** \brief Constructor */
	RRT(const base::SpaceInformationPtr &si, double = 2);

	virtual ~RRT();

	virtual void getPlannerData(base::PlannerData &data) const;

	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

	virtual void clear();

	/** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
	void setGoalBias(double goalBias)
	{
		goalBias_ = goalBias;
	}

	/** \brief Get the goal bias the planner is using */
	double getGoalBias() const
	{
		return goalBias_;
	}

	/** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
	void setRange(double distance)
	{
		maxDistance_ = distance;
	}

	/** \brief Get the range the planner is using */
	double getRange() const
	{
		return maxDistance_;
	}

	/** \brief Set a different nearest neighbors datastructure */
	template<template<typename T> class NN>
	void setNearestNeighbors()
	{
		nn_.reset(new NN<Motion*>());
	}

	virtual void setup();

	// ******************** My additions *********************
	// Performance parameters and handle
	double total_runtime; // Total planning time
	clock_t startTime; // Start clock
	clock_t endTime; // End clock
	int nodes_in_path; // Log nodes in path
	int nodes_in_trees; // Log nodes in both trees
	double PlanDistance; // Norm distance from start to goal configurations
	bool final_solved; // Planning query solved?
	double local_connection_time; // Log LC total time
	int local_connection_count; // Log number of LC attempts

	/** Reset log paprameters */
	void initiate_log_parameters() {
		two_robots::IK_counter = 0;
		two_robots::IK_time = 0;
		collisionCheck_counter = 0;
		collisionCheck_time = 0;
		isValid_counter = 0;
		nodes_in_path = 0;
		nodes_in_trees = 0;
		local_connection_time = 0;
		local_connection_count = 0;
	}

	// Maximum local connection distance
	double Range;

protected:


	/** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
	class Motion
	{
	public:

		Motion() : state(nullptr), parent(nullptr)
	{
	}

		/** \brief Constructor that allocates memory for the state */
		Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(nullptr)
		{
		}

		~Motion()
		{
		}

		/** \brief The state contained by the motion */
		base::State       *state;

		/** \brief The parent motion in the exploration tree */
		Motion            *parent;

        int 			  ik_q1_active;
        int 			  ik_q2_active;
        int 			  a_chain;

	};

	/** \brief Free the memory allocated by this planner */
	void freeMemory();

	/** \brief Compute distance between motions (actually distance between contained states) */
	double distanceFunction(const Motion *a, const Motion *b) const
	{
		return si_->distance(a->state, b->state);
	}

	/** \brief State sampler */
	base::StateSamplerPtr                          sampler_;

	/** \brief A nearest-neighbors datastructure containing the tree of motions */
	std::shared_ptr< NearestNeighbors<Motion*> > nn_;

	/** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
	double                                         goalBias_;

	/** \brief The maximum length of a motion to be added to a tree */
	double                                         maxDistance_;

	/** \brief The random number generator */
	RNG                                            rng_;

	/** \brief The most recent goal motion.  Used for PlannerData computation */
	Motion                                         *lastGoalMotion_;

    // ***************** My additional functions ************************
    /** \brief Save solution path to two files */
	void save2file(vector<Motion*>);

	/** \brief Log performance data of the planning to perf_log.txt */
	void LogPerf2file();
};

}
}

#endif
