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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_CONNECT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_CONNECT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <iostream>
#include <fstream>

#include "../validity_checkers/StateValidityCheckerPCS.h"

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gRRTC
           @par Short description
           The basic idea is to grow two RRTs, one from the start and
           one from the goal, and attempt to connect them.
           @par External documentation
           J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI: [10.1109/ROBOT.2000.844730](http://dx.doi.org/10.1109/ROBOT.2000.844730)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
        */

        /** \brief RRT-Connect (RRTConnect) */
        class RRTConnect : public base::Planner, public StateValidityChecker  // Avishai
        {
        public:

            /** \brief Constructor */
            RRTConnect(const base::SpaceInformationPtr &si, double = 2);

            virtual ~RRTConnect();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

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
                tStart_.reset(new NN<Motion*>());
                tGoal_.reset(new NN<Motion*>());
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
            int grow_iterations; // Total number of iterations in the growTree
            int grow_calls; // Number of calls to the growTree function
            double minPathtime;
            double pathLength;

            /** Reset log paprameters */
            void initiate_log_parameters() {
            	IK_counter = 0;
            	IK_time = 0;
            	collisionCheck_counter = 0;
            	collisionCheck_time = 0;
            	isValid_counter = 0;
            	nodes_in_path = 0;
            	nodes_in_trees = 0;
            	local_connection_time = 0;
            	local_connection_count = 0;
            	grow_iterations = 0;
            	grow_calls = 0;
            	minPathtime = 0;
            	iden = 0;
            	pathLength = 0;
            }

            // Maximum local-connection distance
            double Range;

            // Flag of successful reach to point in the growTree function
            bool growTree_reached;

            int active_chain; // 0 - (q1,a) is the active chain, 1 - (q2,a) is the active chain
            int a_chain_connection;


        protected:

            /** \brief Representation of a motion */
            class Motion
            {
            public:

                Motion() : root(nullptr), state(nullptr), parent(nullptr)
                {
                    parent = nullptr;
                    state  = nullptr;
                }

                Motion(const base::SpaceInformationPtr &si) : root(nullptr), state(si->allocState()), parent(nullptr)
                {
                }

                ~Motion()
                {
                }

                const base::State *root;
                base::State       *state;
                Motion            *parent;
                int 			  ik_q1_active;
                int 			  ik_q2_active;
                int 			  a_chain;
            };

            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            typedef std::shared_ptr< NearestNeighbors<Motion*> > TreeData;

            /** \brief Information attached to growing a tree of motions (used internally) */
            struct TreeGrowingInfo
            {
                base::State         *xstate;
                Motion              *xmotion;
                bool                 start;
            };

            /** \brief The state of the tree after an attempt to extend it */
            enum GrowState
                {
                    /// no progress has been made
                    TRAPPED,
                    /// progress has been made towards the randomly sampled state
                    ADVANCED,
                    /// the randomly sampled state was reached
                    REACHED
                };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler */
            base::StateSamplerPtr         sampler_;

            /** \brief The start tree */
            TreeData                      tStart_;

            /** \brief The goal tree */
            TreeData                      tGoal_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                        maxDistance_;

            /** \brief The random number generator */
            RNG                           rng_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State*, base::State*>      connectionPoint_;

            // ***************** My additional functions ************************
            /** \brief Measured the distance between to states only over the active chain */
            double activeDistance(const Motion *a, const Motion *b);

            /** \brief Grow a tree towards a random state */
            Motion* growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *nmotion, Motion *tmotion, int, int);

            /** \brief Computes the minimum distance between files !!! Do not use in actual planning - very inefficient !!! */
            double distanceBetweenTrees(TreeData &tree1, TreeData &tree2);

            /** \brief Save solution path to two files */
            void save2file(vector<Motion*>, vector<Motion*>);

            /** \brief Log performance data of the planning to perf_log.txt */
            void LogPerf2file();

            void timeMinPath(vector<Motion*>);

            // Smooth the solution path
            void smoothPath(vector<Motion*> &path);

        };

    }
}

#endif
