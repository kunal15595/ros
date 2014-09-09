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

#ifndef OMPL_GEOMETRIC_PATH_GEOMETRIC_
#define OMPL_GEOMETRIC_PATH_GEOMETRIC_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/Path.h"
#include <vector>
#include <utility>

namespace ompl
{

    /** \brief This namespace contains code that is specific to planning under geometric constraints */
    namespace geometric
    {
        /** \brief Definition of a geometric path.

            This is the type of path computed by geometric planners. */
        class PathGeometric : public base::Path
        {
        public:

            /** \brief Construct a path instance for a given space information */
            PathGeometric(const base::SpaceInformationPtr &si) : base::Path(si)
            {
            }

            /** \brief Copy constructor */
            PathGeometric(const PathGeometric &path);

            /** \brief Construct a path instance from a single state */
            PathGeometric(const base::SpaceInformationPtr &si, const base::State *state);

            /** \brief Construct a path instance from two states (thus making a segment) */
            PathGeometric(const base::SpaceInformationPtr &si, const base::State *state1, const base::State *state2);

            virtual ~PathGeometric()
            {
                freeMemory();
            }

            /** \brief Assignment operator */
            PathGeometric& operator=(const PathGeometric &other);

            /** \brief Compute the length of a geometric path (sum of lengths of segments that make up the path) */
            virtual double length() const;

            /** \brief Check if the path is valid */
            virtual bool check() const;

            /** \brief Compute a notion of smoothness for this
                path. The closer the value is to 0, the smoother the
                path. Detailed formula follows.

                The idea is to look at the triangles formed by consecutive path segments and compute the angle between those segments using
                Pythagora's theorem. Then, the outside angle for the computed angle is normalized by the path segments and contributes to the path smoothness.
                For a straight line path, the smoothness will be 0.
                \f[
                    \mbox{smoothness} = \sum\limits_{i=2}^{n-1}\left(\frac{2\left(\pi - \arccos\left(\frac{a_i^2+b_i^2-c_i^2}{2 a_i b_i}\right)\right)}{a_i + b_i}\right)^2
                \f]
                where \f$a_i = \mbox{dist}(s_{i-2}, s_{i-1}), b_i = \mbox{dist}(s_{i-1}, s_{i}), c_i = \mbox{dist}(s_{i-2}, s_i)\f$, \f$s_i\f$ is the i<sup>th</sup>
                state along the path (see getState()) and \f$\mbox{dist}(s_i, s_j)\f$ gives the distance between two states (see ompl::base::StateSpace::distance()).
            */
            double smoothness() const;

            /** \brief Compute the clearance of the way-points along
                the path (no interpolation is performed). Detailed formula follows.

                The formula used for computing clearance is:
                \f[
                    \mbox{clearance} = \frac{1}{n}\sum\limits_{i=0}^{n-1}cl(s_i)
                \f]
                \f$n\f$ is the number of states along the path (see getStateCount())
                \f$s_i\f$ is the i<sup>th</sup> state along the path (see getState())
                \f$cl()\f$ gives the distance to the nearest invalid state for a particular state (see ompl::base::StateValidityChecker::clearance())
            */
            double clearance() const;

            /** \brief Print the path to a stream */
            virtual void print(std::ostream &out) const;
            /** \brief Print the path as a real-valued matrix where the
                i-th row represents the i-th state along the path. Each
                row contains the state components as returned by
                ompl::base::StateSpace::copyToReals. */
            virtual void printAsMatrix(std::ostream &out) const;

            /** @name Path operations
                @{ */

            /** \brief Insert a number of states in a path so that the
                path is made up of exactly \e count states. States are
                inserted uniformly (more states on longer
                segments). Changes are performed only if a path has
                less than \e count states. */
            void interpolate(unsigned int count);

            /** \brief Insert a number of states in a path so that the
                path is made up of (approximately) the states checked
                for validity when a discrete motion validator is
                used. */
            void interpolate();

            /** \brief Add a state at the middle of each segment */
            void subdivide();

            /** \brief Reverse the path */
            void reverse();

            /** \brief Check if the path is valid. If it is not,
                attempts are made to fix the path by sampling around
                invalid states. Not more than \e attempts samples are
                drawn. 
                \return A pair of boolean values is returned. The first
                value represents the validity of the path before any
                change was made. The second value represents the
                validity of the path after changes were attempted. If
                no changes are attempted, the both values are true.

                \note If repairing a path fails, the path may still be altered */
            std::pair<bool, bool> checkAndRepair(unsigned int attempts);

            /** \brief Overlay the path \e over on top of the current
                path. States are added to the current path if needed
                (by copying the last state).

                If \e over consists of states form a different
                state space than the existing path, the data from those
                states is copied over, for the corresponding
                components. If \e over is from the same state space as this path,
                and \e startIndex is 0, this function's result will be the same
                as with operator=() */
            void overlay(const PathGeometric &over, unsigned int startIndex = 0);

            /** \brief Append \e state to the end of this path. The memory for \e state is copied. */
            void append(const base::State *state);

            /** \brief Append \e path at the end of this path. States from \e path are copied.

                Let the existing path consist of states [ \e s1, \e
                s2, ..., \e sk ]. Let \e path consist of states [\e y1, ..., \e yp].

                If the existing path and \e path consist of states
                from the same state space, [\e y1, ..., \e yp] are added after \e sk.
                If they are not from the same state space, states [\e z1, ..., \e zp]
                are added, where each \e zi is a copy of \e sk that
                has components overwritten with ones in \e yi (if there are any common subspaces).
            */
            void append(const PathGeometric &path);

            /** \brief Keep the part of the path that is after \e state (getClosestIndex() is used to find out which way-point is closest to \e state) */
            void keepAfter(const base::State *state);

            /** \brief Keep the part of the path that is before \e state (getClosestIndex() is used to find out which way-point is closest to \e state) */
            void keepBefore(const base::State *state);

            /** \brief Set this path to a random segment */
            void random();

            /** \brief Set this path to a random valid segment. Sample \e attempts times for valid segments. Returns true on success.*/
            bool randomValid(unsigned int attempts);
            /** @} */

            /** @name Functionality for accessing states
                @{ */

            /** \brief Get the index of the way-point along the path that is closest to \e state. Returns -1 for an empty path. */
            int getClosestIndex(const base::State *state) const;

            /** \brief Get the states that make up the path (as a reference, so it can be modified, hence the function is not const) */
            std::vector<base::State*>& getStates()
            {
                return states_;
            }

            /** \brief Get the state located at \e index along the path */
            base::State* getState(unsigned int index)
            {
                return states_[index];
            }

            /** \brief Get the state located at \e index along the path */
            const base::State* getState(unsigned int index) const
            {
                return states_[index];
            }

            /** \brief Get the number of states (way-points) that make up this path */
            std::size_t getStateCount() const
            {
                return states_.size();
            }

            /** @} */

        protected:

            /** \brief Free the memory corresponding to the states on this path */
            void freeMemory();

            /** \brief Copy data to this path from another path instance */
            void copyFrom(const PathGeometric &other);

            /** \brief The list of states that make up the path */
            std::vector<base::State*> states_;
        };

    }
}

#endif
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

#ifndef OMPL_GEOMETRIC_PATH_SIMPLIFIER_
#define OMPL_GEOMETRIC_PATH_SIMPLIFIER_

#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/util/Console.h"
#include <limits>

namespace ompl
{

    namespace geometric
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::PathSimplifier */
        OMPL_CLASS_FORWARD(PathSimplifier);
        /// @endcond

        /** \class ompl::geometric::PathSimplifierPtr
            \brief A boost shared pointer wrapper for ompl::geometric::PathSimplifier */

        /** \brief This class contains routines that attempt to simplify geometric paths.

            Some of these are in fact routines that shorten the path, and do not
            necessarily make it smoother. */
        class PathSimplifier
        {
        public:

            /** \brief Create an instance for a specified space information */
            PathSimplifier(const base::SpaceInformationPtr &si) : si_(si), freeStates_(true)
            {
            }

            virtual ~PathSimplifier()
            {
            }


            /** \brief Given a path, attempt to remove vertices from
                it while keeping the path valid. This is an iterative
                process that attempts to do "short-cutting" on the
                path.  Connection is attempted between non-consecutive
                way-points on the path. If the connection is
                successful, the path is shortened by removing the
                in-between way-points. This function returns true if
                changes were made to the path.

                \param path the path to reduce vertices from

                \param maxSteps the maximum number of attempts to
                "short-cut" the path.  If this value is set to 0 (the
                default), the number of attempts made is equal to the
                number of states in \e path.

                \param maxEmptySteps not all iterations of this function
                produce a simplification. If an iteration does not
                produce a simplification, it is called an empty
                step. \e maxEmptySteps denotes the maximum number of
                consecutive empty steps before the simplification
                process terminates. If this value is set to 0 (the
                default), the number of attempts made is equal to the
                number of states in \e path.

                \param rangeRatio the maximum distance between states
                a connection is attempted, as a fraction relative to
                the total number of states (between 0 and 1).

            */
            bool reduceVertices(PathGeometric &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0, double rangeRatio = 0.33);

            /** \brief Given a path, attempt to shorten it while
                maintaining its validity. This is an iterative process
                that attempts to do "short-cutting" on the path.
                Connection is attempted between random points along
                the path segments. Unlike the reduceVertices()
                function, this function does not sample only vertices
                produced by the planner, but intermediate points on
                the path. If the connection is successful, the path is
                shortened by removing the in-between states (and new
                vertices are created when needed). This function returns
                true if changes were made to the path.

                \param path the path to reduce vertices from

                \param maxSteps the maximum number of attempts to
                "short-cut" the path.  If this value is set to 0 (the
                default), the number of attempts made is equal to the
                number of states in \e path.

                \param maxEmptySteps not all iterations of this function
                produce a simplification. If an iteration does not
                produce a simplification, it is called an empty
                step. \e maxEmptySteps denotes the maximum number of
                consecutive empty steps before the simplification
                process terminates. If this value is set to 0 (the
                default), the number of attempts made is equal to the
                number of states in \e path.

                \param rangeRatio the maximum distance between states
                a connection is attempted, as a fraction relative to
                the total number of states (between 0 and 1).

                \param snapToVertex While sampling random points on
                the path, sometimes the points may be close to
                vertices on the original path (way-points).  This
                function will then "snap to the near vertex", if the
                distance is less than \e snapToVertex fraction of the
                total path length. This should usually be a small
                value (e.g., one percent of the total path length:
                0.01; the default is half a percent)

                \note This function assumes the triangle inequality holds and should not be run on non-metric spaces.
            */
            bool shortcutPath(PathGeometric &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0, double rangeRatio = 0.33, double snapToVertex = 0.005);

            /** \brief Given a path, attempt to remove vertices from
                it while keeping the path valid. This is an iterative
                process that attempts to do "short-cutting" on the
                path.  Connection is attempted between non-consecutive
                states that are close along the path. If the
                connection is successful, the path is shortened by
                removing the in-between states. This function returns
                true if changes were made to the path.

                \param path the path to reduce vertices from

                \param maxSteps the maximum number of attempts to
                "short-cut" the path.  If this value is set to 0 (the
                default), the number of attempts made is equal to the
                number of states in \e path. If this value is set to 0 (the
                default), the number of attempts made is equal to the
                number of states in \e path.

                \param maxEmptySteps not all iterations of this function
                produce a simplification. If an iteration does not
                produce a simplification, it is called an empty
                step. \e maxEmptySteps denotes the maximum number of
                consecutive empty steps before the simplification
                process terminates.
            */
            bool collapseCloseVertices(PathGeometric &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0);

            /** \brief Given a path, attempt to smooth it (the
                validity of the path is maintained).

                This function applies \e maxSteps steps of smoothing
                with B-Splines. Fewer steps are applied if no progress
                is detected: states are either not updated or their
                update is smaller than \e minChange.  At each step the
                path is subdivided and states along it are updated
                such that the smoothness is improved.

                \note This function may significantly increase the number of states along the solution path.
                \note This function assumes the triangle inequality holds and should not be run on non-metric spaces.
                */
            void smoothBSpline(PathGeometric &path, unsigned int maxSteps = 5, double minChange = std::numeric_limits<double>::epsilon());

            /** \brief Given a path, attempt to remove vertices from it while keeping the path valid.  Then, try to smooth
                the path. This function applies the same set of default operations to the path, except in non-metric spaces,
                with the intention of simplifying it. In non-metric spaces, some operations are skipped because they do
                not work correctly when the triangle inequality may not hold. */
            void simplifyMax(PathGeometric &path);

            /** \brief Run simplification algorithms on the path for at most \e maxTime seconds */
            void simplify(PathGeometric &path, double maxTime);

            /** \brief Run simplification algorithms on the path as long as the termination condition does not become true */
            void simplify(PathGeometric &path, const base::PlannerTerminationCondition &ptc);

            /** \brief Set this flag to false to avoid freeing the memory allocated for states that are removed from a path during simplification.
                Setting this to true makes this free memory. Memory is freed by default (flag is true by default) */
            void freeStates(bool flag);

            /** \brief Return true if the memory of states is freed when they are removed from a path during simplification */
            bool freeStates() const;

        protected:

            /** \brief The space information this path simplifier uses */
            base::SpaceInformationPtr si_;

            /** \brief Flag indicating whether the states removed from a motion should be freed */
            bool                      freeStates_;

            /** \brief Instance of random number generator */
            RNG                       rng_;

        };
    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef OMPL_GEOMETRIC_PATH_HYBRIDIZATION_
#define OMPL_GEOMETRIC_PATH_HYBRIDIZATION_

#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <iostream>
#include <set>

namespace ompl
{
    namespace geometric
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::PathHybridization */
        OMPL_CLASS_FORWARD(PathHybridization);
        /// @endcond

        /** \class ompl::geometric::PathHybridizationPtr
            \brief A boost shared pointer wrapper for ompl::geometric::PathHybridization */

        /** \brief Given multiple geometric paths, attempt to combine them in order to obtain a shorter solution.

            @par External documentation

            B. Raveh, A. Enosh, and D. Halperin,
            A little more, a lot better: Improving path quality by a path-merging algorithm,
            <em>IEEE Trans. on Robotics</em>, vol. 27, pp. 365–371, Apr. 2011.
            DOI: <a href="http://dx.doi.org/10.1109/TRO.2010.2098622">10.1109/TRO.2010.2098622</a><br>
            <a href="http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5686946">[PDF]</a>
        */
        class PathHybridization
        {
        public:

            /** \brief The constructor needs to know about the space information of the paths it will operate on */
            PathHybridization(const base::SpaceInformationPtr &si);
            ~PathHybridization();

            /** \brief Get the currently computed hybrid path. computeHybridPath() needs to have been called before. */
            const base::PathPtr& getHybridPath() const;

            /** \brief Run Dijkstra's algorithm to find out the shortest path among the mixed ones */
            void computeHybridPath();

            /** \brief Add a path to the hybridization. If \e matchAcrossGaps is true, more possible edge connections are evaluated.
                Return the number of attempted connections between paths. */
            unsigned int recordPath(const base::PathPtr &pp, bool matchAcrossGaps);

            /** \brief Get the number of paths that are currently considered as part of the hybridization */
            std::size_t pathCount() const;

            /** \brief Given two geometric paths \e p and \e q, compute the alignment of the paths using dynamic
                programming in an edit-distance like fashion, as described in the referenced paper. The cost of a gap
                is considered to be \e gapCost. The output of the computation is two arrays \e indexP and \e indexQ of
                equal length, such that these arrays contain matching index positions from the states in \e p and \e q,
                respectively. Gaps are marked by -1. */
            void matchPaths(const geometric::PathGeometric &p, const geometric::PathGeometric &q, double gapCost,
                            std::vector<int> &indexP, std::vector<int> &indexQ) const;

            /** \brief Clear all the stored paths */
            void clear();

            /** \brief Print information about the computed path */
            void print(std::ostream &out = std::cout) const;

            /** \brief Get the name of the algorithm */
            const std::string& getName() const;

        private:

            /// @cond IGNORE
            struct vertex_state_t {
                typedef boost::vertex_property_tag kind;
            };

            typedef boost::adjacency_list <
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property < vertex_state_t, base::State*,
                                  boost::property < boost::vertex_predecessor_t, unsigned long int,
                                                    boost::property < boost::vertex_rank_t, unsigned long int > > >,
                boost::property < boost::edge_weight_t, double >
                > HGraph;

            typedef boost::graph_traits<HGraph>::vertex_descriptor Vertex;
            typedef boost::graph_traits<HGraph>::edge_descriptor   Edge;

            struct PathInfo
            {
                PathInfo(const base::PathPtr &path) : path_(path), states_(static_cast<PathGeometric*>(path.get())->getStates()), length_(0.0)
                {
                    vertices_.reserve(states_.size());
                }

                bool operator==(const PathInfo &other) const
                {
                    return path_ == other.path_;
                }

                bool operator<(const PathInfo &other) const
                {
                    return path_ < other.path_;
                }

                base::PathPtr                    path_;
                const std::vector<base::State*> &states_;
                double                           length_;
                std::vector<Vertex>              vertices_;
            };
            /// @endcond

            void attemptNewEdge(const PathInfo &p, const PathInfo &q, int indexP, int indexQ);

            base::SpaceInformationPtr                         si_;
            HGraph                                            g_;
            boost::property_map<HGraph, vertex_state_t>::type stateProperty_;
            Vertex                                            root_;
            Vertex                                            goal_;
            std::set<PathInfo>                                paths_;
            base::PathPtr                                     hpath_;

            /** \brief The name of the path hybridization algorithm, used for tracking planner solution sources */
            std::string                                       name_;
        };
    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

#ifndef OMPL_GEOMETRIC_SIMPLE_SETUP_
#define OMPL_GEOMETRIC_SIMPLE_SETUP_

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

namespace ompl
{

    namespace geometric
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SimpleSetup);
        /// @endcond

        /** \class ompl::geometric::SimpleSetupPtr
            \brief A boost shared pointer wrapper for ompl::geometric::SimpleSetup */

        /** \brief Create the set of classes typically needed to solve a
            geometric problem */
        class SimpleSetup
        {
        public:

            /** \brief Constructor needs the state space used for planning. */
            explicit
            SimpleSetup(const base::SpaceInformationPtr &si);

            /** \brief Constructor needs the state space used for planning. */
            explicit
            SimpleSetup(const base::StateSpacePtr &space);

            virtual ~SimpleSetup()
            {
            }

            /** \brief Get the current instance of the space information */
            const base::SpaceInformationPtr& getSpaceInformation() const
            {
                return si_;
            }

            /** \brief Get the current instance of the problem definition */
            const base::ProblemDefinitionPtr& getProblemDefinition() const
            {
                return pdef_;
            }

            /** \brief Get the current instance of the state space */
            const base::StateSpacePtr& getStateSpace() const
            {
                return si_->getStateSpace();
            }

            /** \brief Get the current instance of the state validity checker */
            const base::StateValidityCheckerPtr& getStateValidityChecker() const
            {
                return si_->getStateValidityChecker();
            }

            /** \brief Get the current goal definition */
            const base::GoalPtr& getGoal() const
            {
                return pdef_->getGoal();
            }

            /** \brief Get the current planner */
            const base::PlannerPtr& getPlanner() const
            {
                return planner_;
            }

            /** \brief Get the planner allocator */
            const base::PlannerAllocator& getPlannerAllocator() const
            {
                return pa_;
            }

            /** \brief Get the path simplifier */
            const PathSimplifierPtr& getPathSimplifier() const
            {
                return psk_;
            }

            /** \brief Get the path simplifier */
            PathSimplifierPtr& getPathSimplifier()
            {
                return psk_;
            }

            /** \brief Get the optimization objective to use */
            const base::OptimizationObjectivePtr& getOptimizationObjective()
            {
                return pdef_->getOptimizationObjective();
            }

            /** \brief Return true if a solution path is available (previous call to solve() was successful) and the solution is exact (not approximate) */
            bool haveExactSolutionPath() const;

            /** \brief Return true if a solution path is available (previous call to solve() was successful). The solution may be approximate. */
            bool haveSolutionPath() const
            {
                return pdef_->getSolutionPath().get();
            }

            /** \brief Get the best solution's planer name. Throw an exception if no solution is available */
            const std::string getSolutionPlannerName(void) const;

            /** \brief Get the solution path. Throw an exception if no solution is available */
            PathGeometric& getSolutionPath() const;

            /** \brief Get information about the exploration data structure the motion planner used. */
            void getPlannerData(base::PlannerData &pd) const;

            /** \brief Set the state validity checker to use */
            void setStateValidityChecker(const base::StateValidityCheckerPtr &svc)
            {
                si_->setStateValidityChecker(svc);
            }

            /** \brief Set the state validity checker to use */
            void setStateValidityChecker(const base::StateValidityCheckerFn &svc)
            {
                si_->setStateValidityChecker(svc);
            }

            /** \brief Set the optimization objective to use */
            void setOptimizationObjective(const base::OptimizationObjectivePtr &optimizationObjective)
            {
                pdef_->setOptimizationObjective(optimizationObjective);
            }

            /** \brief Set the start and goal states to use. */
            void setStartAndGoalStates(const base::ScopedState<> &start, const base::ScopedState<> &goal,
                                       const double threshold = std::numeric_limits<double>::epsilon())
            {
                pdef_->setStartAndGoalStates(start, goal, threshold);

                // Clear any past solutions since they no longer correspond to our start and goal states
                pdef_->clearSolutionPaths();
            }

            /** \brief Add a starting state for planning. This call is not
                needed if setStartAndGoalStates() has been called. */
            void addStartState(const base::ScopedState<> &state)
            {
                pdef_->addStartState(state);
            }

            /** \brief Clear the currently set starting states */
            void clearStartStates()
            {
                pdef_->clearStartStates();
            }

            /** \brief Clear the currently set starting states and add \e state as the starting state */
            void setStartState(const base::ScopedState<> &state)
            {
                clearStartStates();
                addStartState(state);
            }

            /** \brief A simple form of setGoal(). The goal will be an instance of ompl::base::GoalState */
            void setGoalState(const base::ScopedState<> &goal, const double threshold = std::numeric_limits<double>::epsilon())
            {
                pdef_->setGoalState(goal, threshold);
            }

            /** \brief Set the goal for planning. This call is not
                needed if setStartAndGoalStates() has been called. */
            void setGoal(const base::GoalPtr &goal)
            {
                pdef_->setGoal(goal);
            }

            /** \brief Set the planner to use. If the planner is not
                set, an attempt is made to use the planner
                allocator. If no planner allocator is available
                either, a default planner is set. */
            void setPlanner(const base::PlannerPtr &planner)
            {
                if (planner && planner->getSpaceInformation().get() != si_.get())
                    throw Exception("Planner instance does not match space information");
                planner_ = planner;
                configured_ = false;
            }

            /** \brief Set the planner allocator to use. This is only
                used if no planner has been set. This is optional -- a default
                planner will be used if no planner is otherwise specified. */
            void setPlannerAllocator(const base::PlannerAllocator &pa)
            {
                pa_ = pa;
                planner_.reset();
                configured_ = false;
            }

            /** \brief Run the planner for up to a specified amount of time (default is 1 second) */
            virtual base::PlannerStatus solve(double time = 1.0);

            /** \brief Run the planner until \e ptc becomes true (at most) */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Return the status of the last planning attempt */
            base::PlannerStatus getLastPlannerStatus() const
            {
                return lastStatus_;
            }

            /** \brief Get the amount of time (in seconds) spent during the last planning step */
            double getLastPlanComputationTime() const
            {
                return planTime_;
            }

            /** \brief Get the amount of time (in seconds) spend during the last path simplification step */
            double getLastSimplificationTime() const
            {
                return simplifyTime_;
            }

            /** \brief Attempt to simplify the current solution path. Spent at most \e duration seconds in the simplification process.
                If \e duration is 0 (the default), a default simplification procedure is executed. */
            void simplifySolution(double duration = 0.0);

            /** \brief Attempt to simplify the current solution path. Stop computation when \e ptc becomes true at the latest. */
            void simplifySolution(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear all planning data. This only includes
                data generated by motion plan computation. Planner
                settings, start & goal states are not affected. */
            virtual void clear();

            /** \brief Print information about the current setup */
            virtual void print(std::ostream &out = std::cout) const;

            /** \brief This method will create the necessary classes
                for planning. The solve() method will call this
                function automatically. */
            virtual void setup();

            /** \brief Get the  parameters for this planning context */
            base::ParamSet& params()
            {
                return params_;
            }

            /** \brief Get the  parameters for this planning context */
            const base::ParamSet& params() const
            {
                return params_;
            }

        protected:

            /// The created space information
            base::SpaceInformationPtr     si_;

            /// The created problem definition
            base::ProblemDefinitionPtr    pdef_;

            /// The maintained planner instance
            base::PlannerPtr              planner_;

            /// The optional planner allocator
            base::PlannerAllocator        pa_;

            /// The instance of the path simplifier
            PathSimplifierPtr             psk_;

            /// Flag indicating whether the classes needed for planning are set up
            bool                          configured_;

            /// The amount of time the last planning step took
            double                        planTime_;

            /// The amount of time the last path simplification step took
            double                        simplifyTime_;

            /// The status of the last planning request
            base::PlannerStatus           lastStatus_;

            /// The parameters that describe the planning context
            base::ParamSet                params_;
        };

        /** \brief Given a goal specification, decide on a planner for that goal */
        base::PlannerPtr getDefaultPlanner(const base::GoalPtr &goal);
    }

}
#endif
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: James D. Marble */

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_CONNECTION_STRATEGY_
#define OMPL_GEOMETRIC_PLANNERS_PRM_CONNECTION_STRATEGY_

#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/math/constants/constants.hpp>
#include <vector>

namespace ompl
{

    namespace geometric
    {

        /**
         * Attempt to connect to the k nearest neighbors.
         */
        template <class Milestone>
        class KStrategy
        {
        public:

            /** \brief Constructor takes the maximum number of nearest neighbors to return (\e k) and the
                nearest neighbors datastruture to use (\e nn) */
            KStrategy(const unsigned int k,
                      const boost::shared_ptr< NearestNeighbors<Milestone> > &nn) :
                k_(k), nn_(nn)
            {
                neighbors_.reserve(k_);
            }

            virtual ~KStrategy()
            {
            }

            /** \brief Set the nearest neighbors datastructure to use */
            void setNearestNeighbors(const boost::shared_ptr< NearestNeighbors<Milestone> > &nn)
            {
                nn_ = nn;
            }

            /** \brief Given a milestone \e m, find the number of nearest
                neighbors connection attempts that should be made from it,
                according to the connection strategy */
            std::vector<Milestone>& operator()(const Milestone &m)
            {
                nn_->nearestK(m, k_, neighbors_);
                return neighbors_;
            }

        protected:

            /** \brief Maximum number of nearest neighbors to attempt to connect new milestones to */
            unsigned int                                     k_;

            /** \brief Nearest neighbors data structure */
            boost::shared_ptr< NearestNeighbors<Milestone> > nn_;

            /** \brief Scratch space for storing k-nearest neighbors */
            std::vector<Milestone>                           neighbors_;
        };

        /**
         * \brief Make the minimal number of connections required to ensure
         * asymptotic optimality.
         *
         * This connection strategy attempts to connect a milestone to its
         * k-nearest neighbors where k is a function of the number of milestones
         * that have already been added to the roadmap (n).
         *
         * k(n) = kPRMConstant * log(n)
         *
         * where
         *
         * kPRMConstant > kStarPRMConstant = e(1 + 1/d)
         *
         * and d is the number of dimensions in the state space. Note that
         * kPRMConstant = 2e is a valid choice for any problem instance and so,
         * if d is not provided, this value is used.
         *
         * The user must provide a function that returns the value of n.
         *
         * @par External documentation
         * S. Karaman and E. Frazzoli
         * Sampling-based algorithms for optimal motion planning,
         * <em>Int. Journal of Robotics Research</em> Volume 30, Number 7, June 2010
         */
        template <class Milestone>
        class KStarStrategy : public KStrategy<Milestone>
        {
        public:
            typedef boost::function<unsigned int()> NumNeighborsFn;
            /**
             * \brief Constructor
             *
             * \param n a function that returns the number of milestones that have already been added to the roadmap
             * \param nn the nearest neighbors datastruture to use
             * \param d the dimensionality of the state space.
             * The default is 1, which will make kPRMConstant=2e which
             * is valid for all problem instances.
             */
            KStarStrategy(const NumNeighborsFn& n,
                          const boost::shared_ptr< NearestNeighbors<Milestone> > &nn,
                          const unsigned int d = 1) :
                KStrategy<Milestone>(n(), nn), n_(n),
                kPRMConstant_(boost::math::constants::e<double>() + (boost::math::constants::e<double>()/(double)d))
            {
            }

            std::vector<Milestone>& operator()(const Milestone &m)
            {
                KStrategy<Milestone>::k_ = static_cast<unsigned int>(ceil(kPRMConstant_ * log((double)n_())));
                return static_cast<KStrategy<Milestone>&>(*this)(m);
            }

        protected:

            /** \brief Function returning the number of milestones added to the roadmap so far */
            const NumNeighborsFn  n_;
            const double          kPRMConstant_;

        };

    }

}

#endif
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ioan Sucan, James D. Marble, Ryan Luna */

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_PRM_
#define OMPL_GEOMETRIC_PLANNERS_PRM_PRM_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <utility>
#include <vector>
#include <map>

namespace ompl
{

    namespace base
    {
        // Forward declare for use in implementation
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }

    namespace geometric
    {

        /**
           @anchor gPRM
           @par Short description
           PRM is a planner that constructs a roadmap of milestones
           that approximate the connectivity of the state space. The
           milestones are valid states in the state space. Near-by
           milestones are connected by valid motions. Finding a motion
           plan that connects two given states is reduced to a
           discrete search (this implementation uses A*) in the
           roadmap.
           @par External documentation
           L.E. Kavraki, P.Švestka, J.-C. Latombe, and M.H. Overmars,
           Probabilistic roadmaps for path planning in high-dimensional configuration spaces,
           <em>IEEE Trans. on Robotics and Automation</em>, vol. 12, pp. 566–580, Aug. 1996.
           DOI: <a href="http://dx.doi.org/10.1109/70.508439">10.1109/70.508439</a><br>
           <a href="http://ieeexplore.ieee.org/ielx4/70/11078/00508439.pdf?tp=&arnumber=508439&isnumber=11078">[PDF]</a>
           <a href="http://www.kavrakilab.org/robotics/prm.html">[more]</a>
        */

        /** \brief Probabilistic RoadMap planner */
        class PRM : public base::Planner
        {
        public:

            struct vertex_state_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_total_connection_attempts_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_successful_connection_attempts_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_flags_t {
                typedef boost::vertex_property_tag kind;
            };

            struct edge_flags_t {
                typedef boost::edge_property_tag kind;
            };

            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix.

             @par Obviously, a ompl::base::State* vertex property is required.
             The incremental connected components algorithm requires
             vertex_predecessor_t and vertex_rank_t properties.
             If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             @par Edges should be undirected and have a weight property.
             */
            typedef boost::adjacency_list <
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property < vertex_state_t, base::State*,
                boost::property < vertex_total_connection_attempts_t, unsigned int,
                boost::property < vertex_successful_connection_attempts_t, unsigned int,
                boost::property < vertex_flags_t, unsigned int,
                boost::property < boost::vertex_predecessor_t, unsigned long int,
                boost::property < boost::vertex_rank_t, unsigned long int > > > > > >,
                boost::property < boost::edge_weight_t, base::Cost,
                boost::property < boost::edge_index_t, unsigned int,
                boost::property < edge_flags_t, unsigned int > > >
            > Graph;

            typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
            typedef boost::graph_traits<Graph>::edge_descriptor   Edge;

            typedef boost::shared_ptr< NearestNeighbors<Vertex> > RoadmapNeighbors;

            /** @brief A function returning the milestones that should be
             * attempted to connect to
             *
             * @note Can't use the prefered boost::function syntax here because
             * the Python bindings don't like it.
             */
            typedef boost::function<std::vector<Vertex>&(const Vertex)>
                ConnectionStrategy;

            /** @brief A function that can reject connections.

             This is called after previous connections from the neighbor list
             have been added to the roadmap.
             */
            typedef boost::function<bool(const Vertex&, const Vertex&)> ConnectionFilter;

            /** \brief Constructor */
            PRM(const base::SpaceInformationPtr &si, bool starStrategy = false);

            virtual ~PRM();

            virtual void setProblemDefinition(const base::ProblemDefinitionPtr &pdef);

            /** \brief Set the connection strategy function that specifies the
             milestones that connection attempts will be make to for a
             given milestone.

             \par The behavior and performance of PRM can be changed drastically
             by varying the number and properties if the milestones that are
             connected to each other.

             \param pdef A function that takes a milestone as an argument and
             returns a collection of other milestones to which a connection
             attempt must be made. The default connection strategy is to connect
             a milestone's 10 closest neighbors.
             */
            void setConnectionStrategy(const ConnectionStrategy& connectionStrategy)
            {
                connectionStrategy_ = connectionStrategy;
                userSetConnectionStrategy_ = true;
            }
            /** \brief Convenience function that sets the connection strategy to the
             default one with k nearest neighbors.
             */
            void setMaxNearestNeighbors(unsigned int k);

            /** \brief Set the function that can reject a milestone connection.

             \par The given function is called immediately before a connection
             is checked for collision and added to the roadmap. Other neighbors
             may have already been connected before this function is called.
             This allows certain heuristics that use the structure of the
             roadmap (like connected components or useful cycles) to be
             implemented by changing this function.

             \param connectionFilter A function that takes the new milestone,
             a neighboring milestone and returns whether a connection should be
             attempted.
             */
            void setConnectionFilter(const ConnectionFilter& connectionFilter)
            {
                connectionFilter_ = connectionFilter;
            }

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief While the termination condition allows, this function will construct the roadmap (using growRoadmap() and expandRoadmap(),
                maintaining a 2:1 ratio for growing/expansion of roadmap) */
            virtual void constructRoadmap(const base::PlannerTerminationCondition &ptc);

            /** \brief If the user desires, the roadmap can be
                improved for the given time (seconds). The solve()
                method will also improve the roadmap, as needed.*/
            virtual void growRoadmap(double growTime);

            /** \brief If the user desires, the roadmap can be
                improved until a given condition is true. The solve()
                method will also improve the roadmap, as needed.*/
            virtual void growRoadmap(const base::PlannerTerminationCondition &ptc);

            /** \brief Attempt to connect disjoint components in the roadmap
                using random bouncing motions (the PRM expansion step) for the
                given time (seconds). */
            virtual void expandRoadmap(double expandTime);

            /** \brief Attempt to connect disjoint components in the roadmap
                using random bouncing motions (the PRM expansion step) until the
                given condition evaluates true. */
            virtual void expandRoadmap(const base::PlannerTerminationCondition &ptc);

            /** \brief Function that can solve the motion planning
                problem. Grows a roadmap using
                constructRoadmap(). This function can be called
                multiple times on the same problem, without calling
                clear() in between. This allows the planner to
                continue work for more time on an unsolved problem,
                for example. Start and goal states from the currently
                specified ProblemDefinition are cached. This means
                that between calls to solve(), input states are only
                added, not removed. When using PRM as a multi-query
                planner, the input states should be however cleared,
                without clearing the roadmap itself. This can be done
                using the clearQuery() function. */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for PRM. */
            void clearQuery();

            virtual void clear();

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors()
            {
                nn_.reset(new NN<Vertex>());
                if (!userSetConnectionStrategy_)
                    connectionStrategy_.clear();
                if (isSetup())
                    setup();
            }

            virtual void setup();

            const Graph& getRoadmap() const
            {
                return g_;
            }

            /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
            double distanceFunction(const Vertex a, const Vertex b) const
            {
                return si_->distance(stateProperty_[a], stateProperty_[b]);
            }

            /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
            unsigned int milestoneCount() const
            {
                return boost::num_vertices(g_);
            }

            const RoadmapNeighbors& getNearestNeighbors()
            {
                return nn_;
            }

        protected:

            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data structure
                and then connect it to the roadmap in accordance to the connection strategy. */
            virtual Vertex addMilestone(base::State *state);

            /** \brief Make two milestones (\e m1 and \e m2) be part of the same connected component. The component with fewer elements will get the id of the component with more elements. */
            void uniteComponents(Vertex m1, Vertex m2);

            /** \brief Check if two milestones (\e m1 and \e m2) are part of the same connected component. This is not a const function since we use incremental connected components from boost */
            bool sameComponent(Vertex m1, Vertex m2);

            /** \brief Randomly sample the state space, add and connect milestones
                 in the roadmap. Stop this process when the termination condition
                 \e ptc returns true.  Use \e workState as temporary memory. */
            virtual void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState);

            /** \brief Attempt to connect disjoint components in the
                roadmap using random bounding motions (the PRM
                expansion step) */
            virtual void expandRoadmap(const base::PlannerTerminationCondition &ptc, std::vector<base::State*> &workStates);

            /** Thread that checks for solution */
            void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, the path is saved. */
            bool haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution);

            /** \brief Returns the value of the addedSolution_ member. */
            bool addedNewSolution() const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
            virtual base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

            /** \brief Given a solution represented as a vector of predecesors in the roadmap, construct a geometric path */
            virtual base::PathPtr constructGeometricPath(const boost::vector_property_map<Vertex> &prev, const Vertex &start, const Vertex &goal);

            /** \brief Flag indicating whether the default connection strategy is the Star strategy */
            bool                                                   starStrategy_;

            /** \brief Sampler user for generating valid samples in the state space */
            base::ValidStateSamplerPtr                             sampler_;

            /** \brief Sampler user for generating random in the state space */
            base::StateSamplerPtr                                  simpleSampler_;

            /** \brief Nearest neighbors data structure */
            RoadmapNeighbors                                       nn_;

            /** \brief Connectivity graph */
            Graph                                                  g_;

            /** \brief Array of start milestones */
            std::vector<Vertex>                                    startM_;

            /** \brief Array of goal milestones */
            std::vector<Vertex>                                    goalM_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type       stateProperty_;

            /** \brief Access to the number of total connection attempts for a vertex */
            boost::property_map<Graph,
                vertex_total_connection_attempts_t>::type          totalConnectionAttemptsProperty_;

            /** \brief Access to the number of successful connection attempts for a vertex */
            boost::property_map<Graph,
                vertex_successful_connection_attempts_t>::type     successfulConnectionAttemptsProperty_;

            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

            /** \brief Access to the indices of each Edge */
            boost::property_map<Graph, boost::edge_index_t>::type  edgeIDProperty_;

            /** \brief Data structure that maintains the connected components */
            boost::disjoint_sets<
                boost::property_map<Graph, boost::vertex_rank_t>::type,
                boost::property_map<Graph, boost::vertex_predecessor_t>::type >
                                                                   disjointSets_;

            /** \brief Maximum unique id number used so for for edges */
            unsigned int                                           maxEdgeID_;

            /** \brief Function that returns the milestones to attempt connections with */
            ConnectionStrategy                                     connectionStrategy_;

            /** \brief Function that can reject a milestone connection */
            ConnectionFilter                                       connectionFilter_;

            /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are assumed) */
            bool                                                   userSetConnectionStrategy_;

            /** \brief Random number generator */
            RNG                                                    rng_;

            /** \brief A flag indicating that a solution has been added during solve() */
            bool                                                   addedSolution_;

            /** \brief Mutex to guard access to the Graph member (g_) */
            mutable boost::mutex                                   graphMutex_;

            /** \brief Objective cost function for PRM graph edges */
            base::OptimizationObjectivePtr                         opt_;

            /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them. This method wraps OptimizationObjective::motionCostHeuristic */
            base::Cost costHeuristic(Vertex u, Vertex v) const;
        };

    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ioan Sucan, James D. Marble */

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_PRM_STAR_
#define OMPL_GEOMETRIC_PLANNERS_PRM_PRM_STAR_

#include "ompl/geometric/planners/prm/PRM.h"

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gPRMstar
           Run PRM with the "star strategy". Instead of setting the
           value "k" for how many neighbors to connect, automatically
           compute it based on the coverage of the space, guaranteeing
           optimality of solutions.
           @par Short description
           @par External documentation
           L.E. Kavraki, P.Švestka, J.-C. Latombe, and M.H. Overmars,
           Probabilistic roadmaps for path planning in high-dimensional configuration spaces,
           <em>IEEE Trans. on Robotics and Automation</em>, vol. 12, pp. 566–580, Aug. 1996.
           DOI: <a href="http://dx.doi.org/10.1109/70.508439">10.1109/70.508439</a><br>
           S. Karaman and E. Frazzoli, Sampling-based
           Algorithms for Optimal Motion Planning, International Journal of Robotics
           Research, vol. 30, no.7, pp. 846-894, 2011.
           DOI: <a href="http://dx.doi.org/10.1177/0278364911406761">10.1177/0278364911406761</a><br>
           <a href="http://www.kavrakilab.org/robotics/lazyprm.html">[more]</a>
        */

        /** \brief PRM* planner */
        class PRMstar : public PRM
        {
        public:

            /** \brief Constructor */
            PRMstar(const base::SpaceInformationPtr &si);
        };
    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
*  All Rights Reserved.
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Author: Andrew Dobson */

#ifndef OMPL_GEOMETRIC_PLANNERS_SPARSE_ROADMAP_SPANNER_
#define OMPL_GEOMETRIC_PLANNERS_SPARSE_ROADMAP_SPANNER_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Time.h"

#include <boost/range/adaptor/map.hpp>
#include <boost/unordered_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <deque>
#include <map>
#include <set>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gSPARS
           @par Short description
           SPARS is an algorithm which operates similarly to the Visibility-based
           PRM.  It has several desirable properties, including asymptotic
           near-optimality, and a meaningful stopping criterion.
           @par External documentation
           A. Dobson, A. Krontiris, K. Bekris,
           Sparse Roadmap Spanners,
           <em>Workshop on the Algorithmic Foundations of Robotics (WAFR)</em> 2012.
           <a href="http://www.cs.rutgers.edu/~kb572/pubs/sparse_roadmap_spanner.pdf">[PDF]</a>
         */

        /** \brief <b> SPArse Roadmap Spanner technique. </b> */
        class SPARS : public base::Planner
        {
        public:
            /** \brief Enumeration which specifies the reason a guard is added to the spanner. */
            enum GuardType
            {
                START,
                GOAL,
                COVERAGE,
                CONNECTIVITY,
                INTERFACE,
                QUALITY,
            };

            struct vertex_state_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_representative_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_color_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_list_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_interface_list_t {
                typedef boost::vertex_property_tag kind;
            };

            /** \brief The type used internally for representing vertex IDs */
            typedef unsigned long int VertexIndexType;

            /** \brief Hash for storing interface information. */
            typedef boost::unordered_map<VertexIndexType, std::set<VertexIndexType>, boost::hash<VertexIndexType> > InterfaceHash;

            /** \brief Internal representation of a dense path */
            typedef std::deque<base::State*> DensePath;

            // The InterfaceHash structure is wrapped inside of this struct due to a compilation error on
            // GCC 4.6 with Boost 1.48.  An implicit assignment operator overload does not compile with these
            // components, so an explicit overload is given here.
            // Remove this struct when the minimum Boost requirement is > v1.48.
            struct InterfaceHashStruct
            {
                InterfaceHashStruct& operator=(const InterfaceHashStruct &rhs) { interfaceHash = rhs.interfaceHash; return *this; }
                InterfaceHash interfaceHash;
            };

            /**
             @brief The constructed roadmap spanner.

             @par Any BGL graph representation could be used here, but the
             spanner should be very sparse (m<n^2), so we use an adjacency_list.

             @par Nodes in the spanner contain extra information needed by the
             spanner technique, including nodes in the dense graph which nodes
             in the spanner represent.

             @par SparseEdges should be undirected and have a weight property.
             */
            typedef boost::adjacency_list <
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property < vertex_state_t, base::State*,
                boost::property < boost::vertex_predecessor_t, VertexIndexType,
                boost::property < boost::vertex_rank_t, VertexIndexType,
                boost::property < vertex_color_t, GuardType,
                boost::property < vertex_list_t, std::set<VertexIndexType>,
                boost::property < vertex_interface_list_t, InterfaceHashStruct > > > > > >,
                boost::property < boost::edge_weight_t, double >
            > SpannerGraph;

            /** \brief A vertex in the sparse roadmap that is constructed */
            typedef boost::graph_traits<SpannerGraph>::vertex_descriptor SparseVertex;

            /** \brief An edge in the sparse roadmap that is constructed */
            typedef boost::graph_traits<SpannerGraph>::edge_descriptor   SparseEdge;

            /** \brief Nearest neighbor structure which works over the SpannerGraph */
            typedef boost::shared_ptr< NearestNeighbors<SparseVertex> > SparseNeighbors;

            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix.

             @par Obviously, a ompl::base::State* vertex property is required.
             The incremental connected components algorithm requires
             vertex_predecessor_t and vertex_rank_t properties.
             If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             @par DenseEdges should be undirected and have a weight property.
             */
            typedef boost::adjacency_list <
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property < vertex_state_t, base::State*,
                boost::property < boost::vertex_predecessor_t, VertexIndexType,
                boost::property < boost::vertex_rank_t, VertexIndexType,
                boost::property < vertex_representative_t, SparseVertex > > > >,
                boost::property < boost::edge_weight_t, double >
            > DenseGraph;

            /** \brief A vertex in DenseGraph */
            typedef boost::graph_traits<DenseGraph>::vertex_descriptor DenseVertex;

            /** \brief An edge in DenseGraph */
            typedef boost::graph_traits<DenseGraph>::edge_descriptor   DenseEdge;

            /** \brief Nearest neighbor structure which works over the DenseGraph */
            typedef boost::shared_ptr< NearestNeighbors<DenseVertex> > DenseNeighbors;

            /** \brief Constructor. */
            SPARS(const base::SpaceInformationPtr &si);
            /** \brief Destructor. */
            virtual ~SPARS();

            virtual void setProblemDefinition(const base::ProblemDefinitionPtr &pdef);

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief While the termination condition permits, construct the spanner graph */
            void constructRoadmap(const base::PlannerTerminationCondition &ptc);

            /** \brief While the termination condition permits, construct the spanner graph. If \e stopOnMaxFail is true,
                the function also terminates when the failure limit set by setMaxFailures() is reached. */
            void constructRoadmap(const base::PlannerTerminationCondition &ptc, bool stopOnMaxFail);

            /** \brief Function that can solve the motion planning
                problem. This function can be called multiple times on
                the same problem, without calling clear() in
                between. This allows the planner to continue work for
                more time on an unsolved problem, for example. Start
                and goal states from the currently specified
                ProblemDefinition are cached. This means that between
                calls to solve(), input states are only added, not
                removed. When using PRM as a multi-query planner, the
                input states should be however cleared, without
                clearing the roadmap itself. This can be done using
                the clearQuery() function. */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for PRM. */
            void clearQuery();

            virtual void clear();

            /** \brief Set a different nearest neighbors datastructure for the roadmap graph.
                This nearest neighbor structure contains only information on the nodes
                existing in the underlying dense roadmap.  This structure is used for
                near-neighbor queries for the construction of that graph as well as for
                determining which dense samples the sparse roadmap nodes should represent.*/
            template<template<typename T> class NN>
            void setDenseNeighbors()
            {
                nn_.reset(new NN<DenseVertex>());
                connectionStrategy_.clear();
                if (isSetup())
                    setup();
            }

            /** \brief Set a different nearest neighbors datastructure for the spanner graph.
                This structure is stores only nodes in the roadmap spanner, and is used in
                the construction of the spanner.  It can also be queried to determine which
                node in the spanner should represent a given state.*/
            template<template<typename T> class NN>
            void setSparseNeighbors()
            {
                snn_.reset(new NN<SparseVertex>());
                if (isSetup())
                    setup();
            }

            /** \brief Set the maximum consecutive failures to augment the spanner before termination.
                In general, if the algorithm fails to add to the spanner for M consecutive iterations,
                then we can probabilistically estimate how close to attaining the desired properties
                the SPARS spanner is.*/
            void setMaxFailures(unsigned int m)
            {
                maxFailures_ = m;
            }

            /** \brief Set the delta fraction for interface detection.  If two nodes in the dense graph
                are more than a delta fraction of the max. extent apart, then the algorithm cannot
                consider them to have accurately approximated the location of an interface. */
            void setDenseDeltaFraction(double d)
            {
                denseDeltaFraction_ = d;
                if (denseDelta_ > 0.0) // setup was previously called
                    denseDelta_ = d * si_->getMaximumExtent();
            }

            /** \brief Set the delta fraction for connection distance on the sparse spanner.  This
                value represents the visibility range of sparse samples.  A sparse node
                represents all dense nodes within a delta fraction of the max. extent if it is
                also the closest sparse node to that dense node. */
            void setSparseDeltaFraction(double d)
            {
                sparseDeltaFraction_ = d;
                if (sparseDelta_ > 0.0) // setup was previously called
                    sparseDelta_ = d * si_->getMaximumExtent();
            }

            /** \brief Set the roadmap spanner stretch factor.  This value represents a multiplicative upper bound on path
                quality that should be produced by the roadmap spanner. The produced sparse graph with solutions that
                are less than \e t times the optimap path length.  It does not make sense to make this parameter more than 3. */
            void setStretchFactor(double t)
            {
                stretchFactor_ = t;
            }

            /** \brief Retrieve the maximum consecutive failure limit. */
            unsigned getMaxFailures() const
            {
                return maxFailures_;
            }

            /** \brief Retrieve the dense graph interface support delta fraction. */
            double getDenseDeltaFraction() const
            {
                return denseDeltaFraction_;
            }

            /** \brief Retrieve the sparse graph visibility range delta fraction. */
            double getSparseDeltaFraction() const
            {
                return sparseDeltaFraction_;
            }

            /** \brief Retrieve the spanner's set stretch factor. */
            double getStretchFactor() const
            {
                return stretchFactor_;
            }

            virtual void setup();

            /** \brief Retrieve the underlying dense graph structure.  This is built as a PRM* and asymptotically approximates best paths through the space. */
            const DenseGraph& getDenseGraph() const
            {
                return g_;
            }

            /** \brief Retrieve the sparse roadmap structure.  This is the structure which
                answers given queries, and has the desired property of asymptotic near-optimality.*/
            const SpannerGraph& getRoadmap() const
            {
                return s_;
            }

            /** \brief Returns the number of milestones added to D */
            unsigned int milestoneCount() const
            {
                return boost::num_vertices(g_);
            }

            /** \brief Returns the number of guards added to S */
            unsigned int guardCount() const
            {
                return boost::num_vertices(s_);
            }

            /** \brief Get the number of iterations the algorithm performed */
            long unsigned int getIterations() const
            {
                return iterations_;
            }

            /** \brief Returns the average valence of the spanner graph */
            double averageValence() const;

            /** \brief Print debug information about planner */
            void printDebug(std::ostream &out = std::cout) const;

            /** \brief Returns true if we have reached the iteration failures limit, \e maxFailures_  */
            bool reachedFailureLimit() const;

        protected:

            /** \brief Attempt to add a single sample to the roadmap. */
            DenseVertex addSample(base::State *workState, const base::PlannerTerminationCondition &ptc);

            /** \brief Check that the query vertex is initialized (used for internal nearest neighbor searches) */
            void checkQueryStateInitialization();

            /** \brief Check that two vertices are in the same connected component */
            bool sameComponent(SparseVertex m1, SparseVertex m2);

            /** \brief Construct a milestone for a given state (\e state) and store it in the nearest neighbors data structure */
            DenseVertex addMilestone(base::State *state);

            /** \brief Construct a node with the given state (\e state) for the spanner and store it in the nn structure */
            SparseVertex addGuard(base::State *state, GuardType type);

            /** \brief Convenience function for creating an edge in the Spanner Roadmap */
            void connectSparsePoints(SparseVertex v, SparseVertex vp);

            /** \brief Connects points in the dense graph */
            void connectDensePoints(DenseVertex v, DenseVertex vp);

            /** \brief Checks the latest dense sample for the coverage property, and adds appropriately. */
            bool checkAddCoverage(const base::State *lastState, const std::vector<SparseVertex> &neigh);

            /** \brief Checks the latest dense sample for connectivity, and adds appropriately. */
            bool checkAddConnectivity(const base::State *lastState, const std::vector<SparseVertex> &neigh);

            /** \brief Checks the latest dense sample for bridging an edge-less interface */
            bool checkAddInterface(const std::vector<DenseVertex>& graphNeighborhood, const std::vector<DenseVertex>& visibleNeighborhood, DenseVertex q);

            /** \brief Checks for adding an entire dense path to the Sparse Roadmap */
            bool checkAddPath( DenseVertex q, const std::vector<DenseVertex>& neigh );

            /** \brief Get the first neighbor of q who has representative rep and is within denseDelta_. */
            DenseVertex getInterfaceNeighbor(DenseVertex q, SparseVertex rep);

            /** \brief Method for actually adding a dense path to the Roadmap Spanner, S. */
            bool addPathToSpanner(const DensePath &p, SparseVertex vp, SparseVertex vpp);

            /** \brief Automatically updates the representatives of all dense samplse within sparseDelta_ of v */
            void updateRepresentatives(SparseVertex v);

            /** \brief Calculates the representative for a dense sample */
            void calculateRepresentative( DenseVertex q );

            /** \brief Adds a dense sample to the appropriate lists of its representative */
            void addToRepresentatives( DenseVertex q, SparseVertex rep, const std::set<SparseVertex>& oreps );

            /** \brief Removes the node from its representative's lists */
            void removeFromRepresentatives( DenseVertex q, SparseVertex rep );

            /** \brief Computes all nodes which qualify as a candidate v" for v and vp */
            void computeVPP(DenseVertex v, DenseVertex vp, std::vector<SparseVertex> &VPPs);

            /** \brief Computes all nodes which qualify as a candidate x for v, v', and v" */
            void computeX(DenseVertex v, DenseVertex vp, DenseVertex vpp, std::vector<SparseVertex> &Xs);

            /** \brief A reset function for resetting the failures count */
            void resetFailures();

            /** Thread that checks for solution */
            void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, the path is saved. */
            bool haveSolution(const std::vector<DenseVertex> &start, const std::vector<DenseVertex> &goal, base::PathPtr &solution);

            /** \brief Returns true if we have reached the iteration failures limit, \e maxFailures_ or if a solution was added */
            bool reachedTerminationCriterion() const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
            base::PathPtr constructSolution(const SparseVertex start, const SparseVertex goal) const;

            /** \brief Constructs the dense path between the start and goal vertices (if connected) */
            void computeDensePath(const DenseVertex start, const DenseVertex goal, DensePath &path) const;

            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Get all nodes in the sparse graph which are within sparseDelta_ of the given state. */
            void getSparseNeighbors(base::State *inState, std::vector< SparseVertex > &graphNeighborhood);

            /** \brief Get the visible neighbors */
            void filterVisibleNeighbors(base::State *inState, const std::vector<SparseVertex> &graphNeighborhood, std::vector<SparseVertex> &visibleNeighborhood) const;

            /** \brief Gets the representatives of all interfaces that q supports */
            void getInterfaceNeighborRepresentatives(DenseVertex q, std::set<SparseVertex> &interfaceRepresentatives);

            /** \brief Gets the neighbors of q who help it support an interface */
            void getInterfaceNeighborhood(DenseVertex q, std::vector<DenseVertex> &interfaceNeighborhood);

            /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
            double distanceFunction(const DenseVertex a, const DenseVertex b) const
            {
                return si_->distance(stateProperty_[a], stateProperty_[b]);
            }

            /** \brief Compute distance between two nodes in the sparse roadmap spanner. */
            double sparseDistanceFunction( const SparseVertex a, const SparseVertex b ) const
            {
                return si_->distance( sparseStateProperty_[a], sparseStateProperty_[b] );
            }

            /** \brief Sampler user for generating valid samples in the state space */
            base::ValidStateSamplerPtr                                          sampler_;

            /** \brief Sampler user for generating random in the state space */
            base::StateSamplerPtr                                               simpleSampler_;

            /** \brief Nearest neighbors data structure */
            DenseNeighbors                                                      nn_;

            /** \brief Nearest Neighbors structure for the sparse roadmap */
            SparseNeighbors                                                     snn_;

            /** \brief The dense graph, D */
            DenseGraph                                                          g_;

            /** \brief The sparse roadmap, S */
            SpannerGraph                                                        s_;

            /** \brief Array of start guards */
            std::vector<SparseVertex>                                           startM_;

            /** \brief Array of goal guards */
            std::vector<SparseVertex>                                           goalM_;

            /** \brief DenseVertex for performing nearest neighbor queries on the SPARSE roadmap. */
            DenseVertex                                                         sparseQueryVertex_;

            /** \brief Vertex for performing nearest neighbor queries on the DENSE graph. */
            DenseVertex                                                         queryVertex_;

            /** \brief Geometric Path variable used for smoothing out paths. */
            PathGeometric                                                       geomPath_;

            /** \brief Access to the internal base::state at each DenseVertex */
            boost::property_map<DenseGraph, vertex_state_t>::type               stateProperty_;

            /** \brief Access to the internal base::State for each SparseVertex of S */
            boost::property_map<SpannerGraph, vertex_state_t>::type             sparseStateProperty_;

            /** \brief Access to draw colors for the SparseVertexs of S, to indicate addition type */
            boost::property_map<SpannerGraph, vertex_color_t>::type             sparseColorProperty_;

            /** \brief Access to the representatives of the Dense vertices */
            boost::property_map<DenseGraph, vertex_representative_t>::type      representativesProperty_;

            /** \brief Access to all non-interface supporting vertices of the sparse nodes */
            boost::property_map<SpannerGraph, vertex_list_t>::type              nonInterfaceListsProperty_;

            /** \brief Access to the interface-supporting vertice hashes of the sparse nodes */
            boost::property_map<SpannerGraph, vertex_interface_list_t>::type    interfaceListsProperty_;

            /** \brief A path simplifier used to simplify dense paths added to S */
            PathSimplifierPtr                                                   psimp_;

            /** \brief Access to the weights of each DenseEdge */
            boost::property_map<DenseGraph, boost::edge_weight_t>::type         weightProperty_;

            /** \brief Data structure that maintains the connected components of S */
            boost::disjoint_sets<
                boost::property_map<SpannerGraph, boost::vertex_rank_t>::type,
                boost::property_map<SpannerGraph, boost::vertex_predecessor_t>::type >
                                                                                sparseDJSets_;

            /** \brief Function that returns the milestones to attempt connections with */
            boost::function<std::vector<DenseVertex>&(const DenseVertex)>       connectionStrategy_;

            /** \brief A counter for the number of consecutive failed iterations of the algorithm */
            unsigned int                                                        consecutiveFailures_;

            /** \brief A counter for the number of iterations of the algorithm */
            long unsigned int                                                   iterations_;

            /** \brief The stretch factor in terms of graph spanners for SPARS to check against */
            double                                                              stretchFactor_;

            /** \brief The maximum number of failures before terminating the algorithm */
            unsigned int                                                        maxFailures_;

            /** \brief A flag indicating that a solution has been added during solve() */
            bool                                                                addedSolution_;

            /** \brief SPARS parameter for dense graph connection distance as a fraction of max. extent */
            double                                                              denseDeltaFraction_;

            /** \brief SPARS parameter for Sparse Roadmap connection distance as a fraction of max. extent */
            double                                                              sparseDeltaFraction_;

            /** \brief SPARS parameter for dense graph connection distance */
            double                                                              denseDelta_;

            /** \brief SPARS parameter for Sparse Roadmap connection distance */
            double                                                              sparseDelta_;

            /** \brief Random number generator */
            RNG                                                                 rng_;

            /** \brief Mutex to guard access to the graphs */
            mutable boost::mutex                                                graphMutex_;

        };

    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
*  All Rights Reserved.
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Author: Andrew Dobson */

#ifndef OMPL_GEOMETRIC_PLANNERS_SPARS_TWO_
#define OMPL_GEOMETRIC_PLANNERS_SPARS_TWO_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Time.h"

#include <boost/range/adaptor/map.hpp>
#include <boost/unordered_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <map>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gSPARStwo
           @par Short description
           SPARStwo is a variant of the SPARS algorithm which removes the
           dependency on having the dense graph, D.  It works through similar
           mechanics, but uses a different approach to identifying interfaces
           and computing shortest paths through said interfaces.
           @par External documentation
           A. Dobson, K. Bekris,
           Improving Sparse Roadmap Spanners,
           <em>IEEE International Conference on Robotics and Automation (ICRA)</em> May 2013.
           <a href="http://www.cs.rutgers.edu/~kb572/pubs/spars2.pdf">[PDF]</a>
        */

        /** \brief <b> SPArse Roadmap Spanner Version 2.0 </b> */
        class SPARStwo : public base::Planner
        {
        public:

            /** \brief Enumeration which specifies the reason a guard is added to the spanner. */
            enum GuardType
            {
                START,
                GOAL,
                COVERAGE,
                CONNECTIVITY,
                INTERFACE,
                QUALITY,
            };

            /** \brief The type used internally for representing vertex IDs */
            typedef unsigned long int VertexIndexType;

            /** \brief Pair of vertices which support an interface. */
            typedef std::pair< VertexIndexType, VertexIndexType > VertexPair;

            /** \brief Interface information storage class, which does bookkeeping for criterion four. */
            struct InterfaceData
            {
                /** \brief States which lie inside the visibility region of a vertex and support an interface. */
                base::State *pointA_;
                base::State *pointB_;

                /** \brief States which lie just outside the visibility region of a vertex and support an interface. */
                base::State *sigmaA_;
                base::State *sigmaB_;

                /** \brief Last known distance between the two interfaces supported by points_ and sigmas. */
                double       d_;

                /** \brief Constructor */
                InterfaceData() :
                    pointA_(NULL),
                    pointB_(NULL),
                    sigmaA_(NULL),
                    sigmaB_(NULL),
                    d_(std::numeric_limits<double>::infinity())
                {
                }

                /** \brief Clears the given interface data. */
                void clear(const base::SpaceInformationPtr& si)
                {
                    if (pointA_)
                    {
                        si->freeState(pointA_);
                        pointA_ = NULL;
                    }
                    if (pointB_)
                    {
                        si->freeState(pointB_);
                        pointB_ = NULL;
                    }
                    if (sigmaA_)
                    {
                        si->freeState(sigmaA_);
                        sigmaA_ = NULL;
                    }
                    if (sigmaB_)
                    {
                        si->freeState(sigmaB_);
                        sigmaB_ = NULL;
                    }
                    d_ = std::numeric_limits<double>::infinity();
                }

                /** \brief Sets information for the first interface (i.e. interface with smaller index vertex). */
                void setFirst(const base::State *p, const base::State *s, const base::SpaceInformationPtr& si)
                {
                    if (pointA_)
                        si->copyState(pointA_, p);
                    else
                        pointA_ = si->cloneState(p);
                    if (sigmaA_)
                        si->copyState(sigmaA_, s);
                    else
                        sigmaA_ = si->cloneState(s);
                    if (pointB_)
                        d_ = si->distance(pointA_, pointB_);
                }

                /** \brief Sets information for the second interface (i.e. interface with larger index vertex). */
                void setSecond(const base::State *p, const base::State *s, const base::SpaceInformationPtr& si)
                {
                    if (pointB_)
                        si->copyState(pointB_, p);
                    else
                        pointB_ = si->cloneState(p);
                    if (sigmaB_)
                        si->copyState(sigmaB_, s);
                    else
                        sigmaB_ = si->cloneState(s);
                    if (pointA_)
                        d_ = si->distance(pointA_, pointB_);
                }
            };

            /** \brief the hash which maps pairs of neighbor points to pairs of states */
            typedef boost::unordered_map< VertexPair, InterfaceData, boost::hash< VertexPair > > InterfaceHash;

            // The InterfaceHash structure is wrapped inside of this struct due to a compilation error on
            // GCC 4.6 with Boost 1.48.  An implicit assignment operator overload does not compile with these
            // components, so an explicit overload is given here.
            // Remove this struct when the minimum Boost requirement is > v1.48.
            struct InterfaceHashStruct
            {
                InterfaceHashStruct& operator=(const InterfaceHashStruct &rhs) { interfaceHash = rhs.interfaceHash; return *this; }
                InterfaceHash interfaceHash;
            };

            struct vertex_state_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_color_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_interface_data_t {
                typedef boost::vertex_property_tag kind;
            };

            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix.

             @par Obviously, a ompl::base::State* vertex property is required.
             The incremental connected components algorithm requires
             vertex_predecessor_t and vertex_rank_t properties.
             If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             @par Edges should be undirected and have a weight property.
             */
            typedef boost::adjacency_list <
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property < vertex_state_t, base::State*,
                boost::property < boost::vertex_predecessor_t, VertexIndexType,
                boost::property < boost::vertex_rank_t, VertexIndexType,
                boost::property < vertex_color_t, GuardType,
                boost::property < vertex_interface_data_t, InterfaceHashStruct > > > > >,
                boost::property < boost::edge_weight_t, double >
            > Graph;

            /** \brief Vertex in Graph */
            typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

            /** \brief Edge in Graph */
            typedef boost::graph_traits<Graph>::edge_descriptor   Edge;

            /** \brief Constructor */
            SPARStwo(const base::SpaceInformationPtr &si);

            /** \brief Destructor */
            virtual ~SPARStwo();

            virtual void setProblemDefinition(const base::ProblemDefinitionPtr &pdef);

            /** \brief Sets the stretch factor */
            void setStretchFactor(double t)
            {
                stretchFactor_ = t;
            }

            /** \brief Sets vertex visibility range as a fraction of max. extent. */
            void setSparseDeltaFraction( double D )
            {
                sparseDeltaFraction_ = D;
                if (sparseDelta_ > 0.0) // setup was previously called
                    sparseDelta_ = D * si_->getMaximumExtent();
            }

            /** \brief Sets interface support tolerance as a fraction of max. extent. */
            void setDenseDeltaFraction( double d )
            {
                denseDeltaFraction_ = d;
                if (denseDelta_ > 0.0) // setup was previously called
                    denseDelta_ = d * si_->getMaximumExtent();
            }

            /** \brief Sets the maximum failures until termination */
            void setMaxFailures( unsigned int m )
            {
                maxFailures_ = m;
            }

            /** \brief Retrieve the maximum consecutive failure limit. */
            unsigned int getMaxFailures( ) const
            {
                return maxFailures_;
            }

            /** \brief Retrieve the dense graph interface support delta. */
            double getDenseDeltaFraction( ) const
            {
                return denseDeltaFraction_;
            }

            /** \brief Retrieve the sparse graph visibility range delta. */
            double getSparseDeltaFraction( ) const
            {
                return sparseDeltaFraction_;
            }

            /** \brief Retrieve the spanner's set stretch factor. */
            double getStretchFactor( ) const
            {
                return stretchFactor_;
            }

            /** \brief While the termination condition permits, construct the spanner graph */
            void constructRoadmap(const base::PlannerTerminationCondition &ptc);

            /** \brief While the termination condition permits, construct the spanner graph. If \e stopOnMaxFail is true,
                the function also terminates when the failure limit set by setMaxFailures() is reached. */
            void constructRoadmap(const base::PlannerTerminationCondition &ptc, bool stopOnMaxFail);

            /** \brief Function that can solve the motion planning
                problem. This function can be called multiple times on
                the same problem, without calling clear() in
                between. This allows the planner to continue work for
                more time on an unsolved problem, for example. Start
                and goal states from the currently specified
                ProblemDefinition are cached. This means that between
                calls to solve(), input states are only added, not
                removed. When using PRM as a multi-query planner, the
                input states should be however cleared, without
                clearing the roadmap itself. This can be done using
                the clearQuery() function. */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for PRM. */
            void clearQuery();

            virtual void clear();

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors()
            {
                nn_.reset(new NN< Vertex >());
                if (isSetup())
                    setup();
            }

            virtual void setup();

            /** \brief Retrieve the computed roadmap. */
            const Graph& getRoadmap() const
            {
                return g_;
            }

            /** \brief Get the number of vertices in the sparse roadmap. */
            unsigned int milestoneCount() const
            {
                return boost::num_vertices(g_);
            }

            /** \brief Get the number of iterations the algorithm performed */
            long unsigned int getIterations() const
            {
                return iterations_;
            }

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:

            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Check that the query vertex is initialized (used for internal nearest neighbor searches) */
            void checkQueryStateInitialization();

            /** \brief Checks to see if the sample needs to be added to ensure coverage of the space */
            bool checkAddCoverage(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood);

            /** \brief Checks to see if the sample needs to be added to ensure connectivity */
            bool checkAddConnectivity(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood);

            /** \brief Checks to see if the current sample reveals the existence of an interface, and if so, tries to bridge it. */
            bool checkAddInterface(const base::State *qNew, std::vector<Vertex> &graphNeighborhood, std::vector<Vertex> &visibleNeighborhood);

            /** \brief Checks vertex v for short paths through its region and adds when appropriate. */
            bool checkAddPath( Vertex v );

            /** \brief A reset function for resetting the failures count */
            void resetFailures();

            /** \brief Finds visible nodes in the graph near st */
            void findGraphNeighbors(base::State *st, std::vector<Vertex> &graphNeighborhood, std::vector<Vertex> &visibleNeighborhood);

            /** \brief Approaches the graph from a given vertex */
            void approachGraph( Vertex v );

            /** \brief Finds the representative of the input state, st  */
            Vertex findGraphRepresentative(base::State *st);

            /** \brief Finds representatives of samples near qNew_ which are not his representative */
            void findCloseRepresentatives(base::State *workArea, const base::State *qNew, Vertex qRep,
                                          std::map<Vertex, base::State*> &closeRepresentatives, const base::PlannerTerminationCondition &ptc);

            /** \brief High-level method which updates pair point information for repV_ with neighbor r */
            void updatePairPoints(Vertex rep, const base::State *q, Vertex r, const base::State *s);

            /** \brief Computes all nodes which qualify as a candidate v" for v and vp */
            void computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs);

            /** \brief Computes all nodes which qualify as a candidate x for v, v', and v" */
            void computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs);

            /** \brief Rectifies indexing order for accessing the vertex data */
            VertexPair index( Vertex vp, Vertex vpp );

            /** \brief Retrieves the Vertex data associated with v,vp,vpp */
            InterfaceData& getData( Vertex v, Vertex vp, Vertex vpp );

            /** \brief Performs distance checking for the candidate new state, q against the current information */
            void distanceCheck(Vertex rep, const base::State *q, Vertex r, const base::State *s, Vertex rp);

            /** \brief When a new guard is added at state st, finds all guards who must abandon their interface information and deletes that information */
            void abandonLists(base::State *st);

            /** \brief Construct a guard for a given state (\e state) and store it in the nearest neighbors data structure */
            Vertex addGuard(base::State *state, GuardType type);

            /** \brief Connect two guards in the roadmap */
            void connectGuards( Vertex v, Vertex vp );

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, the path is saved. */
            bool haveSolution(const std::vector<Vertex> &start, const std::vector<Vertex> &goal, base::PathPtr &solution);

            /** Thread that checks for solution */
            void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);

            /** \brief Returns true if we have reached the iteration failures limit, \e maxFailures_ or if a solution was added */
            bool reachedTerminationCriterion() const;

            /** \brief Returns whether we have reached the iteration failures limit, maxFailures_ */
            bool reachedFailureLimit () const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
            base::PathPtr constructSolution(const Vertex start, const Vertex goal) const;

            /** \brief Check if two milestones (\e m1 and \e m2) are part of the same connected component. This is not a const function since we use incremental connected components from boost */
            bool sameComponent(Vertex m1, Vertex m2);

            /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
            double distanceFunction(const Vertex a, const Vertex b) const
            {
                return si_->distance(stateProperty_[a], stateProperty_[b]);
            }

            /** \brief Sampler user for generating valid samples in the state space */
            base::ValidStateSamplerPtr                                          sampler_;

            /** \brief Sampler user for generating random in the state space */
            base::StateSamplerPtr                                               simpleSampler_;

            /** \brief Nearest neighbors data structure */
            boost::shared_ptr< NearestNeighbors<Vertex> >                       nn_;

            /** \brief Connectivity graph */
            Graph                                                               g_;

            /** \brief Array of start milestones */
            std::vector<Vertex>                                                 startM_;

            /** \brief Array of goal milestones */
            std::vector<Vertex>                                                 goalM_;

            /** \brief Vertex for performing nearest neighbor queries. */
            Vertex                                                              queryVertex_;

            /** \brief Stretch Factor as per graph spanner literature (multiplicative bound on path quality) */
            double                                                              stretchFactor_;

            /** \brief Maximum visibility range for nodes in the graph as a fraction of maximum extent. */
            double                                                              sparseDeltaFraction_;

            /** \brief Maximum range for allowing two samples to support an interface as a fraction of maximum extent. */
            double                                                              denseDeltaFraction_;

            /** \brief The number of consecutive failures to add to the graph before termination */
            unsigned int                                                        maxFailures_;

            /** \brief Number of sample points to use when trying to detect interfaces. */
            unsigned int                                                        nearSamplePoints_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type                    stateProperty_;

            /** \brief A path simplifier used to simplify dense paths added to the graph */
            PathSimplifierPtr                                                   psimp_;

            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type              weightProperty_;

            /** \brief Access to the colors for the vertices */
            boost::property_map<Graph, vertex_color_t>::type                    colorProperty_;

            /** \brief Access to the interface pair information for the vertices */
            boost::property_map<Graph, vertex_interface_data_t>::type           interfaceDataProperty_;

            /** \brief Data structure that maintains the connected components */
            boost::disjoint_sets<
                boost::property_map<Graph, boost::vertex_rank_t>::type,
                boost::property_map<Graph, boost::vertex_predecessor_t>::type >
                                                                                disjointSets_;
            /** \brief Random number generator */
            RNG                                                                 rng_;

            /** \brief A flag indicating that a solution has been added during solve() */
            bool                                                                addedSolution_;

            /** \brief A counter for the number of consecutive failed iterations of the algorithm */
            unsigned int                                                        consecutiveFailures_;

            /** \brief A counter for the number of iterations of the algorithm */
            long unsigned int                                                   iterations_;

            /** \brief Maximum visibility range for nodes in the graph */
            double                                                              sparseDelta_;

            /** \brief Maximum range for allowing two samples to support an interface */
            double                                                              denseDelta_;

            /** \brief Mutex to guard access to the Graph member (g_) */
            mutable boost::mutex                                                graphMutex_;

        };

    }
}

#endif
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

#ifndef OMPL_GEOMETRIC_PLANNERS_EST_EST_
#define OMPL_GEOMETRIC_PLANNERS_EST_EST_

#include "ompl/datastructures/Grid.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/PDF.h"
#include <vector>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gEST
           @par Short description
           EST is a tree-based motion planner that attempts to detect
           the less explored area of the space through the use of a
           grid imposed on a projection of the state space. Using this
           information, EST continues tree expansion primarily from
           less explored areas.  It is important to set the projection
           the algorithm uses (setProjectionEvaluator() function). If
           no projection is set, the planner will attempt to use the
           default projection associated to the state space. An
           exception is thrown if no default projection is available
           either.
           @par External documentation
           D. Hsu, J.-C. Latombe, and R. Motwani, Path planning in expansive configuration spaces,
           <em>Intl. J. Computational Geometry and Applications</em>,
           vol. 9, no. 4-5, pp. 495–512, 1999. DOI: <a href="http://dx.doi.org/10.1142/S0218195999000285">10.1142/S0218195999000285</a><br>
           <a href="http://bigbird.comp.nus.edu.sg/pmwiki/farm/motion/uploads/Site/ijcga96.pdf">[PDF]</a>
        */

        /** \brief Expansive Space Trees */
        class EST : public base::Planner
        {
        public:

            /** \brief Constructor */
            EST(const base::SpaceInformationPtr &si);

            virtual ~EST();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            /** \brief In the process of randomly selecting states in
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

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state.  */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
                the ones registered with the state space). */
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /** \brief Get the projection evaluator */
            const base::ProjectionEvaluatorPtr& getProjectionEvaluator() const
            {
                return projectionEvaluator_;
            }

            virtual void setup();

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:

            /** \brief The definition of a motion */
            class Motion
            {
            public:

                Motion() : state(NULL), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;
            };

            struct MotionInfo;

            /** \brief A grid cell */
            typedef Grid<MotionInfo>::Cell GridCell;

            /** \brief A PDF of grid cells */
            typedef PDF<GridCell*>        CellPDF;

            /** \brief A struct containing an array of motions and a corresponding PDF element */
            struct MotionInfo
            {
                Motion* operator[](unsigned int i)
                {
                    return motions_[i];
                }
                const Motion* operator[](unsigned int i) const
                {
                    return motions_[i];
                }
                void push_back(Motion *m)
                {
                    motions_.push_back(m);
                }
                unsigned int size() const
                {
                    return motions_.size();
                }
                bool empty() const
                {
                    return motions_.empty();
                }
                std::vector<Motion*> motions_;
                CellPDF::Element    *elem_;
            };


            /** \brief The data contained by a tree of exploration */
            struct TreeData
            {
                TreeData() : grid(0), size(0)
                {
                }

                /** \brief A grid where each cell contains an array of motions */
                Grid<MotionInfo> grid;

                /** \brief The total number of motions in the grid */
                unsigned int    size;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Add a motion to the exploration tree */
            void addMotion(Motion *motion);

            /** \brief Select a motion to continue the expansion of the tree from */
            Motion* selectMotion();

            /** \brief Valid state sampler */
            base::ValidStateSamplerPtr   sampler_;

            /** \brief The exploration tree constructed by this algorithm */
            TreeData                     tree_;

            /** \brief This algorithm uses a discretization (a grid) to guide the exploration. The exploration is imposed on a projection of the state space. */
            base::ProjectionEvaluatorPtr projectionEvaluator_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                       goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                       maxDistance_;

            /** \brief The random number generator */
            RNG                          rng_;

            /** \brief The PDF used for selecting a cell from which to sample a motion */
            CellPDF                      pdf_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                       *lastGoalMotion_;
        };

    }
}

#endif
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

#ifndef OMPL_GEOMETRIC_PLANNERS_KPIECE_KPIECE1_
#define OMPL_GEOMETRIC_PLANNERS_KPIECE_KPIECE1_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/planners/kpiece/Discretization.h"

namespace ompl
{

    namespace geometric
    {


        /**
           @anchor gKPIECE1
           @par Short description
           KPIECE is a tree-based planner that uses a discretization
           (multiple levels, in general) to guide the exploration of
           the continuous space. This implementation is a simplified
           one, using a single level of discretization: one grid. The
           grid is imposed on a projection of the state space. When
           exploring the space, preference is given to the boundary of
           this grid. The boundary is computed to be the set of grid
           cells that have less than 2n non-diagonal neighbors in an
           n-dimensional projection space.
           It is important to set the projection the algorithm uses
           (setProjectionEvaluator() function). If no projection is
           set, the planner will attempt to use the default projection
           associated to the state space. An exception is thrown if
           no default projection is available either.
           @par External documentation
           I.A. Şucan and L.E. Kavraki, Kinodynamic motion planning by interior-exterior cell exploration,
           in <em>Workshop on the Algorithmic Foundations of Robotics</em>, Dec. 2008.<br>
           <a href="http://ioan.sucan.ro/files/pubs/wafr2008.pdf">[PDF]</a>
        */

        /** \brief Kinematic Planning by Interior-Exterior Cell Exploration */
        class KPIECE1 : public base::Planner
        {
        public:

            /** \brief Constructor */
            KPIECE1(const base::SpaceInformationPtr &si);

            virtual ~KPIECE1();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            /** \brief Set the goal bias.

                In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
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

            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). This is the minimum fraction
                used to select cells that are exterior (minimum
                because if 95% of cells are on the border, they will
                be selected with 95% chance, even if this fraction is
                set to 90%)*/
            void setBorderFraction(double bp)
            {
                disc_.setBorderFraction(bp);
            }

            /** \brief Get the fraction of time to focus exploration
                on boundary */
            double getBorderFraction() const
            {
                return disc_.getBorderFraction();
            }

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion. This function sets the minimum acceptable
                fraction (between 0 and 1). */
            void setMinValidPathFraction(double fraction)
            {
                minValidPathFraction_ = fraction;
            }

            /** \brief Get the value of the fraction set by setMinValidPathFraction() */
            double getMinValidPathFraction() const
            {
                return minValidPathFraction_;
            }

            /** \brief When extending a motion from a cell, the
                extension can be successful or it can fail. If the
                extension fails, the score of the cell is multiplied
                by \e factor. These number should be in the range (0, 1]. */
            void setFailedExpansionCellScoreFactor(double factor)
            {
                failedExpansionScoreFactor_ = factor;
            }

            /** \brief Get the factor that is multiplied to a cell's
                score if extending a motion from that cell failed. */
            double getFailedExpansionCellScoreFactor() const
            {
                return failedExpansionScoreFactor_;
            }

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state. */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
                the ones registered with the state space). */
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /** \brief Get the projection evaluator */
            const base::ProjectionEvaluatorPtr& getProjectionEvaluator() const
            {
                return projectionEvaluator_;
            }

            virtual void setup();

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:

            /** \brief Representation of a motion for this algorithm */
            class Motion
            {
            public:

                Motion() : state(NULL), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by this motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;
            };

            /** \brief Free the memory for a motion */
            void freeMotion(Motion *motion);

            /** \brief A state space sampler */
            base::StateSamplerPtr                      sampler_;

            /** \brief The tree datastructure and the grid that covers it */
            Discretization<Motion>                     disc_;

            /** \brief This algorithm uses a discretization (a grid)
                to guide the exploration. The exploration is imposed
                on a projection of the state space. */
            base::ProjectionEvaluatorPtr               projectionEvaluator_;

            /** \brief When extending a motion from a cell, the
                extension can fail. If it is, the score of the cell is
                multiplied by this factor. */
            double                                     failedExpansionScoreFactor_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                     goalBias_;

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion */
            double                                     minValidPathFraction_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                     maxDistance_;

            /** \brief The random number generator */
            RNG                                        rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                     *lastGoalMotion_;
        };

    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Rice University,
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
*   * Neither the name of the Rice University nor the names of its
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

#ifndef OMPL_GEOMETRIC_PLANNERS_KPIECE_BKPIECE1_
#define OMPL_GEOMETRIC_PLANNERS_KPIECE_BKPIECE1_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/planners/kpiece/Discretization.h"

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gBKPIECE1
           @par Short description
           KPIECE is a tree-based planner that uses a discretization
           (multiple levels, in general) to guide the exploration of
           the continuous space. This implementation is a simplified
           one, using a single level of discretization: one grid. The
           grid is imposed on a projection of the state space. When
           exploring the space, preference is given to the boundary of
           this grid. The boundary is computed to be the set of grid
           cells that have less than 2n non-diagonal neighbors in an
           n-dimensional projection space.
           It is important to set the projection the algorithm uses
           (setProjectionEvaluator() function). If no projection is
           set, the planner will attempt to use the default projection
           associated to the state space. An exception is thrown if
           no default projection is available either.
           This variant of the implementation use two trees of
           exploration, hence the B prefix.
           @par External documentation
           - I.A. Şucan and L.E. Kavraki, Kinodynamic motion planning by interior-exterior cell exploration,
           in <em>Workshop on the Algorithmic Foundations of Robotics</em>, Dec. 2008.<br>
           <a href="http://ioan.sucan.ro/files/pubs/wafr2008.pdf">[PDF]</a>
           - R. Bohlin and L.E. Kavraki, Path planning using lazy PRM, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 521–528, 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844107">10.1109/ROBOT.2000.844107</a><br>
           <a href="http://ieeexplore.ieee.org/ielx5/6794/18235/00844107.pdf?tp=&arnumber=844107&isnumber=18235">[PDF]
        */

        /** \brief Bi-directional KPIECE with one level of discretization */
        class BKPIECE1 : public base::Planner
        {
        public:

            /** \brief Constructor */
            BKPIECE1(const base::SpaceInformationPtr &si);

            virtual ~BKPIECE1();

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state. */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
                the ones registered with the state space). */
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /** \brief Get the projection evaluator. */
            const base::ProjectionEvaluatorPtr& getProjectionEvaluator() const
            {
                return projectionEvaluator_;
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

            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). This is the minimum fraction
                used to select cells that are exterior (minimum
                because if 95% of cells are on the border, they will
                be selected with 95% chance, even if this fraction is
                set to 90%)*/
            void setBorderFraction(double bp)
            {
                dStart_.setBorderFraction(bp);
                dGoal_.setBorderFraction(bp);
            }

            /** \brief Get the fraction of time to focus exploration
                on boundary */
            double getBorderFraction() const
            {
                return dStart_.getBorderFraction();
            }

            /** \brief When extending a motion from a cell, the
                extension can be successful or it can fail. If the
                extension fails, the score of the cell is multiplied
                by \e factor. These number should be in the range (0, 1]. */
            void setFailedExpansionCellScoreFactor(double factor)
            {
                failedExpansionScoreFactor_ = factor;
            }

            /** \brief Get the factor that is multiplied to a cell's
                score if extending a motion from that cell failed. */
            double getFailedExpansionCellScoreFactor() const
            {
                return failedExpansionScoreFactor_;
            }

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion. This function sets the minimum acceptable
                fraction. */
            void setMinValidPathFraction(double fraction)
            {
                minValidPathFraction_ = fraction;
            }

            /** \brief Get the value of the fraction set by setMinValidPathFraction() */
            double getMinValidPathFraction() const
            {
                return minValidPathFraction_;
            }

            virtual void setup();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
            virtual void clear();

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:

            /** \brief Representation of a motion for this algorithm */
            class Motion
            {
            public:

                Motion() : root(NULL), state(NULL), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The root state (start state) that leads to this motion */
                const base::State   *root;

                /** \brief The state contained by this motion */
                base::State         *state;

                /** \brief The parent motion in the exploration tree */
                Motion              *parent;
            };

            /** \brief Free the memory for a motion */
            void freeMotion(Motion *motion);

            /** \brief The employed state sampler */
            base::ValidStateSamplerPtr                 sampler_;

            /** \brief The employed projection evaluator */
            base::ProjectionEvaluatorPtr               projectionEvaluator_;

            /** \brief The start tree */
            Discretization<Motion>                     dStart_;

            /** \brief The goal tree */
            Discretization<Motion>                     dGoal_;

            /** \brief When extending a motion from a cell, the
                extension can fail. If it is, the score of the cell is
                multiplied by this factor. */
            double                                     failedExpansionScoreFactor_;

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion */
            double                                     minValidPathFraction_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                     maxDistance_;

            /** \brief The random number generator */
            RNG                                        rng_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State*, base::State*>      connectionPoint_;
        };

    }
}


#endif
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

#ifndef OMPL_GEOMETRIC_PLANNERS_KPIECE_LBKPIECE1_
#define OMPL_GEOMETRIC_PLANNERS_KPIECE_LBKPIECE1_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/planners/kpiece/Discretization.h"

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gLBKPIECE1
           @par Short description
           KPIECE is a tree-based planner that uses a discretization
           (multiple levels, in general) to guide the exploration of
           the continuous space. This implementation is a simplified
           one, using a single level of discretization: one grid. The
           grid is imposed on a projection of the state space. When
           exploring the space, preference is given to the boundary of
           this grid. The boundary is computed to be the set of grid
           cells that have less than 2n non-diagonal neighbors in an
           n-dimensional projection space.
           It is important to set the projection the algorithm uses
           (setProjectionEvaluator() function). If no projection is
           set, the planner will attempt to use the default projection
           associated to the state space. An exception is thrown if
           no default projection is available either.
           This variant of the implementation use two trees of
           exploration with lazy collision checking, hence the LB
           prefix.
           @par External documentation
           - I.A. Şucan and L.E. Kavraki, Kinodynamic motion planning by interior-exterior cell exploration,
           in <em>Workshop on the Algorithmic Foundations of Robotics</em>, Dec. 2008.<br>
           <a href="http://ioan.sucan.ro/files/pubs/wafr2008.pdf">[PDF]</a>
           - R. Bohlin and L.E. Kavraki, Path planning using lazy PRM, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 521–528, 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844107">10.1109/ROBOT.2000.844107</a><br>
           <a href="http://ieeexplore.ieee.org/ielx5/6794/18235/00844107.pdf?tp=&arnumber=844107&isnumber=18235">[PDF]
        */

        /** \brief Lazy Bi-directional KPIECE with one level of discretization */
        class LBKPIECE1 : public base::Planner
        {
        public:

            /** \brief Constructor */
            LBKPIECE1(const base::SpaceInformationPtr &si);

            virtual ~LBKPIECE1();

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state. */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
                the ones registered with the state space). */
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /** \brief Get the projection evaluator. */
            const base::ProjectionEvaluatorPtr& getProjectionEvaluator() const
            {
                return projectionEvaluator_;
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
            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). This is the minimum fraction
                used to select cells that are exterior (minimum
                because if 95% of cells are on the border, they will
                be selected with 95% chance, even if this fraction is
                set to 90%)*/
            void setBorderFraction(double bp)
            {
                dStart_.setBorderFraction(bp);
                dGoal_.setBorderFraction(bp);
            }

            /** \brief Get the fraction of time to focus exploration
                on boundary */
            double getBorderFraction() const
            {
                return dStart_.getBorderFraction();
            }

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion. This function sets the minimum acceptable
                fraction. */
            void setMinValidPathFraction(double fraction)
            {
                minValidPathFraction_ = fraction;
            }

            /** \brief Get the value of the fraction set by setMinValidPathFraction() */
            double getMinValidPathFraction() const
            {
                return minValidPathFraction_;
            }

            virtual void setup();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
            virtual void clear();

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:

            /** \brief Representation of a motion for this algorithm */
            class Motion
            {
            public:

                Motion() : root(NULL), state(NULL), parent(NULL), valid(false)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL), valid(false)
                {
                }

                ~Motion()
                {
                }

                /** \brief The root state (start state) that leads to this motion */
                const base::State   *root;

                /** \brief The state contained by this motion */
                base::State         *state;

                /** \brief The parent motion in the exploration tree */
                Motion              *parent;

                /** \brief Flag indicating whether this motion has been checked for validity. */
                bool                 valid;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion*> children;
            };

            /** \brief Free the memory for a motion */
            void freeMotion(Motion *motion);

            /** \brief Remove a motion from a tree of motions */
            void removeMotion(Discretization<Motion> &disc, Motion *motion);

            /** \brief Since solutions are computed in a lazy fashion,
                once trees are connected, the solution found needs to
                be checked for validity. This function checks whether
                the reverse path from a given motion to a root is
                valid. If this is not the case, invalid motions are removed  */
            bool isPathValid(Discretization<Motion> &disc, Motion *motion, base::State *temp);

            /** \brief The employed state sampler */
            base::StateSamplerPtr              sampler_;

            /** \brief The employed projection evaluator */
            base::ProjectionEvaluatorPtr               projectionEvaluator_;

            /** \brief The start tree */
            Discretization<Motion>                     dStart_;

            /** \brief The goal tree */
            Discretization<Motion>                     dGoal_;

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion */
            double                                     minValidPathFraction_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                     maxDistance_;

            /** \brief The random number generator */
            RNG                                        rng_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State*, base::State*>      connectionPoint_;
        };

    }
}


#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Jonathan Sobieski, Mark Moll */



#ifndef OMPL_GEOMETRIC_PLANNERS_PDST_PDST_
#define OMPL_GEOMETRIC_PLANNERS_PDST_PDST_

#include "ompl/base/Planner.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/PlannerData.h"
#include "ompl/datastructures/BinaryHeap.h"


namespace ompl
{

    namespace geometric
    {

         /**
            @anchor gPDST
            @par Short description
            PDST is a tree-based motion planner that attempts to detect
            the less explored area of the space through the use of a
            binary space partition of a projection of the state space.
            Exploration is biased towards large cells with few path
            segments. Unlike most tree-based planners which expand from
            a randomly select endpoint of a path segment, PDST expands
            from a randomly selected point along a deterministically
            selected path segment. It is important to set the projection
            the algorithm uses (setProjectionEvaluator() function). If
            no projection is set, the planner will attempt to use the
            default projection associated to the state space. An
            exception is thrown if no default projection is available
            either.
            @par External documentation
            A.M. Ladd and L.E. Kavraki, Motion planning in the presence
            of drift, underactuation and discrete system changes, in
            <em>Robotics: Science and Systems I</em>, pp. 233–241, MIT
            Press, June 2005.
            <a href="http://www.roboticsproceedings.org/rss01/p31.pdf">[PDF]</a>
         */

        /// \brief Path-Directed Subdivision Tree
        class PDST : public base::Planner
        {
        public:

            PDST(const base::SpaceInformationPtr &si);

            virtual ~PDST();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
            virtual void clear();
            virtual void setup();

            /// Extracts the planner data from the priority queue into data.
            virtual void getPlannerData(base::PlannerData &data) const;

            /// Set the projection evaluator. This class is able to compute the projection of a given state.
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /// Set the projection evaluator (select one from the ones registered with the state space).
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /// Get the projection evaluator
            const base::ProjectionEvaluatorPtr& getProjectionEvaluator() const
            {
                return projectionEvaluator_;
            }

            /// \brief In the process of randomly selecting states in
            /// the state space to attempt to go towards, the
            /// algorithm may in fact choose the actual goal state, if
            /// it knows it, with some probability. This probability
            /// is a real number between 0.0 and 1.0; its value should
            /// usually be around 0.05 and should not be too large. It
            /// is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }
            /// Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

        protected:
            struct Cell;
            struct Motion;

            /// Comparator used to order motions in the priority queue
            struct MotionCompare
            {
                /// returns true if m1 is lower priority than m2
                bool operator()(Motion *p1, Motion *p2) const
                {
                    // lowest priority means highest score
                    return p1->score() < p2->score();
                }
            };

            /// Class representing the tree of motions exploring the state space
            struct Motion
            {
            public:
                Motion(base::State *startState, base::State *endState, double priority, Motion *parent)
                    : startState_(startState), endState_(endState), priority_(priority),
                    parent_(parent), cell_(NULL), heapElement_(NULL), isSplit_(false)
                {
                }
                /// constructor for start states
                Motion(base::State *state)
                    : startState_(state), endState_(state), priority_(0.),
                    parent_(NULL), cell_(NULL), heapElement_(NULL), isSplit_(false)
                {
                }
                /// The score is used to order motions in a priority queue.
                double score() const
                {
                    return priority_ / cell_->volume_;
                }
                void updatePriority()
                {
                    priority_ = priority_ * 2.0 + 1.0;
                }
                Motion* ancestor() const
                {
                    Motion *m = const_cast<Motion*>(this);
                    while (m->parent_ && m->parent_->endState_ == m->startState_)
                        m = m->parent_;
                    return m;
                }

                /// The starting point of this motion
                ompl::base::State               *startState_;
                /// The state reached by this motion
                ompl::base::State               *endState_;
                /// Priority for selecting this path to extend from in the future
                double                           priority_;
                /// Parent motion from which this one started
                Motion *parent_;
                /// pointer to the cell that contains this path
                Cell*                            cell_;
                /// Handle to the element of the priority queue for this Motion
                ompl::BinaryHeap<Motion*, MotionCompare>::Element *heapElement_;
                /// Whether this motion is the result of a split operation, in which case
                /// its endState_ should not be freed.
                bool                             isSplit_;
            };

            /// Cell is a Binary Space Partition
            struct Cell
            {
                Cell(double volume, const ompl::base::RealVectorBounds &bounds,
                    unsigned int splitDimension = 0)
                    : volume_(volume), splitDimension_(splitDimension), splitValue_(0.0),
                    left_(NULL), right_(NULL), bounds_(bounds)
                {
                }

                ~Cell()
                {
                    if (left_)
                    {
                        delete left_;
                        delete right_;
                    }
                }

                /// Subdivides this cell
                void subdivide(unsigned int spaceDimension);

                /// Locates the cell that this motion begins in
                Cell* stab(const ompl::base::EuclideanProjection& projection) const
                {
                    Cell *containingCell = const_cast<Cell*>(this);
                    while (containingCell->left_ != NULL)
                    {
                        if (projection[containingCell->splitDimension_] <= containingCell->splitValue_)
                            containingCell = containingCell->left_;
                        else
                            containingCell = containingCell->right_;
                    }
                    return containingCell;
                }
                /// Add a motion
                void addMotion(Motion *motion)
                {
                    motions_.push_back(motion);
                    motion->cell_ = this;
                }

                /// Number of cells
                unsigned int size() const
                {
                    unsigned int sz = 1;
                    if (left_)
                        sz += left_->size() + right_->size();
                    return sz;
                }

                /// Volume of the cell
                double                       volume_;
                /// Dimension along which the cell is split into smaller cells
                unsigned int                 splitDimension_;
                /// The midpoint between the bounds_ at the splitDimension_
                double                       splitValue_;
                /// The left child cell (NULL for a leaf cell)
                Cell*                        left_;
                /// The right child cell (NULL for a leaf cell)
                Cell*                        right_;
                /// A bounding box for this cell
                ompl::base::RealVectorBounds bounds_;
                /// The motions contained in this cell. Motions are stored only in leaf nodes.
                std::vector<Motion*>         motions_;
            };


            /// Inserts the motion into the appropriate cell
            void addMotion(Motion *motion, Cell *cell, base::State*, base::EuclideanProjection&);
            /// \brief Either update heap after motion's priority has changed or insert motion into heap.
            void updateHeapElement(Motion *motion)
            {
                if (motion->heapElement_)
                    priorityQueue_.update(motion->heapElement_);
                else
                    motion->heapElement_ = priorityQueue_.insert(motion);
            }
            /// \brief Select a state along motion and propagate a new motion from there.
            /// Return NULL if no valid motion could be generated starting at the
            /// selected state.
            Motion* propagateFrom(Motion *motion, base::State*, base::State*);

            void freeMemory();

            /// State sampler
            ompl::base::StateSamplerPtr              sampler_;
            // Random number generator
            RNG                                      rng_;
            /// \brief Vector holding all of the start states supplied for the problem
            /// Each start motion is the root of its own tree of motions.
            std::vector<Motion*>                     startMotions_;
            /// Priority queue of motions
            ompl::BinaryHeap<Motion*, MotionCompare> priorityQueue_;
            /// Binary Space Partition
            Cell*                                    bsp_;
            /// Projection evaluator for the problem
            ompl::base::ProjectionEvaluatorPtr       projectionEvaluator_;
            /// Number between 0 and 1 specifying the probability with which the goal should be sampled
            double                                   goalBias_;
            /// Objected used to sample the goal
            ompl::base::GoalSampleableRegion        *goalSampler_;
            /// Iteration number and priority of the next Motion that will be generated
            unsigned int                             iteration_;
            /// Closest motion to the goal
            Motion                                  *lastGoalMotion_;
        };
    }
}

#endif
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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

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
           J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844730">10.1109/ROBOT.2000.844730</a><br>
           <a href="http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246">[PDF]</a>
           <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">[more]</a>
        */

        /** \brief Rapidly-exploring Random Trees */
        class RRT : public base::Planner
        {
        public:

            /** \brief Constructor */
            RRT(const base::SpaceInformationPtr &si);

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

        protected:


            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:

                Motion() : state(NULL), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

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
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;
        };

    }
}

#endif
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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_CONNECT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_CONNECT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gRRTC
           @par Short description
           The basic idea is to grow to RRTs, one from the start and
           one from the goal, and attempt to connect them.
           @par External documentation
           J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844730">10.1109/ROBOT.2000.844730</a><br>
           <a href="http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246">[PDF]</a>
           <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">[more]</a>
        */

        /** \brief RRT-Connect (RRTConnect) */
        class RRTConnect : public base::Planner
        {
        public:

            /** \brief Constructor */
            RRTConnect(const base::SpaceInformationPtr &si);

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

        protected:

            /** \brief Representation of a motion */
            class Motion
            {
            public:

                Motion() : root(NULL), state(NULL), parent(NULL)
                {
                    parent = NULL;
                    state  = NULL;
                }

                Motion(const base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                const base::State *root;
                base::State       *state;
                Motion            *parent;

            };

            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            typedef boost::shared_ptr< NearestNeighbors<Motion*> > TreeData;

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

            /** \brief Grow a tree towards a random state */
            GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

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
        };

    }
}

#endif

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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_LAZY_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_LAZY_RRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <vector>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gLazyRRT
           @par Short description
           RRT is a tree-based motion planner that uses the following
           idea: RRT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           The difference between \ref gRRT "RRT" and LazyRRT is that when moving
           towards the new state @b qm, LazyRRT does not check to make
           sure the path is valid. Instead, it is optimistic and
           attempts to find a path as soon as possible. Once a path is
           found, it is checked for collision. If collisions are
           found, the invalid path segments are removed and the search
           process is continued.
           @par External documentation
           - J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844730">10.1109/ROBOT.2000.844730</a><br>
           <a href="http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246">[PDF]</a>
           <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">[more]</a>
           - R. Bohlin and L.E. Kavraki, A Randomized Algorithm for Robot Path Planning Based on Lazy Evaluation, in <em>Handbook on Randomized Computing</em>, pp. 221–249, 2001.<br>
           <a href="http://www.kavrakilab.org/sites/default/files/bohlin2001lazy-evaluation.pdf">[PDF]</a>
           - R. Bohlin and L.E. Kavraki, Path planning using lazy PRM, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 521–528, 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844107">10.1109/ROBOT.2000.844107</a><br>
           <a href="http://ieeexplore.ieee.org/ielx5/6794/18235/00844107.pdf?tp=&arnumber=844107&isnumber=18235">[PDF]
        */

        /** \brief Lazy RRT */
        class LazyRRT : public base::Planner
        {
        public:

            /** \brief Constructor */
            LazyRRT(const base::SpaceInformationPtr &si);

            virtual ~LazyRRT();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            /** \brief Set the goal biasing.

                In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
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

        protected:

            /** \brief Representation of a motion */
            class Motion
            {
            public:

                Motion() : state(NULL), parent(NULL), valid(false)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL), valid(false)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State          *state;

                /** \brief The parent motion in the exploration tree */
                Motion               *parent;

                /** \brief Flag indicating whether this motion has been validated */
                bool                  valid;

                /** \brief The set of motions that descend from this one */
                std::vector<Motion*>  children;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Remove a motion from the tree datastructure */
            void removeMotion(Motion *motion);

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;

        };

    }
}

#endif
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

/* Author: Dave Coleman */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_TRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_TRRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/OptimizationObjective.h"

/*
  NOTES:
  **Variable Names that have been converted to longer versions from standards:
  nearest_neighbors_ -> nn_
  planner_termination_condition -> ptc

  **Inherited Member Variables Key:
  si_ -> SpaceInformation
  pdef_ -> ProblemDefinition
  pis_ -> PlannerInputStates - Utility class to extract valid input states
*/


namespace ompl
{

    namespace geometric
    {
        /**
           @anchor gTRRT
           @par Short description
           T-RRT is an RRT variant and tree-based motion planner that takes into consideration state costs
           to compute low-cost paths that follow valleys and saddle points of the configuration-space
           costmap. It uses transition tests from stoachastic optimization methods to accept or reject new
           potential sates.
           @par Example usage
           Please see <a href="https://github.com/davetcoleman/ompl_rviz_viewer/">Dave Coleman's example</a> to see how TRRT can be used.
           @par External documentation
           L. Jaillet, J. Cortés, T. Siméon, Sampling-Based Path Planning on Configuration-Space Costmaps, in <em>IEEE TRANSACTIONS ON ROBOTICS, VOL. 26, NO. 4, AUGUST 2010</em>. DOI: <a href="http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5477164&isnumber=5540533">IEEE</a><br />
           <a href="http://homepages.laas.fr/nic/Papers/10TRO.pdf">[PDF]</a>
        */

        /** \brief Transition-based Rapidly-exploring Random Trees */
        class TRRT : public base::Planner
        {
        public:

            /** \brief Constructor */
            TRRT(const base::SpaceInformationPtr &si);

            virtual ~TRRT();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &plannerTerminationCondition);

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

            /** \brief Set the maximum number of states that can be rejected before the temperature starts to rise */
            void setMaxStatesFailed( double maxStatesFailed )
            {
                maxStatesFailed_ = maxStatesFailed;
            }

            /** \brief Get the maximum number of states that can be rejected before the temperature starts to rise */
            double getMaxStatesFailed( void ) const
            {
                return maxStatesFailed_;
            }

            /** \brief Set the factor by which the temperature rises or falls based on current acceptance/rejection rate */
            void setTempChangeFactor( double tempChangeFactor )
            {
                tempChangeFactor_ = tempChangeFactor;
            }

            /** \brief Get the factor by which the temperature rises or falls based on current acceptance/rejection rate */
            double getTempChangeFactor( void ) const
            {
                return tempChangeFactor_;
            }

            /** \brief Set the minimum the temperature can drop to before being floored at that value */
            void setMinTemperature( double minTemperature )
            {
                minTemperature_ = minTemperature;
            }

            /** \brief Get the minimum the temperature can drop to before being floored at that value */
            double getMinTemperature( void ) const
            {
                return minTemperature_;
            }

            /** \brief Set the initial temperature at the beginning of the algorithm. Should be low */
            void setInitTemperature( double initTemperature )
            {
                initTemperature_ = initTemperature;
            }

            /** \brief Get the initial temperature at the beginning of the algorithm. Should be low */
            double getInitTemperature( void ) const
            {
                return initTemperature_;
            }

            /** \brief Set the distance between a new state and the nearest neighbor
                that qualifies that state as being a frontier */
            void setFrontierThreshold( double frontier_threshold )
            {
                frontierThreshold_ = frontier_threshold;
            }

            /** \brief Get the distance between a new state and the nearest neighbor
                that qualifies that state as being a frontier */
            double getFrontierThreshold( void ) const
            {
                return frontierThreshold_;
            }

            /** \brief Set the ratio between adding nonfrontier nodes to frontier nodes,
                for example .1 is 1/10 or one nonfrontier node for every 10 frontier nodes added */
            void setFrontierNodeRatio( double frontierNodeRatio )
            {
                frontierNodeRatio_ = frontierNodeRatio;
            }

            /** \brief Get the ratio between adding nonfrontier nodes to frontier nodes,
                for example .1 is 1/10 or one nonfrontier node for every 10 frontier nodes added */
            double getFrontierNodeRatio( void ) const
            {
                return frontierNodeRatio_;
            }

            /** \brief Set the constant value used to normalize the expression */
            void setKConstant( double kConstant )
            {
                kConstant_ = kConstant;
            }

            /** \brief Get the constant value used to normalize the expression */
            double getKConstant( void ) const
            {
                return kConstant_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors()
            {
                nearestNeighbors_.reset(new NN<Motion*>());
            }

            virtual void setup();

        protected:


            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:

                Motion() : state(NULL), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

                /** \brief Cost of the state */
                base::Cost            cost;

            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Filter irrelevant configuration regarding the search of low-cost paths before inserting into tree
                \param childCost - cost of current state
                \param parentCost - cost of its ancestor parent state
                \param distance - distance between parent and child
            */
            bool transitionTest( double childCost, double parentCost, double distance );

            /** \brief Use ratio to prefer frontier nodes to nonfrontier ones */
            bool minExpansionControl( double randMotionDistance );

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nearestNeighbors_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;

            /** \brief Output debug info */
            bool                                            verbose_;

            // *********************************************************************************************************
            // TRRT-Specific Variables
            // *********************************************************************************************************

            // Transtion Test -----------------------------------------------------------------------

            /** \brief Temperature parameter used to control the difficulty level of transition tests. Low temperatures
                limit the expansion to a slightly positive slopes, high temps enable to climb the steeper slopes.
                Dynamically tuned according to the information acquired during exploration */
            double                                          temp_;

            /** \brief Constant value used to normalize expression. Based on order of magnitude of the considered costs.
                Average cost of the query configurtaions since they are the only cost values known at the
                beginning of the search process. */
            double                                          kConstant_;

            /// Max number of rejections allowed
            unsigned int                                    maxStatesFailed_;

            /// Failure temperature factor used when max_num_failed_ failures occur
            double                                          tempChangeFactor_;

            /// Prevent temperature from dropping too far
            double                                          minTemperature_;

            /// A very low value at initialization to authorize very easy positive slopes
            double                                          initTemperature_;

            /// Failure counter for states that are rejected
            unsigned int                                    numStatesFailed_;


            // Minimum Expansion Control --------------------------------------------------------------

            /// Ratio counters for nodes that expand the search space versus those that do not
            double                                          nonfrontierCount_;
            double                                          frontierCount_;

            /// The distance between an old state and a new state that qualifies it as a frontier state
            double                                          frontierThreshold_;

            /// Target ratio of nonfrontier nodes to frontier nodes. rho
            double                                          frontierNodeRatio_;

            /// The optimization objective being optimized by TRRT
            ompl::base::OptimizationObjectivePtr            opt_;
        };
    }
}

#endif
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
*   * Neither the name of the Rice University nor the names of its
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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan */

#ifndef OMPL_CONTRIB_RRT_STAR_RRTSTAR_
#define OMPL_CONTRIB_RRT_STAR_RRTSTAR_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <limits>
#include <vector>
#include <utility>


namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gRRTstar
           @par Short description
           \ref gRRTstar "RRT*" (optimal RRT) is an asymptotically-optimal incremental
           sampling-based motion planning algorithm. \ref gRRTstar "RRT*" algorithm is
           guaranteed to converge to an optimal solution, while its
           running time is guaranteed to be a constant factor of the
           running time of the \ref gRRT "RRT". The notion of optimality is with
           respect to the distance function defined on the state space
           we are operating on. See ompl::base::Goal::setMaximumPathLength() for
           how to set the maximally allowed path length to reach the goal.
           If a solution path that is shorter than ompl::base::Goal::getMaximumPathLength() is
           found, the algorithm terminates before the elapsed time.
           @par External documentation
           S. Karaman and E. Frazzoli, Sampling-based
           Algorithms for Optimal Motion Planning, International Journal of Robotics
           Research (to appear), 2011.
           <a href="http://arxiv.org/abs/1105.1186">http://arxiv.org/abs/1105.1186</a>
        */

        /** \brief Optimal Rapidly-exploring Random Trees */
        class RRTstar : public base::Planner
        {
        public:

            RRTstar(const base::SpaceInformationPtr &si);

            virtual ~RRTstar();

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

            /** \brief Option that delays collision checking procedures.
                When it is enabled, all neighbors are sorted by cost. The
                planner then goes through this list, starting with the lowest
                cost, checking for collisions in order to find a parent. The planner
                stops iterating through the list when a collision free parent is found.
                This prevents the planner from collsion checking each neighbor, reducing
                computation time in scenarios where collision checking procedures are expensive.*/
            void setDelayCC(bool delayCC)
            {
                delayCC_ = delayCC;
            }

            /** \brief Get the state of the delayed collision checking option */
            bool getDelayCC() const
            {
                return delayCC_;
            }

            virtual void setup();

            ///////////////////////////////////////
            // Planner progress property functions
            std::string getIterationCount() const;

            std::string getCollisionCheckCount() const;

            std::string getBestCost() const;
            ///////////////////////////////////////

        protected:

            /** \brief Representation of a motion */
            class Motion
            {
            public:
                /** \brief Constructor that allocates memory for the state. This constructor automatically allocates memory for \e state, \e cost, and \e incCost */
                Motion(const base::SpaceInformationPtr &si) :
                    state(si->allocState()),
                    parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

                /** \brief The cost up to this motion */
                base::Cost        cost;

                /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance computations in the updateChildCosts() method) */
                base::Cost        incCost;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion*> children;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            // For sorting a list of costs and getting only their sorted indices
            struct CostIndexCompare
            {
                CostIndexCompare(const std::vector<base::Cost>& costs,
                                 const base::OptimizationObjective &opt) :
                    costs_(costs), opt_(opt)
                {}
                bool operator()(unsigned i, unsigned j)
                {
                    return opt_.isCostBetterThan(costs_[i],costs_[j]);
                }
                const std::vector<base::Cost>& costs_;
                const base::OptimizationObjective &opt_;
            };

            enum DistanceDirection { FROM_NEIGHBORS, TO_NEIGHBORS };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                switch (distanceDirection_)
                {
                case FROM_NEIGHBORS:
                    return si_->distance(a->state, b->state);
                case TO_NEIGHBORS:
                    return si_->distance(b->state, a->state);
                }
                return 0; // remove warning
            }

            /** \brief Removes the given motion from the parent's child list */
            void removeFromParent(Motion *m);

            /** \brief Updates the cost of the children of this node if the cost up to this node has changed */
            void updateChildCosts(Motion *m);

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief Option to delay and reduce collision checking within iterations */
            bool                                           delayCC_;

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;

            /** \brief A list of states in the tree that satisfy the goal condition */
            std::vector<Motion*>                           goalMotions_;

            //////////////////////////////
            // Planner progress properties

            /** \brief Number of iterations the algorithm performed */
            unsigned int                                   iterations_;

            /** \brief Number of collisions checks performed by the algorithm */
            unsigned int                                   collisionChecks_;

            /** \brief Best cost found so far by algorithm */
            base::Cost                                     bestCost_;

            /** \brief Directionality of distance computation for
                nearest neighbors. Either from neighbors to new state,
                or from new state to neighbors. */
            DistanceDirection                              distanceDirection_;
        };

    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Oren Salzman
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

/* Author: Oren Salzman, Sertac Karaman, Ioan Sucan */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_LBT_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_LBT_RRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <fstream>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gLBTRRT
           @par Short description
           \ref gLBTRRT "LBTRRT" (Lower Bound Tree RRT) is a near asymptotically-optimal
           incremental sampling-based motion planning algorithm. \ref gLBTRRT "LBTRRT"
           algorithm is guaranteed to converge to a solution that is within a constant
           factor of the optimal solution. The notion of optimality is with respect to
           the distance function defined on the state space we are operating on.

           @par External documentation
           O. Salzman and D. Halperin, Sampling-based
           Asymptotically near-optimal RRT for fast, high-quality, motion planning, 2013.
           [[PDF]](http://arxiv.org/abs/1308.0189)
        */

        /** \brief Lower Bound Tree Rapidly-exploring Random Trees */
        class LBTRRT : public base::Planner
        {
        public:

            /** \brief Constructor */
            LBTRRT (const base::SpaceInformationPtr &si);

            virtual ~LBTRRT ();

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

            /** \brief Set the apprimation factor */
            void setApproximationFactor (double epsilon)
            {
                epsilon_ = epsilon;
            }

            /** \brief Get the apprimation factor */
            double getApproximationFactor () const
            {
                return epsilon_;
            }
        protected:

            /** \brief kRRG = 2e~5.5 is a valid choice for all problem instances */
            static const double kRRG; // = 5.5

            /** \brief Representation of a motion

                a motion is a simultunaeous represntation of the two trees used by LBT-RRT
                a lower bound tree named Tlb and an approximaion tree named Tapx. */
            class Motion
            {
            public:

                Motion() : state(NULL), parentLb_(NULL), parentApx_(NULL), costLb_(0.0), costApx_(0.0)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parentLb_(NULL), parentApx_(NULL), costLb_(0.0), costApx_(0.0)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parentLb_;

                /** \brief The parent motion in the exploration tree */
                Motion            *parentApx_;

                double             costLb_, costApx_;

                std::vector<Motion*> childrenLb_;
                std::vector<Motion*> childrenApx_;
            };

            struct IsLessThan
            {
                IsLessThan (LBTRRT *plannerPtr, Motion * motion_): plannerPtr_(plannerPtr), motion(motion_)
                {
                }

                bool operator() (const Motion * motionA, const Motion * motionB)
                {
                    double sqDistA = plannerPtr_->distanceFunction(motionA, motion);
                    double distA = std::sqrt(sqDistA);

                    double sqDistB = plannerPtr_->distanceFunction(motionB, motion);
                    double distB = std::sqrt(sqDistB);

                    return (motionA->costLb_ + distA < motionB->costLb_ + distB);
                }
                LBTRRT *plannerPtr_;
                Motion *motion;
            }; //IsLessThan

            /** \brief attempt to rewire the trees */
            void attemptNodeUpdate(Motion *potentialParent, Motion *child);

            /** \brief update the child cost of the lower bound tree */
            void updateChildCostsLb(Motion *m, double delta);

            /** \brief update the child cost of the approximation tree */
            void updateChildCostsApx(Motion *m, double delta);

            /** \brief remove motion from its parent in the lower bound tree*/
            void removeFromParentLb(Motion *m);

            /** \brief remove motion from its parent in the approximation tree*/
            void removeFromParentApx(Motion *m);

            /** \brief remove motion from a vector*/
            void removeFromParent(const Motion *m, std::vector<Motion*>& vec);

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
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief approximation factor*/
            double                                          epsilon_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;
        };

    }
}

#endif //OMPL_GEOMETRIC_PLANNERS_RRT_LBT_RRT_
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

#ifndef OMPL_GEOMETRIC_PLANNERS_SBL_SBL_
#define OMPL_GEOMETRIC_PLANNERS_SBL_SBL_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/Grid.h"
#include "ompl/datastructures/PDF.h"
#include <vector>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gSBL
           @par Short description
           SBL is a tree-based motion planner that attempts to grow two
           trees at once: one grows from the starting state and the other
           from the goal state. The tree expansion strategy is the same as for \ref gEST "EST".
           Attempts are made to connect these trees
           at every step of the expansion. If they are connected, a
           solution path is obtained. However, this solution path is not
           certain to be valid (the lazy part of the algorithm) so it is
           checked for validity. If invalid parts are found, they are
           removed from the tree and exploration of the state space
           continues until a solution is found.
           To guide the exploration, an additional grid data
           structure is maintained. Grid cells contain states that
           have been previously visited. When deciding which state to
           use for further expansion, this grid is used;
           least-filled grid cells have most chances of being selected. The
           grid is usually imposed on a projection of the state
           space. This projection needs to be set before using the
           planner (setProjectionEvaluator() function). Connection of states in different trees is
           attempted if they fall in the same grid cell. If no projection is
           set, the planner will attempt to use the default projection
           associated to the state space. An exception is thrown if
           no default projection is available either.
           @par External documentation
           G. Sánchez and J.-C. Latombe, A single-query bi-directional probabilistic roadmap planner with lazy collision checking, in <em>The Tenth International Symposium on Robotics Research</em>, pp. 403–417, 2001.
           DOI: <a href="http://dx.doi.org/10.1007/3-540-36460-9_27">10.1007/3-540-36460-9_27</a><br>
           <a href="http://www.springerlink.com/content/9843341054386hh6/fulltext.pdf">[PDF]</a>
        */

        /** \brief Single-Query Bi-Directional Probabilistic Roadmap
           Planner with Lazy Collision Checking */
        class SBL : public base::Planner
        {
        public:

            /** \brief The constructor needs the instance of the space information */
            SBL(const base::SpaceInformationPtr &si);

            virtual ~SBL();

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state. */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
                the ones registered with the state space). */
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /** \brief Get the projection evaluator. */
            const base::ProjectionEvaluatorPtr& getProjectionEvaluator() const
            {
                return projectionEvaluator_;
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

            virtual void setup();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:

            struct MotionInfo;

            /** \brief A grid cell */
            typedef Grid<MotionInfo>::Cell GridCell;

            /** \brief A PDF of grid cells */
            typedef PDF<GridCell*>         CellPDF;

            /** \brief Representation of a motion */
            class Motion
            {
            public:

                /** \brief Default constructor. Allocates no memory */
                Motion() : root(NULL), state(NULL), parent(NULL), valid(false)
                {
                }

                /** \brief Constructor that allocates storage for a state */
                Motion(const base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL), valid(false)
                {
                }

                /** \brief The root of the tree this motion would get to, if we were to follow parent pointers */
                const base::State   *root;

                /** \brief The state this motion leads to */
                base::State         *state;

                /** \brief The parent motion -- it contains the state this motion originates at */
                Motion              *parent;

                /** \brief Flag indicating whether this motion has been checked for validity. */
                bool                 valid;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion*> children;
            };

            /** \brief A struct containing an array of motions and a corresponding PDF element */
            struct MotionInfo
            {
                Motion* operator[](unsigned int i)
                {
                    return motions_[i];
                }
                std::vector<Motion*>::iterator begin()
                {
                    return motions_.begin();
                }
                void erase(std::vector<Motion*>::iterator iter)
                {
                    motions_.erase(iter);
                }
                void push_back(Motion *m)
                {
                    motions_.push_back(m);
                }
                unsigned int size() const
                {
                    return motions_.size();
                }
                bool empty() const
                {
                    return motions_.empty();
                }

                std::vector<Motion*> motions_;
                CellPDF::Element    *elem_;
            };

            /** \brief Representation of a search tree. Two instances will be used. One for start and one for goal */
            struct TreeData
            {
                TreeData() : grid(0), size(0)
                {
                }

                /** \brief The grid of motions corresponding to this tree */
                Grid<MotionInfo> grid;

                /** \brief The number of motions (in total) from the tree */
                unsigned int     size;

                /** \brief The PDF used for selecting a cell from which to sample a motion */
                CellPDF          pdf;
            };

            /** \brief Free the memory allocated by the planner */
            void freeMemory()
            {
                freeGridMotions(tStart_.grid);
                freeGridMotions(tGoal_.grid);
            }

            /** \brief Free the memory used by the motions contained in a grid */
            void freeGridMotions(Grid<MotionInfo> &grid);

            /** \brief Add a motion to a tree */
            void addMotion(TreeData &tree, Motion *motion);

            /** \brief Select a motion from a tree */
            Motion* selectMotion(TreeData &tree);

            /** \brief Remove a motion from a tree */
            void removeMotion(TreeData &tree, Motion *motion);

            /** \brief Since solutions are computed in a lazy fashion,
                once trees are connected, the solution found needs to
                be checked for validity. This function checks whether
                the reverse path from a given motion to a root is
                valid. If this is not the case, invalid motions are removed  */
            bool isPathValid(TreeData &tree, Motion *motion);

            /** \brief Check if a solution can be obtained by connecting two trees using a specified motion */
            bool checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion *motion, std::vector<Motion*> &solution);

            /** \brief The employed state sampler */
            base::ValidStateSamplerPtr                 sampler_;

            /** \brief The employed projection evaluator */
            base::ProjectionEvaluatorPtr               projectionEvaluator_;

            /** \brief The start tree */
            TreeData                                   tStart_;

            /** \brief The goal tree */
            TreeData                                   tGoal_;

            /** \brief The maximum length of a motion to be added in the tree */
            double                                     maxDistance_;

            /** \brief The random number generator to be used */
            RNG                                        rng_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State*, base::State*>      connectionPoint_;
        };

    }
}

#endif
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rice University
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

/* Author: Bryant Gipson, Mark Moll */

#ifndef OMPL_GEOMETRIC_PLANNERS_STRIDE_STRIDE_
#define OMPL_GEOMETRIC_PLANNERS_STRIDE_STRIDE_

#include "ompl/datastructures/Grid.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/PDF.h"
#include <boost/unordered_map.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>

namespace ompl
{

    template<typename _T>
    class NearestNeighborsGNAT;

    namespace geometric
    {

        /**
          @anchor gSTRIDE

          @par Short description
          STRIDE (Search Tree with Resolution Independent Density Estimation)
          is a tree-based motion planner that attempts to detect
          the less explored area of the space through the use of a
          GNAT nearest-neighbor data structure. It is similar to EST,
          but unlike the EST implementation in OMPL does not require
          a projection. However, in case the state space has many
          dimensions, a projection can be specified and the GNAT
          can be built using distances in the projected space. This
          has the advantage over the EST implementation that no grid
          cell sizes have to be specified.

          @par External documentation
          B. Gipson, M. Moll, and L.E. Kavraki, Resolution independent density
          estimation for motion planning in high-dimensional spaces, in
          <em>IEEE Intl. Conf. on Robotics and Automation</em>, pp. 2429-2435, 2013.
          <a href="http://dx.doi.org/10.1109/ICRA.2013.6630908">[PDF]</a>
        */

        /** \brief Search Tree with Resolution Independent Density Estimation */
        class STRIDE : public base::Planner
        {
            public:

                /** \brief Constructor */
                STRIDE(const base::SpaceInformationPtr &si, bool useProjectedDistance = false,
                        unsigned int degree = 16, unsigned int minDegree = 12, unsigned int maxDegree = 18,
                        unsigned int maxNumPtsPerLeaf = 6, double estimatedDimension = 0.0);
                virtual ~STRIDE();

                virtual void setup();

                virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

                virtual void clear();

                /** \brief In the process of randomly selecting states in
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

                /** \brief Set whether nearest neighbors are computed based
                  on distances in a _projection_ of the state rather distances
                  in the state space itself. */
                void setUseProjectedDistance(bool useProjectedDistance)
                {
                    useProjectedDistance_ = useProjectedDistance;
                }
                /** \brief Return whether nearest neighbors are computed based
                  on distances in a _projection_ of the state rather distances
                  in the state space itself. */
                bool getUseProjectedDistance() const
                {
                    return useProjectedDistance_;
                }

                /** \brief Set desired degree of a node in the GNAT. */
                void setDegree(unsigned int degree)
                {
                    degree_ = degree;
                }
                /** \brief Get desired degree of a node in the GNAT. */
                unsigned int getDegree() const
                {
                    return degree_;
                }
                /** \brief Set minimum degree of a node in the GNAT. */
                void setMinDegree(unsigned int minDegree)
                {
                    minDegree_ = minDegree;
                }
                /** \brief Get minimum degree of a node in the GNAT. */
                unsigned int getMinDegree() const
                {
                    return minDegree_;
                }
                /** \brief Set maximum degree of a node in the GNAT. */
                void setMaxDegree(unsigned int maxDegree)
                {
                    maxDegree_ = maxDegree;
                }
                /** \brief Set maximum degree of a node in the GNAT. */
                unsigned int getMaxDegree() const
                {
                    return maxDegree_;
                }
                /** \brief Set maximum number of elements stored in a leaf
                  node of the GNAT. */
                void setMaxNumPtsPerLeaf(unsigned int maxNumPtsPerLeaf)
                {
                    maxNumPtsPerLeaf_ = maxNumPtsPerLeaf;
                }
                /** \brief Get maximum number of elements stored in a leaf
                  node of the GNAT. */
                unsigned int getMaxNumPtsPerLeaf() const
                {
                    return maxNumPtsPerLeaf_;
                }
                /** \brief Set estimated dimension of the free space, which
                  is needed to compute the sampling weight for a node in the
                  GNAT. */
                void setEstimatedDimension(double estimatedDimension)
                {
                    estimatedDimension_ = estimatedDimension;
                }
                /** \brief Get estimated dimension of the free space, which
                  is needed to compute the sampling weight for a node in the
                  GNAT. */
                double getEstimatedDimension() const
                {
                    return estimatedDimension_;
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
                /** \brief When extending a motion, the planner can decide
                  to keep the first valid part of it, even if invalid
                  states are found, as long as the valid part represents
                  a sufficiently large fraction from the original
                  motion. This function sets the minimum acceptable
                  fraction (between 0 and 1). */
                void setMinValidPathFraction(double fraction)
                {
                    minValidPathFraction_ = fraction;
                }

                /** \brief Get the value of the fraction set by setMinValidPathFraction() */
                double getMinValidPathFraction() const
                {
                    return minValidPathFraction_;
                }
                /** \brief Set the projection evaluator. This class is
                  able to compute the projection of a given state.  */
                void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
                {
                    projectionEvaluator_ = projectionEvaluator;
                }

                /** \brief Set the projection evaluator (select one from
                  the ones registered with the state space). */
                void setProjectionEvaluator(const std::string &name)
                {
                    projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
                }

                /** \brief Get the projection evaluator */
                const base::ProjectionEvaluatorPtr& getProjectionEvaluator() const
                {
                    return projectionEvaluator_;
                }

                virtual void getPlannerData(base::PlannerData &data) const;

            protected:

                /** \brief The definition of a motion */
                class Motion
                {
                public:
                    Motion() : state(NULL), parent(NULL)
                    {
                    }

                    /** \brief Constructor that allocates memory for the state */
                    Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL)
                    {
                    }

                    ~Motion()
                    {
                    }

                    /** \brief The state contained by the motion */
                    base::State       *state;

                    /** \brief The parent motion in the exploration tree */
                    Motion            *parent;
                };


                /** \brief Free the memory allocated by this planner */
                void freeMemory();

                /** \brief Initialize GNAT data structure */
                void setupTree();

                /** \brief Compute distance between motions (actually distance between contained states) */
                double distanceFunction(const Motion *a, const Motion *b) const
                {
                    return si_->distance(a->state, b->state);
                }

                /** \brief Compute distance between motions (actually distance between projections of contained states) */
                double projectedDistanceFunction(const Motion *a, const Motion *b) const
                {
                    unsigned int num_dims = projectionEvaluator_->getDimension();
                    ompl::base::EuclideanProjection aproj(num_dims), bproj(num_dims);
                    projectionEvaluator_->project(a->state, aproj);
                    projectionEvaluator_->project(b->state, bproj);
                    return boost::numeric::ublas::norm_2(aproj - bproj);
                }

                /** \brief Add a motion to the exploration tree */
                void addMotion(Motion *motion);

                /** \brief Select a motion to continue the expansion of the tree from */
                Motion* selectMotion();

                /** \brief Valid state sampler */
                base::ValidStateSamplerPtr   sampler_;

                /** \brief This algorithm can optionally use a projection to guide the exploration. */
                base::ProjectionEvaluatorPtr projectionEvaluator_;

                /** \brief The exploration tree constructed by this algorithm */
                boost::scoped_ptr<NearestNeighborsGNAT<Motion*> >
                                             tree_;

                /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
                double                       goalBias_;

                /** \brief The maximum length of a motion to be added to a tree */
                double                       maxDistance_;

                /** \brief Whether to use distance in the projection (instead of distance in the state space) for the GNAT */
                bool                         useProjectedDistance_;
                /** \brief Desired degree of an internal node in the GNAT */
                unsigned int                 degree_;
                /** \brief Minimum degree of an internal node in the GNAT */
                unsigned int                 minDegree_;
                /** \brief Maximum degree of an internal node in the GNAT */
                unsigned int                 maxDegree_;
                /** \brief Maximum number of points stored in a leaf node in the GNAT */
                unsigned int                 maxNumPtsPerLeaf_;
                /** \brief Estimate of the local dimensionality of the free space around a state */
                double                       estimatedDimension_;
                /** \brief When extending a motion, the planner can decide
                     to keep the first valid part of it, even if invalid
                     states are found, as long as the valid part represents
                     a sufficiently large fraction from the original
                     motion. This is used only when extendWhileValid_ is true. */
                double                       minValidPathFraction_;

                /** \brief The random number generator */
                RNG                          rng_;
        };

    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Autonomous Systems Laboratory, Stanford University
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
*   * Neither the name of Stanford University nor the names of its
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

/* Authors: Ashley Clark (Stanford) and Wolfgang Pointner (AIT) */
/* Co-developers: Brice Rebsamen (Stanford) and Tim Wheeler (Stanford) */
/* Algorithm design: Lucas Janson (Stanford) and Marco Pavone (Stanford) */
/* Acknowledgements for insightful comments: Edward Schmerling (Stanford),
 * Oren Salzman (Tel Aviv University), Joseph Starek (Stanford), and Evan Clark (Stanford) */

#ifndef OMPL_GEOMETRIC_PLANNERS_FMT_
#define OMPL_GEOMETRIC_PLANNERS_FMT_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/BinaryHeap.h>
#include <ompl/base/OptimizationObjective.h>
#include <map>


namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gFMT
           @par Short description
           \ref gFMT "FMT*" is an asymptotically-optimal sampling-based motion
            planning algorithm, which is guaranteed to converge to a shortest
            path solution. The algorithm is specifically aimed at solving complex
            motion planning problems in high-dimensional configuration spaces.
            The \ref gFMT "FMT*" algorithm essentially performs a lazy dynamic
            programming recursion on a set of probabilistically-drawn samples to
            grow a tree of paths, which moves steadily outward in cost-to-come space.
           @par External documentation
           L. Janson, A. Clark, and M. Pavone, Fast Marching Trees: a Fast Marching
           Sampling-Based Method for Optimal Motion Planning in Many Dimensions
           , International Journal on Robotics Research, 2013.
           <a href="http://arxiv.org/pdf/1306.3532v3.pdf">http://arxiv.org/pdf/1306.3532v3.pdf</a>
        */
        /** @brief Asymptotically Optimal Fast Marching Tree algorithm developed
            by L. Janson and M. Pavone. */
        class FMT : public ompl::base::Planner
        {
        public:

            FMT(const base::SpaceInformationPtr &si);

            virtual ~FMT();

            virtual void setup();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief Set the number of states that the planner should sample.
                The planner will sample this number of states in addition to the
                initial states. If any of the goal states are not reachable from
                the randomly sampled states, those goal states will also be
                added. The default value is 1000 */
            void setNumSamples(const unsigned int numSamples)
            {
                numSamples_ = numSamples;
            }

            /** \brief Get the number of states that the planner will sample */
            unsigned int getNumSamples() const
            {
                return numSamples_;
            }

            /** \brief The planner searches for neighbors of a node within a
                cost r, where r is the value described for FMT* in Section 4
                of [L. Janson, A. Clark, and M. Pavone, "Fast Marching Trees: a Fast
                Marching Sampling-Based Method for Optimal Motion Planning in
                Many Dimensions," International Symposium on
                Robotics Research, 2013. <a href="http://arxiv.org/pdf/1306.3532v3.pdf">
                http://arxiv.org/pdf/1306.3532v3.pdf</a>] For guaranteed asymptotic
                convergence, the user should choose a constant multiplier for
                the search radius that is greater than one. The default value is 1.1.
                In general, a radius multiplier between 0.9 and 5 appears to
                perform the best */
            void setRadiusMultiplier(const double radiusMultiplier)
            {
                if (radiusMultiplier <= 0.0)
                    throw Exception("Radius multiplier must be greater than zero");
                radiusMultiplier_ = radiusMultiplier;
            }

            /** \brief Get the multiplier used for the nearest neighbors search
                radius */
            double getRadiusMultiplier() const
            {
                return radiusMultiplier_;
            }

            /** \brief Store the volume of the obstacle-free configuration space.
                If no value is specified, the default assumes an obstacle-free
                unit hypercube, freeSpaceVolume = (maximumExtent/sqrt(dimension))^(dimension) */
            void setFreeSpaceVolume(const double freeSpaceVolume)
            {
                if (freeSpaceVolume < 0.0)
                    throw Exception("Free space volume should be greater than zero");
                freeSpaceVolume_ = freeSpaceVolume;
            }

            /** \brief Get the volume of the free configuration space that is
                being used by the planner */
            double getFreeSpaceVolume() const
            {
                return freeSpaceVolume_;
            }

        protected:
            /** \brief Representation of a motion
              */
            class Motion
            {
                public:

                    /** \brief The FMT* planner begins with all nodes included in
                        set W "Waiting for optimal connection". As nodes are
                        connected to the tree, they are transferred into set H
                        "Horizon of explored tree." Once a node in H is no longer
                        close enough to the frontier to connect to any more nodes in
                        W, it is removed from H. These three SetTypes are flags
                        indicating which set the node belongs to; H, W, or neither */
                    enum SetType { SET_NULL, SET_H, SET_W };

                    Motion()
                        : state_(NULL), parent_(NULL), cost_(0.0), currentSet_(SET_NULL)
                    {
                    }

                    /** \brief Constructor that allocates memory for the state */
                    Motion(const base::SpaceInformationPtr &si)
                        : state_(si->allocState()), parent_(NULL), cost_(0.0), currentSet_(SET_NULL)
                    {
                    }

                    ~Motion()
                    {
                    }

                    /** \brief Set the state associated with the motion */
                    void setState(base::State *state)
                    {
                        state_ = state;
                    }

                    /** \brief Get the state associated with the motion */
                    base::State* getState() const
                    {
                        return state_;
                    }

                    /** \brief Set the parent motion of the current motion */
                    void setParent(Motion *parent)
                    {
                        parent_ = parent;
                    }

                    /** \brief Get the parent motion of the current motion */
                    Motion* getParent() const
                    {
                        return parent_;
                    }

                    /** \brief Set the cost-to-come for the current motion */
                    void setCost(const base::Cost cost)
                    {
                        cost_ = cost;
                    }

                    /** \brief Get the cost-to-come for the current motion */
                    base::Cost getCost() const
                    {
                        return cost_;
                    }

                    /** \brief Specify the set that this motion belongs to */
                    void setSetType(const SetType currentSet)
                    {
                        currentSet_ = currentSet;
                    }

                    /** \brief Get the set that this motion belongs to */
                    SetType getSetType() const
                    {
                        return currentSet_;
                    }

                protected:

                    /** \brief The state contained by the motion */
                    base::State *state_;

                    /** \brief The parent motion in the exploration tree */
                    Motion *parent_;

                    /** \brief The cost of this motion */
                    base::Cost cost_;

                    /** \brief The flag indicating which set a motion belongs to */
                    SetType currentSet_;
            };

            /** \brief Comparator used to order motions in a binary heap */
            struct MotionCompare
            {
                MotionCompare() : opt_(NULL)
                {
                }

                /* Returns true if m1 is lower cost than m2. m1 and m2 must
                   have been instantiated with the same optimization objective */
                bool operator()(const Motion *m1, const Motion *m2) const
                {
                    return opt_->isCostBetterThan(m1->getCost(), m2->getCost());
                }

                base::OptimizationObjective* opt_;
            };

            /** \brief Compute the distance between two motions as the cost
                between their contained states. Note that for computationally
                intensive cost functions, the cost between motions should be
                stored to avoid duplicate calculations */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return opt_->motionCost(a->getState(), b->getState()).v;
            }

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Sample a state from the free configuration space and save
                it into the nearest neighbors data structure */
            void sampleFree(const ompl::base::PlannerTerminationCondition &ptc);

            /** \brief For each goal region, check to see if any of the sampled
                states fall within that region. If not, add a goal state from
                that region directly into the set of vertices. In this way, FMT
                is able to find a solution, if one exists. If no sampled nodes
                are within a goal region, there would be no way for the
                algorithm to successfully find a path to that region */
            void assureGoalIsSampled(const ompl::base::GoalSampleableRegion *goal);

            /** \brief Compute the volume of the unit ball in a given dimension */
            double calculateUnitBallVolume(const unsigned int dimension) const;

            /** \brief Calculate the radius to use for nearest neighbor searches,
                using the bound given in [L. Janson, A. Clark, and M.
                Pavone, "Fast Marching Trees: a Fast Marching Sampling-Based
                Method for Optimal Motion Planning in Many Dimensions,"
                International Journal on Robotics Research,
                2013. <a href="http://arxiv.org/pdf/1306.3532v3.pdf">
                http://arxiv.org/pdf/1306.3532v3.pdf</a>]. The radius depends on
                the radiusMultiplier parameter, the volume of the free
                configuration space, the volume of the unit ball in the current
                dimension, and the number of nodes in the graph */
            double calculateRadius(unsigned int dimension, unsigned int n) const;

            /** \brief Save the neighbors within a given radius of a state */
            void saveNeighborhood(Motion *m, const double r);

            /** \brief Trace the path from a goal state back to the start state
                and save the result as a solution in the Problem Definiton.
             */
            void traceSolutionPathThroughTree(Motion *goalMotion);

            /** \brief Complete one iteration of the main loop of the FMT* algorithm:
		Find all nodes in set W within a radius r of the node z.
                Attempt to connect them to their optimal cost-to-come parent
                in set H. Remove all newly connected nodes from W and insert
                them into H. Remove motion z from H, and update z to be the
                current lowest cost-to-come node in H */
            bool expandTreeFromNode(Motion *&z, const double r);

            /** \brief A binary heap for storing explored motions in
                cost-to-come sorted order */
            typedef ompl::BinaryHeap<Motion*, MotionCompare> MotionBinHeap;

            /** \brief A binary heap for storing explored motions in
                cost-to-come sorted order. The motions in H have been explored,
                yet are still close enough to the frontier of the explored set H
                to be connected to nodes in the unexplored set W */
            MotionBinHeap H_;

            /** \brief A map of all of the elements stored within the
                MotionBinHeap H, used to convert between Motion *and Element* */
            std::map<Motion*, MotionBinHeap::Element*> hElements_;

            /** \brief A map linking a motion to all of the motions within a
                distance r of that motion */
            std::map<Motion*, std::vector<Motion*> > neighborhoods_;

            /** \brief The number of samples to use when planning */
            unsigned int numSamples_;

            /** \brief The volume of the free configuration space */
            double freeSpaceVolume_;

            /** \brief This planner uses a nearest neighbor search radius
                proportional to the lower bound for optimality derived for FMT*
                in Section 4 of [L. Janson, A. Clark, and M. Pavone, "Fast
                Marching Trees: a Fast Marching Sampling-Based Method for
                Optimal Motion Planning in Many Dimensions," International
                Journal on Robotics Research, 2013.
                <a href="http://arxiv.org/pdf/1306.3532v3.pdf">
                http://arxiv.org/pdf/1306.3532v3.pdf</a>].  The radius multiplier
                is the multiplier for the lower bound. For guaranteed asymptotic
                convergence, the user should choose a multiplier for the search
                radius that is greater than one. The default value is 1.1.
                In general, a radius between 0.9 and 5 appears to perform the best
             */
            double radiusMultiplier_;

            /** \brief A nearest-neighbor datastructure containing the set of all motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief The cost objective function */
            base::OptimizationObjectivePtr opt_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_;

        };
    }
}


#endif // OMPL_GEOMETRIC_PLANNERS_FMT_
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

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_

#include <vector>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <ompl/base/StateSpace.h>

namespace ompl
{

    /** \brief Abstract representation of a container that can perform nearest neighbors queries */
    template<typename _T>
    class NearestNeighbors
    {
    public:

        /** \brief The definition of a distance function */
        typedef boost::function<double(const _T&, const _T&)> DistanceFunction;

        NearestNeighbors()
        {
        }

        virtual ~NearestNeighbors()
        {
        }

        /** \brief Set the distance function to use */
        virtual void setDistanceFunction(const DistanceFunction &distFun)
        {
            distFun_ = distFun;
        }

        /** \brief Get the distance function used */
        const DistanceFunction& getDistanceFunction() const
        {
            return distFun_;
        }

        /** \brief Clear the datastructure */
        virtual void clear() = 0;

        /** \brief Add an element to the datastructure */
        virtual void add(const _T &data) = 0;

        /** \brief Add a vector of points */
        virtual void add(const std::vector<_T> &data)
        {
            for (typename std::vector<_T>::const_iterator elt = data.begin() ; elt != data.end() ; ++elt)
                add(*elt);
        }

        /** \brief Remove an element from the datastructure */
        virtual bool remove(const _T &data) = 0;

        /** \brief Get the nearest neighbor of a point */
        virtual _T nearest(const _T &data) const = 0;

        /** \brief Get the k-nearest neighbors of a point
         *
         * All the nearest neighbor structures currently return the neighbors in
         * sorted order, but this is not required.
         */
        virtual void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const = 0;

        /** \brief Get the nearest neighbors of a point, within a specified radius
         *
         * All the nearest neighbor structures currently return the neighbors in
         * sorted order, but this is not required.
         */
        virtual void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const = 0;

        /** \brief Get the number of elements in the datastructure */
        virtual std::size_t size() const = 0;

        /** \brief Get all the elements in the datastructure */
        virtual void list(std::vector<_T> &data) const = 0;

    protected:

        /** \brief The used distance function */
        DistanceFunction distFun_;

    };
}

#endif
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

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_LINEAR_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_LINEAR_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/util/Exception.h"
#include <algorithm>

namespace ompl
{

    /** \brief A nearest neighbors datastructure that uses linear
        search.

        \li Search for nearest neighbor is O(n).
        \li Search for k-nearest neighbors is  O(n log(k)).
        \li Search for neighbors within a range is O(n log(n)).
        \li Adding an element to the datastructure is O(1).
        \li Removing an element from the datastructure O(n).
    */
    template<typename _T>
    class NearestNeighborsLinear : public NearestNeighbors<_T>
    {
    public:
        NearestNeighborsLinear() : NearestNeighbors<_T>()
        {
        }

        virtual ~NearestNeighborsLinear()
        {
        }

        virtual void clear()
        {
            data_.clear();
        }

        virtual void add(const _T &data)
        {
            data_.push_back(data);
        }

        virtual void add(const std::vector<_T> &data)
        {
            data_.reserve(data_.size() + data.size());
            data_.insert(data_.end(), data.begin(), data.end());
        }

        virtual bool remove(const _T &data)
        {
            if (!data_.empty())
                for (int i = data_.size() - 1 ; i >= 0 ; --i)
                    if (data_[i] == data)
                    {
                        data_.erase(data_.begin() + i);
                        return true;
                    }
            return false;
        }

        virtual _T nearest(const _T &data) const
        {
            const std::size_t sz = data_.size();
            std::size_t pos = sz;
            double dmin = 0.0;
            for (std::size_t i = 0 ; i < sz ; ++i)
            {
                double distance = NearestNeighbors<_T>::distFun_(data_[i], data);
                if (pos == sz || dmin > distance)
                {
                    pos = i;
                    dmin = distance;
                }
            }
            if (pos != sz)
                return data_[pos];

            throw Exception("No elements found in nearest neighbors data structure");
        }

        /// Return the k nearest neighbors in sorted order
        virtual void nearestK(const _T &data, std::size_t k, std::vector<_T> &nbh) const
        {
            nbh = data_;
            if (nbh.size() > k)
            {
                std::partial_sort(nbh.begin(), nbh.begin() + k, nbh.end(),
                                  ElemSort(data, NearestNeighbors<_T>::distFun_));
                nbh.resize(k);
            }
            else
            {
                std::sort(nbh.begin(), nbh.end(), ElemSort(data, NearestNeighbors<_T>::distFun_));
            }
        }

        /// Return the nearest neighbors within distance \c radius in sorted order
        virtual void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const
        {
            nbh.clear();
            for (std::size_t i = 0 ; i < data_.size() ; ++i)
                if (NearestNeighbors<_T>::distFun_(data_[i], data) <= radius)
                    nbh.push_back(data_[i]);
            std::sort(nbh.begin(), nbh.end(), ElemSort(data, NearestNeighbors<_T>::distFun_));
        }

        virtual std::size_t size() const
        {
            return data_.size();
        }

        virtual void list(std::vector<_T> &data) const
        {
            data = data_;
        }

    protected:

        /** \brief The data elements stored in this structure */
        std::vector<_T>   data_;

    private:

        struct ElemSort
        {
            ElemSort(const _T &e, const typename NearestNeighbors<_T>::DistanceFunction &df) : e_(e), df_(df)
            {
            }

            bool operator()(const _T &a, const _T &b) const
            {
                return df_(a, e_) < df_(b, e_);
            }

            const _T                                              &e_;
            const typename NearestNeighbors<_T>::DistanceFunction &df_;
        };

    };


}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Mark Moll */

#ifndef PY_BINDINGS_OMPL_PY_GEOMETRIC_
#define PY_BINDINGS_OMPL_PY_GEOMETRIC_

#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "py_boost_function.hpp"


namespace ompl
{
    namespace geometric
    {
        inline int dummyFn() { return 1; }
        inline int dummyConnectionStrategy()
        {
            NearestNeighborsLinear<PRM::Vertex> nn;
            boost::shared_ptr<NearestNeighbors<PRM::Vertex> > nnPtr(&nn);
            return sizeof(KStrategy<PRM::Vertex>(1, nnPtr)) + sizeof(KStarStrategy<PRM::Vertex>(dummyFn, nnPtr, 1)) + sizeof(nn);
        }
    }
}

#endif
