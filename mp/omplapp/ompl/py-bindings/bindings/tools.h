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

#ifndef OMPL_TOOLS_BENCHMARK_BENCHMARK_
#define OMPL_TOOLS_BENCHMARK_BENCHMARK_

#include "ompl/geometric/SimpleSetup.h"
#include "ompl/control/SimpleSetup.h"

namespace ompl
{
    namespace tools
    {
        /** \brief Benchmark a set of planners on a problem instance */
        class Benchmark
        {
        public:

            /** \brief This structure contains information about the
                activity of a benchmark instance.  If the instance is
                running, it is possible to find out information such
                as which planner is currently being tested or how much */
            struct Status
            {
                Status()
                {
                    running = false;
                    activeRun = 0;
                    progressPercentage = 0.0;
                }

                /// Flag indicating whether benchmarking is running
                bool         running;

                /// The name of the planner currently being tested
                std::string  activePlanner;

                /// The number of the run currently being executed
                unsigned int activeRun;

                /// Total progress (0 to 100)
                double       progressPercentage;
            };

            /** \brief The data collected from a run of a planner is
                stored as key-value pairs. */
            typedef std::map<std::string, std::string> RunProperties;

            typedef std::vector<std::map<std::string, std::string> > RunProgressData;

            /** \brief Signature of function that can be called before a planner execution is started */
            typedef boost::function<void(const base::PlannerPtr&)> PreSetupEvent;

            /** \brief Signature of function that can be called after a planner execution is completed */
            typedef boost::function<void(const base::PlannerPtr&, RunProperties&)> PostSetupEvent;

            /** \brief The data collected after running a planner multiple times */
            struct PlannerExperiment
            {
                /// The name of the planner
                std::string                name;

                /// Data collected for each run
                std::vector<RunProperties> runs;

                /// Names of each of the planner progress properties
                /// reported by planner
                std::vector<std::string>     progressPropertyNames;

                /// For each run of the planner, this stores the set
                /// of planner progress data reported by the planner
                std::vector<RunProgressData> runsProgressData;

                /// Some common properties for all the runs
                RunProperties              common;

                bool operator==(const PlannerExperiment& p) const
                {
                    return name==p.name && runs==p.runs && common==p.common;
                }
            };

            /** \brief This structure holds experimental data for a set of planners */
            struct CompleteExperiment
            {
                /** \brief The name of the experiment */
                std::string                    name;

                /// The collected experimental data; each element of the array (an experiment) corresponds to a planner
                std::vector<PlannerExperiment> planners;

                /// The maximum allowed time for planner computation during the experiment (seconds)
                double                         maxTime;

                /// The maximum allowed memory for planner computation during the experiment (MB)
                double                         maxMem;

                /// The number of runs to execute for each planner
                unsigned int                   runCount;

                /// The point in time when the experiment was started
                time::point                    startTime;

                /// The amount of time spent to collect the information in this structure (seconds)
                double                         totalDuration;

                /// The output of SimpleSetup::print() before the experiment was started
                std::string                    setupInfo;

                /// The random seed that was used at the start of the benchmark program
                boost::uint32_t                seed;

                /// Hostname that identifies the machine the benchmark ran on
                std::string                    host;

                /// Information about the CPU of the machine the benchmark ran on
                std::string                    cpuInfo;
            };

            /** \brief Representation of a benchmark request */
            struct Request
            {
                /** \brief Constructor that provides default values for all members */
                Request(double maxTime = 5.0, double maxMem = 4096.0,
                        unsigned int runCount = 100,
                        double timeBetweenUpdates = 0.001,
                        bool displayProgress = true,
                        bool saveConsoleOutput = true, bool useThreads = true)
                    : maxTime(maxTime), maxMem(maxMem), runCount(runCount),
                    displayProgress(displayProgress), saveConsoleOutput(saveConsoleOutput),
                    useThreads(useThreads)
                {
                }

                /// \brief the maximum amount of time a planner is allowed to run (seconds); 5.0 by default
                double       maxTime;

                /// \brief the maximum amount of memory a planner is allowed to use (MB); 4096.0 by default
                double       maxMem;

                /// \brief the number of times to run each planner; 100 by default
                unsigned int runCount;

                /// \brief When collecting time-varying data from a planner during its execution, the planner's progress will be queried every \c timeBetweenUpdates seconds.
                double       timeBetweenUpdates;

                /// \brief flag indicating whether progress is to be displayed or not; true by default
                bool         displayProgress;

                /// \brief flag indicating whether console output is saved (in an automatically generated filename); true by default
                bool         saveConsoleOutput;

                /// \brief flag indicating whether planner runs should be run in a separate thread. It is advisable to set this to \c true, so that a crashing planner doesn't result in a crash of the benchmark program. However, in the Python bindings this is set to \c false to avoid multi-threading problems in Python.
                bool         useThreads;
            };

            /** \brief Constructor needs the SimpleSetup instance needed for planning. Optionally, the experiment name (\e name) can be specified */
            Benchmark(geometric::SimpleSetup &setup, const std::string &name = std::string()) : gsetup_(&setup), csetup_(NULL)
            {
                exp_.name = name;
            }

            /** \brief Constructor needs the SimpleSetup instance needed for planning. Optionally, the experiment name (\e name) can be specified */
            Benchmark(control::SimpleSetup &setup, const std::string &name = std::string()) : gsetup_(NULL), csetup_(&setup)
            {
                exp_.name = name;
            }

            virtual ~Benchmark()
            {
            }

            /** \brief Set the name of the experiment */
            void setExperimentName(const std::string &name)
            {
                exp_.name = name;
            }

            /** \brief Get the name of the experiment */
            const std::string& getExperimentName() const
            {
                return exp_.name;
            }

            /** \brief Add a planner to use. */
            void addPlanner(const base::PlannerPtr &planner)
            {
                if (planner && planner->getSpaceInformation().get() !=
                    (gsetup_ ? gsetup_->getSpaceInformation().get() : csetup_->getSpaceInformation().get()))
                    throw Exception("Planner instance does not match space information");
                planners_.push_back(planner);
            }

            /** \brief Add a planner allocator to use. */
            void addPlannerAllocator(const base::PlannerAllocator &pa)
            {
                planners_.push_back(pa(gsetup_ ? gsetup_->getSpaceInformation() : csetup_->getSpaceInformation()));
            }

            /** \brief Clear the set of planners to be benchmarked */
            void clearPlanners()
            {
                planners_.clear();
            }

            /// Set the event to be called before any runs of a particular planner (when the planner is switched)
            void setPlannerSwitchEvent(const PreSetupEvent &event)
            {
                plannerSwitch_ = event;
            }

            /// Set the event to be called before the run of a planner
            void setPreRunEvent(const PreSetupEvent &event)
            {
                preRun_ = event;
            }

            /// Set the event to be called after the run of a planner
            void setPostRunEvent(const PostSetupEvent &event)
            {
                postRun_ = event;
            }

            /** \brief Benchmark the added planners on the defined problem. Repeated calls clear previously gathered data.
                \param req The parameters for the execution of the benchmark
                \note The values returned for memory consumption may
                be misleading. Memory allocators often free memory in
                a lazy fashion, so the returned values for memory
                consumption indicate the increase in memory usage for
                each run. Since not all the memory for the previous
                run was freed, the increase in usage may be close to
                0. To get correct averages for memory usage, use \e
                req.runCount = 1 and run the process multiple times.
            */
            virtual void benchmark(const Request &req);

            /** \brief Get the status of the benchmarking code. This function can be called in a separate thread to check how much progress has been made */
            const Status& getStatus() const
            {
                return status_;
            }

            /** \brief Return all the experiment data that would be
                written to the results file. The data should not be
                changed, but it could be useful to quickly extract cartain
                statistics. */
            const CompleteExperiment& getRecordedExperimentData() const
            {
                return exp_;
            }

            /** \brief Save the results of the benchmark to a stream. */
            virtual bool saveResultsToStream(std::ostream &out = std::cout) const;

            /** \brief Save the results of the benchmark to a file. */
            bool saveResultsToFile(const char *filename) const;

            /** \brief Save the results of the benchmark to a file. The name of the file is the current date and time. */
            bool saveResultsToFile() const;

        protected:

            /** \brief The instance of the problem to benchmark (if geometric planning) */
            geometric::SimpleSetup       *gsetup_;

            /** \brief The instance of the problem to benchmark (if planning with controls) */
            control::SimpleSetup         *csetup_;

            /// The set of planners to be tested
            std::vector<base::PlannerPtr> planners_;

            /// The collected experimental data (for all planners)
            CompleteExperiment            exp_;

            /// The current status of this benchmarking instance
            Status                        status_;

            /// Event to be called when the evaluated planner is switched
            PreSetupEvent                 plannerSwitch_;

            /// Event to be called before the run of a planner
            PreSetupEvent                 preRun_;

            /// Event to be called after the run of a planner
            PostSetupEvent                postRun_;

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

#ifndef OMPL_TOOLS_BENCHMARK_MACHINE_SPECS_
#define OMPL_TOOLS_BENCHMARK_MACHINE_SPECS_

#include <string>

namespace ompl
{

    /** \brief This namespace contains routines that read specifications of the machine in use */
    namespace machine
    {

        /** \brief Amount of memory used, in bytes */
        typedef unsigned long long MemUsage_t;

        /** \brief Get the amount of memory the current process is using. This should work on major platforms (Windows, Mac OS, Linux) */
        MemUsage_t getProcessMemoryUsage();

        /** \brief Get the hostname of the machine in use */
        std::string getHostname();

        /** \brief Get information about the CPU of the machine in use */
        std::string getCPUInfo();
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

/* Author: Ioan Sucan */

#ifndef OMPL_TOOLS_CONFIG_MAGIC_CONSTANTS_
#define OMPL_TOOLS_CONFIG_MAGIC_CONSTANTS_

namespace ompl
{

    /** \brief This namespace includes magic constants used in various places in OMPL.

        We strive to minimize the use of constants in the code, but at
        places, this is necessary. These constants typically do not
        have to be changed, but we chose to expose their functionality
        for the more curious user. The constants in this namespace
        include values typically used in the computation of default
        settings. */
    namespace magic
    {

        /** \brief When the cell sizes for a projection are
            automatically computed, this value defines the number of
            parts into which each dimension is split. */
        static const double PROJECTION_DIMENSION_SPLITS = 20.0;


        /** \brief When no cell sizes are specified for a projection, they are inferred like so:
            1. approximate extent of projected space by taking a number of samples (the constant below)
            2. compute the cell sizes by dividing the extent by PROJECTION_DIMENSION_SPLITS */
        static const unsigned int PROJECTION_EXTENTS_SAMPLES = 100;

        /** \brief When a bounding box of projected states cannot be inferred,
            it will be estimated by sampling states. To get closer to the true
            bounding box, we grow the bounding box of the projected sampled
            states by 5% on each side. */
        static const double PROJECTION_EXPAND_FACTOR = 0.05;

        /** \brief For planners: if default values are to be used for
            the maximum length of motions, this constant defines what
            fraction of the space extent (computed with
            ompl::base::SpaceInformation::getMaximumExtent()) is to be
            used as the maximum length of a motion */
        static const double MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION = 0.2;

        /** \brief For cost-based planners it has been observed that smaller ranges
            are typically suitable. The same range computation strategy is used for all
            planners, but for cost planners an additional factor (smaller than 1) is
            multiplied in. */
        static const double COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION = 0.175;

        /** \brief When standard deviation is needed for Gaussian
            sampling in the state space, by default the value of the
            standard deviation is computed as a fraction of the extent
            of the space */
        static const double STD_DEV_AS_SPACE_EXTENT_FRACTION = 0.1;

        /** \brief When running algorithms such as  RRT*, rewire updates are made when the cost of a path appears better than the cost of another.
            The minimum margin for a path to be better than another one is specified by this parameter. This is used to avoid
            numerical issues that can otherise arise. */
        static const double BETTER_PATH_COST_MARGIN = std::numeric_limits<double>::epsilon() * 1e3;

        /** \brief When multiple attempts are needed to generate valid
            samples, this value defines the default number of
            attempts */
        static const unsigned int MAX_VALID_SAMPLE_ATTEMPTS = 100;

        /** \brief Maximum number of sampling attempts to find a valid state,
            without checking whether the allowed time elapsed. This value
            should not really be changed. */
        static const unsigned int FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK = 2;

        /** \brief When multiple states need to be generated as part
            of the computation of various information (usually through
            stochastic processes), this parameter controls how many
            samples are generated. */
        static const unsigned int TEST_STATE_COUNT = 1000;

    }
}

#endif
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University.
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

#ifndef OMPL_TOOLS_SELF_CONFIG_
#define OMPL_TOOLS_SELF_CONFIG_

#include "ompl/config.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include <iostream>
#include <string>

namespace ompl
{
    /** \brief Includes various tools such as self config, benchmarking, etc. */
    namespace tools
    {

        /** \brief This class contains methods that automatically
            configure various parameters for motion planning. If expensive
            computation is performed, the results are cached. */
        class SelfConfig
        {
        public:

            /** \brief Construct an instance that can configure the space
                encapsulated by \e si. Any information printed to the
                console is prefixed by \e context */
            SelfConfig(const base::SpaceInformationPtr &si, const std::string &context = std::string());

            ~SelfConfig();

            /** \brief Get the probability of a sampled state being valid (calls base::SpaceInformation::probabilityOfValidState())*/
            double getProbabilityOfValidState();

            /** \brief Get the probability of a sampled state being valid (calls base::SpaceInformation::averageValidMotionLength())*/
            double getAverageValidMotionLength();

            /** \brief Instances of base::ValidStateSampler need a number of attempts to be specified -- the maximum number of times
                a new sample is selected and checked to be valid. This function computes a number of \e attempts such that the probability
                of obtaining a valid sample is 90\% */
            void configureValidStateSamplingAttempts(unsigned int &attempts);

            /** \brief Compute what a good length for motion segments is */
            void configurePlannerRange(double &range);

            /** \brief If \e proj is undefined, it is set to the default
                projection reported by base::StateSpace::getDefaultProjection().
                If no default projection is available either, an exception is thrown. */
            void configureProjectionEvaluator(base::ProjectionEvaluatorPtr &proj);

            /** \brief Print the computed configuration parameters */
            void print(std::ostream &out = std::cout) const;

            /** \brief Select a default nearest neighbor datastructure for the given space */
            template<typename _T>
            static NearestNeighbors<_T>* getDefaultNearestNeighbors(const base::StateSpacePtr &space)
            {
                if (space->isMetricSpace())
                    return new NearestNeighborsGNAT<_T>();
                else
                    return new NearestNeighborsSqrtApprox<_T>();
            }

        private:

            /// @cond IGNORE
            class SelfConfigImpl;

            SelfConfigImpl *impl_;
            std::string     context_;
            /// @endcond
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

#ifndef OMPL_TOOLS_MULTIPLAN_PARALLEL_PLAN_
#define OMPL_TOOLS_MULTIPLAN_PARALLEL_PLAN_

#include "ompl/base/Planner.h"
#include "ompl/geometric/PathGeometric.h"
#include <boost/thread.hpp>

namespace ompl
{
    /// @cond IGNORE
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathHybridization);
    }
    /// @endcond

    namespace tools
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ParallelPlan);
        /// @endcond

        /** \brief This is a utility that allows executing multiple
            planners in parallel, until one or more find a
            solution. Optionally, the results are automatically hybridized
            using ompl::geometric::PathHybridization. Between calls to
            solve(), the set of known solutions (maintained by
            ompl::base::Goal) are not cleared, and neither is the
            hybridization datastructure.*/
        class ParallelPlan
        {
        public:

            /** \brief Create an instance for a specified space information */
            ParallelPlan(const base::ProblemDefinitionPtr &pdef);

            virtual ~ParallelPlan();

            /** \brief Add a planner to use. */
            void addPlanner(const base::PlannerPtr &planner);

            /** \brief Add a planner allocator to use. */
            void addPlannerAllocator(const base::PlannerAllocator &pa);

            /** \brief Clear the set of paths recorded for hybrididzation */
            void clearHybridizationPaths();

            /** \brief Clear the set of planners to be executed */
            void clearPlanners();

            /** \brief Get the problem definition used */
            const base::ProblemDefinitionPtr& getProblemDefinition() const
            {
                return pdef_;
            }

            /** \brief Call Planner::solve() for all planners, in parallel, each planner running for at most \e solveTime seconds.
                If \e hybridize is false, when the first solution is found, the rest of the planners are stopped as well.
                If \e hybridize is true, all planners are executed until termination and the obtained solution paths are hybridized. */
            base::PlannerStatus solve(double solveTime, bool hybridize = true);

            /** \brief Call Planner::solve() for all planners, in parallel, until the termination condition \e ptc becomes true.
                If \e hybridize is false, when the first solution is found, the rest of the planners are stopped as well.
                If \e hybridize is true, all planners are executed until termination and the obtained solution paths are hybridized. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc, bool hybridize = true);

            /** \brief Call Planner::solve() for all planners, in parallel, each planner running for at most \e solveTime seconds.
                If \e hybridize is false, when \e minSolCount new solutions are found (added to the set of solutions maintained by ompl::base::Goal), the rest of the planners are stopped as well.
                If \e hybridize is true, all planners are executed until termination or until \e maxSolCount new solutions were obtained.
                While \e hybridize is true, if \e minSolCount or more solution paths are available, they are hybridized. */
            base::PlannerStatus solve(double solveTime, std::size_t minSolCount, std::size_t maxSolCount, bool hybridize = true);

            /** \brief Call Planner::solve() for all planners, in parallel, until the termination condition \e ptc becomes true.
                If \e hybridize is false, when \e minSolCount new solutions are found (added to the set of solutions maintained by ompl::base::Goal), the rest of the planners are stopped as well.
                If \e hybridize is true, all planners are executed until termination or until \e maxSolCount new solutions were obtained.
                While \e hybridize is true, if \e minSolCount or more solution paths are available, they are hybridized. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc, std::size_t minSolCount, std::size_t maxSolCount, bool hybridize = true);

        protected:

            /** \brief Run the planner and call ompl::base::PlannerTerminationCondition::terminate() for the other planners once a first solution is found */
            void solveOne(base::Planner *planner, std::size_t minSolCount, const base::PlannerTerminationCondition *ptc);

            /** \brief Run the planner and collect the solutions. This function is only called if hybridize_ is true. */
            void solveMore(base::Planner *planner, std::size_t minSolCount, std::size_t maxSolCount, const base::PlannerTerminationCondition *ptc);

            /** \brief The problem definition used */
            base::ProblemDefinitionPtr      pdef_;

            /** \brief The set of planners to be used */
            std::vector<base::PlannerPtr>   planners_;

            /** \brief The instance of the class that performs path hybridization */
            geometric::PathHybridizationPtr phybrid_;

            /** \brief Lock for phybrid_ */
            boost::mutex                    phlock_;

        private:

            /** \brief Number of solutions found during a particular run */
            unsigned int                    foundSolCount_;

            /** \brief Lock for phybrid_ */
            boost::mutex                    foundSolCountLock_;
        };

    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef OMPL_TOOLS_MULTIPLAN_OPTIMIZE_PLAN_
#define OMPL_TOOLS_MULTIPLAN_OPTIMIZE_PLAN_

#include "ompl/tools/multiplan/ParallelPlan.h"

namespace ompl
{
    namespace tools
    {

        /** \brief Run one or more motion planners repeatedly (using a
            specified number of threads), and hybridize solutions, trying
            to optimize solutions. */
        class OptimizePlan
        {
        public:
            /** \brief Create an instance for a specified space information */
            OptimizePlan(const base::ProblemDefinitionPtr &pdef) : pp_(pdef)
            {
            }

            virtual ~OptimizePlan()
            {
            }

            /** \brief Add a planner to use. */
            void addPlanner(const base::PlannerPtr &planner);

            /** \brief Add a planner allocator to use. */
            void addPlannerAllocator(const base::PlannerAllocator &pa);

            /** \brief Clear the set of planners to be executed */
            void clearPlanners();

            /** \brief Get the problem definition used */
            const base::ProblemDefinitionPtr& getProblemDefinition() const
            {
                return pp_.getProblemDefinition();
            }

            /** \brief Try to solve the specified problem within a \e solveTime seconds, using at most \e nthreads threads. If
                more than \e maxSol solutions are generated, stop generating more. */
            base::PlannerStatus solve(double solveTime, unsigned int maxSol = 10, unsigned int nthreads = 1);

        protected:

            /** \brief Instance of parallel planning to use for computing solutions in parallel */
            ParallelPlan                    pp_;

            /** \brief The set of planners to be used */
            std::vector<base::PlannerPtr>   planners_;
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

/* Author: Mark Moll */

#ifndef PY_BINDINGS_OMPL_PY_TOOLS_
#define PY_BINDINGS_OMPL_PY_TOOLS_

#include "py_boost_function.hpp"

#endif
