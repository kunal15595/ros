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

#ifndef OMPL_UTIL_CLASS_FORWARD_
#define OMPL_UTIL_CLASS_FORWARD_

#include <boost/shared_ptr.hpp>

/** \brief Macro that defines a forward declaration for a class, and
    shared pointers to the class. For example OMPL_CLASS_FORWARD(MyType);
    will produce type definitions for MyType and MyTypePtr. */
#define OMPL_CLASS_FORWARD(C)                                          \
    class C;                                                           \
    typedef boost::shared_ptr<C> C##Ptr

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage
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

#ifndef OMPL_BASE_GENERIC_PARAM_
#define OMPL_BASE_GENERIC_PARAM_

#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <map>

namespace ompl
{
    namespace base
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::GenericParam */
        OMPL_CLASS_FORWARD(GenericParam);
        /// @endcond

        /** \brief Motion planning algorithms often employ parameters
            to guide their exploration process. (e.g., goal
            biasing). Motion planners (and some of their components)
            use this class to declare what the parameters are, in a
            generic way, so that they can be set externally. */
        class GenericParam
        {
        public:

            /** \brief The constructor of a parameter takes the name of the parameter (\e name) */
            GenericParam(const std::string &name) : name_(name)
            {
            }

            virtual ~GenericParam()
            {
            }

            /** \brief Get the name of the parameter */
            const std::string& getName() const
            {
                return name_;
            }

            /** \brief Set the name of the parameter */
            void setName(const std::string &name)
            {
                name_ = name;
            }

            /** \brief Set the value of the parameter. The value is taken in as a string, but converted to the type of that parameter. */
            virtual bool setValue(const std::string &value) = 0;

            /** \brief Retrieve the value of the parameter, as a string. */
            virtual std::string getValue() const = 0;

            /** \brief Assignment operator by type. This is just for convenience, as it just calls setValue() */
            template<typename T>
            GenericParam& operator=(const T &value)
            {
                try
                {
                    setValue(boost::lexical_cast<std::string>(value));
                }
                catch (boost::bad_lexical_cast &e)
                {
                    OMPL_WARN("Invalid value format specified for parameter '%s': %s", name_.c_str(), e.what());
                }
                return *this;
            }

            /** \brief Set a suggested range */
            void setRangeSuggestion(const std::string &rangeSuggestion)
            {
                rangeSuggestion_ = rangeSuggestion;
            }

            /** \brief Get the suggested range of values */
            const std::string& getRangeSuggestion() const
            {
                return rangeSuggestion_;
            }

        protected:

            /** \brief The name of the parameter */
            std::string name_;

            /** \brief Suggested range for the parameter

                This can be used to provide a hint to, e.g., a GUI. The
                convention used in OMPL is to denote ranges for the
                following types as follows:
                - \c bool: "0,1"
                - \c enum: "<enum_val0>,<enum_val1>,<enum_val2>,..."
                - \c int, \c double: either "first:last" or "first:stepsize:last".
                  In the first case, the stepsize is assumed to be 1. It is
                  important to use floating point representations for double
                  ranges (i.e., "1." instead of "1") to make sure the type is
                  deduced correctly.
            */
            std::string rangeSuggestion_;
        };


        /** \brief This is a helper class that instantiates parameters with different data types. */
        template<typename T>
        class SpecificParam : public GenericParam
        {
        public:

            /** \brief The type for the 'setter' function for this parameter */
            typedef boost::function<void(T)> SetterFn;

            /** \brief The type for the 'getter' function for this parameter */
            typedef boost::function<T()>     GetterFn;

            /** \brief An explicit instantiation of a parameter \e name requires the \e setter function and optionally the \e
                getter function. */
            SpecificParam(const std::string &name, const SetterFn &setter, const GetterFn &getter = GetterFn()) :
                GenericParam(name), setter_(setter), getter_(getter)
            {
                if (!setter_)
                    OMPL_ERROR("Setter function must be specified for parameter");
            }

            virtual ~SpecificParam()
            {
            }

            virtual bool setValue(const std::string &value)
            {
                bool result = true;
                try
                {
                    if (setter_)
                        setter_(boost::lexical_cast<T>(value));
                }
                catch (boost::bad_lexical_cast &e)
                {
                    result = false;
                    OMPL_WARN("Invalid value format specified for parameter '%s': %s", name_.c_str(), e.what());
                }

                if (getter_)
                    OMPL_DEBUG("The value of parameter '%s' is now: '%s'", name_.c_str(), getValue().c_str());
                else
                    OMPL_DEBUG("The value of parameter '%s' was set to: '%s'", name_.c_str(), value.c_str());
                return result;
            }

            virtual std::string getValue() const
            {
                if (getter_)
                    try
                    {
                        return boost::lexical_cast<std::string>(getter_());
                    }
                    catch (boost::bad_lexical_cast &e)
                    {
                        OMPL_WARN("Unable to parameter '%s' to string: %s", name_.c_str(), e.what());
                        return "";
                    }
                else
                    return "";
            }

        protected:

            /** \brief The setter function for this parameter */
            SetterFn setter_;

            /** \brief The getter function for this parameter */
            GetterFn getter_;
        };

        /** \brief Maintain a set of parameters */
        class ParamSet
        {
        public:

            /** \brief This function declares a parameter \e name, and specifies the \e setter and \e getter functions. */
            template<typename T>
            void declareParam(const std::string &name, const typename SpecificParam<T>::SetterFn &setter,
                              const typename SpecificParam<T>::GetterFn &getter = typename SpecificParam<T>::GetterFn())
            {
                params_[name].reset(new SpecificParam<T>(name, setter, getter));
            }

            /** \brief Add a parameter to the set */
            void add(const GenericParamPtr &param);

            /** \brief Remove a parameter from the set */
            void remove(const std::string &name);

            /** \brief Include the params of a different ParamSet into this one. Optionally include a prefix for each of the parameters */
            void include(const ParamSet &other, const std::string &prefix = "");

            /** \brief Algorithms in OMPL often have parameters that
                can be set externally. While each algorithm will have
                their own getter and setter functions specifically for
                those parameters, this function allows setting
                parameters generically, for any algorithm that
                declares parameters, by specifying the parameter name
                \e key and its value \e value (both as string, but \e
                value is cast to the type desired by the corresponding
                setter). Under the hood, this calls SpecificParam::setValue().
                This ability makes it easy to automatically configure
                using external sources (e.g., a configuration
                file). The function returns true if the parameter was
                parsed and set successfully and false otherwise. */
            bool setParam(const std::string &key, const std::string &value);

            /** \brief Get the value of the parameter named \e key. Store the value as string in \e value and return true if the parameter was found. Return false otherwise. */
            bool getParam(const std::string &key, std::string &value) const;

            /** \brief Set the values for a set of parameters. The parameter names are the keys in the map \e kv.
                The corresponding key values in \e kv are set as the parameter values.
                Return true if all parameters were set successfully. This function simply calls setParam() multiple times.
                If \e ignoreUnknown is true, then no attempt is made to set unknown
                parameters (and thus no errors are reported) */
            bool setParams(const std::map<std::string, std::string> &kv, bool ignoreUnknown = false);

            /** \brief Get the known parameter as a map from names to their values cast as string */
            void getParams(std::map<std::string, std::string> &params) const;

            /** \brief List the names of the known parameters */
            void getParamNames(std::vector<std::string> &params) const;

            /** \brief List the values of the known parameters, in the same order as getParamNames() */
            void getParamValues(std::vector<std::string> &vals) const;

            /** \brief Get the map from parameter names to parameter descriptions */
            const std::map<std::string, GenericParamPtr>& getParams() const;

            /** \brief Get the parameter that corresponds to a specified name. An empty shared ptr is returned if the parameter does not exist */
            const GenericParamPtr& getParam(const std::string &key) const;

            /** \brief Check whether this set of parameters includes the parameter named \e key */
            bool hasParam(const std::string &key) const;

            /** \brief Access operator for parameters, by name. If the parameter is not defined, an exception is thrown */
            GenericParam& operator[](const std::string &key);

            /** \brief Get the number of parameters maintained by this instance */
            std::size_t size() const
            {
                return params_.size();
            }

            /** \brief Clear all the set parameters */
            void clear();

            /** \brief Print the parameters to a stream */
            void print(std::ostream &out) const;

        private:

            std::map<std::string, GenericParamPtr> params_;
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

#ifndef OMPL_BASE_STATE_SPACE_TYPES_
#define OMPL_BASE_STATE_SPACE_TYPES_

namespace ompl
{
    namespace base
    {

        /** \brief The type of a state space */
        enum StateSpaceType
            {

                /** \brief Unset type; this is the default type */
                STATE_SPACE_UNKNOWN     =  0,

                /** \brief ompl::base::RealVectorStateSpace */
                STATE_SPACE_REAL_VECTOR =  1,

                /** \brief ompl::base::SO2StateSpace */
                STATE_SPACE_SO2         =  2,

                /** \brief ompl::base::SO3StateSpace */
                STATE_SPACE_SO3         =  3,

                /** \brief ompl::base::SE2StateSpace */
                STATE_SPACE_SE2         =  4,

                /** \brief ompl::base::SE3StateSpace */
                STATE_SPACE_SE3         =  5,

                /** \brief ompl::base::TimeStateSpace */
                STATE_SPACE_TIME        =  6,

                /** \brief ompl::base::DiscreteStateSpace */
                STATE_SPACE_DISCRETE    =  7,

                /** \brief Number of state space types; To add new types,
                    use values that are larger than the count*/
                STATE_SPACE_TYPE_COUNT
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

#ifndef OMPL_BASE_STATE_
#define OMPL_BASE_STATE_

#include <boost/concept_check.hpp>

namespace ompl
{
    namespace base
    {

        /** \brief Definition of an abstract state.

            See \ref stateAlloc and \ref stateOps. */
        class State
        {
        private:

            /** \brief Disable copy-constructor */
            State(const State&);

            /** \brief Disable copy operator */
            const State& operator=(const State&);

        protected:

            State()
            {
            }

            virtual ~State()
            {
            }

        public:

            /** \brief Cast this instance to a desired type. */
            template<class T>
            const T* as() const
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

                return static_cast<const T*>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            T* as()
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

                return static_cast<T*>(this);
            }

        };

        /** \brief Definition of a compound state */
        class CompoundState : public State
        {
        public:

            CompoundState() : State(), components(NULL)
            {
            }

            virtual ~CompoundState()
            {
            }

            /** \brief Cast a component of this instance to a desired type. */
            template<class T>
            const T* as(const unsigned int index) const
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

                return static_cast<const T*>(components[index]);
            }

            /** \brief Cast a component of this instance to a desired type. */
            template<class T>
            T* as(const unsigned int index)
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

                return static_cast<T*>(components[index]);
            }

            /** \brief Access element i<sup>th</sup> component. This
                does not check whether the index is within bounds. */
            State* operator[](unsigned int i) const
            {
                return components[i];
            }

            /** \brief The components that make up a compound state */
            State **components;
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

#ifndef OMPL_BASE_STATE_SAMPLER_
#define OMPL_BASE_STATE_SAMPLER_

#include "ompl/base/State.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/util/ClassForward.h"
#include <vector>
#include <string>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>

namespace ompl
{
    namespace base
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::StateSampler */
        OMPL_CLASS_FORWARD(StateSampler);
        /// @endcond

        /** \class ompl::base::StateSamplerPtr
            \brief A boost shared pointer wrapper for ompl::base::StateSampler */

        /** \brief Abstract definition of a state space sampler. */
        class StateSampler : private boost::noncopyable
        {
        public:

            /** \brief Constructor */
            StateSampler(const StateSpace *space) : space_(space)
            {
            }

            virtual ~StateSampler()
            {
            }

            /** \brief Sample a state */
            virtual void sampleUniform(State *state) = 0;

            /** \brief Sample a state near another, within specified distance */
            virtual void sampleUniformNear(State *state, const State *near, const double distance) = 0;

            /** \brief Sample a state using a Gaussian distribution with given \e mean and standard deviation (\e stdDev) */
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev) = 0;

        protected:

            /** \brief The state space this sampler samples */
            const StateSpace *space_;

            /** \brief An instance of a random number generator */
            RNG               rng_;
        };

        /** \brief Definition of a compound state sampler. This is useful to construct samplers for compound states. */
        class CompoundStateSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            CompoundStateSampler(const StateSpace *space) : StateSampler(space), samplerCount_(0)
            {
            }

            /** \brief Destructor. This frees the added samplers as well. */
            virtual ~CompoundStateSampler()
            {
            }

            /** \brief Add a sampler as part of the new compound
                sampler. This sampler is used to sample part of the
                compound state. When sampling near a state, the
                compound sampler calls in to added samplers. The
                distance passed to the called samplers is adjusted
                according to the specified importance. */
            virtual void addSampler(const StateSamplerPtr &sampler, double weightImportance);

            virtual void sampleUniform(State *state);

            /** \brief Call sampleUniformNear for each of the subspace states
                with distance scaled by the corresponding subspace weight. */
            virtual void sampleUniformNear(State *state, const State *near, const double distance);

            /** \brief Call sampleGaussian for each of the subspace states
                with stdDev scaled by the corresponding subspace weight. */
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);

        protected:

            /** \brief The samplers that are composed */
            std::vector<StateSamplerPtr> samplers_;

            /** \brief The weight of each sampler (used when sampling near a state) */
            std::vector<double>          weightImportance_;

        private:

            /** \brief The number of samplers that are composed */
            unsigned int                 samplerCount_;

        };

        /** \brief Construct a sampler that samples only within a subspace of the space */
        class SubspaceStateSampler : public StateSampler
        {
        public:

            /** \brief Construct a sampler for \e space but only sample components common to \e subspace. Use \e weight as a multiplicative factor for \e distance and \e stdDev in the sampleUniformNear() and sampleGaussian() functions. */
            SubspaceStateSampler(const StateSpace *space, const StateSpace *subspace, double weight);
            virtual ~SubspaceStateSampler();

            virtual void sampleUniform(State *state);

            virtual void sampleUniformNear(State *state, const State *near, const double distance);

            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);

        protected:

            /** \brief The subspace to sample */
            const StateSpace        *subspace_;

            /** \brief The sampler for the subspace */
            StateSamplerPtr          subspaceSampler_;

            /** \brief The weigth factor to multiply distance and stdDev when sampling in the vicinity of a state */
            double                   weight_;

            /** \brief The names of common subspaces between \e space_ and \e subspace_; these are the ones copied after sampling a state */
            std::vector<std::string> subspaces_;

        private:

            /** \brief Temporary work area */
            State                   *work_;

            /** \brief Temporary work area */
            State                   *work2_;
        };

        /** \brief Definition of a function that can allocate a state sampler */
        typedef boost::function<StateSamplerPtr(const StateSpace*)> StateSamplerAllocator;
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

#ifndef OMPL_BASE_VALID_STATE_SAMPLER_
#define OMPL_BASE_VALID_STATE_SAMPLER_

#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"
#include "ompl/base/GenericParam.h"
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>
#include <string>

namespace ompl
{
    namespace base
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ValidStateSampler */
        OMPL_CLASS_FORWARD(ValidStateSampler);
        /// @endcond

        /** \class ompl::base::ValidStateSamplerPtr
            \brief A boost shared pointer wrapper for ompl::base::ValidStateSampler */

        /** \brief Abstract definition of a state sampler. */
        class ValidStateSampler : private boost::noncopyable
        {
        public:

            /** \brief Constructor */
            ValidStateSampler(const SpaceInformation *si);

            virtual ~ValidStateSampler();

            /** \brief Get the name of the sampler */
            const std::string& getName() const
            {
                return name_;
            }

            /** \brief Set the name of the sampler */
            void setName(const std::string &name)
            {
                name_ = name;
            }

            /** \brief Sample a state. Return false in case of failure */
            virtual bool sample(State *state) = 0;

            /** \brief Sample a state near another, within specified distance. Return false, in case of failure.
                \note The memory for \e near must be disjoint from the memory for \e state */
            virtual bool sampleNear(State *state, const State *near, const double distance) = 0;

            /** \brief Finding a valid sample usually requires
                performing multiple attempts. This call allows setting
                the number of such attempts. */
            void setNrAttempts(unsigned int attempts)
            {
                attempts_ = attempts;
            }

            /** \brief Get the number of attempts to be performed by the sampling routine */
            unsigned int getNrAttempts() const
            {
                return attempts_;
            }

            /** \brief Get the parameters for the valid state sampler */
            ParamSet& params()
            {
                return params_;
            }

            /** \brief Get the parameters for the valid state sampler */
            const ParamSet& params() const
            {
                return params_;
            }

        protected:

            /** \brief The state space this sampler samples */
            const SpaceInformation *si_;

            /** \brief Number of attempts to find a valid sample */
            unsigned int            attempts_;

            /** \brief The name of the sampler */
            std::string             name_;

            /** \brief The parameters for this instance of the valid state sampler */
            ParamSet                params_;
        };

        /** \brief Definition of a function that can allocate a valid state sampler */
        typedef boost::function<ValidStateSamplerPtr(const SpaceInformation*)> ValidStateSamplerAllocator;
    }
}


#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage
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

#include "ompl/base/StateSampler.h"
#include <vector>

namespace ompl
{
    namespace base
    {

        /** \brief State space sampler for discrete states */
        class PrecomputedStateSampler : public StateSampler
        {
        public:

            /** \brief Constructor. Takes the state space to be sampled (\e space) and the set of states to draw samples from (\e states) */
            PrecomputedStateSampler(const StateSpace *space, const std::vector<const State*> &states);

            /** \brief Constructor. Takes the state space to be sampled (\e space), the set of states to draw samples from (\e states) and a range to sample from: [\e minIndex, \e maxIndex]*/
            PrecomputedStateSampler(const StateSpace *space, const std::vector<const State*> &states, std::size_t minIndex, std::size_t maxIndex);

            virtual void sampleUniform(State *state);
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);

        protected:

            /** \brief The states to sample from */
            const std::vector<const State*> &states_;

            /** \brief The minimum index to start sampling at */
            std::size_t                      minStateIndex_;

            /** \brief The maximum index to stop sampling at */
            std::size_t                      maxStateIndex_;
        };
    }
}
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

#ifndef OMPL_BASE_SPACES_REAL_VECTOR_BOUNDS_
#define OMPL_BASE_SPACES_REAL_VECTOR_BOUNDS_

#include <vector>

namespace ompl
{
    namespace base
    {

        /** \brief The lower and upper bounds for an R<sup>n</sup> space */
        class RealVectorBounds
        {
        public:

            /** \brief Constructor; \e dim represents the dimension of the space these bounds are for. */
            RealVectorBounds(unsigned int dim)
            {
                resize(dim);
            }

            /** \brief Set the lower bound in each dimension to a specific value */
            void setLow(double value);

            /** \brief Set the upper bound in each dimension to a specific value */
            void setHigh(double value);

            /** \brief Set the lower bound of a dimension to a specific value */
            void setLow(unsigned int index, double value);

            /** \brief Set the upper bound of a dimension to a specific value */
            void setHigh(unsigned int index, double value);

            /** \brief Change the number of dimensions for the bounds */
            void resize(std::size_t size);

            /** \brief Compute the volume of the space enclosed by the bounds */
            double getVolume() const;

            /** \brief Get the difference between the high and low bounds for each dimension: result[i] = high[i] - low[i] */
            std::vector<double> getDifference() const;

            /** \brief Check if the bounds are valid (same length for
                low and high, high[i] > low[i]). Throw an exception if
                this is not the case. */
            void check() const;

            /** \brief Lower bound */
            std::vector<double> low;

            /** \brief Upper bound */
            std::vector<double> high;
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

#ifndef OMPL_BASE_PROJECTION_EVALUATOR_
#define OMPL_BASE_PROJECTION_EVALUATOR_

#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/base/GenericParam.h"
#include "ompl/base/spaces/RealVectorBounds.h"

#include <vector>
#include <valarray>
#include <iostream>
#include <boost/noncopyable.hpp>
#include <boost/numeric/ublas/matrix.hpp>

namespace ompl
{

    namespace base
    {

        /** \brief Grid cells corresponding to a projection value are described in terms of their coordinates. */
        typedef std::vector<int> ProjectionCoordinates;

        /** \brief The datatype for state projections. This class contains a real vector. */
        typedef boost::numeric::ublas::vector<double> EuclideanProjection;


        /** \brief A projection matrix -- it allows multiplication of
            real vectors by a specified matrix. The matrix can also be
            randomly generated. */
        class ProjectionMatrix
        {
        public:

            /** \brief Datatype for projection matrices */
            typedef boost::numeric::ublas::matrix<double> Matrix;

            /** \brief Compute a random projection matrix with \e from
                columns and \e to rows. A vector with \e from elements
                can be multiplied by this matrix in order to produce a
                vector with \e to elements.

                If the \e scale argument is specified, the columns of
                the matrix are divided by the corresponding scaling
                argument: all elements (rows) in column[i] are divided
                by scale[i]. This is useful to specify if scaling of
                the elements of the state is to be applied before
                projection. If the scale for a column is 0, the column
                itself is set to 0.

                Each element is sampled with a Gaussian distribution
                with mean 0 and variance 1 and the matrix rows are
                made orthonormal. */
            static Matrix ComputeRandom(const unsigned int from, const unsigned int to, const std::vector<double> &scale);

            /** \brief Compute a random projection matrix with \e from
                columns and \e to rows. A vector with \e from elements
                can be multiplied by this matrix in order to produce a
                vector with \e to elements. This uses the function above
                called with an empty \e scale vector.

                Each element is sampled with a Gaussian distribution
                with mean 0 and variance 1 and the matrix rows are
                made orthonormal. */
            static Matrix ComputeRandom(const unsigned int from, const unsigned int to);

            /** \brief Wrapper for ComputeRandom(from, to, scale) */
            void computeRandom(const unsigned int from, const unsigned int to, const std::vector<double> &scale);

            /** \brief Wrapper for ComputeRandom(from, to) */
            void computeRandom(const unsigned int from, const unsigned int to);

            /** \brief Multiply the vector \e from by the contained projection matrix to obtain the vector \e to. */
            void project(const double *from, EuclideanProjection& to) const;

            /** \brief Print the contained projection matrix to a stram */
            void print(std::ostream &out = std::cout) const;

            /** \brief Projection matrix */
            Matrix mat;
        };

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ProjectionEvaluator */
        OMPL_CLASS_FORWARD(ProjectionEvaluator);
        /// @endcond

        /** \class ompl::base::ProjectionEvaluatorPtr
            \brief A boost shared pointer wrapper for ompl::base::ProjectionEvaluator */

        /** \brief Abstract definition for a class computing
            projections to R<sup>n</sup>. Implicit integer grids are
            imposed on this projection space by setting cell
            sizes. Before use, the user must supply cell sizes
            for the integer grid (setCellSizes()). The
            implementation of this class is thread safe. */
        class ProjectionEvaluator : private boost::noncopyable
        {
        public:

            /** \brief Construct a projection evaluator for a specific state space */
            ProjectionEvaluator(const StateSpace *space);

            /** \brief Construct a projection evaluator for a specific state space */
            ProjectionEvaluator(const StateSpacePtr &space);

            virtual ~ProjectionEvaluator();

            /** \brief Return the dimension of the projection defined by this evaluator */
            virtual unsigned int getDimension() const = 0;

            /** \brief Compute the projection as an array of double values */
            virtual void project(const State *state, EuclideanProjection &projection) const = 0;

            /** \brief Define the size (in each dimension) of a grid
                cell. The number of sizes set here must be the
                same as the dimension of the projection computed by
                the projection evaluator. After a call to this
                function, setup() will not call
                defaultCellSizes() or inferCellSizes() any
                more. */
            virtual void setCellSizes(const std::vector<double> &cellSizes);

            /** \brief Set the cell sizes to \e cellSize for a particular dimension \e dim. This function simply calls getCellSizes(),
                modifies the desired dimension and then calls setCellSizes(). This is done intentionally to enforce a call to setCellSizes(). */
            void setCellSizes(unsigned int dim, double cellSize);

            /** \brief Multiply the cell sizes in each dimension by a specified factor \e factor. This function does nothing
                if cell sizes have not been set. If cell sizes have been set (irrespective of source; e.g., user, default or inferred),
                this function will then call setCellSizes(), so the source of the cell sizes will be considered to be the user. */
            void mulCellSizes(double factor);

            /** \brief Return true if any user configuration has been done to this projection evaluator (setCellSizes() was called) */
            bool userConfigured() const;

            /** \brief Get the size (each dimension) of a grid cell  */
            const std::vector<double>& getCellSizes() const
            {
                return cellSizes_;
            }

            /** \brief Get the size of a particular dimension of a grid cell  */
            double getCellSizes(unsigned int dim) const;

            /** \brief Check if cell dimensions match projection dimension */
            void checkCellSizes() const;

            /** \brief Sample the state space and decide on default
                cell sizes. This function is called by setup() if
                no cell dsizes have been set and
                defaultCellSizes() does not fill the cell
                sizes either. */
            void inferCellSizes();

            /** \brief Set the default cell dimensions for this
                projection. The default implementation of this
                function is empty. setup() calls this function if no
                cell dimensions have been previously set. */
            virtual void defaultCellSizes();

            /** \brief Check if the projection dimension matched the dimension of the bounds */
            void checkBounds() const;

            /** \brief Check if bounds were specified for this projection */
            bool hasBounds() const
            {
                return !bounds_.low.empty();
            }

            /** \brief Set bounds on the projection. The PDST planner
                 needs to known the bounds on the projection. Default bounds
                 are automatically computed by inferCellSizes(). */
            void setBounds(const RealVectorBounds &bounds);

            /** \brief Get the bounds computed/set for this projection */
            const RealVectorBounds& getBounds()
            {
                return bounds_;
            }

            /** \brief Compute an approximation of the bounds for this projection space. getBounds() will then report the computed bounds. */
            void inferBounds();

            /** \brief Perform configuration steps, if needed */
            virtual void setup();

            /** \brief Compute integer coordinates for a projection */
            void computeCoordinates(const EuclideanProjection &projection, ProjectionCoordinates &coord) const;

            /** \brief Compute integer coordinates for a state */
            void computeCoordinates(const State *state, ProjectionCoordinates &coord) const
            {
                EuclideanProjection projection(getDimension());
                project(state, projection);
                computeCoordinates(projection, coord);
            }

            /** \brief Get the parameters for this projection */
            ParamSet& params()
            {
                return params_;
            }

            /** \brief Get the parameters for this projection */
            const ParamSet& params() const
            {
                return params_;
            }

            /** \brief Print settings about this projection */
            virtual void printSettings(std::ostream &out = std::cout) const;

            /** \brief Print a euclidean projection */
            virtual void printProjection(const EuclideanProjection &projection, std::ostream &out = std::cout) const;

        protected:

            /** \brief Fill estimatedBounds_ with an approximate bounding box for the projection space (via sampling) */
            void estimateBounds();

            /** \brief The state space this projection operates on */
            const StateSpace    *space_;

            /** \brief The size of a cell, in every dimension of the
                projected space, in the implicitly defined integer
                grid. */
            std::vector<double>  cellSizes_;

            /** \brief A bounding box for projected state values */
            RealVectorBounds     bounds_;

            /** \brief An approximate bounding box for projected state values;
                This is the cached result of estimateBounds() which may later be copied
                to bounds_ if bounds are needed but were not specified. */
            RealVectorBounds     estimatedBounds_;

            /** \brief Flag indicating whether cell sizes have
                been set by the user, or whether they were inferred
                automatically. This flag becomes false if
                setCellSizes() is called. */
            bool                 defaultCellSizes_;

            /** \brief Flag indicating whether projection cell sizes
                were automatically inferred. */
            bool                 cellSizesWereInferred_;

            /** \brief The set of parameters for this projection */
            ParamSet             params_;
        };

        /** \brief If the projection for a CompoundStateSpace is
            supposed to be the same as the one for one of its included
            subspaces, this class facilitates selecting a projection
            of that subspace. */
        class SubspaceProjectionEvaluator : public ProjectionEvaluator
        {
        public:

            /** \brief The constructor states that for space \e space,
                the projection to use is the same as the component at
                position \e index of space \e space. The actual
                projection to use can be specified by \e projToUse. If
                the projection is not specified, the default one for
                the subspace at position \e index is used. */
            SubspaceProjectionEvaluator(const StateSpace *space, unsigned int index, const ProjectionEvaluatorPtr &projToUse = ProjectionEvaluatorPtr());

            virtual void setup();

            virtual unsigned int getDimension() const;

            virtual void project(const State *state, EuclideanProjection &projection) const;

        protected:

            /** \brief The index of the subspace from which to project */
            unsigned int           index_;

            /** \brief The projection to use. This is either the same
                as \e specifiedProj_ or, if specifiedProj_ is not
                initialized, it is the default projection for the
                subspace at index \e index_ */
            ProjectionEvaluatorPtr proj_;

            /** \brief The projection that is optionally specified by the user in the constructor argument (\e projToUse) */
            ProjectionEvaluatorPtr specifiedProj_;
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

#ifndef OMPL_BASE_STATE_SPACE_
#define OMPL_BASE_STATE_SPACE_

#include "ompl/base/State.h"
#include "ompl/base/StateSpaceTypes.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/GenericParam.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include <boost/concept_check.hpp>
#include <boost/noncopyable.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>

namespace ompl
{
    namespace base
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::StateSpace */
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond

        /** \class ompl::base::StateSpacePtr
            \brief A boost shared pointer wrapper for ompl::base::StateSpace */


        /** \brief Representation of a space in which planning can be
            performed. Topology specific sampling, interpolation and distance
            are defined.

            See \ref implementingStateSpaces. */
        class StateSpace : private boost::noncopyable
        {
        public:

            /** \brief Define the type of state allocated by this space */
            typedef State StateType;

            /** \brief Constructor. Assigns a @b unique name to the space */
            StateSpace();

            virtual ~StateSpace();

            /** \brief Cast this instance to a desired type. */
            template<class T>
            T* as()
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateSpace*>));

                return static_cast<T*>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            const T* as() const
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateSpace*>));

                return static_cast<const T*>(this);
            }

            /** \brief Representation of the address of a substate in a state. This structure stores the indexing information needed to access a particular substate of a state */
            struct SubstateLocation
            {
                /** \brief In a complex state space there may be multiple
                    compound state spaces that make up an even larger
                    compound space.  This array indicates the sequence of
                    indices of the subspaces that need to be followed to
                    get to the component of the state that is of interest. */
                std::vector<std::size_t> chain;

                /** \brief The space that is reached if the chain above is followed on the state space */
                const StateSpace        *space;
            };

            /** \brief Representation of the address of a value in a state. This structure stores the indexing information needed to access elements of a state (no pointer values are stored) */
            struct ValueLocation
            {
                /** \brief Location of the substate that contains the pointed to value */
                SubstateLocation stateLocation;

                /** \brief The index of the value to be accessed, within the substate location above */
                std::size_t      index;
            };

            /** \brief Flags to use in a bit mask for state space sanity checks. Some basic checks do not have flags associated (they are always executed; for example,
                whether copyState() works as expected) */
            enum SanityChecks
                {

                    /// \brief Check whether the distances between non-equal states is strictly positive (StateSpace::distance())
                    STATESPACE_DISTANCE_DIFFERENT_STATES = (1<<1),

                    /// \brief Check whether the distance function is symmetric (StateSpace::distance())
                    STATESPACE_DISTANCE_SYMMETRIC        = (1<<2),

                    /// \brief Check whether calling StateSpace::interpolate() works as expected
                    STATESPACE_INTERPOLATION             = (1<<3),

                    /// \brief Check whether the triangle inequality holds when using StateSpace::interpolate() and StateSpace::distance()
                    STATESPACE_TRIANGLE_INEQUALITY       = (1<<4),

                    /// \brief Check whether the StateSpace::distance() is bounded by StateSpace::getExtent()
                    STATESPACE_DISTANCE_BOUND            = (1<<5),

                    /// \brief Check whether sampled states are always within bounds
                    STATESPACE_RESPECT_BOUNDS            = (1<<6),

                    /// \brief Check that enforceBounds() does not modify the contents of states that are within bounds
                    STATESPACE_ENFORCE_BOUNDS_NO_OP      = (1<<7),

                    /// \brief Check whether the StateSpace::serialize() and StateSpace::deserialize() work as expected
                    STATESPACE_SERIALIZATION             = (1<<8)
                };

            /** @name Generic functionality for state spaces
                @{ */

            /** \brief Check if the state space is compound */
            virtual bool isCompound() const;

            /** \brief Check if the set of states is discrete

                \note In fact, because of limited numerical precision,
                the representation of all spaces is discrete; this
                function returns true if the corresponding
                mathematical object is a discrete one. */
            virtual bool isDiscrete() const;

            /** \brief Check if this is a hybrid state space (i.e., both discrete and continuous components exist)*/
            virtual bool isHybrid() const;

            /** \brief Return true if the distance function associated with the space
                is a metric */
            virtual bool isMetricSpace() const
            {
                return true;
            }

            /** \brief Check if the distance function on this state space is symmetric, i.e. distance(s1,s2) = distance(s2,s1). Default implementation returns true.*/
            virtual bool hasSymmetricDistance() const;

            /** \brief Check if the interpolation function on this state space is symmetric, i.e. interpolate(from, to, t, state) = interpolate(to, from, 1-t, state). Default implementation returns true.*/
            virtual bool hasSymmetricInterpolate() const;

            /** \brief Get the name of the state space */
            const std::string& getName() const;

            /** \brief Set the name of the state space */
            void setName(const std::string &name);

            /** \brief Get the type of the state space. The type can be
                used to verify whether two space instances are of
                the same type (e.g., SO2) */
            int getType() const
            {
                return type_;
            }

            /** \brief Return true if \e other is a space included (perhaps equal, perhaps a subspace) in this one. */
            bool includes(const StateSpacePtr &other) const;

            /** \brief Return true if \e other is a space included (perhaps equal, perhaps a subspace) in this one. */
            bool includes(const StateSpace *other) const;

            /** \brief Return true if \e other is a space that is either included (perhaps equal, perhaps a subspace)
                in this one, or all of its subspaces are included in this one. */
            bool covers(const StateSpacePtr &other) const;

            /** \brief Return true if \e other is a space that is either included (perhaps equal, perhaps a subspace)
                in this one, or all of its subspaces are included in this one. */
            bool covers(const StateSpace *other) const;

            /** \brief Get the parameters for this space */
            ParamSet& params()
            {
                return params_;
            }

            /** \brief Get the parameters for this space */
            const ParamSet& params() const
            {
                return params_;
            }

            /** \brief When performing discrete validation of motions,
                the length of the longest segment that does not
                require state validation needs to be specified. This
                function returns this length, for this state space, as a
                fraction of the space's maximum extent. */
            virtual double getLongestValidSegmentFraction() const;

            /** \brief When performing discrete validation of motions,
                the length of the longest segment that does not
                require state validation needs to be specified. This
                function sets this length as a fraction of the
                space's maximum extent.

                \note This function's effect is not considered until
                after setup() has been called. For immediate effects
                (i.e., during planning) use
                setValidSegmentCountFactor() */
            virtual void setLongestValidSegmentFraction(double segmentFraction);

            /** \brief Count how many segments of the "longest valid length" fit on the motion from \e state1 to \e state2 */
            virtual unsigned int validSegmentCount(const State *state1, const State *state2) const;

            /** \brief Set \e factor to be the value to multiply the
                return value of validSegmentCount(). By default, this
                value is 1. The higher the value, the smaller the size
                of the segments considered valid. The effect of this
                function is immediate (setup() does not need to be
                called). */
            void setValidSegmentCountFactor(unsigned int factor);

            /** \brief Get the value used to multiply the return value of validSegmentCount().*/
            unsigned int getValidSegmentCountFactor() const;

            /** \brief Compute an array of ints that uniquely identifies the structure of the state space.
                The first element of the signature is the number of integers that follow */
            void computeSignature(std::vector<int> &signature) const;

            /** @} */

            /** @name Functionality specific to state spaces (to be implemented by derived state spaces)
                @{ */

            /** \brief Get the dimension of the space (not the dimension of the surrounding ambient space) */
            virtual unsigned int getDimension() const = 0;

            /** \brief Get the maximum value a call to distance() can return (or an upper bound).
                For unbounded state spaces, this function can return infinity.

                \note Tight upper bounds are preferred because the value of the extent is used in
                the automatic computation of parameters for planning. If the bounds are less tight,
                the automatically computed parameters will be less useful.*/
            virtual double getMaximumExtent() const = 0;

            /** \brief Get a measure of the space (this can be thought of as a generalization of volume) */
            virtual double getMeasure() const = 0;

            /** \brief Bring the state within the bounds of the state space. For unbounded spaces this
                function can be a no-op. */
            virtual void enforceBounds(State *state) const = 0;

            /** \brief Check if a state is inside the bounding box. For unbounded spaces this function
                can always return true. */
            virtual bool satisfiesBounds(const State *state) const = 0;

            /** \brief Copy a state to another. The memory of source and destination should NOT overlap.
                \note For more advanced state copying methods (partial copy, for example), see \ref advancedStateCopy. */
            virtual void copyState(State *destination, const State *source) const = 0;

            /** \brief Computes distance between two states. This function satisfies the properties of a
                metric if isMetricSpace() is true, and its return value will always be between 0 and getMaximumExtent() */
            virtual double distance(const State *state1, const State *state2) const = 0;

            /** \brief Get the number of chars in the serialization of a state in this space */
            virtual unsigned int getSerializationLength() const;

            /** \brief Write the binary representation of \e state to \e serialization */
            virtual void serialize(void *serialization, const State *state) const;

            /** \brief Read the binary representation of a state from \e serialization and write it to \e state */
            virtual void deserialize(State *state, const void *serialization) const;

            /** \brief Checks whether two states are equal */
            virtual bool equalStates(const State *state1, const State *state2) const = 0;

            /** \brief Computes the state that lies at time @e t in [0, 1] on the segment that connects @e from state to @e to state.
                The memory location of @e state is not required to be different from the memory of either
                @e from or @e to. */
            virtual void interpolate(const State *from, const State *to, const double t, State *state) const = 0;

            /** \brief Allocate an instance of the default uniform state sampler for this space */
            virtual StateSamplerPtr allocDefaultStateSampler() const = 0;

            /** \brief Allocate an instance of the state sampler for this space. This sampler will be allocated with the
                sampler allocator that was previously specified by setStateSamplerAllocator() or, if no sampler allocator was specified,
                allocDefaultStateSampler() is called */
            virtual StateSamplerPtr allocStateSampler() const;

            /** \brief Set the sampler allocator to use */
            void setStateSamplerAllocator(const StateSamplerAllocator &ssa);

            /** \brief Clear the state sampler allocator (reset to default) */
            void clearStateSamplerAllocator();

            /** \brief Allocate a state that can store a point in the described space */
            virtual State* allocState() const = 0;

            /** \brief Free the memory of the allocated state */
            virtual void freeState(State *state) const = 0;

            /** @} */

            /** @name Functionality specific to accessing real values in a state
                @{ */

            /** \brief Many states contain a number of double values. This function provides a means to get the
                memory address of a double value from state \e state located at position \e index. The first double value
                is returned for \e index = 0. If \e index is too large (does not point to any double values in the state),
                the return value is NULL.

                \note This function does @b not map a state to an
                array of doubles. There may be components of a state
                that do not correspond to double values and they are
                'invisible' to this function. Furthermore, this
                function is @b slow and is not intended for use in the
                implementation of planners. Ideally, state values should not be accessed by index. If accessing of individual state elements
                is however needed, getValueAddressAtLocation() provides a faster implementation. */
            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

            /** \brief Const variant of the same function as above; */
            const double* getValueAddressAtIndex(const State *state, const unsigned int index) const;

            /** \brief Get the locations of values of type double contained in a state from this space. The order of the values is
                consistent with getValueAddressAtIndex(). The setup() function must have been previously called. */
            const std::vector<ValueLocation>& getValueLocations() const;

            /** \brief Get the named locations of values of type double contained in a state from this space.
                The setup() function must have been previously called. */
            const std::map<std::string, ValueLocation>& getValueLocationsByName() const;

            /** \brief Get a pointer to the double value in \e state that \e loc points to */
            double* getValueAddressAtLocation(State *state, const ValueLocation &loc) const;

            /** \brief Const variant of the same function as above; */
            const double* getValueAddressAtLocation(const State *state, const ValueLocation &loc) const;

            /** \brief Get a pointer to the double value in \e state that \e name points to */
            double* getValueAddressAtName(State *state, const std::string &name) const;

            /** \brief Const variant of the same function as above; */
            const double* getValueAddressAtName(const State *state, const std::string &name) const;

            /** \brief Copy all the real values from a state \e source to the array \e reals using getValueAddressAtLocation() */
            void copyToReals(std::vector<double> &reals, const State *source) const;

            /** \brief Copy the values from \e reals to the state \e destination using getValueAddressAtLocation() */
            void copyFromReals(State *destination, const std::vector<double> &reals) const;

            /** @} */

            /** @name Management of projections from this state space to Euclidean spaces
                @{ */

            /** \brief Register a projection for this state space under a specified name */
            void registerProjection(const std::string &name, const ProjectionEvaluatorPtr &projection);

            /** \brief Register the default projection for this state space */
            void registerDefaultProjection(const ProjectionEvaluatorPtr &projection);

            /** \brief Register the projections for this state space. Usually, this is at least the default
                projection. These are implicit projections, set by the implementation of the state space. This is called by setup(). */
            virtual void registerProjections();

            /** \brief Get the projection registered under a specific name */
            ProjectionEvaluatorPtr getProjection(const std::string &name) const;

            /** \brief Get the default projection */
            ProjectionEvaluatorPtr getDefaultProjection() const;

            /** \brief Check if a projection with a specified name is available */
            bool hasProjection(const std::string &name) const;

            /** \brief Check if a default projection is available */
            bool hasDefaultProjection() const;

            /** \brief Get all the registered projections */
            const std::map<std::string, ProjectionEvaluatorPtr>& getRegisteredProjections() const;

            /** @} */

            /** @name Debugging tools
                @{ */

            /** \brief Print a state to a stream */
            virtual void printState(const State *state, std::ostream &out) const;

            /** \brief Print the settings for this state space to a stream */
            virtual void printSettings(std::ostream &out) const;

            /** \brief Print the list of registered projections. This function is also called by printSettings() */
            virtual void printProjections(std::ostream &out) const;

            /** \brief Perform sanity checks for this state space. Throws an exception if failures are found.
                \note This checks if distances are always positive, whether the integration works as expected, etc. */
            virtual void sanityChecks(double zero, double eps, unsigned int flags) const;

            /** \brief Convenience function that allows derived state spaces to choose which checks
                should pass (see SanityChecks flags) and how strict the checks are. This just calls sanityChecks() with some default arguments. */
            virtual void sanityChecks() const;

            /** \brief Print a Graphviz digraph that represents the containment diagram for the state space */
            void diagram(std::ostream &out) const;

            /** \brief Print the list of all contained state space instances */
            void list(std::ostream &out) const;

            /** \brief Print a Graphviz digraph that represents the containment diagram for all the instantiated state spaces */
            static void Diagram(std::ostream &out);

            /** \brief Print the list of available state space instances */
            static void List(std::ostream &out);

            /** @} */

            /** @name Operations with substates
                @{ */

            /** \brief Allocate a sampler that actually samples only components that are part of \e subspace */
            StateSamplerPtr allocSubspaceStateSampler(const StateSpacePtr &subspace) const;

            /** \brief Allocate a sampler that actually samples only components that are part of \e subspace */
            virtual StateSamplerPtr allocSubspaceStateSampler(const StateSpace *subspace) const;

            /** \brief Get the substate of \e state that is pointed to by \e loc */
            State* getSubstateAtLocation(State *state, const SubstateLocation &loc) const;

            /** \brief Get the substate of \e state that is pointed to by \e loc */
            const State* getSubstateAtLocation(const State *state, const SubstateLocation &loc) const;

            /** \brief Get the list of known substate locations (keys of the map corrspond to names of subspaces) */
            const std::map<std::string, SubstateLocation>& getSubstateLocationsByName() const;

            /** \brief Get the set of subspaces that this space and \e other have in common. The computed list of \e subspaces does
                not contain spaces that cover each other, even though they may be common, as that is redundant information. */
            void getCommonSubspaces(const StateSpacePtr &other, std::vector<std::string> &subspaces) const;

            /** \brief Get the set of subspaces that this space and \e other have in common. The computed list of \e subspaces does
                not contain spaces that cover each other, even though they may be common, as that is redundant information. */
            void getCommonSubspaces(const StateSpace *other, std::vector<std::string> &subspaces) const;

            /** \brief Compute the location information for various components of the state space. Either this function or setup() must be
                called before any calls to getValueAddressAtName(), getValueAddressAtLocation() (and other functions where those are used). */
            virtual void computeLocations();

            /** @} */

            /** \brief Perform final setup steps. This function is
                automatically called by the SpaceInformation. If any
                default projections are to be registered, this call
                will set them and call their setup() functions. It is
                safe to call this function multiple times. At a
                subsequent call, projections that have been previously
                user configured are not re-instantiated, but their
                setup() method is still called. */
            virtual void setup();

        protected:

            /** \brief The name used for the default projection */
            static const std::string DEFAULT_PROJECTION_NAME;

            /** \brief A type assigned for this state space */
            int                                           type_;

            /** \brief An optional state sampler allocator */
            StateSamplerAllocator                         ssa_;

            /** \brief The extent of this space at the time setup() was called */
            double                                        maxExtent_;

            /** \brief The fraction of the longest valid segment */
            double                                        longestValidSegmentFraction_;

            /** \brief The longest valid segment at the time setup() was called */
            double                                        longestValidSegment_;

            /** \brief The factor to multiply the value returned by validSegmentCount(). Rarely used but useful for things like doubling the resolution */
            unsigned int                                  longestValidSegmentCountFactor_;

            /** \brief List of available projections */
            std::map<std::string, ProjectionEvaluatorPtr> projections_;

            /** \brief The set of parameters for this space */
            ParamSet                                      params_;

            /** \brief The value locations for all varliables of type double contained in a state;
                The locations point to values in the same order as that returned by getValueAddressAtIndex() */
            std::vector<ValueLocation>                    valueLocationsInOrder_;

            /** \brief All the known value locations, by name. The names of state spaces access the first element of a state.
                RealVectorStateSpace dimensions are used to access individual dimensions. */
            std::map<std::string, ValueLocation>          valueLocationsByName_;

            /** \brief All the known substat locations, by name. */
            std::map<std::string, SubstateLocation>       substateLocationsByName_;

        private:

            /** \brief State space name */
            std::string                                   name_;
        };

        /** \brief A space to allow the composition of state spaces */
        class CompoundStateSpace : public StateSpace
        {
        public:

            /** \brief Define the type of state allocated by this state space */
            typedef CompoundState StateType;

            /** \brief Construct an empty compound state space */
            CompoundStateSpace();

            /** \brief Construct a compound state space from a list of subspaces (\e components) and their corresponding weights (\e weights) */
            CompoundStateSpace(const std::vector<StateSpacePtr> &components, const std::vector<double> &weights);

            virtual ~CompoundStateSpace()
            {
            }

            /** \brief Cast a component of this instance to a desired type. */
            template<class T>
            T* as(const unsigned int index) const
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateSpace*>));

                return static_cast<T*>(getSubspace(index).get());
            }

            /** \brief Cast a component of this instance to a desired type. */
            template<class T>
            T* as(const std::string &name) const
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateSpace*>));

                return static_cast<T*>(getSubspace(name).get());
            }

            virtual bool isCompound() const;

            virtual bool isHybrid() const;

            /** @name Management of contained subspaces
                @{ */

            /** \brief Adds a new state space as part of the compound state space. For computing distances within the compound
                state space, the weight of the component also needs to be specified. */
            void addSubspace(const StateSpacePtr &component, double weight);

            /** \brief Get the number of state spaces that make up the compound state space */
            unsigned int getSubspaceCount() const;

            /** \brief Get a specific subspace from the compound state space */
            const StateSpacePtr& getSubspace(const unsigned int index) const;

            /** \brief Get a specific subspace from the compound state space */
            const StateSpacePtr& getSubspace(const std::string& name) const;

            /** \brief Get the index of a specific subspace from the compound state space */
            unsigned int getSubspaceIndex(const std::string& name) const;

            /** \brief Check if a specific subspace is contained in this state space */
            bool hasSubspace(const std::string &name) const;

            /** \brief Get the weight of a subspace from the compound state space (used in distance computation) */
            double getSubspaceWeight(const unsigned int index) const;

            /** \brief Get the weight of a subspace from the compound state space (used in distance computation) */
            double getSubspaceWeight(const std::string &name) const;

            /** \brief Set the weight of a subspace in the compound state space (used in distance computation) */
            void setSubspaceWeight(const unsigned int index, double weight);

            /** \brief Set the weight of a subspace in the compound state space (used in distance computation) */
            void setSubspaceWeight(const std::string &name, double weight);

            /** \brief Get the list of components */
            const std::vector<StateSpacePtr>& getSubspaces() const;

            /** \brief Get the list of component weights */
            const std::vector<double>& getSubspaceWeights() const;

            /** \brief Return true if the state space is locked. A value
                of true means that no further spaces can be added
                as components. */
            bool isLocked() const;

            /** \brief Lock this state space. This means no further
                spaces can be added as components.  This function can
                be for instance called from the constructor of a
                state space that inherits from CompoundStateSpace to
                prevent the user to add further components. */
            void lock();
            /** @} */

            /** @name Operations with substates
                @{ */

            virtual StateSamplerPtr allocSubspaceStateSampler(const StateSpace *subspace) const;

            /** @} */

            /** @name Functionality specific to the state space
                @{ */

            virtual unsigned int getDimension() const;

            virtual double getMaximumExtent() const;

            virtual double getMeasure() const;

            virtual void enforceBounds(State *state) const;

            virtual bool satisfiesBounds(const State *state) const;

            virtual void copyState(State *destination, const State *source) const;

            virtual unsigned int getSerializationLength() const;

            virtual void serialize(void *serialization, const State *state) const;

            virtual void deserialize(State *state, const void *serialization) const;

            virtual double distance(const State *state1, const State *state2) const;

            /** \brief When performing discrete validation of motions,
                the length of the longest segment that does not
                require state validation needs to be specified. This
                function sets this length as a fraction of the space's
                maximum extent. The call is passed to all contained subspaces */
            virtual void setLongestValidSegmentFraction(double segmentFraction);

            /** \brief Count how many segments of the "longest valid length" fit on the motion from \e state1 to \e state2.
                This is the max() of the counts returned by contained subspaces. */
            virtual unsigned int validSegmentCount(const State *state1, const State *state2) const;

            virtual bool equalStates(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual State* allocState() const;

            virtual void freeState(State *state) const;

            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

            /** @} */

            virtual void printState(const State *state, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void computeLocations();

            virtual void setup();

        protected:

            /** \brief Allocate the state components. Called by allocState(). Usually called by derived state spaces. */
            void allocStateComponents(CompoundState *state) const;

            /** \brief The state spaces that make up the compound state space */
            std::vector<StateSpacePtr>    components_;

            /** \brief The number of components */
            unsigned int                  componentCount_;

            /** \brief The weight assigned to each component of the state space when computing the compound distance */
            std::vector<double>           weights_;

            /** \brief The sum of all the weights in \e weights_ */
            double                        weightSum_;

            /** \brief Flag indicating whether adding further components is allowed or not */
            bool                          locked_;

        };

        /** \addtogroup stateAndSpaceOperators
         *  @{
         */

        /** \brief Construct a compound state space from two existing
            state spaces. The components of this compound space are \e
            a (or the components of \e a, if \e a is compound) and \e
            b (or the components of \e b, if \e b is compound).
            State spaces are identified by name. Duplicates are checked
            for and added only once. If the compound state space would
            end up containing solely one component, that component is returned
            instead. */
        StateSpacePtr operator+(const StateSpacePtr &a, const StateSpacePtr &b);

        /** \brief Construct a compound state space that contains
            subspaces only from \e a. If \e a is compound, \e b (or
            the components from \e b, if \e b is compound) are removed
            and the remaining components are returned as a compound
            state space. If the compound space would end up containing solely
            one component, that component is returned instead. */
        StateSpacePtr operator-(const StateSpacePtr &a, const StateSpacePtr &b);

        /** \brief Construct a compound state space that contains
            subspaces only from \e a, except for maybe the one named \e name */
        StateSpacePtr operator-(const StateSpacePtr &a, const std::string &name);

        /** \brief Construct a compound state space that contains
            subspaces that are in both \e a and \e b */
        StateSpacePtr operator*(const StateSpacePtr &a, const StateSpacePtr &b);
        /** @} */


        /** \defgroup advancedStateCopy Advanced methods for copying states
         *  @{
         */

        /** \brief The possible outputs for an advanced copy operation */
        enum AdvancedStateCopyOperation
            {
                /** \brief No data was copied */
                NO_DATA_COPIED = 0,

                /** \brief Some data was copied */
                SOME_DATA_COPIED = 1,

                /** \brief All data was copied */
                ALL_DATA_COPIED  = 2
            };

        /** \brief Copy data from \e source (state from space \e
            sourceS) to \e dest (state from space \e destS) on a
            component by component basis. State spaces are matched by
            name. If the state space \e destS contains any subspace
            whose name matches any subspace of the state space \e
            sourceS, the corresponding state components are
            copied. */
        AdvancedStateCopyOperation copyStateData(const StateSpacePtr &destS, State *dest,
                                                 const StateSpacePtr &sourceS, const State *source);

        /** \brief Copy data from \e source (state from space \e
            sourceS) to \e dest (state from space \e destS) on a
            component by component basis. State spaces are matched by
            name. If the state space \e destS contains any subspace
            whose name matches any subspace of the state space \e
            sourceS, the corresponding state components are
            copied. */
        AdvancedStateCopyOperation copyStateData(const StateSpace *destS, State *dest,
                                                 const StateSpace *sourceS, const State *source);

        /** \brief Copy data from \e source (state from space \e
            sourceS) to \e dest (state from space \e destS) but only
            for the subspaces indicated by name in \e subspaces. This
            uses StateSpace::getSubstateLocationsByName().
            \note For efficiency reasons it is a good idea usually to make sure the elements of \e subspaces are not subspaces of each other */
        AdvancedStateCopyOperation copyStateData(const StateSpacePtr &destS, State *dest,
                                                 const StateSpacePtr &sourceS, const State *source,
                                                 const std::vector<std::string> &subspaces);

        /** \brief Copy data from \e source (state from space \e
            sourceS) to \e dest (state from space \e destS) but only
            for the subspaces indicated by name in \e subspaces. This
            uses StateSpace::getSubstateLocationsByName().
            \note For efficiency reasons it is a good idea usually to make sure the elements of \e subspaces are not subspaces of each other */
        AdvancedStateCopyOperation copyStateData(const StateSpace *destS, State *dest,
                                                 const StateSpace *sourceS, const State *source,
                                                 const std::vector<std::string> &subspaces);
        /** @} */

    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage
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

#ifndef OMPL_BASE_STATE_STORAGE_
#define OMPL_BASE_STATE_STORAGE_

#include "ompl/base/StateSpace.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/function.hpp>
#include <iostream>

namespace ompl
{
    namespace base
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::StateStorage */
        OMPL_CLASS_FORWARD(StateStorage);
        /// @endcond

        /** \brief Manage loading and storing for a set of states of a specified state space

            \deprecated This class is deprecated and will be removed in the future. Please use the improved PlannerDataStorage.
         */
        class StateStorage
        {
        public:

            /** \brief The state space to store states for is specified as argument */
            StateStorage(const StateSpacePtr &space);
            virtual ~StateStorage();

            /** \brief Get the state space this class maintains states for */
            const StateSpacePtr& getStateSpace() const
            {
                return space_;
            }

            /** \brief Load a set of states from a specified file */
            void load(const char *filename);

            /** \brief Load a set of states from a stream */
            virtual void load(std::istream &in);

            /** \brief Save a set of states to a file */
            void store(const char *filename);

            /** \brief Save a set of states to a stream */
            virtual void store(std::ostream &out);

            /** \brief Add a state to the set of states maintained by
                this storage structure. The state is copied to internal storage */
            virtual void addState(const State *state);

            /** \brief Generate \e count states uniformly at random and store them in this structure */
            virtual void generateSamples(unsigned int count);

            /** \brief Clear the stored states. This frees all the memory */
            virtual void clear();

            /** \brief Return the number of stored states */
            std::size_t size() const
            {
                return states_.size();
            }

            /** \brief Get the stored states */
            const std::vector<const State*>& getStates() const
            {
                return states_;
            }

            /** \brief Get a particular state for non-const access */
            State* getState(unsigned int index)
            {
                assert(states_.size() > index);
                return const_cast<State*>(states_[index]);
            }

            /** \brief Get a particular state */
            const State* getState(unsigned int index) const
            {
                assert(states_.size() > index);
                return states_[index];
            }

            /** \brief Return a flag that indicates whether there is metadata associated to the states in this storage */
            bool hasMetadata() const
            {
                return hasMetadata_;
            }

            /** \brief Sort the states according to the less-equal operator \e op. Metadata is NOT sorted;
                if metadata was added, the index values of the metadata will not match after the sort. */
            void sort(const boost::function<bool(const State*, const State*)> &op);

            /** \brief Get a sampler allocator to a sampler that can be specified for a StateSpace, such that all sampled
                states are actually from this storage structure. */
            StateSamplerAllocator getStateSamplerAllocator() const;

            /** \brief Get a sampler allocator to a sampler that can be specified for a StateSpace, such that all sampled
                states are actually from this storage structure at an index less than or equal to \e until */
            StateSamplerAllocator getStateSamplerAllocatorRangeUntil(std::size_t until) const;

            /** \brief Get a sampler allocator to a sampler that can be specified for a StateSpace, such that all sampled
                states are actually from this storage structure at an index above or equal to \e after */
            StateSamplerAllocator getStateSamplerAllocatorRangeAfter(std::size_t after) const;

            /** \brief Get a sampler allocator to a sampler that can be specified for a StateSpace, such that all sampled
                states are actually from this storage structure at an index in the range [\e from, \e to] (inclusive) */
            virtual StateSamplerAllocator getStateSamplerAllocatorRange(std::size_t from, std::size_t to) const;

            /** \brief Output the set of states to a specified stream, in a human readable fashion */
            virtual void print(std::ostream &out = std::cout) const;

        protected:

            /** \brief Information stored at the beginning of the archive */
            struct Header
            {
                /** \brief OMPL specific marker (fixed value) */
                boost::uint32_t  marker;

                /** \brief Number of states stored in the archive */
                std::size_t      state_count;

                /** \brief Signature of state space that allocated the saved states (see ompl::base::StateSpace::computeSignature()) */
                std::vector<int> signature;

                /** \brief boost::serialization routine */
                template<typename Archive>
                void serialize(Archive & ar, const unsigned int /*version*/)
                {
                    ar & marker;
                    ar & state_count;
                    ar & signature;
                }
            };

            /** \brief Load the states from a binary archive \e ia, given the loaded header is \e h */
            virtual void loadStates(const Header &h, boost::archive::binary_iarchive &ia);

            /** \brief Load the state metadata from a binary archive
                \e ia, given the loaded header is \e h. No metadata is
                actually loaded unless the StateStorageWithMetadata
                class is used.*/
            virtual void loadMetadata(const Header &h, boost::archive::binary_iarchive &ia);

            /** \brief Store the states to a binary archive \e oa, given the stored header is \e h */
            virtual void storeStates(const Header &h, boost::archive::binary_oarchive &oa);

            /** \brief Save the state metadata to a binary archive
                \e oa, given the stored header is \e h. No metadata is
                actually saved unless the StateStorageWithMetadata
                class is used.*/
            virtual void storeMetadata(const Header &h, boost::archive::binary_oarchive &oa);

            /** \brief Free the memory allocated for states */
            void freeMemory();

            /** \brief State space that corresponds to maintained states */
            StateSpacePtr             space_;

            /** \brief The list of maintained states */
            std::vector<const State*> states_;

            /** \brief Flag indicating whether there is metadata associated to the states in this storage */
            bool                      hasMetadata_;
        };

        /** \brief State storage that allows storing state metadata as well
            \tparam M the datatype for the stored metadata. boost::serialization operation needs to be defined */
        template<typename M>
        class StateStorageWithMetadata : public StateStorage
        {
        public:

            /** \brief the datatype of the metadata */
            typedef M MetadataType;

            /** \brief The state space to store states for is specified as argument */
            StateStorageWithMetadata(const StateSpacePtr &space) : StateStorage(space)
            {
                hasMetadata_ = true;
            }

            /** \brief Add a state to the set of states maintained by
                this storage structure. The state is copied to
                internal storage and metadata with default values is stored as well. */
            virtual void addState(const State *state)
            {
                addState(state, M());
            }

            /** \brief Add a state to the set of states maintained by
                this storage structure. The state is copied to internal storage. Corresponding metadata is stored too. */
            virtual void addState(const State *state, const M& metadata)
            {
                StateStorage::addState(state);
                metadata_.push_back(metadata);
            }

            virtual void clear()
            {
                StateStorage::clear();
                metadata_.clear();
            }

            /** \brief Get const access to the metadata of a state at a particular index */
            const M& getMetadata(unsigned int index) const
            {
                assert(metadata_.size() > index);
                return metadata_[index];
            }

            /** \brief Get write access to the metadata of a state at a particular index */
            M& getMetadata(unsigned int index)
            {
                assert(metadata_.size() > index);
                return metadata_[index];
            }

        protected:

            virtual void loadMetadata(const Header& /*h*/, boost::archive::binary_iarchive &ia)
            {
                ia >> metadata_;
            }

            virtual void storeMetadata(const Header& /*h*/, boost::archive::binary_oarchive &oa)
            {
                oa << metadata_;
            }

            /** \brief The metadata for each state */
            std::vector<M> metadata_;

        };

        /** \brief Storage of states where the metadata is a vector of indices. This is is typically used to store a graph */
        typedef StateStorageWithMetadata<std::vector<std::size_t> > GraphStateStorage;
        typedef boost::shared_ptr<GraphStateStorage> GraphStateStoragePtr;
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

#ifndef OMPL_BASE_STATE_VALIDITY_CHECKER_
#define OMPL_BASE_STATE_VALIDITY_CHECKER_

#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{

    namespace base
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::StateValidityChecker */
        OMPL_CLASS_FORWARD(StateValidityChecker);
        /// @endcond

        /** \class ompl::base::StateValidityCheckerPtr
            \brief A boost shared pointer wrapper for ompl::base::StateValidityChecker */

        /** \brief Properties that a state validity checker may have */
        struct StateValidityCheckerSpecs
        {
	    /** \brief Specify the type of clearance computation */
            enum ClearanceComputationType
            {
                /// Clearance computation is not implemented.
                NONE = 0,
                /// Exact clearance computation is available.
                EXACT,
                /// Some approximation of clearance is computed, but it is not clear if this is above or below the exact clearance.
                APPROXIMATE,
                /// A lower bound on clearance is computed.
                BOUNDED_APPROXIMATE,
            };

            StateValidityCheckerSpecs() : clearanceComputationType(NONE), hasValidDirectionComputation(false)
            {
            }

            /** \brief Value indicating the kind of clearance computation this
                StateValidityChecker can compute (if any). */
            ClearanceComputationType clearanceComputationType;

            /** \brief Flag indicating that this state validity checker can return
                a direction that moves a state away from being invalid. */
            bool                     hasValidDirectionComputation;
        };

        /** \brief Abstract definition for a class checking the
            validity of states. The implementation of this class must
            be thread safe. */
        class StateValidityChecker
        {
        public:

            /** \brief Constructor */
            StateValidityChecker(SpaceInformation *si) : si_(si)
            {
            }

            /** \brief Constructor */
            StateValidityChecker(const SpaceInformationPtr &si) : si_(si.get())
            {
            }

            virtual ~StateValidityChecker()
            {
            }

            /** \brief Return true if the state \e state is valid. Usually, this means at least collision checking. If it is
                possible that ompl::base::StateSpace::interpolate() or ompl::control::ControlSpace::propagate() return states that
                are outside of bounds, this function should also make a call to ompl::base::SpaceInformation::satisfiesBounds(). */
            virtual bool isValid(const State *state) const = 0;

            /** \brief Return true if the state \e state is valid. In addition, set \e dist to the distance to the nearest invalid state. */
            virtual bool isValid(const State *state, double &dist) const
            {
                dist = clearance(state);
                return isValid(state);
            }

            /** \brief Return true if the state \e state is valid. In addition, set \e dist to the distance to the nearest
                invalid state (using clearance()). If a direction that moves \e state away from being invalid is available,
                a valid state in that direction is also set (\e validState). \e validStateAvailable is set to true if \e validState
                is updated. */
            virtual bool isValid(const State *state, double &dist, State *validState, bool &validStateAvailable) const
            {
                dist = clearance(state, validState, validStateAvailable);
                return isValid(state);
            }

            /** \brief Report the distance to the nearest invalid state when starting from \e state. If the distance is
                negative, the value of clearance is the penetration depth.*/
            virtual double clearance(const State* /*state*/) const
            {
                return 0.0;
            }

            /** \brief Report the distance to the nearest invalid state when starting from \e state, and if possible,
                also specify a valid state \e validState in the direction that moves away from the colliding
                state. The \e validStateAvailable flag is set to true if \e validState is updated. */
            virtual double clearance(const State *state, State* /*validState*/, bool &validStateAvailable) const
            {
                validStateAvailable = false;
                return clearance(state);
            }

            /** \brief Return the specifications (capabilities of this state validity checker) */
            const StateValidityCheckerSpecs& getSpecs() const
            {
                return specs_;
            }

        protected:

            /** \brief The instance of space information this state validity checker operates on */
            SpaceInformation         *si_;

            /** \brief The specifications of the state validity checker (its capabilities) */
            StateValidityCheckerSpecs specs_;

        };

        /** \brief The simplest state validity checker: all states are valid */
        class AllValidStateValidityChecker : public StateValidityChecker
        {
        public:

            /** \brief Constructor */
            AllValidStateValidityChecker(SpaceInformation *si) : StateValidityChecker(si)
            {
            }

            /** \brief Constructor */
            AllValidStateValidityChecker(const SpaceInformationPtr &si) : StateValidityChecker(si)
            {
            }

            /** \brief Always return true (all states are considered valid) */
            virtual bool isValid(const State * /* state */ ) const
            {
                return true;
            }
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

#ifndef OMPL_BASE_MOTION_VALIDATOR_
#define OMPL_BASE_MOTION_VALIDATOR_

#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"
#include <utility>

namespace ompl
{

    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::MotionValidator */
        OMPL_CLASS_FORWARD(MotionValidator);
        /// @endcond

        /** \class ompl::base::MotionValidatorPtr
            \brief A boost shared pointer wrapper for ompl::base::MotionValidator */

        /** \brief Abstract definition for a class checking the
            validity of motions -- path segments between states. This
            is often called a local planner. The implementation of
            this class must be thread safe. */
        class MotionValidator
        {
        public:

            /** \brief Constructor */
            MotionValidator(SpaceInformation *si) : si_(si), valid_(0), invalid_(0)
            {
            }

            /** \brief Constructor */
            MotionValidator(const SpaceInformationPtr &si) : si_(si.get()), valid_(0), invalid_(0)
            {
            }

            virtual ~MotionValidator()
            {
            }

            /** \brief Check if the path between two states (from \e s1 to \e s2) is valid. This function assumes \e s1 is valid.

                \note This function updates the number of valid and invalid segments. */
            virtual bool checkMotion(const State *s1, const State *s2) const = 0;

            /** \brief Check if the path between two states is valid. Also compute the last state that was
                valid and the time of that state. The time is used to parametrize the motion from \e s1 to \e s2, \e s1 being at t =
                0 and \e s2 being at t = 1. This function assumes \e s1 is valid.
                \param s1 start state of the motion to be checked (assumed to be valid)
                \param s2 final state of the motion to be checked
                \param lastValid first: storage for the last valid state (may be NULL, if the user does not care about the exact state); this need not be different from \e s1 or \e s2. second: the time (between 0 and 1) of the last valid state, on the motion from \e s1 to \e s2. If the function returns false, \e lastValid.first must be set to a valid state, even if that implies copying \e s1 to \e lastValid.first (in case \e lastValid.second = 0). If the function returns true, \e lastValid.first and \e lastValid.second should \b not be modified.

                \note This function updates the number of valid and invalid segments. */
            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const = 0;

            /** \brief Get the number of segments that tested as valid */
            unsigned int getValidMotionCount() const
            {
                return valid_;
            }

            /** \brief Get the number of segments that tested as invalid */
            unsigned int getInvalidMotionCount() const
            {
                return invalid_;
            }

            /** \brief Get the fraction of segments that tested as valid */
            double getValidMotionFraction() const
            {
                return valid_ == 0 ? 0.0 : (double)valid_ / (double)(invalid_ + valid_);
            }

            /** \brief Reset the counters for valid and invalid segments */
            void resetMotionCounter()
            {
                valid_ = invalid_ = 0;
            }

        protected:

            /** \brief The instance of space information this state validity checker operates on */
            SpaceInformation    *si_;

            /** \brief Number of valid segments */
            mutable unsigned int valid_;

            /** \brief Number of invalid segments */
            mutable unsigned int invalid_;

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

#ifndef OMPL_BASE_SPACE_INFORMATION_
#define OMPL_BASE_SPACE_INFORMATION_

#include "ompl/base/State.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/ValidStateSampler.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <utility>
#include <cstdlib>
#include <vector>
#include <iostream>

/** \brief Main namespace. Contains everything in this library */
namespace ompl
{

    /** \brief This namespace contains sampling based planning
        routines shared by both planning under geometric constraints
        (geometric) and planning under differential constraints
        (dynamic) */
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::SpaceInformation */
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /** \class ompl::base::SpaceInformationPtr
            \brief A boost shared pointer wrapper for ompl::base::SpaceInformation */

        /** \brief If no state validity checking class is specified
            (StateValidityChecker), a boost function can be specified
            instead */
        typedef boost::function<bool(const State*)> StateValidityCheckerFn;


        /** \brief The base class for space information. This contains
            all the information about the space planning is done in.
            setup() needs to be called as well, before use */
        class SpaceInformation : private boost::noncopyable
        {
        public:

            /** \brief Constructor. Sets the instance of the state space to plan with. */
            SpaceInformation(const StateSpacePtr &space);

            virtual ~SpaceInformation()
            {
            }

            /** \brief Check if a given state is valid or not */
            bool isValid(const State *state) const
            {
                return stateValidityChecker_->isValid(state);
            }

            /** \brief Return the instance of the used state space */
            const StateSpacePtr& getStateSpace() const
            {
                return stateSpace_;
            }

            /** @name Topology-specific state operations (as in the state space)
                @{ */

            /** \brief Check if two states are the same */
            bool equalStates(const State *state1, const State *state2) const
            {
                return stateSpace_->equalStates(state1, state2);
            }

            /** \brief Check if a state is inside the bounding box */
            bool satisfiesBounds(const State *state) const
            {
                return stateSpace_->satisfiesBounds(state);
            }

            /** \brief Compute the distance between two states */
            double distance(const State *state1, const State *state2) const
            {
                return stateSpace_->distance(state1, state2);
            }

            /** \brief Bring the state within the bounds of the state space */
            void enforceBounds(State *state) const
            {
                stateSpace_->enforceBounds(state);
            }

            /** \brief Print a state to a stream */
            void printState(const State *state, std::ostream &out = std::cout) const
            {
                stateSpace_->printState(state, out);
            }

            /** @} */

            /** @name Configuration of state validity checking
                @{ */

            /** \brief Set the instance of the state validity checker
                to use. Parallel implementations of planners assume
                this validity checker is thread safe. */
            void setStateValidityChecker(const StateValidityCheckerPtr &svc)
            {
                stateValidityChecker_ = svc;
                setup_ = false;
            }

            /** \brief If no state validity checking class is
                specified (StateValidityChecker), a boost function can
                be specified instead. This version however incurs a
                small additional overhead when calling the function,
                since there is one more level of indirection */
            void setStateValidityChecker(const StateValidityCheckerFn &svc);

            /** \brief Return the instance of the used state validity checker */
            const StateValidityCheckerPtr& getStateValidityChecker() const
            {
                return stateValidityChecker_;
            }

            /** \brief Set the instance of the motion validity checker
                to use. Parallel implementations of planners assume
                this validity checker is thread safe.  */
            void setMotionValidator(const MotionValidatorPtr &mv)
            {
                motionValidator_ = mv;
                setup_ = false;
            }

            /** \brief Return the instance of the used state validity checker */
            const MotionValidatorPtr& getMotionValidator() const
            {
                return motionValidator_;
            }

            /** \brief Set the resolution at which state validity
                needs to be verified in order for a motion between two
                states to be considered valid. This value is specified
                as a fraction of the space's extent. This call is only
                applicable if a ompl::base::DiscreteMotionValidator is
                used. See \ref stateValidation. */
            void setStateValidityCheckingResolution(double resolution)
            {
                stateSpace_->setLongestValidSegmentFraction(resolution);
                setup_ = false;
            }

            /** \brief Get the resolution at which state validity is
                verified. This call is only applicable if a
                ompl::base::DiscreteMotionValidator is used. See \ref
                stateValidation. */
            double getStateValidityCheckingResolution() const
            {
                return stateSpace_->getLongestValidSegmentFraction();
            }


            /** @}*/

            /** \brief Return the dimension of the state space */
            unsigned int getStateDimension() const
            {
                return stateSpace_->getDimension();
            }

            /** @name State memory management
                @{ */

            /** \brief Allocate memory for a state */
            State* allocState() const
            {
                return stateSpace_->allocState();
            }

            /** \brief Allocate memory for each element of the array \e states */
            void allocStates(std::vector<State*> &states) const
            {
                for (unsigned int i = 0 ; i < states.size() ; ++i)
                    states[i] = stateSpace_->allocState();
            }

            /** \brief Free the memory of a state */
            void freeState(State *state) const
            {
                stateSpace_->freeState(state);
            }

            /** \brief Free the memory of an array of states */
            void freeStates(std::vector<State*> &states) const
            {
                for (unsigned int i = 0 ; i < states.size() ; ++i)
                    stateSpace_->freeState(states[i]);
            }

            /** \brief Copy a state to another */
            void copyState(State *destination, const State *source) const
            {
                stateSpace_->copyState(destination, source);
            }

            /** \brief Clone a state */
            State* cloneState(const State *source) const
            {
                State *copy = stateSpace_->allocState();
                stateSpace_->copyState(copy, source);
                return copy;
            }

            /**  @} */


            /** @name Sampling of valid states
                @{ */

            /** \brief Allocate a uniform state sampler for the state space */
            StateSamplerPtr allocStateSampler() const
            {
                return stateSpace_->allocStateSampler();
            }

            /** \brief Allocate an instance of a valid state sampler for this space. If setValidStateSamplerAllocator() was previously called,
                the specified allocator is used to produce the state sampler.  Otherwise, a ompl::base::UniformValidStateSampler() is
                allocated. */
            ValidStateSamplerPtr allocValidStateSampler() const;

            /** \brief Set the allocator to use for a valid state sampler. This replaces the default uniform valid state
                sampler. This call can be made at any time, but it should not be changed while ompl::base::Planner::solve() is executing */
            void setValidStateSamplerAllocator(const ValidStateSamplerAllocator &vssa);

            /** \brief Clear the allocator used for the valid state sampler. This will revert to using the uniform valid state sampler (the default). */
            void clearValidStateSamplerAllocator();

            /** @}*/

            /** @name Primitives typically used by motion planners
                @{ */

            /** \brief Get the maximum extent of the space we are
                planning in. This is the maximum distance that could
                be reported between any two given states */
            double getMaximumExtent() const
            {
                return stateSpace_->getMaximumExtent();
            }

            /** \brief Find a valid state near a given one. If the given state is valid, it will be returned itself.
             *  The two passed state pointers need not point to different memory. Returns true on success.
             *  \param state the location at which to store the valid state, if one is found. This location may be modified even if no valid state is found.
             *  \param near a state that may be invalid near which we would like to find a valid state
             *  \param distance the maximum allowed distance between \e state and \e near
             *  \param attempts the algorithm works by sampling states near state \e near. This parameter defines the maximum number of sampling attempts
             */
            bool searchValidNearby(State *state, const State *near, double distance, unsigned int attempts) const;

            /** \brief Find a valid state near a given one. If the given state is valid, it will be returned itself.
             *  The two passed state pointers need not point to different memory. Returns true on success.
             *  \param sampler the valid state sampler to use when attemting to find a valid sample.
             *  \param state the location at which to store the valid state, if one is found. This location may be modified even if no valid state is found.
             *  \param near a state that may be invalid near which we would like to find a valid state
             *  \param distance the maximum allowed distance between \e state and \e near
             */
            bool searchValidNearby(const ValidStateSamplerPtr &sampler, State *state, const State *near, double distance) const;

            /** \brief Produce a valid motion starting at \e start by randomly bouncing off of invalid states. The start state \e start is not included in the computed motion (\e states). Returns the number of elements written to \e states (less or equal to \e steps).
             *  \param sss the state space sampler to use
             *  \param start the state at which to start bouncing
             *  \param steps the number of bouncing steps to take
             *  \param states the location at which generated states will be stored
             *  \param alloc flag indicating whether memory should be allocated for \e states */
            unsigned int randomBounceMotion(const StateSamplerPtr &sss, const State *start, unsigned int steps, std::vector<State*> &states, bool alloc) const;

            /** \brief Incrementally check if the path between two motions is valid. Also compute the last state that was
                valid and the time of that state. The time is used to parametrize the motion from s1 to s2, s1 being at t =
                0 and s2 being at t = 1. This function assumes s1 is valid.
                \param s1 start state of the motion to be checked (assumed to be valid)
                \param s2 final state of the motion to be checked
                \param lastValid first: storage for the last valid state (may be NULL); this need not be different from \e s1 or \e s2. second: the time (between 0 and 1) of  the last valid state, on the motion from \e s1 to \e s2 */
            bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const
            {
                return motionValidator_->checkMotion(s1, s2, lastValid);
            }


            /** \brief Check if the path between two states (from \e s1 to \e s2) is valid, using subdivision. This function assumes \e s1 is valid. */
            bool checkMotion(const State *s1, const State *s2) const
            {
                return motionValidator_->checkMotion(s1, s2);
            }

            /** \brief Incrementally check if a sequence of states is valid. Given a vector of states, this routine only
                checks the first \e count elements and marks the index of the first invalid state
                \param states the array of states to be checked
                \param count the number of states to be checked in the array (0 to \e count)
                \param firstInvalidStateIndex location to store the first invalid state index. Unmodified if the function returns true */
            bool checkMotion(const std::vector<State*> &states, unsigned int count, unsigned int &firstInvalidStateIndex) const;

            /** \brief Check if a sequence of states is valid using subdivision. */
            bool checkMotion(const std::vector<State*> &states, unsigned int count) const;

            /** \brief Get \e count states that make up a motion between \e s1 and \e s2. Returns the number of states that were added to \e states.
                If \e states.size() >= count or \e alloc is true, the returned value is equal to \e count (or \e count + 2, if \e endpoints is true).
                Otherwise, fewer states can be returned.
                \param s1 the start state of the considered motion
                \param s2 the end state of the considered motion
                \param states the computed set of states along the specified motion
                \param count the number of intermediate states to compute
                \param endpoints flag indicating whether \e s1 and \e s2 are to be included in states
                \param alloc flag indicating whether memory is to be allocated automatically
            */
            unsigned int getMotionStates(const State *s1, const State *s2, std::vector<State*> &states, unsigned int count, bool endpoints, bool alloc) const;

            /** @}*/

            /** @name Routines for inferring information about the state space
                @{ */

            /** \brief Estimate probability of sampling a valid state. setup() is assumed to have been called. */
            double probabilityOfValidState(unsigned int attempts) const;

            /** \brief Estimate the length of a valid motion. setup() is assumed to have been called.*/
            double averageValidMotionLength(unsigned int attempts) const;

            /** \brief Estimate the number of samples that can be drawn per second, using the sampler returned by allocStateSampler() */
            void samplesPerSecond(double &uniform, double &near, double &gaussian, unsigned int attempts) const;

            /** \brief Print information about the current instance of the state space */
            virtual void printSettings(std::ostream &out = std::cout) const;

            /** \brief Print properties of the current instance of the state space */
            virtual void printProperties(std::ostream &out = std::cout) const;

            /** \brief Get the combined parameters for the classes that the space information manages */
            ParamSet& params()
            {
                return params_;
            }

            /** \brief Get the combined parameters for the classes that the space information manages */
            const ParamSet& params() const
            {
                return params_;
            }

            /** \brief Perform additional setup tasks (run once,
                before use). If state validity checking resolution has
                not been set, estimateMaxResolution() is called to
                estimate it. */
            virtual void setup();

            /** \brief Return true if setup was called */
            bool isSetup() const;

        protected:
            /** \brief Set default motion validator for the state space */
            void setDefaultMotionValidator();

            /** \brief The state space planning is to be performed in */
            StateSpacePtr              stateSpace_;

            /** \brief The instance of the state validity checker used for determining the validity of states in the planning process */
            StateValidityCheckerPtr    stateValidityChecker_;

            /** \brief The instance of the motion validator to use when determining the validity of motions in the planning process */
            MotionValidatorPtr         motionValidator_;

            /** \brief Flag indicating whether setup() has been called on this instance */
            bool                       setup_;

            /** \brief The optional valid state sampler allocator */
            ValidStateSamplerAllocator vssa_;

            /** \brief Combined parameters for the contained classes */
            ParamSet                   params_;
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

#ifndef OMPL_BASE_STATE_SAMPLER_ARRAY_
#define OMPL_BASE_STATE_SAMPLER_ARRAY_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include <vector>

namespace ompl
{
    namespace base
    {

        /** \brief Depending on the type of state sampler, we have different allocation routines

            This struct will provide that allocation routine,
            depending on the template argument of ompl::base::SamplerType.*/
        template<typename T>
        struct SamplerSelector
        {
        };

        /** \cond IGNORE */
        template<>
        struct SamplerSelector<StateSampler>
        {
            typedef StateSampler    Sampler;
            typedef StateSamplerPtr SamplerPtr;

            SamplerPtr allocStateSampler(const SpaceInformation *si)
            {
                return si->allocStateSampler();
            }

        };

        template<>
        struct SamplerSelector<ValidStateSampler>
        {
            typedef ValidStateSampler    Sampler;
            typedef ValidStateSamplerPtr SamplerPtr;

            SamplerPtr allocStateSampler(const SpaceInformation *si)
            {
                return si->allocValidStateSampler();
            }
        };
        /** \endcond */

        /** \brief Class to ease the creation of a set of samplers. This is especially useful for multi-threaded planners. */
        template<typename T>
        class StateSamplerArray
        {
        public:

            /** \brief Pointer to the type of sampler allocated */
            typedef typename SamplerSelector<T>::SamplerPtr SamplerPtr;

            /** \brief The type of sampler allocated */
            typedef typename SamplerSelector<T>::Sampler    Sampler;

            /** \brief Constructor */
            StateSamplerArray(const SpaceInformationPtr &si) : si_(si.get())
            {
            }

            /** \brief Constructor */
            StateSamplerArray(const SpaceInformation *si) : si_(si)
            {
            }

            ~StateSamplerArray()
            {
            }

            /** \brief Access operator for a specific sampler. For
                performance reasons, the bounds are not checked. */
            Sampler* operator[](std::size_t index) const
            {
                return samplers_[index].get();
            }

            /** \brief Create or release some state samplers */
            void resize(std::size_t count)
            {
                if (samplers_.size() > count)
                    samplers_.resize(count);
                else
                    if (samplers_.size() < count)
                    {
                        std::size_t c = samplers_.size();
                        samplers_.resize(count);
                        for (std::size_t i = c ; i < count ; ++i)
                            samplers_[i] = ss_.allocStateSampler(si_);
                    }
            }

            /** \brief Get the count of samplers currently available */
            std::size_t size() const
            {
                return samplers_.size();
            }

            /** \brief Clear all allocated samplers */
            void clear()
            {
                resize(0);
            }

        private:

            const SpaceInformation  *si_;
            SamplerSelector<T>       ss_;
            std::vector<SamplerPtr>  samplers_;

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

#ifndef OMPL_BASE_SCOPED_STATE_
#define OMPL_BASE_SCOPED_STATE_

#include "ompl/base/SpaceInformation.h"
#include <boost/concept_check.hpp>
#include <iostream>

namespace ompl
{
    namespace base
    {

        /** \brief Definition of a scoped state.

            This class allocates a state of a desired type using the
            allocation mechanism of the corresponding state space.
            The state is then freed when the instance goes out of
            scope using the corresponding free mechanism. */
        template<class T = StateSpace>
        class ScopedState
        {
            /** \brief Make sure the type we are allocating is indeed from a state space */
            BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateSpace*>));

            /** \brief Make sure the type we are allocating is indeed a state */
            BOOST_CONCEPT_ASSERT((boost::Convertible<typename T::StateType*, State*>));

        public:

            /** \brief The type of the contained state */
            typedef typename T::StateType StateType;

            /** \brief Given the space that we are working with,
                allocate a state from the corresponding
                state space.  */
            explicit
            ScopedState(const SpaceInformationPtr &si) : space_(si->getStateSpace())
            {
                State *s = space_->allocState();

                // ideally, this should be a dynamic_cast and we
                // should throw an exception in case of
                // failure. However, RTTI may not be available across
                // shared library boundaries, so we do not use it
                state_ = static_cast<StateType*>(s);
            }

            /** \brief Given the state space that we are working with,
                allocate a state. */
            explicit
            ScopedState(const StateSpacePtr &space) : space_(space)
            {
                State *s = space_->allocState();

                // ideally, this should be a dynamic_cast and we
                // should throw an exception in case of
                // failure. However, RTTI may not be available across
                // shared library boundaries, so we do not use it
                state_ = static_cast<StateType*>(s);
            }

            /** \brief Copy constructor */
            ScopedState(const ScopedState<T> &other) : space_(other.getSpace())
            {
                State *s = space_->allocState();
                state_ = static_cast<StateType*>(s);
                space_->copyState(s, static_cast<const State*>(other.get()));
            }

            /** \brief Copy constructor that allows instantiation from states of other type */
            template<class O>
            ScopedState(const ScopedState<O> &other) : space_(other.getSpace())
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O*, StateSpace*>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType*, State*>));

                // ideally, we should use a dynamic_cast and throw an
                // exception in case other.get() does not cast to
                // const StateType*. However, RTTI may not be
                // available across shared library boundaries, so we
                // do not use it

                State *s = space_->allocState();
                state_ = static_cast<StateType*>(s);
                space_->copyState(s, static_cast<const State*>(other.get()));
            }

            /** \brief Given the state space that we are working with,
                allocate a state and fill that state with a given value. */
            ScopedState(const StateSpacePtr &space, const State *state) : space_(space)
            {
                State *s = space_->allocState();
                space_->copyState(s, state);

                // ideally, this should be a dynamic_cast and we
                // should throw an exception in case of
                // failure. However, RTTI may not be available across
                // shared library boundaries, so we do not use it
                state_ = static_cast<StateType*>(s);
            }

            /** \brief Free the memory of the internally allocated state */
            ~ScopedState()
            {
                space_->freeState(state_);
            }

            /** \brief Get the state space that the state corresponds to */
            const StateSpacePtr& getSpace() const
            {
                return space_;
            }

            /** \brief Assignment operator */
            ScopedState<T>& operator=(const ScopedState<T> &other)
            {
                if (&other != this)
                {
                    space_->freeState(state_);
                    space_ = other.getSpace();

                    State *s = space_->allocState();
                    state_ = static_cast<StateType*>(s);
                    space_->copyState(s, static_cast<const State*>(other.get()));
                }
                return *this;
            }

            /** \brief Assignment operator */
            ScopedState<T>& operator=(const State *other)
            {
                if (other != static_cast<State*>(state_))
                {
                    // ideally, we should use a dynamic_cast and throw an
                    // exception in case other does not cast to
                    // const StateType*. However, RTTI may not be
                    // available across shared library boundaries, so we
                    // do not use it

                    space_->copyState(static_cast<State*>(state_), other);
                }
                return *this;
            }

            /** \brief Assignment operator */
            ScopedState<T>& operator=(const State &other)
            {
                if (&other != static_cast<State*>(state_))
                {
                    // ideally, we should use a dynamic_cast and throw an
                    // exception in case &other does not cast to
                    // const StateType*. However, RTTI may not be
                    // available across shared library boundaries, so we
                    // do not use it

                    space_->copyState(static_cast<State*>(state_), &other);
                }
                return *this;
            }

            /** \brief Assignment operator that allows conversion of states */
            template<class O>
            ScopedState<T>& operator=(const ScopedState<O> &other)
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O*, StateSpace*>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType*, State*>));

                // ideally, we should use a dynamic_cast and throw an
                // exception in case other.get() does not cast to
                // const StateType*. However, RTTI may not be
                // available across shared library boundaries, so we
                // do not use it

                if (reinterpret_cast<const void*>(&other) != reinterpret_cast<const void*>(this))
                {
                    space_->freeState(state_);
                    space_ = other.getSpace();

                    State *s = space_->allocState();
                    state_ = static_cast<StateType*>(s);
                    space_->copyState(s, static_cast<const State*>(other.get()));
                }
                return *this;
            }

            /** \brief Partial assignment operator. Only sets the double values of the state to specified real values */
            ScopedState<T>& operator=(const std::vector<double> &reals)
            {
                for (unsigned int i = 0 ; i < reals.size() ; ++i)
                    if (double *va = space_->getValueAddressAtIndex(state_, i))
                        *va = reals[i];
                    else
                        break;
                return *this;
            }

            /** \brief Partial assignment operator. Only sets the double values of the state to a fixed value */
            ScopedState<T>& operator=(const double value)
            {
                unsigned int index = 0;
                while (double *va = space_->getValueAddressAtIndex(state_, index++))
                    *va = value;
                return *this;
            }

            /** \brief Checks equality of two states */
            template<class O>
            bool operator==(const ScopedState<O> &other) const
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O*, StateSpace*>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType*, State*>));

                // ideally, we should use a dynamic_cast and throw an
                // exception in case other.get() does not cast to
                // const StateType*. However, RTTI may not be
                // available across shared library boundaries, so we
                // do not use it

                return space_->equalStates(static_cast<const State*>(state_), static_cast<const State*>(other.get()));
            }

            /** \brief Checks equality of two states */
            template<class O>
            bool operator!=(const ScopedState<O> &other) const
            {
                return !(*this == other);
            }

            /** \brief Extract a state that corresponds to the
                components in state space \e s. Those components will
                have the same value as the current state (only the
                ones included in the current state; others will be
                uninitialised). Note: a new state is constructed and data is copied. */
            const ScopedState<> operator[](const StateSpacePtr &s) const;

            /** \brief Access the \e index<sup>th</sup> double value this state contains. */
            double& operator[](const unsigned int index)
            {
                double *val = space_->getValueAddressAtIndex(state_, index);
                if (!val)
                    throw Exception("Index out of bounds");
                return *val;
            }

            /** \brief Access the \e index<sup>th</sup> double value this state contains. */
            double operator[](const unsigned int index) const
            {
                const double *val = space_->getValueAddressAtIndex(state_, index);
                if (!val)
                    throw Exception("Index out of bounds");
                return *val;
            }

            /** \brief Access a double value from this state contains using its name. */
            double& operator[](const std::string &name)
            {
                const std::map<std::string, StateSpace::ValueLocation> &vm = space_->getValueLocationsByName();
                std::map<std::string, StateSpace::ValueLocation>::const_iterator it = vm.find(name);
                if (it != vm.end())
                {
                    double *val = space_->getValueAddressAtLocation(state_, it->second);
                    if (val)
                        return *val;
                }
                throw Exception("Name '" + name + "' not known");
            }

            /** \brief Access a double value from this state contains using its name. */
            double operator[](const std::string &name) const
            {
                const std::map<std::string, StateSpace::ValueLocation> &vm = space_->getValueLocationsByName();
                std::map<std::string, StateSpace::ValueLocation>::const_iterator it = vm.find(name);
                if (it != vm.end())
                {
                    const double *val = space_->getValueAddressAtLocation(state_, it->second);
                    if (val)
                        return *val;
                }
                throw Exception("Name '" + name + "' not known");
            }

            /** \brief Compute the distance to another state. */
            template<class O>
            double distance(const ScopedState<O> &other) const
            {
                BOOST_CONCEPT_ASSERT((boost::Convertible<O*, StateSpace*>));
                BOOST_CONCEPT_ASSERT((boost::Convertible<typename O::StateType*, State*>));
                return distance(other.get());
            }

            /** \brief Compute the distance to another state. */
            double distance(const State *state) const
            {
                return space_->distance(static_cast<const State*>(state_), state);
            }

            /** \brief Set this state to a random value (uniform) */
            void random()
            {
                if (!sampler_)
                    sampler_ = space_->allocStateSampler();
                sampler_->sampleUniform(state_);
            }

            /** \brief Enforce the bounds on the maintained state */
            void enforceBounds()
            {
                space_->enforceBounds(state_);
            }

            /** \brief Check if the maintained state satisfies bounds */
            bool satisfiesBounds() const
            {
                return space_->satisfiesBounds(state_);
            }

            /** \brief Return the real values corresponding to this
                state. If a conversion is not possible, an exception
                is thrown.*/
            std::vector<double> reals() const
            {
                std::vector<double> r;
                unsigned int index = 0;
                while (double *va = space_->getValueAddressAtIndex(state_, index++))
                    r.push_back(*va);
                return r;
            }

            /** \brief Print this state to a stream */
            void print(std::ostream &out = std::cout) const
            {
                space_->printState(state_, out);
            }

            /** \brief De-references to the contained state */
            StateType& operator*()
            {
                return *state_;
            }

            /** \brief De-references to the contained state */
            const StateType& operator*() const
            {
                return *state_;
            }

            /** \brief Returns a pointer to the contained state */
            StateType* operator->()
            {
                return state_;
            }

            /** \brief Returns a pointer to the contained state */
            const StateType* operator->() const
            {
                return state_;
            }

            /** \brief Returns a pointer to the contained state */
            StateType* get()
            {
                return state_;
            }

            /** \brief Returns a pointer to the contained state */
            const StateType* get() const
            {
                return state_;
            }

            /** \brief Returns a pointer to the contained state (used for Python bindings) */
            StateType* operator()() const
            {
                return state_;
            }

        private:

            StateSpacePtr                   space_;
            StateSamplerPtr                 sampler_;
            StateType                      *state_;
        };

        /** \addtogroup stateAndSpaceOperators Operators for States and State Spaces

           These operators are intended to simplify code that
           manipulates states and state spaces. They rely on the fact
           that state spaces have unique names. Here are some examples
           for using these operators:
          \code
           // Assume X, Y, Z, W are state space instances, none of
           // which inherits from ompl::base::CompoundStateSpace.
           // Denote a compound state space as C[...], where "..." is the
           // list of subspaces.

           ompl::base::StateSpacePtr X;
           ompl::base::StateSpacePtr Y;
           ompl::base::StateSpacePtr Z;
           ompl::base::StateSpacePtr W;

           // the following line will construct a state space C1 = C[X, Y]
           ompl::base::StateSpacePtr C1 = X + Y;

           // the following line will construct a state space C2 = C[X, Y, Z]
           ompl::base::StateSpacePtr C2 = C1 + Z;

           // the following line will leave C2 as C[X, Y, Z]
           ompl::base::StateSpacePtr C2 = C1 + C2;

           // the following line will construct a state space C2 = C[X, Y, Z, W]
           ompl::base::StateSpacePtr C2 = C2 + W;

           // the following line will construct a state space C3 = C[X, Z, Y]
           ompl::base::StateSpacePtr C3 = X + Z + Y;

           // the following line will construct a state space C4 = C[Z, W]
           ompl::base::StateSpacePtr C4 = C2 - C1;

           // the following line will construct a state space C5 = W
           ompl::base::StateSpacePtr C5 = C2 - C3;

           // the following line will construct an empty state space C6 = C[]
           ompl::base::StateSpacePtr C6 = X - X;

           // the following line will construct a state space C7 = Y
           ompl::base::StateSpacePtr C7 = Y + C6;
          \endcode
           These state spaces can be used when operating with states:
          \code
           ompl::base::ScopedState<> sX(X);
           ompl::base::ScopedState<> sXY(X + Y);
           ompl::base::ScopedState<> sY(Y);
           ompl::base::ScopedState<> sZX(Z + X);
           ompl::base::ScopedState<> sXZW(X + Z + W);

           // the following line will copy the content of the state sX to
           // the corresponding locations in sXZW. The components of the state
           // corresponding to the Z and W state spaces are not touched
           sX >> sXZW;

           // the following line will initialize the X component of sXY with
           // the X component of sXZW;
           sXY << sXZW;

           // the following line will initialize both components of sZX, using
           // the X and Z components of sXZW;
           sZX << sXZW;

           // the following line compares the concatenation of states sX and sY with sXY
           // the concatenation will automatically construct the state space X + Y and a state
           // from that state space containing the information from sX and sY. Since sXY is
           // constructed from the state space X + Y, the two are comparable.
           bool eq = (sX ^ sY) == sXY;
          \endcode
            @{
         */

        /** \brief Overload stream output operator. Calls ompl::base::StateSpace::printState() */
        template<class T>
        inline
        std::ostream& operator<<(std::ostream &out, const ScopedState<T> &state)
        {
            state.print(out);
            return out;
        }

        /** \brief This is a fancy version of the assignment
            operator. It is a partial assignment, in some sense. The
            difference is that if the states are part of compound
            state spaces, the data is copied from \e from to \e to on a
            component by component basis. State spaces are matched by
            name. If the state space for \e to contains any subspace
            whose name matches any subspace of the state space for \e
            from, the corresponding state components are copied. */
        template<class T, class Y>
        inline
        ScopedState<T>& operator<<(ScopedState<T> &to, const ScopedState<Y> &from)
        {
            copyStateData(to.getSpace(), to.get(), from.getSpace(), from.get());
            return to;
        }

        /** \brief This is a fancy version of the assignment
            operator. It is a partial assignment, in some sense. The
            difference is that if the states are part of compound
            state spaces, the data is copied from \e from to \e to on a
            component by component basis. State spaces are matched by
            name. If the state space for \e to contains any subspace
            whose name matches any subspace of the state space for \e
            from, the corresponding state components are copied. */
        template<class T, class Y>
        inline
        const ScopedState<T>& operator>>(const ScopedState<T> &from, ScopedState<Y> &to)
        {
            copyStateData(to.getSpace(), to.get(), from.getSpace(), from.get());
            return from;
        }

        /** \brief Given state \e a from state space A and state \e b
            from state space B, construct a state from state space A
            + B. The resulting state contains all the information from
            the input states (the states are concatenated). */
        template<class T, class Y>
        inline
        const ScopedState<> operator^(const ScopedState<T> &a, const ScopedState<Y> &b)
        {
            ScopedState<> r(a.getSpace() + b.getSpace());
            return r << a << b;
        }

        /** @} */

        template<class T>
        const ScopedState<> ScopedState<T>::operator[](const StateSpacePtr &s) const
        {
            ScopedState<> r(s);
            return r << *this;
        }

        /** \brief Shared pointer to a ScopedState<> */
        typedef boost::shared_ptr< ScopedState<> > ScopedStatePtr;
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

/* Author: Luis G. Torres, Ioan Sucan */

#ifndef OMPL_BASE_COST_
#define OMPL_BASE_COST_

#include <iostream>

namespace ompl
{
    namespace base
    {
        /** \brief Definition of a cost value. Can represent the cost of a motion or the cost of a state. */
        struct Cost
        {
            /** \brief Construct cost with a specified value */
            explicit Cost(double v = 0.0) : v(v) {}
            /** \brief The value of the cost */
            double v;
        };

        /** \brief Output operator for Cost */
        std::ostream& operator<<(std::ostream& stream, Cost c);
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

#ifndef OMPL_BASE_PATH_
#define OMPL_BASE_PATH_

#include "ompl/util/ClassForward.h"
#include "ompl/base/Cost.h"
#include <iostream>
#include <boost/noncopyable.hpp>

namespace ompl
{
    namespace base
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(OptimizationObjective);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Path */
        OMPL_CLASS_FORWARD(Path);
        /// @endcond

        /** \class ompl::base::PathPtr
            \brief A boost shared pointer wrapper for ompl::base::Path */

        /** \brief Abstract definition of a path */
        class Path : private boost::noncopyable
        {
        public:

            /** \brief Constructor. A path must always know the space information it is part of */
            Path(const SpaceInformationPtr &si) : si_(si)
            {
            }

            /** \brief Destructor */
            virtual ~Path()
            {
            }

            /** \brief Get the space information associated to this class */
            const SpaceInformationPtr& getSpaceInformation() const
            {
                return si_;
            }

            /** \brief Return the length of a path */
            virtual double length() const = 0;

            /** \brief Return the cost of the path with respect to a
                specified optimization objective. */
            virtual Cost cost(const OptimizationObjectivePtr& obj) const;

            /** \brief Check if the path is valid */
            virtual bool check() const = 0;

            /** \brief Print the path to a stream */
            virtual void print(std::ostream &out) const = 0;

        protected:

            /** \brief The space information this path is part of */
            SpaceInformationPtr si_;
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_GOAL_TYPES_
#define OMPL_BASE_GOAL_TYPES_

namespace ompl
{
    namespace base
    {

        /** \brief The type of goal */
        enum GoalType
            {
                /** \brief This bit is set if casting to generic goal regions (ompl::base::Goal) is possible. This bit shold always be set */
                GOAL_ANY               = 1,

                /** \brief This bit is set if casting to goal regions (ompl::base::GoalRegion) is possible */
                GOAL_REGION            = GOAL_ANY + 2,

                /** \brief This bit is set if casting to sampleable goal regions (ompl::base::GoalSampleableRegion) is possible */
                GOAL_SAMPLEABLE_REGION = GOAL_REGION + 4,

                /** \brief This bit is set if casting to goal state (ompl::base::GoalState) is possible */
                GOAL_STATE             = GOAL_SAMPLEABLE_REGION + 8,

                /** \brief This bit is set if casting to goal states (ompl::base::GoalStates) is possible */
                GOAL_STATES            = GOAL_SAMPLEABLE_REGION + 16,

                /** \brief This bit is set if casting to goal states (ompl::base::GoalLazySamples) is possible */
                GOAL_LAZY_SAMPLES      = GOAL_STATES + 32
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

#ifndef OMPL_BASE_GOAL_
#define OMPL_BASE_GOAL_

#include "ompl/base/State.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include "ompl/base/GoalTypes.h"
#include "ompl/util/Console.h"
#include <iostream>
#include <boost/noncopyable.hpp>
#include <boost/concept_check.hpp>
#include <vector>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Goal */
        OMPL_CLASS_FORWARD(Goal);
        /// @endcond

        /** \class ompl::base::GoalPtr
            \brief A boost shared pointer wrapper for ompl::base::Goal */

        /** \brief Abstract definition of goals.*/
        class Goal : private boost::noncopyable
        {
        public:

            /** \brief Constructor. The goal must always know the space information it is part of */
            Goal(const SpaceInformationPtr &si);

            /** \brief Destructor.*/
            virtual ~Goal()
            {
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            T* as()
            {
                /** \brief Make sure the type we are casting to is indeed a goal */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Goal*>));

                return static_cast<T*>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            const T* as() const
            {
                /** \brief Make sure the type we are casting to is indeed a goal */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Goal*>));

                return static_cast<const T*>(this);
            }

            /** \brief Return the goal type */
            GoalType getType() const
            {
                return type_;
            }

            /** \brief Check if this goal can be cast to a particular goal type */
            bool hasType(GoalType type) const
            {
                return (type_ & type) == type;
            }

            /** \brief Get the space information this goal is for */
            const SpaceInformationPtr& getSpaceInformation() const
            {
                return si_;
            }

            /** \brief Return true if the state satisfies the goal
             *  constraints. */
            virtual bool isSatisfied(const State *st) const = 0;

            /** \brief Return true if the state satisfies the goal
             *  constraints and compute the distance between the state
             *  given as argument and the goal (even if the goal is
             *  not satisfied). This distance can be an
             *  approximation. It can even be set to a constant, if
             *  such a computation is not possible.
             *  \param st the state to check for validity
             *  \param distance location at which distance to goal will be stored
             *  \note The default implementation sets the distance to a constant.
             *  \note If this function returns true,
             *  isStartGoalPairValid() need not be called. */
            virtual bool isSatisfied(const State *st, double *distance) const;

            /** \brief Since there can be multiple starting states
                (and multiple goal states) it is possible certain
                pairs are not to be allowed. By default we however
                assume all such pairs are allowed. Note: if this
                function returns true, isSatisfied() need not be
                called. */
            virtual bool isStartGoalPairValid(const State * /* start */, const State * /* goal */) const
            {
                return true;
            }

            /** \brief Print information about the goal */
            virtual void print(std::ostream &out = std::cout) const;

        protected:

            /** \brief Goal type */
            GoalType                     type_;

            /** \brief The space information for this goal */
            SpaceInformationPtr          si_;
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

#ifndef OMPL_BASE_GOALS_GOAL_REGION_
#define OMPL_BASE_GOALS_GOAL_REGION_

#include "ompl/base/Goal.h"


namespace ompl
{

    namespace base
    {

        /** \brief Definition of a goal region */
        class GoalRegion : public Goal
        {
        public:

            /** \brief Create a goal region */
            GoalRegion(const SpaceInformationPtr &si);

            virtual ~GoalRegion()
            {
            }

            /** \brief Equivalent to calling isSatisfied(const State *, double *) with a NULL second argument. */
            virtual bool isSatisfied(const State *st) const;

            /** \brief Decide whether a given state is part of the
                goal region. Returns true if the distance to goal is
                less than the threshold (using distanceGoal()) */
            virtual bool isSatisfied(const State *st, double *distance) const;

            /** \brief Compute the distance to the goal
                (heuristic). This function is the one used in
                computing the distance to the goal in a call to
                isSatisfied() */
            virtual double distanceGoal(const State *st) const = 0;

            /** \brief Print information about the goal data structure
                to a stream */
            virtual void print(std::ostream &out = std::cout) const;

            /** \brief Set the distance to the goal that is allowed
                for a state to be considered in the goal region */
            void setThreshold(double threshold)
            {
                threshold_ = threshold;
            }

            /** \brief Get the distance to the goal that is allowed
                for a state to be considered in the goal region */
            double getThreshold() const
            {
                return threshold_;
            }

        protected:

            /** \brief The maximum distance that is allowed to the
                goal. By default, this is initialized to the minimum
                epsilon value a double can represent */
            double threshold_;
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

#ifndef OMPL_BASE_GOALS_GOAL_SAMPLEABLE_REGION_
#define OMPL_BASE_GOALS_GOAL_SAMPLEABLE_REGION_

#include "ompl/base/goals/GoalRegion.h"

namespace ompl
{

    namespace base
    {

        /** \brief Abstract definition of a goal region that can be sampled */
        class GoalSampleableRegion : public GoalRegion
        {
        public:

            /** \brief Create a goal region that can be sampled */
            GoalSampleableRegion(const SpaceInformationPtr &si) : GoalRegion(si)
            {
                type_ = GOAL_SAMPLEABLE_REGION;
            }

            virtual ~GoalSampleableRegion()
            {
            }

            /** \brief Sample a state in the goal region */
            virtual void sampleGoal(State *st) const = 0;

            /** \brief Return the maximum number of samples that can be asked for before repeating */
            virtual unsigned int maxSampleCount() const = 0;

            /** \brief Return true if maxSampleCount() > 0, since in this case samples can certainly be produced */
            bool canSample() const
            {
                return maxSampleCount() > 0;
            }

            /** \brief Return true if samples could be generated by this sampler at some point in the future. By default this is equivalent to canSample(),
             but for GoalLazySamples, this call also reflects the fact that a sampling thread is active and although no samples are produced yet, some may
             become available at some point in the future. */
            virtual bool couldSample() const
            {
                return canSample();
            }

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

#ifndef OMPL_BASE_GOALS_GOAL_STATE_
#define OMPL_BASE_GOALS_GOAL_STATE_

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/ScopedState.h"

namespace ompl
{

    namespace base
    {

        /** \brief Definition of a goal state */
        class GoalState : public GoalSampleableRegion
        {
        public:

            /** \brief Create a goal representation that is in fact a state  */
            GoalState(const SpaceInformationPtr &si) : GoalSampleableRegion(si), state_(NULL)
            {
                type_ = GOAL_STATE;
            }

            virtual ~GoalState();

            /** \brief Sample a state in the goal region */
            virtual void sampleGoal(State *st) const;

            /** \brief Return the maximum number of samples that can be asked for before repeating */
            virtual unsigned int maxSampleCount() const;

            /** \brief Compute the distance to the goal (heuristic) */
            virtual double distanceGoal(const State *st) const;

            /** \brief Print information about the goal data structure
                to a stream */
            virtual void print(std::ostream &out = std::cout) const;

            /** \brief Set the goal state */
            void setState(const State *st);

            /** \brief Set the goal state */
            void setState(const ScopedState<> &st);

            /** \brief Get the goal state */
            const State* getState() const;

            /** \brief Get the goal state */
            State* getState();

        protected:

            /** \brief The goal state */
            State *state_;
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

#ifndef OMPL_BASE_GOALS_GOAL_STATES_
#define OMPL_BASE_GOALS_GOAL_STATES_

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/ScopedState.h"
#include <vector>

namespace ompl
{
    namespace base
    {

        /** \brief Definition of a set of goal states */
        class GoalStates : public GoalSampleableRegion
        {
        public:

            /** \brief Create a goal representation that is in fact a set of states  */
            GoalStates(const SpaceInformationPtr &si) : GoalSampleableRegion(si), samplePosition_(0)
            {
                type_ = GOAL_STATES;
            }

            virtual ~GoalStates();

            virtual void sampleGoal(State *st) const;

            virtual unsigned int maxSampleCount() const;

            virtual double distanceGoal(const State *st) const;

            virtual void print(std::ostream &out = std::cout) const;

            /** \brief Add a goal state */
            virtual void addState(const State *st);

            /** \brief Add a goal state (calls the previous definition of addState())*/
            void addState(const ScopedState<> &st);

            /** \brief Clear all goal states */
            virtual void clear();

            /** \brief Check if there are any states in this goal region */
            virtual bool hasStates() const;

            /** \brief Return a pointer to the indexth state in the state list */
            virtual const State* getState(unsigned int index) const;

            /** \brief Return the number of valid goal states */
            virtual std::size_t getStateCount() const;

        protected:

            /** \brief The goal states. Only ones that are valid are considered by the motion planner. */
            std::vector<State*> states_;

        private:

            /** \brief The index of the next sample to be returned  */
            mutable unsigned int samplePosition_;

            /** \brief Free allocated memory */
            void freeMemory();

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

#ifndef OMPL_BASE_GOALS_GOAL_LAZY_SAMPLES_
#define OMPL_BASE_GOALS_GOAL_LAZY_SAMPLES_

#include "ompl/base/goals/GoalStates.h"
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <limits>

namespace ompl
{

    namespace base
    {

        class GoalLazySamples;

        /** \brief Goal sampling function. Returns false when no further calls should be made to it.
            Fills its second argument (the state) with the sampled goal state. This function need not
            be thread safe. */
        typedef boost::function<bool(const GoalLazySamples*, State*)> GoalSamplingFn;

        /** \brief Definition of a goal region that can be sampled,
         but the sampling process can be slow.  This class allows
         sampling the happen in a separate thread, and the number of
         goals may increase, as the planner is running, in a
         thread-safe manner.


         \todo The Python bindings for GoalLazySamples class are still broken.
         The OMPL C++ code creates a new thread from which you should be able
         to call a python Goal sampling function. Acquiring the right threads
         and locks and messing around with the Python Global Interpreter Lock
         (GIL) is very tricky. See ompl/py-bindings/generate_bindings.py for
         an initial attempt to make this work.
         */
        class GoalLazySamples : public GoalStates
        {
        public:

            /** \brief When new samples are generated and added to the
                list of possible samples, a callback can be
                called. This type specifies the signature of that callback */
            typedef boost::function<void(const base::State*)> NewStateCallbackFn;

            /** \brief Create a goal region that can be sampled in a
                lazy fashion. A function (\e samplerFunc) that
                produces samples from that region needs to be passed
                to this constructor. The sampling thread is
                automatically started if \e autoStart is true. The
                sampling function is not called in parallel by
                OMPL. Hence, the function is not required to be thread
                safe, unless the user issues additional calls in
                parallel. The instance of GoalLazySamples remains
                thread safe however.

                The function \e samplerFunc returns a truth value. If
                the return value is true, further calls to the
                function can be made. If the return is false, no more
                calls should be made. The function takes two
                arguments: the instance of GoalLazySamples making the
                call and the state to fill with a goal state. For
                every state filled in by \e samplerFunc,
                addStateIfDifferent() is called.  A state computed by
                the sampling thread is added if it is "sufficiently
                different" from previously added states. A state is
                considered "sufficiently different" if it is at least
                \e minDist away from previously added states.  */
            GoalLazySamples(const SpaceInformationPtr &si, const GoalSamplingFn &samplerFunc,
                            bool autoStart = true, double minDist = std::numeric_limits<double>::epsilon());

            virtual ~GoalLazySamples();

            virtual void sampleGoal(State *st) const;

            virtual double distanceGoal(const State *st) const;

            virtual void addState(const State *st);

            /** \brief Start the goal sampling thread */
            void startSampling();

            /** \brief Stop the goal sampling thread */
            void stopSampling();

            /** \brief Return true if the sampling thread is active */
            bool isSampling() const;

            /** \brief Set the minimum distance that a new state returned by the sampling thread needs to be away from
                previously added states, so that it is added to the list of goal states. */
            void setMinNewSampleDistance(double dist)
            {
                minDist_ = dist;
            }

            /** \brief Get the minimum distance that a new state returned by the sampling thread needs to be away from
                previously added states, so that it is added to the list of goal states. */
            double getMinNewSampleDistance() const
            {
                return minDist_;
            }

            /** \brief The number of times the sampling function was called and it returned true */
            unsigned int samplingAttemptsCount() const
            {
                return samplingAttempts_;
            }

            /** \brief Set the callback function to be called when a new state is added to the list of possible samples. This function
                is not required to be thread safe, as calls are made one at a time. */
            void setNewStateCallback(const NewStateCallbackFn &callback);

            /** \brief Add a state \e st if it further away that \e minDistance from previously added states. Return true if the state was added. */
            bool addStateIfDifferent(const State *st, double minDistance);

            /** \brief Return true if GoalStates::couldSample() is true or if the sampling thread is active, as in this case it is possible a sample can be produced at some point. */
            virtual bool couldSample() const;

            virtual bool hasStates() const;
            virtual const State* getState(unsigned int index) const;
            virtual std::size_t getStateCount() const;

            virtual void clear();

        protected:

            /** \brief The function that samples goals by calling \e samplerFunc_ in a separate thread */
            void goalSamplingThread();

            /** \brief Lock for updating the set of states */
            mutable boost::mutex           lock_;

            /** \brief Function that produces samples */
            GoalSamplingFn                 samplerFunc_;

            /** \brief Flag used to notify the sampling thread to terminate sampling */
            bool                           terminateSamplingThread_;

            /** \brief Additional thread for sampling goal states */
            boost::thread                 *samplingThread_;

            /** \brief The number of times the sampling function was called and it returned true */
            unsigned int                   samplingAttempts_;

            /** \brief Samples returned by the sampling thread are added to the list of states only if
                they are at least minDist_ away from already added samples. */
            double                         minDist_;

            /** \brief If defined, this function is called when a new state is added to the list of possible samples */
            NewStateCallbackFn             callback_;
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

#ifndef OMPL_BASE_DISCRETE_MOTION_VALIDATOR_
#define OMPL_BASE_DISCRETE_MOTION_VALIDATOR_

#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"

namespace ompl
{

    namespace base
    {

        /** \brief A motion validator that only uses the state validity checker. Motions are checked for validity at a specified resolution. */
        class DiscreteMotionValidator : public MotionValidator
        {
        public:

            /** \brief Constructor */
            DiscreteMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }

            /** \brief Constructor */
            DiscreteMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }

            virtual ~DiscreteMotionValidator()
            {
            }

            virtual bool checkMotion(const State *s1, const State *s2) const;

            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const;

        private:

            StateSpace *stateSpace_;

            void defaultSettings();

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

/* Author: Luis G. Torres, Ioan Sucan */

#ifndef OMPL_BASE_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/Cost.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include <boost/noncopyable.hpp>
#include <boost/concept_check.hpp>

namespace ompl
{
    namespace base
    {
        class Goal;

        /** \brief The definition of a function which returns an admissible estimate of the optimal path cost from a given state to a goal. */
        typedef boost::function<Cost (const State*, const Goal*)> CostToGoHeuristic;

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::OptimizationObjective */
        OMPL_CLASS_FORWARD(OptimizationObjective);
        /// @endcond

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Path);
        /// @endcond

        /** \class ompl::base::OptimizationObjectivePtr
            \brief A boost shared pointer wrapper for ompl::base::OptimizationObjective */

        /** \brief Abstract definition of optimization objectives.

            \note This implementation has greatly benefited from discussions with <a href="http://www.cs.indiana.edu/~hauserk/">Kris Hauser</a> */
        class OptimizationObjective : private boost::noncopyable
        {
        public:
            /** \brief Constructor. The objective must always know the space information it is part of. The cost threshold for objective satisfaction defaults to 0.0. */
            OptimizationObjective(const SpaceInformationPtr &si);

            virtual ~OptimizationObjective()
            {
            }

            /** \brief Get the description of this optimization objective */
            const std::string& getDescription() const;

            /** \brief Verify that our objective is satisfied already and we can stop planning */
            virtual bool isSatisfied(Cost c) const;

            /** \brief Returns the cost threshold currently being checked for objective satisfaction */
            Cost getCostThreshold() const;

            /** \brief Set the cost threshold for objective satisfaction. When a path is found with a cost better than the cost threshold, the objective is considered satisfied. */
            void setCostThreshold(Cost c);

            /** \brief Get the cost that corresponds to an entire path. This implementation assumes \e Path is of type \e PathGeometric.*/
            virtual Cost getCost(const Path &path) const;

            /** \brief Check whether the the cost \e c1 is considered better than the cost \e c2. By default, this returns true only if c1 is less by at least some threshold amount, for numerical robustness. */
            virtual bool isCostBetterThan(Cost c1, Cost c2) const;

            /** \brief Evaluate a cost map defined on the state space at a state \e s. Default implementation maps all states to 1.0. */
            virtual Cost stateCost(const State *s) const;

            /** \brief Get the cost that corresponds to the motion segment between \e s1 and \e s2 */
            virtual Cost motionCost(const State *s1, const State *s2) const = 0;

            /** \brief Get the cost that corresponds to combining the costs \e c1 and \e c2. Default implementation defines this combination as an addition. */
            virtual Cost combineCosts(Cost c1, Cost c2) const;

            /** \brief Get the identity cost value. The identity cost value is the cost c_i such that, for all costs c, combineCosts(c, c_i) = combineCosts(c_i, c) = c. In other words, combining a cost with the identity cost does not change the original cost. By default, a cost with the value 0.0 is returned. It's very important to override this with the proper identity value for your optimization objectives, or else optimal planners may not work. */
            virtual Cost identityCost() const;

            /** \brief Get a cost which is greater than all other costs in this OptimizationObjective; required for use in Dijkstra/Astar. Defaults to returning the double value inf.*/
            virtual Cost infiniteCost() const;

            /** \brief Returns a cost value corresponding to starting at a state \e s. No optimal planners currently support this method. Defaults to returning the objective's identity cost. */
            virtual Cost initialCost(const State *s) const;

            /** \brief Returns a cost value corresponding to a path ending at a state \e s. No optimal planners currently support this method. Defaults to returning the objective's identity cost. */
            virtual Cost terminalCost(const State *s) const;

            /** \brief Check if this objective has a symmetric cost metric, i.e. motionCost(s1, s2) = motionCost(s2, s1). Default implementation returns whether the underlying state space has symmetric interpolation. */
            virtual bool isSymmetric() const;

            /** \brief Compute the average state cost of this objective by taking a sample of \e numStates states */
            virtual Cost averageStateCost(unsigned int numStates) const;

            /** \brief Set the cost-to-go heuristic function for this objective. The cost-to-go heuristic is a function which returns an admissible estimate of the optimal path cost from a given state to a goal, where "admissible" means that the estimated cost is always less than the true optimal cost. */
            void setCostToGoHeuristic(const CostToGoHeuristic& costToGo);

            /** \brief Uses a cost-to-go heuristic to calculate an admissible estimate of the optimal cost from a given state to a given goal. If no cost-to-go heuristic has been specified with setCostToGoHeuristic(), this function just returns the identity cost, which is sure to be an admissible heuristic if there are no negative costs. */
            Cost costToGo(const State *state, const Goal *goal) const;

            /** \brief Defines an admissible estimate on the optimal cost on the motion between states \e s1 and \e s2. An admissible estimate always undervalues the true optimal cost of the motion. Used by some planners to speed up planning. The default implementation of this method returns this objective's identity cost, which is sure to be an admissible heuristic if there are no negative costs. */
            virtual Cost motionCostHeuristic(const State *s1, const State *s2) const;

            /** \brief Returns this objective's SpaceInformation. Needed for operators in MultiOptimizationObjective */
            const SpaceInformationPtr& getSpaceInformation() const;

        protected:
            /** \brief The space information for this objective */
            SpaceInformationPtr si_;

            /** \brief The description of this optimization objective */
            std::string         description_;

            /** \brief The cost threshold used for checking whether this objective has been satisfied during planning */
            Cost                threshold_;

            /** \brief The function used for returning admissible estimates on the optimal cost of the path between a given state and goal */
            CostToGoHeuristic   costToGoFn_;
        };

        /**
            \brief For use when goal region's distanceGoal() is
            equivalent to the cost-to-go of a state under the
            optimization objective. This function assumes that all states
            within the goal region's threshold have a cost-to-go of
            exactly zero. Note: \e goal is assumed to be of type
            ompl::base::GoalRegion
        */
        Cost goalRegionCostToGo(const State *state, const Goal *goal);

        /** \brief This class allows for the definition of multiobjective optimal planning problems. Objectives are added to this compound object, and motion costs are computed by taking a weighted sum of the individual objective costs. */
        class MultiOptimizationObjective : public OptimizationObjective
        {
        public:
            MultiOptimizationObjective(const SpaceInformationPtr &si);

            /** \brief Adds a new objective for this multiobjective. A weight must also be specified for specifying importance of this objective in planning. */
            void addObjective(const OptimizationObjectivePtr& objective,
                              double weight);

            /** \brief Returns the number of objectives that make up this multiobjective. */
            std::size_t getObjectiveCount() const;

            /** \brief Returns a specific objective from this multiobjective, where the individual objectives are in order of addition to the multiobjective, and \e idx is the zero-based index into this ordering. */
            const OptimizationObjectivePtr& getObjective(unsigned int idx) const;

            /** \brief Returns the weighing factor of a specific objective */
            double getObjectiveWeight(unsigned int idx) const;

            /** \brief Sets the weighing factor of a specific objective */
            void setObjectiveWeight(unsigned int idx, double weight);

            /** \brief This method "freezes" this multiobjective so that no more objectives can be added to it */
            void lock();

            /** \brief Returns whether this multiobjective has been locked from adding further objectives */
            bool isLocked() const;

            /** The default implementation of this method is to use
              addition to add up all the individual objectives' state cost
              values, where each individual value is scaled by its
              weight */
            virtual Cost stateCost(const State *s) const;

            /** The default implementation of this method is to use
              addition to add up all the individual objectives' motion
              cost values, where each individual value is scaled by
              its weight */
            virtual Cost motionCost(const State *s1, const State *s2) const;

        protected:

            /** \brief Defines a pairing of an objective and its weight */
            struct Component
            {
                Component(const OptimizationObjectivePtr& obj, double weight);
                OptimizationObjectivePtr objective;
                double weight;
            };

            /** \brief List of objective/weight pairs */
            std::vector<Component> components_;

            /** \brief Whether this multiobjective is locked from further additions */
            bool                   locked_;

            // Friend functions for operator overloads for easy multiobjective creation
            friend OptimizationObjectivePtr operator+(const OptimizationObjectivePtr &a,
                                                      const OptimizationObjectivePtr &b);

            friend OptimizationObjectivePtr operator*(double w, const OptimizationObjectivePtr &a);

            friend OptimizationObjectivePtr operator*(const OptimizationObjectivePtr &a, double w);
        };

        /** \brief Given two optimization objectives, returns a MultiOptimizationObjective that combines the two objectives with both weights equal to 1.0. */
        OptimizationObjectivePtr operator+(const OptimizationObjectivePtr &a,
                                           const OptimizationObjectivePtr &b);

        /** \brief Given a weighing factor and an optimization objective, returns a MultiOptimizationObjective containing only this objective weighted by the given weight */
        OptimizationObjectivePtr operator*(double w, const OptimizationObjectivePtr &a);

        /** \brief Given a weighing factor and an optimization objective, returns a MultiOptimizationObjective containing only this objective weighted by the given weight */
        OptimizationObjectivePtr operator*(const OptimizationObjectivePtr &a, double w);
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

/* Author: Luis G. Torres */

#ifndef OMPL_BASE_OBJECTIVES_MINIMAX_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_MINIMAX_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief The cost of a path is defined as the worst state
            cost over the entire path. This objective attempts to find
            the path with the "best worst cost" over all paths. */
        class MinimaxObjective : public OptimizationObjective
        {
        public:
            MinimaxObjective(const SpaceInformationPtr &si);

            /** \brief Interpolates between \e s1 and \e s2 to check for
                state costs along the motion between the two
                states. Assumes all costs are worse than identity */
            virtual Cost motionCost(const State *s1, const State *s2) const;

            /** \brief Since we're only concerned about the "worst"
                state cost in the path, combining two costs just
                returns the worse of the two. */
            virtual Cost combineCosts(Cost c1, Cost c2) const;
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

/* Author: Luis G. Torres */

#ifndef OMPL_BASE_OBJECTIVES_MAXIMIZE_MIN_CLEARANCE_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_MAXIMIZE_MIN_CLEARANCE_OBJECTIVE_

#include "ompl/base/objectives/MinimaxObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief Objective for attempting to maximize the minimum clearance along a path. */
        class MaximizeMinClearanceObjective : public MinimaxObjective
        {
        public:
            MaximizeMinClearanceObjective(const SpaceInformationPtr &si);

            /** \brief Defined as the clearance of the state \e s, which is computed using the StateValidityChecker in this objective's SpaceInformation */
            virtual Cost stateCost(const State *s) const;

            /** \brief Since we wish to maximize clearance, and costs are equivalent to path clearance, we return the greater of the two cost values. */
            virtual bool isCostBetterThan(Cost c1, Cost c2) const;

            /** \brief Returns +infinity, since any cost combined with +infinity under this objective will always return the other cost. */
            virtual Cost identityCost() const;

            /** \brief Returns -infinity, since no path clearance value can be considered worse than this. */
            virtual Cost infiniteCost() const;
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

/* Author: Luis G. Torres */

#ifndef OMPL_BASE_OBJECTIVES_MECHANICAL_WORK_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_MECHANICAL_WORK_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective which defines path cost using the idea of mechanical work. To be used in conjunction with TRRT. */
        class MechanicalWorkOptimizationObjective : public OptimizationObjective
        {
        public:
            /** \brief The mechanical work formulation requires a weighing factor to use for the length of a path in order to disambiguate optimal paths. This weighing factor should be small. The default value for this weight is 0.00001. */
            MechanicalWorkOptimizationObjective(const SpaceInformationPtr &si,
                                                double pathLengthWeight = 0.00001);

            /** \brief Set the factor to use for weighing path length in the mechanical work objective formulation. */
            virtual double getPathLengthWeight() const;

            /** \brief Defines motion cost in terms of the mechanical work formulation used for TRRT. */
            virtual Cost motionCost(const State *s1, const State *s2) const;

        protected:
            /** \brief The weighing factor for the path length in the mechanical work objective formulation. */
            double pathLengthWeight_;
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

/* Author: Luis G. Torres */

#ifndef OMPL_BASE_OBJECTIVES_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective which corresponds to optimizing path length. */
        class PathLengthOptimizationObjective : public OptimizationObjective
        {
        public:
            PathLengthOptimizationObjective(const SpaceInformationPtr &si);

            /** \brief Motion cost for this objective is defined as
                the configuration space distance between \e s1 and \e
                s2, using the method SpaceInformation::distance(). */
            virtual Cost motionCost(const State *s1, const State *s2) const;

            /** \brief the motion cost heuristic for this objective is
                simply the configuration space distance between \e s1
                and \e s2, since this is the optimal cost between any
                two states assuming no obstacles. */
            virtual Cost motionCostHeuristic(const State *s1, const State *s2) const;
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

/* Author: Luis G. Torres */

#ifndef OMPL_BASE_OBJECTIVES_STATE_COST_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_STATE_COST_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {

        /** \brief Defines optimization objectives where path cost can
            be represented as a path integral over a cost function
            defined over the state space. This cost function is
            specified by implementing the stateCost() method. */
        class StateCostIntegralObjective : public OptimizationObjective
        {
        public:
            /** \brief If enableMotionCostInterpolation is set to
                true, then calls to motionCost() will divide the
                motion segment into smaller parts (the number of parts
                being defined by StateSpace::validSegmentCount()) for
                more accurate cost integral computation (but this
                takes more computation time). If
                enableMotionCostInterpolation is false (the default),
                only the two endpoint states are used for motion cost
                computation.
            */
            StateCostIntegralObjective(const SpaceInformationPtr &si,
                                       bool enableMotionCostInterpolation = false);

            /** \brief Compute the cost of a path segment from \e s1 to \e s2 (including endpoints)
                \param s1 start state of the motion to be evaluated
                \param s2 final state of the motion to be evaluated
                \param cost the cost of the motion segment

                By default, this function computes
                \f{eqnarray*}{
                \mbox{cost} &=& \frac{cost(s_1) + cost(s_2)}{2}\vert s_1 - s_2 \vert
                \f}

                If enableMotionCostInterpolation was specified as true
                in constructing this object, the cost will be computed
                by separating the motion into
                StateSpace::validSegmentCount() segments, using the
                above formula to compute the cost of each of those
                segments, and adding them up.
            */
            virtual Cost motionCost(const State *s1, const State *s2) const;

            /** \brief Returns whether this objective subdivides
                motions into smaller segments for more accurate motion
                cost computation. Motion cost interpolation is
                disabled by default.
            */
            bool isMotionCostInterpolationEnabled() const;

        protected:

            /** \brief If true, then motionCost() will more accurately compute
                the cost of a motion by taking small steps along the
                motion and accumulating the cost. This sacrifices speed
                for accuracy. If false, the motion cost will be
                approximated by taking the average of the costs at the
                two end points, and normalizing by the distance between
                the two end points. */
            bool interpolateMotionCost_;

            /** \brief Helper method which uses the trapezoidal rule
                to approximate the integral of the cost between two
                states of distance \e dist and costs \e c1 and \e
                c2 */
            Cost trapezoid(Cost c1, Cost c2, double dist) const
            {
                return Cost(0.5 * dist * (c1.v + c2.v));
            }
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

#ifndef OMPL_BASE_PROBLEM_DEFINITION_
#define OMPL_BASE_PROBLEM_DEFINITION_

#include "ompl/base/State.h"
#include "ompl/base/Goal.h"
#include "ompl/base/Path.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/SolutionNonExistenceProof.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include "ompl/base/ScopedState.h"

#include <vector>
#include <cstdlib>
#include <iostream>
#include <limits>

#include <boost/noncopyable.hpp>

namespace ompl
{
    namespace base
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ProblemDefinition */
        OMPL_CLASS_FORWARD(ProblemDefinition);
        /// @endcond

        /** \class ompl::base::ProblemDefinitionPtr
            \brief A boost shared pointer wrapper for ompl::base::ProblemDefinition */

        /** \brief Representation of a solution to a planning problem */
        struct PlannerSolution
        {
            /** \brief Construct a solution that consists of a \e path and its attributes (whether it is \e approximate and the \e difference to the desired goal) */
            PlannerSolution(const PathPtr &path, bool approximate = false, double difference = -1.0, const std::string& plannerName = "") :
                index_(-1), path_(path), length_(path->length()),
                approximate_(approximate), difference_(difference),
                optimized_(false), plannerName_(plannerName)
            {
            }

            /** \brief Return true if two solutions are the same */
            bool operator==(const PlannerSolution& p) const
            {
                return path_ == p.path_;
            }

            /** \brief Define a ranking for solutions */
            bool operator<(const PlannerSolution &b) const
            {
                if (!approximate_ && b.approximate_)
                    return true;
                if (approximate_ && !b.approximate_)
                    return false;
                if (approximate_ && b.approximate_)
                    return difference_ < b.difference_;
                if (optimized_ && !b.optimized_)
                    return true;
                if (!optimized_ && b.optimized_)
                    return false;
                return length_ < b.length_;
            }

            /** \brief When multiple solutions are found, each is given a number starting at 0, so that the order in which the solutions was found can be retrieved. */
            int     index_;

            /** \brief Solution path */
            PathPtr path_;

            /** \brief For efficiency reasons, keep the length of the path as well */
            double  length_;

            /** \brief True if goal was not achieved, but an approximate solution was found */
            bool    approximate_;

            /** \brief The achieved difference between the found solution and the desired goal */
            double  difference_;

            /** \brief True of the solution was optimized to meet the specified optimization criterion */
            bool    optimized_;

            /** \brief Name of planner type that generated this solution, as received from Planner.getName() */
            std::string plannerName_;
        };

        OMPL_CLASS_FORWARD(OptimizationObjective);

        /** \brief Definition of a problem to be solved. This includes
            the start state(s) for the system and a goal specification.
            Will contain solutions, if found.  */
        class ProblemDefinition : private boost::noncopyable
        {
        public:

            /** \brief Create a problem definition given the SpaceInformation it is part of */
            ProblemDefinition(const SpaceInformationPtr &si);

            virtual ~ProblemDefinition()
            {
                clearStartStates();
            }

            /** \brief Get the space information this problem definition is for */
            const SpaceInformationPtr& getSpaceInformation() const
            {
                return si_;
            }

            /** \brief Add a start state. The state is copied. */
            void addStartState(const State *state)
            {
                startStates_.push_back(si_->cloneState(state));
            }

            /** \copydoc addStartState() */
            void addStartState(const ScopedState<> &state)
            {
                startStates_.push_back(si_->cloneState(state.get()));
            }

            /** \brief Check whether a specified starting state is
                already included in the problem definition and
                optionally return the index of that starting state */
            bool hasStartState(const State *state, unsigned int *startIndex = NULL);

            /** \brief Clear all start states (memory is freed) */
            void clearStartStates()
            {
                for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
                    si_->freeState(startStates_[i]);
                startStates_.clear();
            }

            /** \brief Returns the number of start states */
            unsigned int getStartStateCount() const
            {
                return startStates_.size();
            }

            /** \brief Returns a specific start state */
            const State* getStartState(unsigned int index) const
            {
                return startStates_[index];
            }

            /** \copydoc getStartState() */
            State* getStartState(unsigned int index)
            {
                return startStates_[index];
            }

            /** \brief Set the goal. */
            void setGoal(const GoalPtr &goal)
            {
                goal_ = goal;
            }

            /** \brief Clear the goal. Memory is freed. */
            void clearGoal()
            {
                goal_.reset();
            }

            /** \brief Return the current goal */
            const GoalPtr& getGoal() const
            {
                return goal_;
            }

            /** \brief Get all the input states. This includes start
                states and states that are part of goal regions that
                can be casted as ompl::base::GoalState or
                ompl::base::GoalStates. */
            void getInputStates(std::vector<const State*> &states) const;

            /** \brief In the simplest case possible, we have a single
                starting state and a single goal state.

                This function simply configures the problem definition
                using these states (performs the needed calls to
                addStartState(), creates an instance of
                ompl::base::GoalState and calls setGoal() on it. */
            void setStartAndGoalStates(const State *start, const State *goal, const double threshold = std::numeric_limits<double>::epsilon());

            /** \brief A simple form of setting the goal. This is called by setStartAndGoalStates(). A more general form is setGoal() */
            void setGoalState(const State *goal, const double threshold = std::numeric_limits<double>::epsilon());

            /** \copydoc setStartAndGoalStates() */
            void setStartAndGoalStates(const ScopedState<> &start, const ScopedState<> &goal, const double threshold = std::numeric_limits<double>::epsilon())
            {
                setStartAndGoalStates(start.get(), goal.get(), threshold);
            }

            /** \copydoc setGoalState() */
            void setGoalState(const ScopedState<> &goal, const double threshold = std::numeric_limits<double>::epsilon())
            {
                setGoalState(goal.get(), threshold);
            }

            /** \brief Check if an optimization objective was defined for planning  */
            bool hasOptimizationObjective() const
            {
                return optimizationObjective_.get();
            }

            /** \brief Get the optimization objective to be considered during planning */
            const OptimizationObjectivePtr& getOptimizationObjective() const
            {
                return optimizationObjective_;
            }

            /** \brief Set the optimization objective to be considered during planning */
            void setOptimizationObjective(const OptimizationObjectivePtr &optimizationObjective)
            {
                optimizationObjective_ = optimizationObjective;
            }

            /** \brief A problem is trivial if a given starting state already
                in the goal region, so we need no motion planning. startID
                will be set to the index of the starting state that
                satisfies the goal. The distance to the goal can
                optionally be returned as well. */
            bool isTrivial(unsigned int *startIndex = NULL, double *distance = NULL) const;

            /** \brief Check if a straight line path is valid. If it
                is, return an instance of a path that represents the
                straight line.

                \note When planning under geometric constraints, this
                works only if the goal region can be sampled. If the
                goal region cannot be sampled, this call is equivalent
                to calling isTrivial()

                \note When planning under differential constraints,
                the system is propagated forward in time using the
                null control. */
            PathPtr isStraightLinePathValid() const;

            /** \brief Many times the start or goal state will barely touch an obstacle. In this case, we may want to automatically
              * find a nearby state that is valid so motion planning can be performed. This function enables this behaviour.
              * The allowed distance for both start and goal states is specified. The number of attempts
              * is also specified. Returns true if all states are valid after completion. */
            bool fixInvalidInputStates(double distStart, double distGoal, unsigned int attempts);

            /** \brief Returns true if a solution path has been found (could be approximate) */
            bool hasSolution() const;

            /** \brief Return true if the top found solution is
                approximate (does not actually reach the desired goal,
                but hopefully is closer to it) */
            bool hasApproximateSolution() const;

            /** \brief Get the distance to the desired goal for the top solution. Return -1.0 if there are no solutions available. */
            double getSolutionDifference() const;

            /** \brief Return true if the top found solution is optimized (satisfies the specified optimization objective) */
            bool hasOptimizedSolution() const;

            /** \brief Return the top solution path, if one is found. The top path is the shortest
                 one that was found, preference being given to solutions that are not approximate.

                This will need to be casted into the specialization computed by the planner */
            PathPtr getSolutionPath() const;

            /** \brief Return true if a top solution is found, with the top solution passed by reference in the function header
                 The top path is the shortest one that was found, preference being given to solutions that are not approximate.
                This will need to be casted into the specialization computed by the planner */
            bool getSolution(PlannerSolution& solution) const;

            /** \brief Add a solution path in a thread-safe manner. Multiple solutions can be set for a goal.
                If a solution does not reach the desired goal it is considered approximate.
                Optionally, the distance between the desired goal and the one actually achieved is set by \e difference.
                Optionally, the name of the planner that generated the solution
            */
            void addSolutionPath(const PathPtr &path, bool approximate = false, double difference = -1.0, const std::string& plannerName = "Unknown") const;

            /** \brief Add a solution path in a thread-safe manner. Multiple solutions can be set for a goal. */
            void addSolutionPath(const PlannerSolution &sol) const;

            /** \brief Get the number of solutions already found */
            std::size_t getSolutionCount() const;

            /** \brief Get all the solution paths available for this goal */
            std::vector<PlannerSolution> getSolutions() const;

            /** \brief Forget the solution paths (thread safe). Memory is freed. */
            void clearSolutionPaths() const;

            /** \brief Returns true if the problem definition has a proof of non existence for a solution */
            bool hasSolutionNonExistenceProof() const;

            /** \brief Removes any existing instance of SolutionNonExistenceProof */
            void clearSolutionNonExistenceProof();

            /** \brief Retrieve a pointer to the SolutionNonExistenceProof instance for this problem definition */
            const SolutionNonExistenceProofPtr& getSolutionNonExistenceProof() const;

            /** \brief Set the instance of SolutionNonExistenceProof for this problem definition */
            void setSolutionNonExistenceProof(const SolutionNonExistenceProofPtr& nonExistenceProof);

            /** \brief Print information about the start and goal states and the optimization objective */
            void print(std::ostream &out = std::cout) const;

        protected:

            /** \brief Helper function for fixInvalidInputStates(). Attempts to fix an individual state */
            bool fixInvalidInputState(State *state, double dist, bool start, unsigned int attempts);

            /** \brief The space information this problem definition is for */
            SpaceInformationPtr          si_;

            /** \brief The set of start states */
            std::vector<State*>          startStates_;

            /** \brief The goal representation */
            GoalPtr                      goal_;

            /** \brief A Representation of a proof of non-existence of a solution for this problem definition */
            SolutionNonExistenceProofPtr nonExistenceProof_;

            /** \brief The objective to be optimized while solving the planning problem */
            OptimizationObjectivePtr     optimizationObjective_;

        private:

            /// @cond IGNORE
            OMPL_CLASS_FORWARD(PlannerSolutionSet);
            /// @endcond

            /** \brief The set of solutions computed for this goal (maintains an array of PlannerSolution) */
            PlannerSolutionSetPtr        solutions_;
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_PLANNER_TERMINATION_CONDITION_
#define OMPL_BASE_PLANNER_TERMINATION_CONDITION_

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

namespace ompl
{

    namespace base
    {

        /** \brief Signature for functions that decide whether termination
            conditions have been met for a planner, even if no
            solution is found. This is usually reaching a time or
            memory limit. If the function returns true, the planner is
            signaled to terminate its computation. Otherwise,
            computation continues while this function returns false,
            until a solution is found. */
        typedef boost::function<bool()> PlannerTerminationConditionFn;

        /** \brief Encapsulate a termination condition for a motion
            planner. Planners will call operator() to decide whether
            they should terminate before a solution is found or
            not. operator() will return true if either the implemented
            condition is met (the call to eval() returns true) or if
            the user called terminate(true). */
        class PlannerTerminationCondition
        {
        public:

            /** \brief Construct a termination condition. By default, eval() will call the externally specified function \e fn to decide whether
                the planner should terminate. */
            PlannerTerminationCondition(const PlannerTerminationConditionFn &fn);

            /** \brief Construct a termination condition that is evaluated every \e period seconds. The evaluation of
                the condition consists of calling \e fn() in a separate thread. Calls to eval() will always return the
        last value computed by the call to \e fn(). */
            PlannerTerminationCondition(const PlannerTerminationConditionFn &fn, double period);

            ~PlannerTerminationCondition()
            {
            }

            /** \brief Return true if the planner should stop its computation */
            bool operator()() const
            {
                return eval();
            }

            /** \brief Cast as true if the planner should stop its computation */
            operator bool() const
            {
                return eval();
            }

            /** \brief Notify that the condition for termination should become true, regardless of what eval() returns.
                This function may be called while the condition is being evaluated by other threads. */
            void terminate() const;

            /** \brief The implementation of some termination condition. By default, this just calls \e fn_() */
            bool eval() const;

        private:

            class PlannerTerminationConditionImpl;
            boost::shared_ptr<PlannerTerminationConditionImpl> impl_;
        };

        /** \brief Simple termination condition that always returns false. The termination condition will never be met */
        PlannerTerminationCondition plannerNonTerminatingCondition();

        /** \brief Simple termination condition that always returns true. The termination condition will always be met */
        PlannerTerminationCondition plannerAlwaysTerminatingCondition();

        /** \brief Combine two termination conditions into one. If either termination condition returns true, this one will return true as well. */
        PlannerTerminationCondition plannerOrTerminationCondition(const PlannerTerminationCondition &c1, const PlannerTerminationCondition &c2);

        /** \brief Combine two termination conditions into one. Both termination conditions need to return true for this one to return true. */
        PlannerTerminationCondition plannerAndTerminationCondition(const PlannerTerminationCondition &c1, const PlannerTerminationCondition &c2);

        /** \brief Return a termination condition that will become true \e duration seconds in the future (wall-time) */
        PlannerTerminationCondition timedPlannerTerminationCondition(double duration);

        /** \brief Return a termination condition that will become true \e duration seconds in the future (wall-time), but is checked in a separate thread, every \e interval seconds; \e interval must be less than \e duration */
        PlannerTerminationCondition timedPlannerTerminationCondition(double duration, double interval);
    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Ryan Luna, Luis G. Torres */

#ifndef OMPL_BASE_PLANNER_DATA_
#define OMPL_BASE_PLANNER_DATA_

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include "ompl/base/State.h"
#include "ompl/base/Cost.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/serialization/access.hpp>

namespace ompl
{
    namespace base
    {
        /// \brief Base class for a vertex in the PlannerData structure.  All
        /// derived classes must implement the clone and equivalence operators.
        /// It is assumed that each vertex in the PlannerData structure is
        /// unique (i.e. no duplicates allowed).
        class PlannerDataVertex
        {
        public:
            /// \brief Constructor.  Takes a state pointer and an optional integer tag.
            PlannerDataVertex(const State *st, int tag = 0) : state_(st), tag_(tag) {}
            /// \brief Copy constructor.
            PlannerDataVertex(const PlannerDataVertex &rhs) : state_(rhs.state_), tag_(rhs.tag_) {}
            virtual ~PlannerDataVertex() {}

            /// \brief Returns the integer tag associated with this vertex.
            virtual int  getTag() const { return tag_; }
            /// \brief Set the integer tag associated with this vertex.
            virtual void setTag(int tag) { tag_ = tag; }
            /// \brief Retrieve the state associated with this vertex.
            virtual const State* getState() const { return state_; }

            /// \brief Return a clone of this object, allocated from the heap.
            virtual PlannerDataVertex* clone() const
            {
                return new PlannerDataVertex(*this);
            }

            /// \brief Equivalence operator.  Return true if the state pointers are equal.
            virtual bool operator==(const PlannerDataVertex &rhs) const
            {
                // States should be unique
                return state_ == rhs.state_;
            }

            /// \brief Returns true if this vertex is not equal to the argument.
            /// This is the complement of the == operator.
            bool operator!=(const PlannerDataVertex &rhs) const
            {
                return !(*this == rhs);
            }

        protected:
            PlannerDataVertex() {}

            friend class boost::serialization::access;
            template <class Archive>
            void serialize(Archive & ar, const unsigned int /*version*/)
            {
                ar & tag_;
                // Serialization of the state pointer is handled by PlannerDataStorage
            }

            /// \brief The state represented by this vertex
            const State *state_;
            /// \brief A generic integer tag for this state.  Not used for equivalence checking.
            int tag_;

            friend class PlannerData;
            friend class PlannerDataStorage;
        };

        /// \brief Base class for a PlannerData edge.
        class PlannerDataEdge
        {
        public:
            PlannerDataEdge() {}
            virtual ~PlannerDataEdge() {}
            /// \brief Return a clone of this object, allocated from the heap.
            virtual PlannerDataEdge* clone() const { return new PlannerDataEdge(); }

            /// \brief Returns true if the edges point to the same memory
            virtual bool operator==(const PlannerDataEdge &rhs) const
            {
                return this == &rhs;
            }

            /// \brief Returns true if the edges do not point to the same memory.
            /// This is the complement of the == operator.
            bool operator!=(const PlannerDataEdge &rhs) const
            {
                return !(*this == rhs);
            }

        protected:

            friend class boost::serialization::access;
            template <class Archive>
            void serialize(Archive & /*ar*/, const unsigned int /*version*/)
            {
            }
        };

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(StateStorage);
        OMPL_CLASS_FORWARD(PlannerData);

        // Forward declaration for PlannerData::computeEdgeWeights
        class OptimizationObjective;
        /// @endcond

        /** \class ompl::base::PlannerDataPtr
            \brief A boost shared pointer wrapper for ompl::base::PlannerData */


        /// \brief Object containing planner generated vertex and edge data.  It
        /// is assumed that all vertices are unique, and only a single directed
        /// edge connects two vertices.
        /// \note The storage for states this class maintains belongs to the planner
        /// instance that filled the data (by default; see PlannerData::decoupleFromPlanner())
        class PlannerData : boost::noncopyable
        {
        public:
            class Graph;

            /// \brief Representation for a non-existant edge
            static const PlannerDataEdge   NO_EDGE;
            /// \brief Representation for a non-existant vertex
            static const PlannerDataVertex NO_VERTEX;
            /// \brief Representation of an invalid vertex index
            static const unsigned int      INVALID_INDEX;

            /// \brief Constructor.  Accepts a SpaceInformationPtr for the space planned in.
            PlannerData(const SpaceInformationPtr &si);
            /// \brief Destructor.
            virtual ~PlannerData();

            /// \name PlannerData construction
            /// \{

            /// \brief Adds the given vertex to the graph data.  The vertex index
            /// is returned.  Duplicates are not added.  If a vertex is duplicated,
            /// the index of the existing vertex is returned instead.
            /// Indexes are volatile and may change after adding/removing a subsequent vertex.
            unsigned int addVertex(const PlannerDataVertex &st);
            /// \brief Adds the given vertex to the graph data, and marks it as a
            /// start vertex.  The vertex index is returned.  Duplicates are not added.
            /// If a vertex is duplicated, the index of the existing vertex is returned instead.
            /// Indexes are volatile and may change after adding/removing a subsequent vertex.
            unsigned int addStartVertex(const PlannerDataVertex &v);
            /// \brief Adds the given vertex to the graph data, and marks it as a
            /// start vertex.  The vertex index is returned.  Duplicates are not added.
            /// If a vertex is duplicated, the index of the existing vertex is returned instead.
            /// Indexes are volatile and may change after adding/removing a subsequent vertex.
            unsigned int addGoalVertex(const PlannerDataVertex &v);
            /// \brief Mark the given state as a start vertex.  If the given state does not exist in a
            /// vertex, false is returned.
            bool markStartState(const State *st);
            /// \brief Mark the given state as a goal vertex.  If the given state does not exist in a
            /// vertex, false is returned.
            bool markGoalState(const State *st);
            /// \brief Set the integer tag associated with the given state.  If the given
            /// state does not exist in a vertex, false is returned.
            bool tagState(const State *st, int tag);
            /// \brief Removes the vertex associated with the given data.  If the
            /// vertex does not exist, false is returned.
            /// This method has O(n) complexity in the number of vertices.
            virtual bool removeVertex(const PlannerDataVertex &st);
            /// \brief Removes the vertex with the given index.  If the index is
            /// out of range, false is returned.
            /// This method has O(n) complexity in the number of vertices.
            virtual bool removeVertex(unsigned int vIndex);
            /// \brief Adds a directed edge between the given vertex indexes.  An optional
            /// edge structure and weight can be supplied.  Success is returned.
            virtual bool addEdge(unsigned int v1, unsigned int v2,
                                 const PlannerDataEdge &edge = PlannerDataEdge(),
                                 Cost weight = Cost(1.0));
            /// \brief Adds a directed edge between the given vertex indexes.  The
            /// vertices are added to the data if they are not already in the
            /// structure.  An optional edge structure and weight can also be supplied.
            /// Success is returned.
            virtual bool addEdge(const PlannerDataVertex &v1, const PlannerDataVertex &v2,
                                 const PlannerDataEdge &edge = PlannerDataEdge(),
                                 Cost weight = Cost(1.0));
            /// \brief Removes the edge between vertex indexes \e v1 and \e v2.  Success is returned.
            virtual bool removeEdge(unsigned int v1, unsigned int v2);
            /// \brief Removes the edge between the vertices associated with the given vertex data.
            /// Success is returned.
            virtual bool removeEdge(const PlannerDataVertex &v1, const PlannerDataVertex &v2);
            /// \brief Clears the entire data structure
            virtual void clear();
            /// \brief Creates a deep copy of the states contained in the vertices of this
            /// PlannerData structure so that when the planner that created this instance goes
            /// out of scope, all data remains intact.
            /// \remarks Shallow state pointers inside of the PlannerDataVertex objects already
            /// in this PlannerData will be replaced with clones which are scoped to this PlannerData
            /// object.  A subsequent call to this method is necessary after any other vertices are
            /// added to ensure that this PlannerData instance is fully decoupled.
            virtual void decoupleFromPlanner();

            /// \}
            /// \name PlannerData Properties
            /// \{

            /// \brief Retrieve the number of edges in this structure
            unsigned int numEdges() const;
            /// \brief Retrieve the number of vertices in this structure
            unsigned int numVertices() const;
            /// \brief Returns the number of start vertices
            unsigned int numStartVertices() const;
            /// \brief Returns the number of goal vertices
            unsigned int numGoalVertices() const;

            /// \}
            /// \name PlannerData vertex lookup
            /// \{

            /// \brief Check whether a vertex exists with the given vertex data
            bool vertexExists(const PlannerDataVertex &v) const;
            /// \brief Retrieve a reference to the vertex object with the given
            /// index.  If this vertex does not exist, NO_VERTEX is returned.
            const PlannerDataVertex& getVertex(unsigned int index) const;
            /// \brief Retrieve a reference to the vertex object with the given
            /// index.  If this vertex does not exist, NO_VERTEX is returned.
            PlannerDataVertex& getVertex(unsigned int index);
            /// \brief Retrieve a reference to the ith start vertex object.  If
            /// \e i is greater than the number of start vertices, NO_VERTEX is returned.
            const PlannerDataVertex& getStartVertex(unsigned int i) const;
            /// \brief Retrieve a reference to the ith start vertex object.  If
            /// \e i is greater than the number of start vertices, NO_VERTEX is returned.
            PlannerDataVertex& getStartVertex(unsigned int i);
            /// \brief Retrieve a reference to the ith goal vertex object.  If
            /// \e i is greater than the number of goal vertices, NO_VERTEX is returned.
            const PlannerDataVertex& getGoalVertex(unsigned int i) const;
            /// \brief Retrieve a reference to the ith goal vertex object.  If
            /// \e i is greater than the number of goal vertices, NO_VERTEX is returned.
            PlannerDataVertex& getGoalVertex(unsigned int i);
            /// \brief Returns the index of the ith start state.
            /// INVALID_INDEX is returned if \e i is out of range.
            /// Indexes are volatile and may change after adding/removing a vertex.
            unsigned int getStartIndex(unsigned int i) const;
            /// \brief Returns the index of the ith goal state.
            /// INVALID_INDEX is returned if \e i is out of range
            /// Indexes are volatile and may change after adding/removing a vertex.
            unsigned int getGoalIndex(unsigned int i) const;
            /// \brief Returns true if the given vertex index is marked as a start vertex
            bool isStartVertex(unsigned int index) const;
            /// \brief Returns true if the given vertex index is marked as a goal vertex
            bool isGoalVertex(unsigned int index) const;
            /// \brief Return the index for the vertex associated with the given data.
            /// INVALID_INDEX is returned if this vertex does not exist.
            /// Indexes are volatile and may change after adding/removing a vertex.
            unsigned int vertexIndex(const PlannerDataVertex &v) const;

            /// \}
            /// \name PlannerData edge lookup
            /// \{

            /// \brief Check whether an edge between vertex index \e v1 and index \e v2 exists
            bool edgeExists(unsigned int v1, unsigned int v2) const;
            /// \brief Retrieve a reference to the edge object connecting vertices
            /// with indexes \e v1 and \e v2. If this edge does not exist, NO_EDGE is returned.
            const PlannerDataEdge& getEdge(unsigned int v1, unsigned int v2) const;
            /// \brief Retrieve a reference to the edge object connecting vertices
            /// with indexes \e v1 and \e v2. If this edge does not exist, NO_EDGE is returned.
            PlannerDataEdge& getEdge(unsigned int v1, unsigned int v2);
            /// \brief Returns a list of the vertex indexes directly connected to
            /// vertex with index \e v (outgoing edges).  The number of outgoing
            /// edges from \e v is returned.
            unsigned int getEdges(unsigned int v, std::vector<unsigned int>& edgeList) const;
            /// \brief Returns a map of outgoing edges from vertex with index \e v.
            /// Key = vertex index, value = edge structure.  The number of outgoing edges from \e v is returned
            unsigned int getEdges(unsigned int v, std::map<unsigned int, const PlannerDataEdge*> &edgeMap) const;
            /// \brief Returns a list of vertices with outgoing edges to the vertex with index \e v.
            /// The number of edges connecting to \e v is returned.
            unsigned int getIncomingEdges(unsigned int v, std::vector<unsigned int>& edgeList) const;
            /// \brief Returns a map of incoming edges to the vertex with index \e v (i.e. if there is an
            /// edge from w to v, w and the edge structure will be in the map.)
            /// Key = vertex index, value = edge structure.  The number of incoming edges to \e v is returned
            unsigned int getIncomingEdges(unsigned int v, std::map<unsigned int, const PlannerDataEdge*> &edgeMap) const;
            /// \brief Returns the weight of the edge between the
            /// given vertex indices.  If there exists an edge between
            /// \e v1 and \v2, the edge weight is placed in the
            /// out-variable \e weight. Otherwise, this function
            /// returns false.
            bool getEdgeWeight(unsigned int v1, unsigned int v2, Cost* weight) const;
            /// \brief Sets the weight of the edge between the given
            /// vertex indices.  If an edge between v1 and v2 does not
            /// exist, this function returns false.
            bool setEdgeWeight(unsigned int v1, unsigned int v2, Cost weight);
            /// \brief Computes the weight for all edges given the
            /// OptimizationObjective \e opt.
            void computeEdgeWeights(const OptimizationObjective &opt);
            /// \brief Computes all edge weights using state space
            /// distance (i.e. getSpaceInformation()->distance())
            void computeEdgeWeights();

            /// \}
            /// \name Output methods
            /// \{

            /// \brief Writes a Graphviz dot file of this structure to the given stream
            void printGraphviz(std::ostream& out = std::cout) const;

            /// \brief Writes a GraphML file of this structure to the given stream
            void printGraphML(std::ostream& out = std::cout) const;

            /// \}
            /// \name Advanced graph extraction
            /// \{

            /// \brief Extracts the minimum spanning tree of the data rooted at the vertex
            /// with index \e v.  The minimum spanning tree is saved into \e mst.
            /// O(|E| log |V|) complexity.
            void extractMinimumSpanningTree(unsigned int v,
                                            const OptimizationObjective &opt,
                                            PlannerData &mst) const;
            /// \brief Extracts the subset of PlannerData reachable from the vertex with index
            /// v.  For tree structures, this will be the sub-tree rooted at v. The reachable set
            /// is saved into \e data.
            void extractReachable(unsigned int v, PlannerData &data) const;

            /// \brief Extract a ompl::base::GraphStateStorage object from this PlannerData. Memory for states is copied
            /// (the resulting ompl::base::StateStorage is independent from this PlannerData)
            StateStoragePtr extractStateStorage() const;

            /// \brief Extract a Boost.Graph object from this PlannerData.
            /// \remarks Use of this method requires inclusion of PlannerDataGraph.h  The object
            /// returned can be used safely for all read-only purposes in Boost.  Adding or
            /// removing vertices and edges should be performed by using the respective method
            /// in PlannerData to ensure proper memory management.  Manipulating the graph directly
            /// will result in undefined behavior with this class.
            Graph& toBoostGraph();
            /// \brief Extract a Boost.Graph object from this PlannerData.
            /// \remarks Use of this method requires inclusion of PlannerDataGraph.h  The object
            /// returned can be used safely for all read-only purposes in Boost.  Adding or
            /// removing vertices and edges should be performed by using the respective method
            /// in PlannerData to ensure proper memory management.  Manipulating the graph directly
            /// will result in undefined behavior with this class.
            const Graph& toBoostGraph() const;

            /// \}

            /// \brief Return the instance of SpaceInformation used in this PlannerData
            const SpaceInformationPtr& getSpaceInformation() const;

          /// \brief Indicate whether any information about controls (ompl::control::Control) is stored in this instance
            virtual bool hasControls() const;

            /// \brief Any extra properties (key-value pairs) the planner can set.
            std::map<std::string, std::string>   properties;

        protected:
            /// \brief A mapping of states to vertex indexes.  For fast lookup of vertex index.
            std::map<const State*, unsigned int> stateIndexMap_;
            /// \brief A mutable listing of the vertices marked as start states.  Stored in sorted order.
            std::vector<unsigned int>            startVertexIndices_;
            /// \brief A mutable listing of the vertices marked as goal states.  Stored in sorted order.
            std::vector<unsigned int>            goalVertexIndices_;

            /// \brief The space information instance for this data.
            SpaceInformationPtr                  si_;
            /// \brief A list of states that are allocated during the decoupleFromPlanner method.
            /// These states are freed by PlannerData in the destructor.
            std::set<State*>                     decoupledStates_;

        private:
            void freeMemory();

            // Abstract pointer that points to the Boost.Graph structure.
            // Obscured to prevent unnecessary inclusion of BGL throughout the
            // rest of the code.
            void* graphRaw_;
        };
    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Ryan Luna */

#ifndef OMPL_BASE_PLANNER_DATA_STORAGE_
#define OMPL_BASE_PLANNER_DATA_STORAGE_

// PlannerDataStorage requires Boost version >= 1.44
#include <boost/version.hpp>
#if BOOST_VERSION < 104400
#warning Boost version >= 1.44 is required for PlannerDataStorage classes
#else

#include "ompl/base/PlannerData.h"
#include "ompl/util/Console.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <fstream>

namespace ompl
{
    namespace base
    {
        /// \brief Object that handles loading/storing a PlannerData object to/from a binary stream.
        /// Serialization of vertices and edges is performed using the Boost archive method
        /// \e serialize.  Derived vertex/edge classes are handled, presuming those classes implement
        /// the \e serialize method.
        /// \remarks Since the \e serialize method for vertices and edges is templated, it cannot
        /// be virtual.  To serialize a derived class AND the base class data, a special call can
        /// be invoked inside of \e serialize that instructs the serializer to also serialize the
        /// base class.  The derived class must also have a GUID exposed to the serializer
        /// for proper deserialization at runtime.  This is performed with the \e BOOST_CLASS_EXPORT
        /// macro.  An example of these items is given below:
        /// \code
        /// #include <boost/serialization/export.hpp>
        ///
        /// class MyVertexClass : public ompl::base::PlannerDataVertex
        /// {
        ///     // ---SNIP---
        ///
        ///     template <class Archive>
        ///     void serialize(Archive & ar, const unsigned int version)
        ///     {
        ///         ar & boost::serialization::base_object<ompl::base::PlannerDataVertex>(*this);
        ///         // ... (The other members of MyVertexClass)
        ///     }
        /// };
        ///
        /// BOOST_CLASS_EXPORT(MyVertexClass);
        /// \endcode
        class PlannerDataStorage
        {
        public:

            /// \brief Default constructor.
            PlannerDataStorage();
            /// \brief Destructor
            virtual ~PlannerDataStorage();

            /// \brief Store (serialize) the PlannerData structure to the given filename.
            virtual void store(const PlannerData &pd, const char *filename);

            /// \brief Store (serialize) the PlannerData structure to the given stream.
            virtual void store(const PlannerData &pd, std::ostream &out);

            /// \brief Load the PlannerData structure from the given stream.
            /// The StateSpace that was used to store the data must match the
            /// StateSpace inside of the argument PlannerData.
            virtual void load(const char *filename, PlannerData &pd);

            /// \brief Load the PlannerData structure from the given stream.
            /// The StateSpace that was used to store the data must match the
            /// StateSpace inside of the argument PlannerData.
            virtual void load(std::istream &in, PlannerData &pd);

        protected:
            /// \brief Information stored at the beginning of the PlannerData archive
            struct Header
            {
                /// \brief OMPL PlannerData specific marker (fixed value)
                boost::uint32_t  marker;

                /// \brief Number of vertices stored in the archive
                std::size_t      vertex_count;

                /// \brief Number of edges stored in the archive
                std::size_t      edge_count;

                /// \brief Signature of state space that allocated the saved states in the vertices (see ompl::base::StateSpace::computeSignature()) */
                std::vector<int> signature;

                /// \brief boost::serialization routine
                template<typename Archive>
                void serialize(Archive & ar, const unsigned int /*version*/)
                {
                    ar & marker;
                    ar & vertex_count;
                    ar & edge_count;
                    ar & signature;
                }
            };

            /// \brief The object containing all vertex data that will be stored
            struct PlannerDataVertexData
            {
                enum VertexType
                {
                    STANDARD = 0,
                    START,
                    GOAL
                };

                template<typename Archive>
                void serialize(Archive & ar, const unsigned int /*version*/)
                {
                    ar & v_;
                    ar & state_;
                    ar & type_;
                }

                const PlannerDataVertex *v_;
                std::vector<unsigned char> state_;
                VertexType type_;
            };

            /// \brief The object containing all edge data that will be stored
            struct PlannerDataEdgeData
            {
                template<typename Archive>
                void serialize(Archive & ar, const unsigned int /*version*/)
                {
                    ar & e_;
                    ar & endpoints_;
                    ar & weight_;
                }

                const PlannerDataEdge *e_;
                std::pair<unsigned int, unsigned int> endpoints_;
                double weight_;
            };

            /// \brief Read \e numVertices from the binary input \e ia and store them as PlannerData.
            virtual void loadVertices(PlannerData &pd, unsigned int numVertices, boost::archive::binary_iarchive &ia)
            {
                const StateSpacePtr &space = pd.getSpaceInformation()->getStateSpace();
                std::vector<State*> states;
                for (unsigned int i = 0; i < numVertices; ++i)
                {
                    PlannerDataVertexData vertexData;
                    ia >> vertexData;

                    // Deserializing all data in the vertex (except the state)
                    const PlannerDataVertex *v = vertexData.v_;

                    // Allocating a new state and deserializing it from the buffer
                    State *state = space->allocState();
                    states.push_back(state);
                    space->deserialize (state, &vertexData.state_[0]);
                    const_cast<PlannerDataVertex*>(v)->state_ = state;

                    // Record the type of the vertex (i.e. start vertex).
                    if (vertexData.type_ == PlannerDataVertexData::START)
                        pd.addStartVertex(*v);
                    else if (vertexData.type_ == PlannerDataVertexData::GOAL)
                        pd.addGoalVertex(*v);
                    else
                        pd.addVertex(*v);

                    // We deserialized the vertex object pointer, and we own it.
                    // Since addEdge copies the object, it is safe to free here.
                    delete vertexData.v_;
                }

                // These vertices are using state pointers allocated here.
                // To avoid a memory leak, we decouple planner data from the
                // 'planner', which will clone all states and properly free the
                // memory when PlannerData goes out of scope.  Then it is safe
                // to free all memory allocated here.
                pd.decoupleFromPlanner();

                for (size_t i = 0; i < states.size(); ++i)
                    space->freeState(states[i]);
            }

            /// \brief Serialize and store all vertices in \e pd to the binary archive.
            virtual void storeVertices(const PlannerData &pd, boost::archive::binary_oarchive &oa)
            {
                const StateSpacePtr &space = pd.getSpaceInformation()->getStateSpace();
                std::vector<unsigned char> state (space->getSerializationLength());
                for (unsigned int i = 0; i < pd.numVertices(); ++i)
                {
                    PlannerDataVertexData vertexData;

                    // Serializing all data in the vertex (except the state)
                    const PlannerDataVertex &v = pd.getVertex(i);
                    vertexData.v_ = &v;

                    // Record the type of the vertex (i.e. start vertex).
                    if (pd.isStartVertex(i))
                        vertexData.type_ = PlannerDataVertexData::START;
                    else if (pd.isGoalVertex(i))
                        vertexData.type_ = PlannerDataVertexData::GOAL;
                    else vertexData.type_ = PlannerDataVertexData::STANDARD;

                    // Serializing the state contained in this vertex
                    space->serialize (&state[0], v.getState());
                    vertexData.state_ = state;

                    oa << vertexData;
                }
            }

            /// \brief Read \e numEdges from the binary input \e ia and store them as PlannerData.
            virtual void loadEdges(PlannerData &pd, unsigned int numEdges, boost::archive::binary_iarchive &ia)
            {
                for (unsigned int i = 0; i < numEdges; ++i)
                {
                    PlannerDataEdgeData edgeData;
                    ia >> edgeData;
                    pd.addEdge(edgeData.endpoints_.first, edgeData.endpoints_.second, *edgeData.e_, Cost(edgeData.weight_));

                    // We deserialized the edge object pointer, and we own it.
                    // Since addEdge copies the object, it is safe to free here.
                    delete edgeData.e_;
                }
            }

            /// \brief Serialize and store all edges in \e pd to the binary archive.
            virtual void storeEdges(const PlannerData &pd, boost::archive::binary_oarchive &oa)
            {
                for (unsigned int i = 0; i < pd.numVertices(); ++i)
                    for (unsigned int j = 0; j < pd.numVertices(); ++j)
                    {
                        if(pd.edgeExists(i, j))
                        {
                            PlannerDataEdgeData edgeData;
                            edgeData.e_ = &pd.getEdge(i, j);
                            edgeData.endpoints_.first = i;
                            edgeData.endpoints_.second = j;
                            Cost weight;
                            pd.getEdgeWeight(i, j, &weight);
                            edgeData.weight_ = weight.v;

                            oa << edgeData;
                        }
                    }
            }
        };
    }
}

#endif

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

#ifndef OMPL_BASE_PLANNER_STATUS_
#define OMPL_BASE_PLANNER_STATUS_

#include <string>
#include <ostream>

namespace ompl
{
    namespace base
    {
        /// A class to store the exit status of Planner::solve()
        struct PlannerStatus
        {
            /// The possible values of the status returned by a planner
            enum StatusType
            {
                /// Uninitialized status
                UNKNOWN = 0,
                /// Invalid start state or no start state specified
                INVALID_START,
                /// Invalid goal state
                INVALID_GOAL,
                /// The goal is of a type that a planner does not recognize
                UNRECOGNIZED_GOAL_TYPE,
                /// The planner failed to find a solution
                TIMEOUT,
                /// The planner found an approximate solution
                APPROXIMATE_SOLUTION,
                /// The planner found an exact solution
                EXACT_SOLUTION,
                /// The planner crashed
                CRASH,
                /// The number of possible status values
                TYPE_COUNT
            };

            /// Default constructor
            PlannerStatus(StatusType status = UNKNOWN) : status_(status)
            {
            }

            /// Convenience constructor that sets status_ based on whether some solution was found (\e hasSolution) and whether that solution was approximate or not (\e isApproximate)
            PlannerStatus(bool hasSolution, bool isApproximate)
                : status_ (hasSolution ? (isApproximate ? APPROXIMATE_SOLUTION : EXACT_SOLUTION) : TIMEOUT)
            {
            }

            /// Return a string representation
            std::string asString() const;
            /// Allow casting to true. The value is true iff an approximate or exact solution was found
            operator bool() const
            {
                return status_ == APPROXIMATE_SOLUTION || status_ == EXACT_SOLUTION;
            }
            /// Allow casting to the enum type StatusType
            operator StatusType() const
            {
                return status_;
            }

        private:
            /// Exit status of calling Planner::solve()
            StatusType status_;
        };

        /// Print a PlannerStatus object
        inline std::ostream& operator<<(std::ostream& out, const PlannerStatus& status)
        {
            return out << status.asString();
        }
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

#ifndef OMPL_BASE_PLANNER_
#define OMPL_BASE_PLANNER_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerStatus.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/GenericParam.h"
#include "ompl/util/Console.h"
#include "ompl/util/Time.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/Deprecation.h"
#include <boost/function.hpp>
#include <boost/concept_check.hpp>
#include <boost/noncopyable.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <map>

namespace ompl
{

    namespace base
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Planner */
        OMPL_CLASS_FORWARD(Planner);
        /// @endcond

        /** \class ompl::base::PlannerPtr
            \brief A boost shared pointer wrapper for ompl::base::Planner */


        /** \brief Helper class to extract valid start & goal
            states. Usually used internally by planners.

            This class is meant to behave correctly if the user
            updates the problem definition between subsequent calls to
            ompl::base::Planner::solve() \b without calling
            ompl::base::Planner::clear() in between. Only allowed
            changes to the problem definition are accounted for:
            adding of starring states or adding of goal states for
            instances inherited from
            ompl::base::GoalSampleableRegion. */
        class PlannerInputStates
        {
        public:

            /** \brief Default constructor. No work is performed. */
            PlannerInputStates(const PlannerPtr &planner) : planner_(planner.get())
            {
                tempState_ = NULL;
                update();
            }

            /** \brief Default constructor. No work is performed. */
            PlannerInputStates(const Planner *planner) : planner_(planner)
            {
                tempState_ = NULL;
                update();
            }

            /** \brief Default constructor. No work is performed. A
                call to use() needs to be made, before making any
                calls to nextStart() or nextGoal(). */
            PlannerInputStates() : planner_(NULL)
            {
                tempState_ = NULL;
                clear();
            }

            /** \brief Destructor. Clear allocated memory. */
            ~PlannerInputStates()
            {
                clear();
            }

            /** \brief Clear all stored information. */
            void clear();

            /** \brief Forget how many states were returned by
                nextStart() and nextGoal() and return all states
                again */
            void restart();

            /** \brief Set the space information and problem
                definition this class operates on, based on the
                available planner instance. Returns true if changes
                were found (different problem definition) and clear()
                was called. */
            bool update();

            /** \brief Set the problem definition this class operates on.
                If a planner is not set in the constructor argument, a call
                to this function is needed before any calls to nextStart()
                or nextGoal() are made. Returns true if changes were found
                (different problem definition) and clear() was called. */
            bool use(const ProblemDefinitionPtr &pdef);

            /** \brief Set the problem definition this class operates on.
                If a planner is not set in the constructor argument, a call
                to this function is needed before any calls to nextStart()
                or nextGoal() are made. Returns true if changes were found
                (different problem definition) and clear() was called. */
            bool use(const ProblemDefinition *pdef);

            /** \brief Check if the problem definition was set, start
                state are available and goal was set */
            void checkValidity() const;

            /** \brief Return the next valid start state or NULL if no
                more valid start states are available. */
            const State* nextStart();

            /** \brief Return the next valid goal state or NULL if no
                more valid goal states are available.  Because
                sampling of goal states may also produce invalid
                goals, this function takes an argument that specifies
                whether a termination condition has been reached.  If
                the termination condition evaluates to true the
                function terminates even if no valid goal has been
                found. */
            const State* nextGoal(const PlannerTerminationCondition &ptc);

            /** \brief Same as above but only one attempt is made to find a valid goal. */
            const State* nextGoal();

            /** \brief Check if there are more potential start states */
            bool haveMoreStartStates() const;

            /** \brief Check if there are more potential goal states */
            bool haveMoreGoalStates() const;

            /** \brief Get the number of start states from the problem
                definition that were already seen, including invalid
                ones. */
            unsigned int getSeenStartStatesCount() const
            {
                return addedStartStates_;
            }

            /** \brief Get the number of sampled goal states, including invalid ones */
            unsigned int getSampledGoalsCount() const
            {
                return sampledGoalsCount_;
            }

        private:

            const Planner              *planner_;

            unsigned int                addedStartStates_;
            unsigned int                sampledGoalsCount_;
            State                      *tempState_;

            const ProblemDefinition    *pdef_;
            const SpaceInformation     *si_;
        };

        /** \brief Properties that planners may have */
        struct PlannerSpecs
        {
            PlannerSpecs() : recognizedGoal(GOAL_ANY), multithreaded(false), approximateSolutions(false), optimizingPaths(false), directed(false), provingSolutionNonExistence(false)
            {
            }

            /** \brief The type of goal specification the planner can use */
            GoalType recognizedGoal;

            /** \brief Flag indicating whether multiple threads are used in the computation of the planner */
            bool     multithreaded;

            /** \brief Flag indicating whether the planner is able to compute approximate solutions */
            bool     approximateSolutions;

            /** \brief Flag indicating whether the planner attempts to optimize the path and reduce its length until the
                maximum path length specified by the goal representation is satisfied */
            bool     optimizingPaths;

            /** \brief Flag indicating whether the planner is able to account for the fact that the validity of a motion from A to B may not be the same as the validity of a motion from B to A.
                If this flag is true, the planner will return solutions that do not make this assumption. Usually roadmap-based planners make this assumption and tree-based planners do not. */
            bool     directed;

            /** \brief Flag indicating whether the planner is able to prove that no solution path exists. */
            bool     provingSolutionNonExistence;
        };

        /** \brief Base class for a planner */
        class Planner : private boost::noncopyable
        {

        public:

            /** \brief Constructor */
            Planner(const SpaceInformationPtr &si, const std::string &name);

            /** \brief Destructor */
            virtual ~Planner()
            {
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            T* as()
            {
                /** \brief Make sure the type we are casting to is indeed a planner */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));

                return static_cast<T*>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            const T* as() const
            {
                /** \brief Make sure the type we are casting to is indeed a Planner */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));

                return static_cast<const T*>(this);
            }

            /** \brief Get the space information this planner is using */
            const SpaceInformationPtr& getSpaceInformation() const;

            /** \brief Get the problem definition the planner is trying to solve */
            const ProblemDefinitionPtr& getProblemDefinition() const;

            /** \brief Get the planner input states */
            const PlannerInputStates& getPlannerInputStates() const;

            /** \brief Set the problem definition for the planner. The
                problem needs to be set before calling solve(). Note:
                If this problem definition replaces a previous one, it
                may also be necessary to call clear(). */
            virtual void setProblemDefinition(const ProblemDefinitionPtr &pdef);

            /** \brief Function that can solve the motion planning
                problem. This function can be called multiple times on
                the same problem, without calling clear() in
                between. This allows the planner to continue work for more
                time on an unsolved problem, for example. If this
                option is used, it is assumed the problem definition
                is not changed (unpredictable results otherwise). The
                only change in the problem definition that is
                accounted for is the addition of starting or goal
                states (but not changing previously added start/goal
                states). The function terminates if the call to \e ptc
                returns true. */
            virtual PlannerStatus solve(const PlannerTerminationCondition &ptc) = 0;

            /** \brief Same as above except the termination condition
                is only evaluated at a specified interval. */
            PlannerStatus solve(const PlannerTerminationConditionFn &ptc, double checkInterval);

            /** \brief Same as above except the termination condition
                is solely a time limit: the number of seconds the
                algorithm is allowed to spend planning. */
            PlannerStatus solve(double solveTime);

            /** \brief Clear all internal datastructures. Planner
                settings are not affected. Subsequent calls to solve()
                will ignore all previous work. */
            virtual void clear();

            /** \brief Get information about the current run of the
                motion planner. Repeated calls to this function will
                update \e data (only additions are made). This is
                useful to see what changed in the exploration
                datastructure, between calls to solve(), for example
                (without calling clear() in between).  */
            virtual void getPlannerData(PlannerData &data) const;

            /** \brief Get the name of the planner */
            const std::string& getName() const;

            /** \brief Set the name of the planner */
            void setName(const std::string &name);

            /** \brief Return the specifications (capabilities of this planner) */
            const PlannerSpecs& getSpecs() const;

            /** \brief Perform extra configuration steps, if
                needed. This call will also issue a call to
                ompl::base::SpaceInformation::setup() if needed. This
                must be called before solving */
            virtual void setup();

            /** \brief Check to see if the planner is in a working
                state (setup has been called, a goal was set, the
                input states seem to be in order). In case of error,
                this function throws an exception.*/
            virtual void checkValidity();

            /** \brief Check if setup() was called for this planner */
            bool isSetup() const;

            /** \brief Get the  parameters for this planner */
            ParamSet& params()
            {
                return params_;
            }

            /** \brief Get the  parameters for this planner */
            const ParamSet& params() const
            {
                return params_;
            }

            /** \brief Definition of a function which returns a property about the planner's progress that can be queried by a benchmarking routine */
            typedef boost::function<std::string ()> PlannerProgressProperty;

            /** \brief A dictionary which maps the name of a progress property to the function to be used for querying that property */
            typedef std::map<std::string, PlannerProgressProperty> PlannerProgressProperties;

            /** \brief Retrieve a planner's planner progress property map */
            const PlannerProgressProperties& getPlannerProgressProperties() const
            {
                return plannerProgressProperties_;
            }

            /** \brief Print properties of the motion planner */
            virtual void printProperties(std::ostream &out) const;

            /** \brief Print information about the motion planner's settings */
            virtual void printSettings(std::ostream &out) const;

        protected:

            /** \brief This function declares a parameter for this planner instance, and specifies the setter and getter functions. */
            template<typename T, typename PlannerType, typename SetterType, typename GetterType>
            void declareParam(const std::string &name, const PlannerType &planner, const SetterType& setter, const GetterType& getter, const std::string &rangeSuggestion = "")
            {
                params_.declareParam<T>(name, boost::bind(setter, planner, _1), boost::bind(getter, planner));
                if (!rangeSuggestion.empty())
                    params_[name].setRangeSuggestion(rangeSuggestion);
            }

            /** \brief This function declares a parameter for this planner instance, and specifies the setter function. */
            template<typename T, typename PlannerType, typename SetterType>
            void declareParam(const std::string &name, const PlannerType &planner, const SetterType& setter, const std::string &rangeSuggestion = "")
            {
                params_.declareParam<T>(name, boost::bind(setter, planner, _1));
                if (!rangeSuggestion.empty())
                    params_[name].setRangeSuggestion(rangeSuggestion);
            }

            /** \brief Add a planner progress property called \e progressPropertyName with a property querying function \e prop to this planner's progress property map */
            void addPlannerProgressProperty(const std::string& progressPropertyName, const PlannerProgressProperty& prop)
            {
                plannerProgressProperties_[progressPropertyName] = prop;
            }

            /** \brief The space information for which planning is done */
            SpaceInformationPtr       si_;

            /** \brief The user set problem definition */
            ProblemDefinitionPtr      pdef_;

            /** \brief Utility class to extract valid input states  */
            PlannerInputStates        pis_;

            /** \brief The name of this planner */
            std::string               name_;

            /** \brief The specifications of the planner (its capabilities) */
            PlannerSpecs              specs_;

            /** \brief A map from parameter names to parameter instances for this planner. This field is populated by the declareParam() function */
            ParamSet                  params_;

            /** \brief A mapping between this planner's progress property names and the functions used for querying those progress properties */
            PlannerProgressProperties plannerProgressProperties_;

            /** \brief Flag indicating whether setup() has been called */
            bool                      setup_;
        };

        /** \brief Definition of a function that can allocate a planner */
        typedef boost::function<PlannerPtr(const SpaceInformationPtr&)> PlannerAllocator;
    }
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Ryan Luna */

#ifndef OMPL_BASE_SOLUTION_NON_EXISTENCE_PROOF_
#define OMPL_BASE_SOLUTION_NON_EXISTENCE_PROOF_

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /// \brief Forward declaration of ompl::base::SolutionNonExistenceProof
        OMPL_CLASS_FORWARD(SolutionNonExistenceProof);
        /// @endcond

        /// \class ompl::base::SolutionNonExistenceProofPtr
        /// \brief A boost shared pointer wrapper for ompl::base::SolutionNonExistenceProof

        /// \brief Abstract definition of a proof for the non-existence of a solution to a problem
        class SolutionNonExistenceProof
        {
            public:
                SolutionNonExistenceProof(const SpaceInformationPtr &si) : si_(si)
                {
                }

                virtual ~SolutionNonExistenceProof()
                {
                }

            protected:
                SpaceInformationPtr si_;
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

#ifndef OMPL_BASE_SAMPLERS_UNIFORM_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_UNIFORM_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"

namespace ompl
{
    namespace base
    {


        /** \brief A state sampler that only samples valid states, uniformly. */
        class UniformValidStateSampler : public ValidStateSampler
        {
        public:

            /** \brief Constructor */
            UniformValidStateSampler(const SpaceInformation *si);

            virtual ~UniformValidStateSampler()
            {
            }

            virtual bool sample(State *state);
            virtual bool sampleNear(State *state, const State *near, const double distance);

        protected:

            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

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

#ifndef OMPL_BASE_SPACES_REAL_VECTOR_STATE_SPACE_
#define OMPL_BASE_SPACES_REAL_VECTOR_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <vector>
#include <string>
#include <map>

namespace ompl
{
    namespace base
    {

        /** \brief State sampler for the R<sup>n</sup> state space */
        class RealVectorStateSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            RealVectorStateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            /** \brief Sample a state such that each component state[i] is
                uniformly sampled from [near[i]-distance, near[i]+distance].
                If this interval exceeds the state space bounds, the
                interval is truncated. */
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            /** \brief Sample a state such that each component state[i] has
                a Gaussian distribution with mean mean[i] and standard
                deviation stdDev. If the sampled value exceeds the state
                space boundary, it is thresholded to the nearest boundary. */
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);
        };

        /** \brief A state space representing R<sup>n</sup>. The distance function is the L2 norm. */
        class RealVectorStateSpace : public StateSpace
        {
        public:

            /** \brief The definition of a state in R<sup>n</sup> */
            class StateType : public State
            {
            public:
                StateType() : State()
                {
                }

                /** \brief Access element i of values.  This does not
                    check whether the index is within bounds */
                double operator[](unsigned int i) const
                {
                    return values[i];
                }

                /** \brief Access element i of values.  This does not
                    check whether the index is within bounds */
                double& operator[](unsigned int i)
                {
                    return values[i];
                }

                /** \brief The value of the actual vector in R<sup>n</sup> */
                double *values;
            };

            /** \brief Constructor. The dimension of of the space needs to be specified. A space representing
                R<sup>dim</sup> will be instantiated */
            RealVectorStateSpace(unsigned int dim = 0) : StateSpace(), dimension_(dim), bounds_(dim),  stateBytes_(dim * sizeof(double))
            {
                type_ = STATE_SPACE_REAL_VECTOR;
                setName("RealVector" + getName());
                dimensionNames_.resize(dim, "");
            }

            virtual ~RealVectorStateSpace()
            {
            }

            /** \brief Increase the dimensionality of the state space by 1. Optionally, bounds can be specified for this added dimension. setup() will need to be called after adding dimensions. */
            void addDimension(double minBound = 0.0, double maxBound = 0.0);

            /** \brief Increase the dimensionality of the state space by 1 and specify the name of this dimension. Optionally, bounds can be specified for this added dimension. setup() will need to be called after adding dimensions. This function is a wrapper for the previous definition of addDimension(). */
            void addDimension(const std::string &name, double minBound = 0.0, double maxBound = 0.0);

            /** \brief Set the bounds of this state space. This defines
                the range of the space in which sampling is performed. */
            void setBounds(const RealVectorBounds &bounds);

            /** \brief Set the bounds of this state space. The bounds for
                each dimension will be the same: [\e low, \e high]. */
            void setBounds(double low, double high);

            /** \brief Get the bounds for this state space */
            const RealVectorBounds& getBounds() const
            {
                return bounds_;
            }

            virtual unsigned int getDimension() const;

            /** \brief Each dimension can optionally have a name associated to it. If it does, this function returns that name.
                Return empty string otherwise */
            const std::string& getDimensionName(unsigned int index) const;

            /** \brief Get the index of a specific dimension, by name. Return -1 if name is not found */
            int getDimensionIndex(const std::string &name) const;

            /** \brief Set the name of a dimension */
            void setDimensionName(unsigned int index, const std::string &name);

            virtual double getMaximumExtent() const;

            virtual double getMeasure() const;

            virtual void enforceBounds(State *state) const;

            virtual bool satisfiesBounds(const State *state) const;

            virtual void copyState(State *destination, const State *source) const;

            virtual unsigned int getSerializationLength() const;

            virtual void serialize(void *serialization, const State *state) const;

            virtual void deserialize(State *state, const void *serialization) const;

            virtual double distance(const State *state1, const State *state2) const;

            virtual bool equalStates(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual State* allocState() const;

            virtual void freeState(State *state) const;

            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

            virtual void printState(const State *state, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void registerProjections();

            virtual void setup();

        protected:

            /** \brief The dimension of the space */
            unsigned int                        dimension_;

            /** \brief The bounds of the space (used for sampling) */
            RealVectorBounds                    bounds_;

            /** \brief Optional names for individual dimensions */
            std::vector<std::string>            dimensionNames_;

            /** \brief Map from names to index values for dimensions */
            std::map<std::string, unsigned int> dimensionIndex_;

        private:

            /** \brief The size of a state, in bytes */
            std::size_t                         stateBytes_;

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

#ifndef OMPL_BASE_SPACES_REAL_VECTOR_STATE_PROJECTIONS_
#define OMPL_BASE_SPACES_REAL_VECTOR_STATE_PROJECTIONS_

#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief Definition for a class computing linear projections
            (multiplication of a k-by-n matrix to the the
            R<sup>n</sup> vector state to produce an R<sup>k</sup>
            projection. The multiplication matrix needs to be supplied
            as input. */
        class RealVectorLinearProjectionEvaluator : public ProjectionEvaluator
        {
        public:

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                \e projection and the cell sizes are \e cellSizes. */
            RealVectorLinearProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes,
                                                const ProjectionMatrix::Matrix &projection);

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                \e projection and the cell sizes are \e cellSizes. */
            RealVectorLinearProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes,
                                                const ProjectionMatrix::Matrix &projection);

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                \e projection and the cell sizes are automatically inferred through sampling. */
            RealVectorLinearProjectionEvaluator(const StateSpace *space,
                                                const ProjectionMatrix::Matrix &projection);

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                \e projection and the cell sizes are automatically inferred through sampling. */
            RealVectorLinearProjectionEvaluator(const StateSpacePtr &space,
                                                const ProjectionMatrix::Matrix &projection);

            virtual unsigned int getDimension() const;

            virtual void project(const State *state, EuclideanProjection &projection) const;

        protected:

            /** \brief The projection matrix */
            ProjectionMatrix projection_;

        };

        /** \brief Definition for a class computing a random linear projections */
        class RealVectorRandomLinearProjectionEvaluator : public RealVectorLinearProjectionEvaluator
        {
        public:

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                sampled at random and the cell sizes are automatically inferred through sampling. */
            RealVectorRandomLinearProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes) :
                RealVectorLinearProjectionEvaluator(space, cellSizes, ProjectionMatrix::ComputeRandom(space->getDimension(), cellSizes.size()))
            {
            }

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                sampled at random and the cell sizes are automatically inferred through sampling. */
            RealVectorRandomLinearProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes) :
                RealVectorLinearProjectionEvaluator(space, cellSizes, ProjectionMatrix::ComputeRandom(space->getDimension(), cellSizes.size()))
            {
            }

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                sampled at random to produce a space of dimension \e dim and the cell sizes are automatically inferred through sampling. */
            RealVectorRandomLinearProjectionEvaluator(const StateSpace *space, unsigned int dim) :
                RealVectorLinearProjectionEvaluator(space, ProjectionMatrix::ComputeRandom(space->getDimension(), dim,
                                                                                           space->as<RealVectorStateSpace>()->getBounds().getDifference()))
            {
            }

            /** \brief Initialize a linear projection evaluator for state space \e space. The used projection matrix is
                sampled at random to produce a space of dimension \e dim and the cell sizes are automatically inferred through sampling. */
            RealVectorRandomLinearProjectionEvaluator(const StateSpacePtr &space, unsigned int dim) :
                RealVectorLinearProjectionEvaluator(space, ProjectionMatrix::ComputeRandom(space->getDimension(), dim,
                                                                                           space->as<RealVectorStateSpace>()->getBounds().getDifference()))
            {
            }

        };

        /** \brief Definition for a class computing orthogonal projections */
        class RealVectorOrthogonalProjectionEvaluator : public ProjectionEvaluator
        {
        public:

            /** \brief Initialize an orthogonal projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are in \e cellSizes */
            RealVectorOrthogonalProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes,
                                                    const std::vector<unsigned int> &components);

            /** \brief Initialize an orthogonal projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are in \e cellSizes */
            RealVectorOrthogonalProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes,
                                                    const std::vector<unsigned int> &components);

            /** \brief Initialize an orthogonal projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are a tenth of the corresponding bounds from the state space. */
            RealVectorOrthogonalProjectionEvaluator(const StateSpace *space, const std::vector<unsigned int> &components);

            /** \brief Initialize an orthogonal projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are a tenth of the corresponding bounds from the state space.  */
            RealVectorOrthogonalProjectionEvaluator(const StateSpacePtr &space, const std::vector<unsigned int> &components);

            virtual unsigned int getDimension() const;

            virtual void defaultCellSizes();

            virtual void project(const State *state, EuclideanProjection &projection) const;

        protected:

            /** \brief Fill bounds_ with bounds from the state space */
            void copyBounds();

            /** \brief The set of components selected by the projection */
            std::vector<unsigned int> components_;

        };

        /** \brief Define the identity projection */
        class RealVectorIdentityProjectionEvaluator : public ProjectionEvaluator
        {
        public:

            /** \brief Initialize the identity projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are in \e cellSizes */
            RealVectorIdentityProjectionEvaluator(const StateSpace *space, const std::vector<double> &cellSizes);

            /** \brief Initialize the identity projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are in \e cellSizes */
            RealVectorIdentityProjectionEvaluator(const StateSpacePtr &space, const std::vector<double> &cellSizes);

            /** \brief Initialize the identity projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are a tenth of the bounds from the state space.  */
            RealVectorIdentityProjectionEvaluator(const StateSpace *space);

            /** \brief Initialize the identity projection evaluator for state space \e space. The indices of the
                kept components are in \e components and the cell sizes are a tenth of the bounds from the state space.  */
            RealVectorIdentityProjectionEvaluator(const StateSpacePtr &space);

            virtual unsigned int getDimension() const;

            virtual void defaultCellSizes();

            virtual void setup();

            virtual void project(const State *state, EuclideanProjection &projection) const;

        private:

            /** \brief Fill bounds_ with bounds from the state space */
            void copyBounds();

            /** \brief The amount of data to copy from projection to state */
            std::size_t copySize_;

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

#ifndef OMPL_BASE_SPACES_SO2_STATE_SPACE_
#define OMPL_BASE_SPACES_SO2_STATE_SPACE_

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief State space sampler for SO(2) */
        class SO2StateSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            SO2StateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);
        };

        /** \brief A state space representing SO(2). The distance
            function and interpolation take into account angle
            wrapping. */
        class SO2StateSpace : public StateSpace
        {
        public:

            /** \brief The definition of a state in SO(2) */
            class StateType : public State
            {
            public:

                /** \brief Set the state to identity -- no rotation (value = 0.0) */
                void setIdentity()
                {
                    value = 0.0;
                }

                /** \brief The value of the angle in the interval (-Pi, Pi] */
                double value;
            };

            SO2StateSpace() : StateSpace()
            {
                setName("SO2" + getName());
                type_ = STATE_SPACE_SO2;
            }

            virtual ~SO2StateSpace()
            {
            }

            virtual unsigned int getDimension() const;

            virtual double getMaximumExtent() const;

            virtual double getMeasure() const;

            /** \brief Normalize the value of the state to the interval (-Pi, Pi] */
            virtual void enforceBounds(State *state) const;

            /** \brief Check if the value of the state is in the interval (-Pi, Pi] */
            virtual bool satisfiesBounds(const State *state) const;

            virtual void copyState(State *destination, const State *source) const;

            virtual unsigned int getSerializationLength() const;

            virtual void serialize(void *serialization, const State *state) const;

            virtual void deserialize(State *state, const void *serialization) const;

            virtual double distance(const State *state1, const State *state2) const;

            virtual bool equalStates(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual State* allocState() const;

            virtual void freeState(State *state) const;

            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

            virtual void printState(const State *state, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void registerProjections();
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

#ifndef OMPL_BASE_SPACES_SO3_STATE_SPACE_
#define OMPL_BASE_SPACES_SO3_STATE_SPACE_

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief State space sampler for SO(3), using quaternion representation  */
        class SO3StateSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            SO3StateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            /** \brief To sample unit quaternions uniformly within some given
                distance, we sample a 3-vector from the R^3 tangent space.
                This vector is drawn uniformly random from a 3D ball centered at
                the origin with radius distance. The vector is then "wrapped"
                around S^3 to obtain a unit quaternion uniformly distributed
                around the identity quaternion within given distance. We
                pre-multiply this quaternion with the quaternion near
                to center the distribution around near. */
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            /** \brief Sample a state such that the expected distance between
                mean and state is stdDev.

                To sample a unit quaternion from a Gaussian
                distribution, we sample a 3-vector from the R^3 tangent space
                using a 3D Gaussian with zero mean and covariance matrix equal
                to diag(stdDev^2, stdDev^2, stdDev^2). This vector is "wrapped"
                around S^3 to obtain a Gaussian quaternion with zero mean.
                We pre-multiply this quaternion with the quaternion mean
                to get the desired mean. */
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);
        };

        /** \brief A state space representing SO(3). The internal
            representation is done with quaternions. The distance
            between states is the angle between quaternions and
            interpolation is done with slerp. */
        class SO3StateSpace : public StateSpace
        {
        public:


            /** \brief The definition of a state in SO(3) represented as a unit quaternion

                \note The order of the elements matters in this
                definition for the SO3StateUniformSampler::sample()
                function. */
            class StateType : public State
            {
            public:

                /** \brief Set the quaternion from axis-angle representation */
                void setAxisAngle(double ax, double ay, double az, double angle);

                /** \brief Set the state to identity -- no rotation */
                void setIdentity();

                /** \brief X component of quaternion vector */
                double x;

                /** \brief Y component of quaternion vector */
                double y;

                /** \brief Z component of quaternion vector */
                double z;

                /** \brief scalar component of quaternion */
                double w;
            };

            SO3StateSpace() : StateSpace()
            {
                setName("SO3" + getName());
                type_ = STATE_SPACE_SO3;
            }

            virtual ~SO3StateSpace()
            {
            }

            /** \brief Compute the norm of a state */
            double norm(const StateType *state) const;

            virtual unsigned int getDimension() const;

            virtual double getMaximumExtent() const;

            virtual double getMeasure() const;

            virtual void enforceBounds(State *state) const;

            virtual bool satisfiesBounds(const State *state) const;

            virtual void copyState(State *destination, const State *source) const;

            virtual unsigned int getSerializationLength() const;

            virtual void serialize(void *serialization, const State *state) const;

            virtual void deserialize(State *state, const void *serialization) const;

            virtual double distance(const State *state1, const State *state2) const;

            virtual bool equalStates(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual State* allocState() const;

            virtual void freeState(State *state) const;

            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

            virtual void printState(const State *state, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void registerProjections();
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

#ifndef OMPL_BASE_SPACES_SE2_STATE_SPACE_
#define OMPL_BASE_SPACES_SE2_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief A state space representing SE(2) */
        class SE2StateSpace : public CompoundStateSpace
        {
        public:

            /** \brief A state in SE(2): (x, y, yaw) */
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() : CompoundStateSpace::StateType()
                {
                }

                /** \brief Get the X component of the state */
                double getX() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[0];
                }

                /** \brief Get the Y component of the state */
                double getY() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[1];
                }

                /** \brief Get the yaw component of the state. This is
                    the rotation in plane, with respect to the Z
                    axis. */
                double getYaw() const
                {
                    return as<SO2StateSpace::StateType>(1)->value;
                }

                /** \brief Set the X component of the state */
                void setX(double x)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[0] = x;
                }

                /** \brief Set the Y component of the state */
                void setY(double y)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[1] = y;
                }

                /** \brief Set the X and Y components of the state */
                void setXY(double x, double y)
                {
                    setX(x);
                    setY(y);
                }

                /** \brief Set the yaw component of the state. This is
                    the rotation in plane, with respect to the Z
                    axis. */
                void setYaw(double yaw)
                {
                    as<SO2StateSpace::StateType>(1)->value = yaw;
                }

            };


            SE2StateSpace() : CompoundStateSpace()
            {
                setName("SE2" + getName());
                type_ = STATE_SPACE_SE2;
                addSubspace(StateSpacePtr(new RealVectorStateSpace(2)), 1.0);
                addSubspace(StateSpacePtr(new SO2StateSpace()), 0.5);
                lock();
            }

            virtual ~SE2StateSpace()
            {
            }

            /** \copydoc RealVectorStateSpace::setBounds() */
            void setBounds(const RealVectorBounds &bounds)
            {
                as<RealVectorStateSpace>(0)->setBounds(bounds);
            }

            /** \copydoc RealVectorStateSpace::getBounds() */
            const RealVectorBounds& getBounds() const
            {
                return as<RealVectorStateSpace>(0)->getBounds();
            }

            virtual State* allocState() const;
            virtual void freeState(State *state) const;

            virtual void registerProjections();

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

#ifndef OMPL_BASE_SPACES_SE3_STATE_SPACE_
#define OMPL_BASE_SPACES_SE3_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief A state space representing SE(3) */
        class SE3StateSpace : public CompoundStateSpace
        {
        public:

            /** \brief A state in SE(3): position = (x, y, z), quaternion = (x, y, z, w) */
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() : CompoundStateSpace::StateType()
                {
                }

                /** \brief Get the X component of the state */
                double getX() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[0];
                }

                /** \brief Get the Y component of the state */
                double getY() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[1];
                }

                /** \brief Get the Z component of the state */
                double getZ() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[2];
                }

                /** \brief Get the rotation component of the state */
                const SO3StateSpace::StateType& rotation() const
                {
                    return *as<SO3StateSpace::StateType>(1);
                }

                /** \brief Get the rotation component of the state and allow changing it as well */
                SO3StateSpace::StateType& rotation()
                {
                    return *as<SO3StateSpace::StateType>(1);
                }

                /** \brief Set the X component of the state */
                void setX(double x)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[0] = x;
                }

                /** \brief Set the Y component of the state */
                void setY(double y)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[1] = y;
                }

                /** \brief Set the Z component of the state */
                void setZ(double z)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[2] = z;
                }

                /** \brief Set the X, Y and Z components of the state */
                void setXYZ(double x, double y, double z)
                {
                    setX(x);
                    setY(y);
                    setZ(z);
                }

            };

            SE3StateSpace() : CompoundStateSpace()
            {
                setName("SE3" + getName());
                type_ = STATE_SPACE_SE3;
                addSubspace(StateSpacePtr(new RealVectorStateSpace(3)), 1.0);
                addSubspace(StateSpacePtr(new SO3StateSpace()), 1.0);
                lock();
            }

            virtual ~SE3StateSpace()
            {
            }

            /** \copydoc RealVectorStateSpace::setBounds() */
            void setBounds(const RealVectorBounds &bounds)
            {
                as<RealVectorStateSpace>(0)->setBounds(bounds);
            }

            /** \copydoc RealVectorStateSpace::getBounds() */
            const RealVectorBounds& getBounds() const
            {
                return as<RealVectorStateSpace>(0)->getBounds();
            }

            virtual State* allocState() const;
            virtual void freeState(State *state) const;

            virtual void registerProjections();
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

/* Author: Elizabeth Fudge */

#ifndef OMPL_BASE_SPACES_DISCRETE_STATE_SPACE_
#define OMPL_BASE_SPACES_DISCRETE_STATE_SPACE_

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief State space sampler for discrete states */
        class DiscreteStateSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            DiscreteStateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);
        };

        /** \brief A space representing discrete states; i.e. there
            are a small number of discrete states the system can be in.
            States are represented as integers [lowerBound, upperBound],
            where lowerBound and upperBound are inclusive.
            States do not wrap around; i.e. the distance between state
            lowerBound and state upperBound is upperBound-lowerBound.
            The dimension of the space is 1. */
        class DiscreteStateSpace : public StateSpace
        {
        public:

            /** \brief The definition of a discrete state */
            class StateType : public State
            {
            public:

                /** \brief The current state - an int in range [lowerBound, upperBound] */
                int value;
            };

            /** \brief Construct a discrete space in wich states can take values in the set [\e lowerBound, \e upperBound] */
            DiscreteStateSpace(int lowerBound, int upperBound) : StateSpace(), lowerBound_(lowerBound), upperBound_(upperBound)
            {
                setName("Discrete" + getName());
                type_ = STATE_SPACE_DISCRETE;
            }

            virtual ~DiscreteStateSpace()
            {
            }

            virtual bool isDiscrete() const;

            virtual unsigned int getDimension() const;

            virtual double getMaximumExtent() const;

            virtual double getMeasure() const;

            virtual void enforceBounds(State *state) const;

            virtual bool satisfiesBounds(const State *state) const;

            virtual unsigned int getSerializationLength() const;

            virtual void serialize(void *serialization, const State *state) const;

            virtual void deserialize(State *state, const void *serialization) const;

            virtual void copyState(State *destination, const State *source) const;

            virtual double distance(const State *state1, const State *state2) const;

            virtual bool equalStates(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual State* allocState() const;

            virtual void freeState(State *state) const;

            virtual void printState(const State *state, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void registerProjections();

            /** \brief Returns the number of states possible */
            unsigned int getStateCount() const
            {
                return upperBound_ - lowerBound_ + 1;
            }

            /** \brief Returns the lowest possible state */
            int getLowerBound() const
            {
                return lowerBound_;
            }

            /** \brief Returns the highest possible state */
            int getUpperBound() const
            {
                return upperBound_;
            }

            /** \brief Set the bounds for the states in this space (the states will be in the set [\e lowerBound, \e upperBound] */
            void setBounds(int lowerBound, int upperBound)
            {
                lowerBound_ = lowerBound;
                upperBound_ = upperBound;
            }

            virtual void setup();

        protected:

            /** \brief The lowest integer state */
            int lowerBound_;

            /** \brief The highest integer state */
            int upperBound_;
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

#ifndef OMPL_BASE_SPACES_TIME_STATE_SPACE_
#define OMPL_BASE_SPACES_TIME_STATE_SPACE_

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {

        /** \brief State space sampler for time */
        class TimeStateSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            TimeStateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);
        };

        /** \brief A state space representing time. The time can be
            unbounded, in which case enforceBounds() is a no-op,
            satisfiesBounds() always returns true, sampling uniform
            time states always produces time 0 and getMaximumExtent()
            returns 1. If time is bounded (setBounds() has been
            previously called), the state space behaves as
            expected. After construction, the state space is
            unbounded. isBounded() can be used to check if the state space
            is bounded or not. */
        class TimeStateSpace : public StateSpace
        {
        public:

            /** \brief The definition of a time state */
            class StateType : public State
            {
            public:

                /** \brief The position in time */
                double position;
            };

            TimeStateSpace() : StateSpace(), bounded_(false), minTime_(0.0), maxTime_(0.0)
            {
                setName("Time" + getName());
                type_ = STATE_SPACE_TIME;
            }

            virtual ~TimeStateSpace()
            {
            }

            virtual unsigned int getDimension() const;

            virtual double getMaximumExtent() const;

            virtual double getMeasure() const;

            /** \brief Set the minimum and maximum time bounds. This
                will make the state space switch into bounded time
                mode. If this function is not called, sampling time
                will always produce position = 0, enforceBounds() is a no-op,
                satisfiesBounds() always returns true and
                getMaximumExtent() returns 1. */
            void setBounds(double minTime, double maxTime);

            /** \brief Get the minimum allowed value of \e position in a state. The function returns 0 if time is not bounded. */
            double getMinTimeBound() const
            {
                return minTime_;
            }

            /** \brief Get the maximum allowed value of \e position in a state. The function returns 0 if time is not bounded. */
            double getMaxTimeBound() const
            {
                return maxTime_;
            }

            /** \brief Check if the time is bounded or not */
            bool isBounded() const
            {
                return bounded_;
            }

            virtual void enforceBounds(State *state) const;

            virtual bool satisfiesBounds(const State *state) const;

            virtual void copyState(State *destination, const State *source) const;

            virtual unsigned int getSerializationLength() const;

            virtual void serialize(void *serialization, const State *state) const;

            virtual void deserialize(State *state, const void *serialization) const;

            virtual double distance(const State *state1, const State *state2) const;

            virtual bool equalStates(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual State* allocState() const;

            virtual void freeState(State *state) const;

            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

            virtual void printState(const State *state, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void registerProjections();

        protected:

            /** \brief Flag indicating whether the state space is considering bounds or not */
            bool   bounded_;

            /** \brief The minimum point in time considered by the state space (if bounds are used) */
            double minTime_;

            /** \brief The maximum point in time considered by the state space (if bounds are used) */
            double maxTime_;

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

#ifndef OMPL_BASE_SPACES_DUBINS_STATE_SPACE_
#define OMPL_BASE_SPACES_DUBINS_STATE_SPACE_

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>

namespace ompl
{
    namespace base
    {

        /** \brief An SE(2) state space where distance is measured by the
            length of Dubins curves.

            Note that this Dubins distance is \b not a proper distance metric,
            so nearest neighbor methods that rely on distance() being a metric
            (such as ompl::NearestNeighborsGNAT) will not always return the
            true nearest neighbors or get stuck in an infinite loop.

            The notation and solutions in the code are taken from:<br>
            A.M. Shkel and V. Lumelsky, “Classification of the Dubins set,”
            Robotics and Autonomous Systems, 34(4):179-202, 2001.
            DOI: <a href="http://dx.doi.org/10.1016/S0921-8890(00)00127-5">10.1016/S0921-8890(00)00127-5</a>

            The classification scheme described there is not actually used,
            since it only applies to “long” paths.
            */
        class DubinsStateSpace : public SE2StateSpace
        {
        public:

            /** \brief The Dubins path segment type */
            enum DubinsPathSegmentType { DUBINS_LEFT=0, DUBINS_STRAIGHT=1, DUBINS_RIGHT=2 };
            /** \brief Dubins path types */
            static const DubinsPathSegmentType dubinsPathType[6][3];
            /** \brief Complete description of a Dubins path */
            class DubinsPath
            {
            public:
                DubinsPath(const DubinsPathSegmentType* type = dubinsPathType[0],
                    double t=0., double p=std::numeric_limits<double>::max(), double q=0.)
                    : type_(type), reverse_(false)
                {
                    length_[0] = t;
                    length_[1] = p;
                    length_[2] = q;
                    assert(t >= 0.);
                    assert(p >= 0.);
                    assert(q >= 0.);
                }
                double length() const
                {
                    return length_[0] + length_[1] + length_[2];
                }

                /** Path segment types */
                const DubinsPathSegmentType* type_;
                /** Path segment lengths */
                double length_[3];
                /** Whether the path should be followed "in reverse" */
                bool reverse_;
            };

            DubinsStateSpace(double turningRadius = 1.0, bool isSymmetric = false)
                : SE2StateSpace(), rho_(turningRadius), isSymmetric_(isSymmetric)
            {
            }

            virtual bool isMetricSpace() const
            {
                return false;
            }

            virtual double distance(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t,
                State *state) const;
            virtual void interpolate(const State *from, const State *to, const double t,
                bool &firstTime, DubinsPath &path, State *state) const;

            virtual bool hasSymmetricDistance() const
            {
                return isSymmetric_;
            }

            virtual bool hasSymmetricInterpolate() const
            {
                return isSymmetric_;
            }

            virtual void sanityChecks() const
            {
                double zero = std::numeric_limits<double>::epsilon();
                double eps = std::numeric_limits<float>::epsilon();
                int flags = ~(STATESPACE_INTERPOLATION | STATESPACE_TRIANGLE_INEQUALITY | STATESPACE_DISTANCE_BOUND);
                if (!isSymmetric_)
                    flags &= ~STATESPACE_DISTANCE_SYMMETRIC;
                StateSpace::sanityChecks(zero, eps, flags);
            }

            /** \brief Return the shortest Dubins path from SE(2) state state1 to SE(2) state state2 */
            DubinsPath dubins(const State *state1, const State *state2) const;

        protected:
            virtual void interpolate(const State *from, const DubinsPath &path, const double t,
                State *state) const;

            /** \brief Turning radius */
            double rho_;

            /** \brief Whether the distance is "symmetrized"

                If true the distance from state s1 to state s2 is the same as the
                distance from s2 to s1. This is done by taking the \b minimum
                length of the Dubins curves that connect s1 to s2 and s2 to s1. If
                isSymmetric_ is true, then the distance no longer satisfies the
                triangle inequality. */
            bool isSymmetric_;
        };

        /** \brief A Dubins motion validator that only uses the state validity checker.
            Motions are checked for validity at a specified resolution.

            This motion validator is almost identical to the DiscreteMotionValidator
            except that it remembers the optimal DubinsPath between different calls to
            interpolate. */
        class DubinsMotionValidator : public MotionValidator
        {
        public:
            DubinsMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }
            DubinsMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }
            virtual ~DubinsMotionValidator()
            {
            }
            virtual bool checkMotion(const State *s1, const State *s2) const;
            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const;
        private:
            DubinsStateSpace *stateSpace_;
            void defaultSettings();
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

#ifndef OMPL_BASE_SPACES_REEDS_SHEPP_STATE_SPACE_
#define OMPL_BASE_SPACES_REEDS_SHEPP_STATE_SPACE_

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>

namespace ompl
{
    namespace base
    {

        /** \brief An SE(2) state space where distance is measured by the
            length of Reeds-Shepp curves.

            The notation and solutions are taken from:
            J.A. Reeds and L.A. Shepp, “Optimal paths for a car that goes both
            forwards and backwards,” Pacific Journal of Mathematics,
            145(2):367–393, 1990.

            This implementation explicitly computes all 48 Reeds-Shepp curves
            and returns the shortest valid solution. This can be improved by
            using the configuration space partition described in:
            P. Souères and J.-P. Laumond, “Shortest paths synthesis for a
            car-like robot,” IEEE Trans. on Automatic Control, 41(5):672–688,
            May 1996.
            */
        class ReedsSheppStateSpace : public SE2StateSpace
        {
        public:

            /** \brief The Reeds-Shepp path segment types */
            enum ReedsSheppPathSegmentType { RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };
            /** \brief Reeds-Shepp path types */
            static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
            /** \brief Complete description of a ReedsShepp path */
            class ReedsSheppPath
            {
            public:
                ReedsSheppPath(const ReedsSheppPathSegmentType* type=reedsSheppPathType[0],
                    double t=std::numeric_limits<double>::max(), double u=0., double v=0.,
                    double w=0., double x=0.);
                double length() const { return totalLength_; }

                /** Path segment types */
                const ReedsSheppPathSegmentType* type_;
                /** Path segment lengths */
                double length_[5];
                /** Total length */
                double totalLength_;
            };

            ReedsSheppStateSpace(double turningRadius = 1.0)
                : SE2StateSpace(), rho_(turningRadius)
            {
            }

            virtual double distance(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t,
                State *state) const;
            virtual void interpolate(const State *from, const State *to, const double t,
                bool &firstTime, ReedsSheppPath &path, State *state) const;

            virtual void sanityChecks() const
            {
                double zero = std::numeric_limits<double>::epsilon();
                double eps = .1; // rarely such a large error will occur
                StateSpace::sanityChecks(zero, eps, ~STATESPACE_INTERPOLATION);
            }

            /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
            ReedsSheppPath reedsShepp(const State *state1, const State *state2) const;

        protected:
            virtual void interpolate(const State *from, const ReedsSheppPath &path, const double t,
                State *state) const;

            /** \brief Turning radius */
            double rho_;
        };

        /** \brief A Reeds-Shepp motion validator that only uses the state validity checker.
            Motions are checked for validity at a specified resolution.

            This motion validator is almost identical to the DiscreteMotionValidator
            except that it remembers the optimal ReedsSheppPath between different calls to
            interpolate. */
        class ReedsSheppMotionValidator : public MotionValidator
        {
        public:
            ReedsSheppMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }
            ReedsSheppMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }
            virtual ~ReedsSheppMotionValidator()
            {
            }
            virtual bool checkMotion(const State *s1, const State *s2) const;
            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const;
        private:
            ReedsSheppStateSpace *stateSpace_;
            void defaultSettings();
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

#ifndef OMPL_BASE_SAMPLERS_GAUSSIAN_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_GAUSSIAN_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"

namespace ompl
{
    namespace base
    {

        /** \brief Generate valid samples using the Gaussian sampling strategy */
        class GaussianValidStateSampler : public ValidStateSampler
        {
        public:

            /** \brief Constructor */
            GaussianValidStateSampler(const SpaceInformation *si);

            virtual ~GaussianValidStateSampler()
            {
            }

            virtual bool sample(State *state);
            virtual bool sampleNear(State *state, const State *near, const double distance);

            /** \brief Get the standard deviation used when sampling */
            double getStdDev() const
            {
                return stddev_;
            }

            /** \brief Set the standard deviation to use when sampling */
            void setStdDev(double stddev)
            {
                stddev_ = stddev;
            }

        protected:

            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

            /** \brief The standard deviation to use in the sampling process */
            double                  stddev_;
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_SAMPLERS_MAXIMIZE_CLEARANCE_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_MAXIMIZE_CLEARANCE_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"

namespace ompl
{
    namespace base
    {


        /** \brief Generate valid samples randomly, but with a bias towards higher clearance. */
        class MaximizeClearanceValidStateSampler : public ValidStateSampler
        {
        public:

            /** \brief Constructor */
            MaximizeClearanceValidStateSampler(const SpaceInformation *si);

            virtual ~MaximizeClearanceValidStateSampler();

            virtual bool sample(State *state);

            virtual bool sampleNear(State *state, const State *near, const double distance);

            /** \brief The number of attempts at improving the clearance of the sampled state. */
            void setNrImproveAttempts(unsigned int attempts)
            {
                improveAttempts_ = attempts;
            }

            /** \brief Get the number of attempts to improve a sampled state */
            unsigned int getNrImproveAttempts() const
            {
                return improveAttempts_;
            }

        protected:

            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

            /** \brief Number of attempts to improve a valid sample */
            unsigned int    improveAttempts_;

        private:
            /** \brief Temporary work area */
            State          *work_;
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

#ifndef OMPL_BASE_SAMPLERS_OBSTACLE_BASED_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_OBSTACLE_BASED_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"

namespace ompl
{
    namespace base
    {


        /** \brief Generate valid samples using obstacle based sampling.  First
            sample an invalid state, then sample a valid state.  Then, interpolate from
            the invalid state to the valid state, returning the first valid state
            encountered.

            @par External documentation
            N. M. Amato, O. B. Bayazit, L. K. Dale, C. Jones, and D. Vallejo, OBPRM: an obstacle-based PRM for 3D workspaces, in <em> Third Workshop on the Algorithmic Foundations of Robotics</em>, pp. 155-168, 1998. DOI: <a href="http://dl.acm.org/citation.cfm?id=299002">http://dl.acm.org/citation.cfm?id=299002</a>
        */
        class ObstacleBasedValidStateSampler : public ValidStateSampler
        {
        public:

            /** \brief Constructor */
            ObstacleBasedValidStateSampler(const SpaceInformation *si);

            virtual ~ObstacleBasedValidStateSampler()
            {
            }

            virtual bool sample(State *state);
            virtual bool sampleNear(State *state, const State *near, const double distance);

        protected:

            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

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

#ifndef OMPL_BASE_SAMPLERS_UNIFORM_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_UNIFORM_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"

namespace ompl
{
    namespace base
    {


        /** \brief A state sampler that only samples valid states, uniformly. */
        class UniformValidStateSampler : public ValidStateSampler
        {
        public:

            /** \brief Constructor */
            UniformValidStateSampler(const SpaceInformation *si);

            virtual ~UniformValidStateSampler()
            {
            }

            virtual bool sample(State *state);
            virtual bool sampleNear(State *state, const State *near, const double distance);

        protected:

            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

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

#ifndef PY_BINDINGS_OMPL_PY_BASE_
#define PY_BINDINGS_OMPL_PY_BASE_

#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"
#include "ompl/base/spaces/TimeStateSpace.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/Goal.h"
#include "ompl/base/PlannerData.h"
#include "py_boost_function.hpp"

#define DeclareStateType(T) \
    inline int __dummy##T() \
    { \
        return sizeof(ompl::base::ScopedState<T##StateSpace>) + \
        sizeof(ompl::base::T##StateSpace::StateType); \
    }

#define DeclareSpecificParamType(n, T) \
    inline int __dummySP##n() \
    { \
        return sizeof(ompl::base::SpecificParam<T>("dummy", \
            ompl::base::SpecificParam<T>::SetterFn())); \
    }

namespace ompl
{
    namespace base
    {
        DeclareStateType();
        DeclareStateType(Compound);
        DeclareStateType(RealVector);
        DeclareStateType(SO2);
        DeclareStateType(SO3);
        DeclareStateType(SE2);
        DeclareStateType(SE3);
        DeclareStateType(Discrete);
        DeclareStateType(Time);
        DeclareStateType(Dubins);
        DeclareStateType(ReedsShepp);

        DeclareSpecificParamType(0, bool);
        DeclareSpecificParamType(1, char);
        DeclareSpecificParamType(2, int);
        DeclareSpecificParamType(3, unsigned int);
        DeclareSpecificParamType(4, float);
        DeclareSpecificParamType(5, double);
        DeclareSpecificParamType(6, std::string);

        inline int dummySTLContainerSize()
        {
            return sizeof(std::vector<ompl::base::PlannerSolution>)
                + sizeof(std::map<unsigned int, ompl::base::PlannerDataEdge const *>);
        }
    }
}

#endif
