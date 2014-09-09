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

#ifndef OMPL_CONTROL_CONTROL_
#define OMPL_CONTROL_CONTROL_

#include <boost/concept_check.hpp>

namespace ompl
{
    namespace control
    {

        /** \brief Definition of an abstract control */
        class Control
        {
        private:

            /** \brief Disable copy-constructor */
            Control(const Control&);

            /** \brief Disable copy operator */
            const Control& operator=(const Control&);

        protected:

            Control()
            {
            }

            virtual ~Control()
            {
            }

        public:

            /** \brief Cast this instance to a desired type. */
            template<class T>
            const T* as() const
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Control*>));

                return static_cast<const T*>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            T* as()
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Control*>));

                return static_cast<T*>(this);
            }

        };

        /** \brief Definition of a compound control */
        class CompoundControl : public Control
        {
        public:

            CompoundControl() : Control()
            {
            }

            virtual ~CompoundControl()
            {
            }

            /** \brief Cast a component of this instance to a desired type. */
            template<class T>
            const T* as(const unsigned int index) const
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Control*>));

                return static_cast<const T*>(components[index]);
            }

            /** \brief Cast a component of this instance to a desired type. */
            template<class T>
            T* as(const unsigned int index)
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Control*>));

                return static_cast<T*>(components[index]);
            }

            /** \brief The components that make up a compound control */
            Control **components;
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

#ifndef OMPL_CONTROL_CONTROL_SAMPLER_
#define OMPL_CONTROL_CONTROL_SAMPLER_

#include "ompl/base/State.h"
#include "ompl/control/Control.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/util/ClassForward.h"
#include <vector>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>

namespace ompl
{
    namespace control
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ControlSpace);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::ControlSampler */
        OMPL_CLASS_FORWARD(ControlSampler);
        /// @endcond

        /** \class ompl::control::ControlSamplerPtr
            \brief A boost shared pointer wrapper for ompl::control::ControlSampler */

        /** \brief Abstract definition of a control sampler. Motion
            planners that need to sample controls will call functions
            from this class. Planners should call the versions of
            sample() and sampleNext() with most arguments, whenever
            this information is available. */
        class ControlSampler : private boost::noncopyable
        {
        public:

            /** \brief Constructor takes the state space to construct samples for as argument */
            ControlSampler(const ControlSpace *space) : space_(space)
            {
            }

            virtual ~ControlSampler()
            {
            }

            /** \brief Sample a control. All other control sampling
                functions default to this one, unless a user-specified
                implementation is given. */
            virtual void sample(Control *control) = 0;

            /** \brief Sample a control, given it is applied to a
                specific state (\e state). The default implementation calls the
                previous definition of sample(). Providing a different
                implementation of this function is useful if, for
                example, the sampling of controls depends on the state
                of the system. When attempting to sample controls that
                keep a system stable, for example, knowing the state
                at which the control is applied is important. */
            virtual void sample(Control *control, const base::State *state);

            /** \brief Sample a control, given the previously applied
                control. The default implementation calls the first
                definition of sample(). For some systems it is
                possible that large changes in controls are not
                desirable. For example, switching from maximum
                acceleration to maximum deceleration is not desirable
                when driving a car. */
            virtual void sampleNext(Control *control, const Control *previous);

            /** \brief Sample a control, given the previously applied
                control and that it is applied to a specific
                state. The default implementation calls the first
                definition of sample(), even if other implementations
                of the sampleNext() shown above are provided. Often
                this function needs to be overridden as it is the
                function planners typically call.  */
            virtual void sampleNext(Control *control, const Control *previous, const base::State *state);

            /** \brief Sample a number of steps to execute a control for */
            virtual unsigned int sampleStepCount(unsigned int minSteps, unsigned int maxSteps);

        protected:

            /** \brief The control space this sampler operates on */
            const ControlSpace    *space_;

            /** \brief Instance of random number generator */
            RNG                    rng_;
        };

        /** \brief Definition of a compound control sampler. This is useful to construct samplers for compound controls. */
        class CompoundControlSampler : public ControlSampler
        {
        public:

            /** \brief Constructor */
            CompoundControlSampler(const ControlSpace *space) : ControlSampler(space)
            {
            }

            /** \brief Destructor. This frees the added samplers as well. */
            virtual ~CompoundControlSampler()
            {
            }

            /** \brief Add a sampler as part of the new compound
                sampler. This sampler is used to sample part of the
                compound control.  */
            virtual void addSampler(const ControlSamplerPtr &sampler);


            virtual void sample(Control *control);
            virtual void sample(Control *control, const base::State *state);
            virtual void sampleNext(Control *control, const Control *previous);
            virtual void sampleNext(Control *control, const Control *previous, const base::State *state);

        protected:

            /** \brief The instances of samplers used for compound sampler */
            std::vector<ControlSamplerPtr> samplers_;

        private:

            /** \brief Number of sampler instances */
            unsigned int                   samplerCount_;

        };

        /** \brief Definition of a function that can allocate a control sampler */
        typedef boost::function<ControlSamplerPtr(const ControlSpace*)> ControlSamplerAllocator;
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

#ifndef OMPL_CONTROL_DIRECTED_CONTROL_SAMPLER_
#define OMPL_CONTROL_DIRECTED_CONTROL_SAMPLER_

#include "ompl/base/State.h"
#include "ompl/control/Control.h"
#include "ompl/util/ClassForward.h"
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>

namespace ompl
{
    namespace control
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        OMPL_CLASS_FORWARD(DirectedControlSampler);
        /// @endcond

        /** \class ompl::control::DirectedControlSamplerPtr
            \brief A boost shared pointer wrapper for ompl::control::DirectedControlSampler */

        /** \brief Abstract definition of a directed control sampler. Motion
            planners that need to sample controls that take the system to a desired direction will call functions
            from this class. Planners should call the versions of sampleTo() with most arguments, whenever this information is available.
            If no direction information is available, the use of a ControlSampler is perhaps more appropriate. */
        class DirectedControlSampler : private boost::noncopyable
        {
        public:

            /** \brief Constructor takes the state space to construct samples for as argument */
            DirectedControlSampler(const SpaceInformation *si) : si_(si)
            {
            }

            virtual ~DirectedControlSampler()
            {
            }

            /** \brief Sample a control given that it will be applied to state
                \e state and the intention is to reach state \e target. This is
                useful for some algorithms that have a notion of direction in
                their exploration (e.g., \ref cRRT). Furthermore, return the
                duration for which this control should be applied. The state
                \e dest is modified to match the state reached with the computed
                control and duration. The motion is checked for validity. */
            virtual unsigned int sampleTo(Control *control, const base::State *source, base::State *dest) = 0;

            /** \brief Sample a control given that it will be applied to state
                \e state and the intention is to reach state \e dest. Also take
                into account the fact that the previously applied control is \e
                previous. This is useful for some algorithms that have a notion
                of direction in their exploration (e.g., \ref cRRT).
                Furthermore, return the duration for which this control should
                be applied. The state \e dest is modified to match the state
                reached with the computed control and duration. The motion is
                checked for validity. */
            virtual unsigned int sampleTo(Control *control, const Control *previous, const base::State *source, base::State *dest) = 0;

        protected:

            /** \brief The space information this sampler operates on */
            const SpaceInformation *si_;

        };

        /** \brief Definition of a function that can allocate a directed control sampler */
        typedef boost::function<DirectedControlSamplerPtr(const SpaceInformation*)> DirectedControlSamplerAllocator;
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

#ifndef OMPL_CONTROL_SIMPLE_DIRECTED_CONTROL_SAMPLER_
#define OMPL_CONTROL_SIMPLE_DIRECTED_CONTROL_SAMPLER_

#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/ControlSampler.h"

namespace ompl
{
    namespace control
    {

        /** \brief Implementation of a simple directed control
            sampler. This is a basic implementation that does not
            actually take direction into account and builds upon ControlSampler.
            Instead, a set of k random controls are sampled, and the control that
            gets the system closest to the target state is returned.

           @par External documentation
           K-control sampling is first believed to be proposed in:

           S.M. LaValle and J.J. Kuffner, Randomized kinodynamic planning, <em>Intl. J. of Robotics Research</em>, vol. 20, pp. 378–400, May 2001. DOI: <a href="http://dx.doi.org/10.1177/02783640122067453">10.1177/02783640122067453</a><br>
           <a href="http://ijr.sagepub.com/content/20/5/378.full.pdf">[PDF]</a>
           <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">[more]</a>*/
        class SimpleDirectedControlSampler : public DirectedControlSampler
        {
        public:

            /** \brief Constructor takes the state space to construct samples for as argument
                Optionally, a \e k value can be given to indicate the number of controls to
                try when directing a system toward a specific state.  Default value is 1. */
            SimpleDirectedControlSampler(const SpaceInformation *si, unsigned int k = 1);

            virtual ~SimpleDirectedControlSampler();

            /** \brief Retrieve the number of controls to generate when finding the best control. */
            unsigned int getNumControlSamples () const
            {
                return numControlSamples_;
            }

            /** \brief Set the number of controls to generate when finding the best control. */
            void setNumControlSamples (unsigned int numSamples)
            {
                numControlSamples_ = numSamples;
            }

            /** \brief Sample a control given that it will be applied
                to state \e state and the intention is to reach state
                \e dest. This is useful for some algorithms that
                have a notion of direction in their exploration (e.g.,
                \ref cRRT). Furthermore, return the duration for which
                this control should be applied. The state
                \e dest is modified to match the state reached with the computed
                control and duration. The motion is checked for validity. */
            virtual unsigned int sampleTo(Control *control, const base::State *source, base::State *dest);

            /** \brief Sample a control given that it will be applied
                to state \e state and the intention is to reach state
                \e dest. Also take into account the fact that the
                previously applied control is \e previous. This is
                useful for some algorithms that have a notion of
                direction in their exploration (e.g.,
                \ref cRRT). Furthermore, return the duration for which
                this control should be applied. The state \e dest is
                modified to match the state reached with the computed
                control and duration. The motion is checked for validity. */
            virtual unsigned int sampleTo(Control *control, const Control *previous, const base::State *source, base::State *dest);

        protected:

            /** \brief Samples \e numControlSamples_ controls, and returns the
                control that brings the system the closest to \e target */
            virtual unsigned int getBestControl (Control *control, const base::State *source, base::State *dest, const Control *previous);

            /** \brief An instance of the control sampler*/
            ControlSamplerPtr       cs_;

            /** \brief The number of controls to sample when finding the best control*/
            unsigned int            numControlSamples_;

        };

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

#ifndef OMPL_CONTROL_CONTROL_SPACE_TYPES_
#define OMPL_CONTROL_CONTROL_SPACE_TYPES_

namespace ompl
{
    namespace control
    {

        /** \brief The type of a control space */
        enum ControlSpaceType
            {

                /** \brief Unset type; this is the default type */
                CONTROL_SPACE_UNKNOWN     =  0,

                /** \brief ompl::control::RealVectorControlSpace */
                CONTROL_SPACE_REAL_VECTOR =  1,

                /** \brief ompl::control::DiscreteControlSpace */
                CONTROL_SPACE_DISCRETE    =  2,

                /** \brief Number of control space types; To add new types,
                    use values that are larger than the count */
                CONTROL_SPACE_TYPE_COUNT
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

#ifndef OMPL_CONTROL_CONTROL_SPACE_
#define OMPL_CONTROL_CONTROL_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/control/Control.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/ControlSpaceTypes.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include <boost/concept_check.hpp>
#include <boost/noncopyable.hpp>
#include <iostream>
#include <vector>

namespace ompl
{

    namespace control
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::ControlSpace */
        OMPL_CLASS_FORWARD(ControlSpace);
        /// @endcond

        /** \class ompl::control::ControlSpacePtr
            \brief A boost shared pointer wrapper for ompl::control::ControlSpace */

        /** \brief A control space representing the space of applicable controls */
        class ControlSpace : private boost::noncopyable
        {
        public:

            /** \brief Construct a control space, given the state space */
            ControlSpace(const base::StateSpacePtr &stateSpace);

            virtual ~ControlSpace();

            /** \brief Cast this instance to a desired type. */
            template<class T>
            T* as()
            {
                /** \brief Make sure the type we are casting to is indeed a control space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ControlSpace*>));

                return static_cast<T*>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            const T* as() const
            {
                /** \brief Make sure the type we are casting to is indeed a control space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ControlSpace*>));

                return static_cast<const T*>(this);
            }

            /** \brief Get the name of the control space */
            const std::string& getName() const;

            /** \brief Set the name of the control space */
            void setName(const std::string &name);

            /** \brief Get the type of the control space. The type can be
                used to verify whether two space instances are of
                the same type */
            int getType() const
            {
                return type_;
            }

            /** \brief Return the state space this control space depends on */
            const base::StateSpacePtr& getStateSpace() const
            {
                return stateSpace_;
            }

            /** \brief Get the dimension of this control space */
            virtual unsigned int getDimension() const = 0;

            /** \brief Allocate memory for a control */
            virtual Control* allocControl() const = 0;

            /** \brief Free the memory of a control */
            virtual void freeControl(Control *control) const = 0;

            /** \brief Copy a control to another */
            virtual void copyControl(Control *destination, const Control *source) const = 0;

            /** \brief Check if two controls are the same */
            virtual bool equalControls(const Control *control1, const Control *control2) const = 0;

            /** \brief Make the control have no effect if it were to be applied to a state for any amount of time. */
            virtual void nullControl(Control *control) const = 0;

            /** \brief Allocate the default control sampler */
            virtual ControlSamplerPtr allocDefaultControlSampler() const = 0;

            /** \brief Allocate an instance of the control sampler for this space. This sampler will be allocated with the
                sampler allocator that was previously specified by setControlSamplerAllocator() or, if no sampler allocator was specified,
                allocDefaultControlSampler() is called */
            virtual ControlSamplerPtr allocControlSampler() const;

            /** \brief Set the sampler allocator to use */
            void setControlSamplerAllocator(const ControlSamplerAllocator &csa);

            /** \brief Clear the control sampler allocator (reset to default) */
            void clearControlSamplerAllocator();

            /** \brief Many controls contain a number of double values. This function provides a means to get the
                memory address of a double value from a control \e control located at position \e index. The first double value
                is returned for \e index = 0. If \e index is too large (does not point to any double values in the control),
                the return value is NULL. */
            virtual double* getValueAddressAtIndex(Control *control, const unsigned int index) const;

            /** \brief Print a control to a stream */
            virtual void printControl(const Control *control, std::ostream &out) const;

            /** \brief Print the settings for this control space to a stream */
            virtual void printSettings(std::ostream &out) const;

            /** \brief Perform final setup steps. This function is automatically called by the SpaceInformation */
            virtual void setup();

            /** \brief Returns the serialization size for a single control in this space */
            virtual unsigned int getSerializationLength() const;

            /** \brief Serializes the given control into the serialization buffer. */
            virtual void serialize(void *serialization, const Control *ctrl) const;

            /** \brief Deserializes a control from the serialization buffer. */
            virtual void deserialize(Control *ctrl, const void *serialization) const;

            /** \brief Compute an array of ints that uniquely identifies the structure of the control space.
                The first element of the signature is the number of integers that follow */
            void computeSignature(std::vector<int> &signature) const;

            /** \brief Check if the control space is compound */
            virtual bool isCompound() const;

        protected:

            /** \brief A type assigned for this control space */
            int                     type_;

            /** \brief The state space controls can be applied to */
            base::StateSpacePtr     stateSpace_;

            /** \brief An optional control sampler allocator */
            ControlSamplerAllocator csa_;

        private:

            /** \brief The name of this control space */
            std::string             name_;
        };

        /** \brief A control space to allow the composition of control spaces */
        class CompoundControlSpace : public ControlSpace
        {
        public:

            /** \brief Define the type of control allocated by this control space */
            typedef CompoundControl ControlType;

            /** \brief Constructor. The corresponding state space needs to be specified. */
            CompoundControlSpace(const base::StateSpacePtr &stateSpace) : ControlSpace(stateSpace), componentCount_(0), locked_(false)
            {
            }

            virtual ~CompoundControlSpace()
            {
            }

            /** \brief Cast a component of this instance to a desired type. */
            template<class T>
            T* as(const unsigned int index) const
            {
                /** \brief Make sure the type we are casting to is indeed a control space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ControlSpace*>));

                return static_cast<T*>(getSubspace(index).get());
            }

            /** \brief Adds a control space as a component of the compound control space. */
            virtual void addSubspace(const ControlSpacePtr &component);

            /** \brief Get the number of control spaces that make up the compound control space */
            unsigned int getSubspaceCount() const;

            /** \brief Get a specific subspace from the compound control space */
            const ControlSpacePtr& getSubspace(const unsigned int index) const;

            /** \brief Get a specific subspace from the compound control space */
            const ControlSpacePtr& getSubspace(const std::string &name) const;

            virtual unsigned int getDimension() const;

            virtual Control* allocControl() const;

            virtual void freeControl(Control *control) const;

            virtual void copyControl(Control *destination, const Control *source) const;

            virtual bool equalControls(const Control *control1, const Control *control2) const;

            virtual void nullControl(Control *control) const;

            virtual ControlSamplerPtr allocDefaultControlSampler() const;

            virtual double* getValueAddressAtIndex(Control *control, const unsigned int index) const;

            virtual void printControl(const Control *control, std::ostream &out = std::cout) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void setup();

            /** \brief Returns the serialization size for a single control in this space */
            virtual unsigned int getSerializationLength() const;

            /** \brief Serializes the given control into the serialization buffer. */
            virtual void serialize(void *serialization, const Control *ctrl) const;

            /** \brief Deserializes a control from the serialization buffer. */
            virtual void deserialize(Control *ctrl, const void *serialization) const;

            virtual bool isCompound() const;

            /** \brief Lock this control space. This means no further
             control spaces can be added as components.  This function can
             be for instance called from the constructor of a state space
             that inherits from CompoundControlSpace to prevent the
             user to add further components. */
            void lock();

        protected:

            /** \brief The component control spaces that make up the compound control space */
            std::vector<ControlSpacePtr>    components_;

            /** \brief The number of contained components */
            unsigned int                    componentCount_;

            /** \brief Flag indicating whether adding further components is allowed or not */
            bool                            locked_;
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

#ifndef OMPL_CONTROL_STATE_PROPAGATOR_
#define OMPL_CONTROL_STATE_PROPAGATOR_

#include "ompl/control/Control.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{

    namespace control
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::StatePropagator */
        OMPL_CLASS_FORWARD(StatePropagator);
        /// @endcond

        /** \class ompl::base::StatePropagatorPtr
            \brief A boost shared pointer wrapper for ompl::control::StatePropagator */

        /** \brief Model the effect of controls on system states */
        class StatePropagator
        {
        public:

            /** \brief Constructor */
            StatePropagator(SpaceInformation *si) : si_(si)
            {
            }

            /** \brief Constructor */
            StatePropagator(const SpaceInformationPtr &si) : si_(si.get())
            {
            }

            virtual ~StatePropagator()
            {
            }

            /** \brief Propagate from a state, given a control, for some specified amount of time (the amount of time can
                also be negative, if canPropagateBackward() returns true)
                \param state the state to start propagating from
                \param control the control to apply
                \param duration the duration for which the control is applied
                \param result the state the system is brought to

                \note This function is <b>not used for integration</b>
                internally. If integrating a system of differential
                equations is needed, this should be implemented inside
                the propagate() function.

                \note The pointer to the starting state and the result
                state may be the same.
            */
            virtual void propagate(const base::State *state, const Control *control, const double duration, base::State *result) const = 0;

            /** \brief Some systems can only propagate forward in time (i.e., the \e duration argument for the propagate()
                function is always positive). If this is the case, this function should return false. Planners that need
                backward propagation (negative durations) will call this function to check. If backward propagation is
                possible, this function should return true (this is the default). */
            virtual bool canPropagateBackward() const
            {
                return true;
            }

            /** \brief Compute the control that can take the system from state \e from to state \e to.
                Store that control in \e result; the duration for which the control should be applied is stored in \e duration;
                return true if the computation was successful; return false otherwise;

                \note If false is returned, the content of \e result and \e duration may have been changed,
                but it does not represent a solution; */
            virtual bool steer(const base::State* /*from*/, const base::State* /*to*/, Control* /*result*/, double& /*duration*/) const
            {
                return false;
            }

            virtual bool canSteer() const
            {
                return false;
            }

        protected:

            /** \brief The instance of space information this state propagator operates on */
            SpaceInformation *si_;

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

#ifndef OMPL_CONTROL_SPACE_INFORMATION_
#define OMPL_CONTROL_SPACE_INFORMATION_

#include "ompl/base/SpaceInformation.h"
#include "ompl/control/ControlSpace.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/control/Control.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{

    /** \brief This namespace contains sampling based planning
        routines used by planning under differential constraints */
    namespace control
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::SpaceInformation */
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /** \class ompl::control::SpaceInformationPtr
            \brief A boost shared pointer wrapper for ompl::control::SpaceInformation */


        /** \brief A function that achieves state propagation.*/
        typedef boost::function<void(const base::State*, const Control*, const double, base::State*)> StatePropagatorFn;

        /** \brief Space information containing necessary information for planning with controls. setup() needs to be called before use. */
        class SpaceInformation : public base::SpaceInformation
        {
        public:

            /** \brief Constructor. Sets the instance of the state and control spaces to plan with. */
            SpaceInformation(const base::StateSpacePtr &stateSpace, const ControlSpacePtr &controlSpace) :
                base::SpaceInformation(stateSpace), controlSpace_(controlSpace),
                minSteps_(0), maxSteps_(0), stepSize_(0.0)
            {
            }

            virtual ~SpaceInformation()
            {
            }

            /** \brief Get the control space */
            const ControlSpacePtr& getControlSpace() const
            {
                return controlSpace_;
            }

            /** @name Control memory management
                @{ */

            /** \brief Allocate memory for a control */
            Control* allocControl() const
            {
                return controlSpace_->allocControl();
            }

            /** \brief Free the memory of a control */
            void freeControl(Control *control) const
            {
                controlSpace_->freeControl(control);
            }

            /** \brief Copy a control to another */
            void copyControl(Control *destination, const Control *source) const
            {
                controlSpace_->copyControl(destination, source);
            }

            /** \brief Clone a control */
            Control* cloneControl(const Control *source) const
            {
                Control *copy = controlSpace_->allocControl();
                controlSpace_->copyControl(copy, source);
                return copy;
            }

            /** @} */

            /** @name Topology-specific control operations (as in the control space)
                @{ */

            /** \brief Print a control to a stream */
            void printControl(const Control *control, std::ostream &out = std::cout) const
            {
                controlSpace_->printControl(control, out);
            }

            /** \brief Check if two controls are the same */
            bool equalControls(const Control *control1, const Control *control2) const
            {
                return controlSpace_->equalControls(control1, control2);
            }

            /** \brief Make the control have no effect if it were to be applied to a state for any amount of time. */
            void nullControl(Control *control) const
            {
                controlSpace_->nullControl(control);
            }

            /** @} */

            /** @name Sampling of controls
                @{ */

            /** \brief Allocate a control sampler */
            ControlSamplerPtr allocControlSampler() const
            {
                return controlSpace_->allocControlSampler();
            }

            /** \brief Set the minimum and maximum number of steps a control is propagated for */
            void setMinMaxControlDuration(unsigned int minSteps, unsigned int maxSteps)
            {
                minSteps_ = minSteps;
                maxSteps_ = maxSteps;
            }

            /** \brief Get the minimum number of steps a control is propagated for */
            unsigned int getMinControlDuration() const
            {
                return minSteps_;
            }

            /** \brief Get the maximum number of steps a control is propagated for */
            unsigned int getMaxControlDuration() const
            {
                return maxSteps_;
            }

            /** \brief Allocate an instance of the DirectedControlSampler to use. This will be the default (SimpleDirectedControlSampler) unless
                setDirectedControlSamplerAllocator() was previously called. */
            DirectedControlSamplerPtr allocDirectedControlSampler() const;

            /** \brief Set the allocator to use for the  DirectedControlSampler */
            void setDirectedControlSamplerAllocator(const DirectedControlSamplerAllocator &dcsa);

            /** \brief Reset the DirectedControlSampler to be the default one */
            void clearDirectedSamplerAllocator();

            /** @} */

            /** @name Configuration of the state propagator
                @{ */

            /** \brief Get the instance of StatePropagator that performs state propagation */
            const StatePropagatorPtr& getStatePropagator() const
            {
                return statePropagator_;
            }

            /** \brief Set the function that performs state propagation */
            void setStatePropagator(const StatePropagatorFn &fn);

            /** \brief Set the instance of StatePropagator to perform state propagation */
            void setStatePropagator(const StatePropagatorPtr &sp);

            /** \brief When controls are applied to states, they are applied for a time duration that is an integer
                multiple of the stepSize, within the bounds specified by setMinMaxControlDuration() */
            void setPropagationStepSize(double stepSize)
            {
                stepSize_ = stepSize;
            }

            /** \brief Propagation is performed at integer multiples of a specified step size. This function returns the value of this step size. */
            double getPropagationStepSize() const
            {
                return stepSize_;
            }
            /** @} */

            /** @name Primitives for propagating the model of the system
                @{ */

            /** \brief Propagate the model of the system forward, starting a a given state, with a given control, for a given number of steps.
                \param state the state to start at
                \param control the control to apply
                \param steps the number of time steps to apply the control for. Each time step is of length getPropagationStepSize()
                \param result the state at the end of the propagation */
            void propagate(const base::State *state, const Control *control, int steps, base::State *result) const;

            /** \brief Some systems can only propagate forward in time (i.e., the \e steps argument for the propagate()
                function is always positive). If this is the case, this function will return false. Planners that need
                backward propagation (negative \e steps) will call this function to check. If backward propagation is
                possible, this function will return true (this is the default). */
            bool canPropagateBackward() const;

            /** \brief Propagate the model of the system forward, starting at a given state, with a given control, for a given number of steps.
                Stop if a collision is found and return the number of steps actually performed without collision. If no collision is found, the returned value is
                equal to the \e steps argument. If a collision is found after the first step, the return value is 0 and \e result = \e state.
                \param state the state to start at
                \param control the control to apply
                \param steps the maximum number of time steps to apply the control for. Each time step is of length getPropagationStepSize(). If \e steps is negative, backward propagation will be performed.
                \param result the state at the end of the propagation or the last valid state if a collision is found */
            unsigned int propagateWhileValid(const base::State *state, const Control *control, int steps, base::State *result) const;

            /** \brief Propagate the model of the system forward, starting a a given state, with a given control, for a given number of steps.
                \param state the state to start at
                \param control the control to apply
                \param steps the number of time steps to apply the control for. Each time step is of length getPropagationStepSize(). If \e steps is negative, backward propagation will be performed.
                \param result the set of states along the propagated motion
                \param alloc flag indicating whether memory for the states in \e result should be allocated

                \note Start state \e state is not included in \e result */
            void propagate(const base::State *state, const Control *control, int steps, std::vector<base::State*> &result, bool alloc) const;

            /** \brief Propagate the model of the system forward, starting at a given state, with a given control, for a given number of steps.
                Stop if a collision is found and return the number of steps actually performed without collision. If no collision is found, the returned value is
                equal to the \e steps argument.  If a collision is found after the first step, the return value is 0 and no states are added to \e result.
                If \e alloc is false and \e result cannot store all the generated states, propagation is stopped prematurely (when \e result is full).
                The starting state (\e state) is not included in \e result. The return value of the function indicates how many states have been written to \e result.

                \param state the state to start at
                \param control the control to apply
                \param steps the maximum number of time steps to apply the control for. Each time step is of length getPropagationStepSize(). If \e steps is negative, backward propagation will be performed.
                \param result the set of states along the propagated motion (only valid states included)
                \param alloc flag indicating whether memory for the states in \e result should be allocated
            */
            unsigned int propagateWhileValid(const base::State *state, const Control *control, int steps, std::vector<base::State*> &result, bool alloc) const;

            /** @} */

            /** \brief Print information about the current instance of the state space */
            virtual void printSettings(std::ostream &out = std::cout) const;

            /** \brief Perform additional setup tasks (run once, before use) */
            virtual void setup();

        protected:

            /** \brief The control space describing the space of controls applicable to states in the state space */
            ControlSpacePtr                 controlSpace_;

            /** \brief The state propagator used to model the motion of the system being planned for */
            StatePropagatorPtr              statePropagator_;

            /** \brief The minimum number of steps to apply a control for */
            unsigned int                    minSteps_;

            /** \brief The maximum number of steps to apply a control for */
            unsigned int                    maxSteps_;

            /** \brief Optional allocator for the DirectedControlSampler. If not specified, the default implementation is used */
            DirectedControlSamplerAllocator dcsa_;

            /** \brief The actual duration of each step */
            double                          stepSize_;

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

/* Author: Ryan Luna */

#ifndef OMPL_CONTROL_ODESOLVER_
#define OMPL_CONTROL_ODESOLVER_

// Boost.OdeInt needs Boost version >= 1.44
#include <boost/version.hpp>
#if BOOST_VERSION < 104400
#warning Boost version >=1.44 is needed for ODESolver classes
#else

#include "ompl/control/Control.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"

#if BOOST_VERSION >= 105300
#include <boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::odeint;
#else
#include <omplext_odeint/boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::omplext_odeint;
#endif
#include <boost/function.hpp>
#include <cassert>
#include <vector>

namespace ompl
{

    namespace control
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ODESolver);
        /// @endcond

        /// \class ompl::control::ODESolverPtr
        /// \brief A boost shared pointer wrapper for ompl::control::ODESolver

        /// \brief Abstract base class for an object that can solve ordinary differential
        /// equations (ODE) of the type q' = f(q,u) using numerical integration.  Classes
        /// deriving from this must implement the solve method.  The user must supply
        /// the ODE to solve.
        class ODESolver
        {
        public:
            /// \brief Portable data type for the state values
            typedef std::vector<double> StateType;

            /// \brief Callback function that defines the ODE.  Accepts
            /// the current state, input control, and output state.
            typedef boost::function<void(const StateType &, const Control*, StateType &)> ODE;

            /// \brief Callback function to perform an event at the end of numerical
            /// integration.  This functionality is optional.
            typedef boost::function<void(const base::State *state, const Control *control, const double duration, base::State *result)> PostPropagationEvent;

            /// \brief Parameterized constructor.  Takes a reference to SpaceInformation,
            /// an ODE to solve, and the integration step size.
            ODESolver (const SpaceInformationPtr& si, const ODE& ode, double intStep) : si_(si), ode_(ode), intStep_(intStep)
            {
            }

            /// \brief Destructor.
            virtual ~ODESolver ()
            {
            }

            /// \brief Set the ODE to solve
            void setODE (const ODE &ode)
            {
                ode_ = ode;
            }

            /// \brief Return the size of a single numerical integration step
            double getIntegrationStepSize () const
            {
                return intStep_;
            }

            /// \brief Set the size of a single numerical integration step
            void setIntegrationStepSize (double intStep)
            {
                intStep_ = intStep;
            }

            /** \brief Get the current instance of the space information */
            const SpaceInformationPtr& getSpaceInformation() const
            {
                return si_;
            }

            /// \brief Retrieve a StatePropagator object that solves a system of ordinary
            /// differential equations defined by an ODESolver.
            /// An optional PostPropagationEvent can also be specified as a callback after
            /// numerical integration is finished for further operations on the resulting
            /// state.
            static StatePropagatorPtr getStatePropagator (ODESolverPtr solver,
                const PostPropagationEvent &postEvent = NULL)
            {
                class ODESolverStatePropagator : public StatePropagator
                {
                    public:
                        ODESolverStatePropagator (ODESolverPtr solver, const PostPropagationEvent &pe) : StatePropagator (solver->si_), solver_(solver), postEvent_(pe)
                        {
                            if (!solver.get())
                                OMPL_ERROR("ODESolverPtr does not reference a valid ODESolver object");
                        }

                        virtual void propagate (const base::State *state, const Control *control, const double duration, base::State *result) const
                        {
                            ODESolver::StateType reals;
                            si_->getStateSpace()->copyToReals(reals, state);
                            solver_->solve (reals, control, duration);
                            si_->getStateSpace()->copyFromReals(result, reals);

                            if (postEvent_)
                                postEvent_ (state, control, duration, result);
                        }

                    protected:
                        ODESolverPtr solver_;
                        ODESolver::PostPropagationEvent postEvent_;
                };
                return StatePropagatorPtr(dynamic_cast<StatePropagator*>(new ODESolverStatePropagator(solver, postEvent)));
            }

        protected:

            /// \brief Solve the ODE given the initial state, and a control to apply for some duration.
            virtual void solve (StateType &state, const Control *control, const double duration) const = 0;

            /// \brief The SpaceInformation that this ODESolver operates in.
            const SpaceInformationPtr     si_;

            /// \brief Definition of the ODE to find solutions for.
            ODE                           ode_;

            /// \brief The size of the numerical integration step.  Should be small to minimize error.
            double                        intStep_;

            /// @cond IGNORE
            // Functor used by the boost::numeric::odeint stepper object
            struct ODEFunctor
            {
                ODEFunctor (const ODE &o, const Control *ctrl) : ode(o), control(ctrl) {}

                // boost::numeric::odeint will callback to this method during integration to evaluate the system
                void operator () (const StateType &current, StateType &output, double /*time*/)
                {
                    ode (current, control, output);
                }

                ODE ode;
                const Control *control;
            };
            /// @endcond
        };

        /// \brief Basic solver for ordinary differential equations of the type q' = f(q, u),
        /// where q is the current state of the system and u is a control applied to the
        /// system.  StateType defines the container object describing the state of the system.
        /// Solver is the numerical integration method used to solve the equations.  The default
        /// is a fourth order Runge-Kutta method.  This class wraps around the simple stepper
        /// concept from boost::numeric::odeint.
        template <class Solver = odeint::runge_kutta4<ODESolver::StateType> >
        class ODEBasicSolver : public ODESolver
        {
        public:

            /// \brief Parameterized constructor.  Takes a reference to the SpaceInformation,
            /// an ODE to solve, and an optional integration step size - default is 0.01
            ODEBasicSolver (const SpaceInformationPtr &si, const ODESolver::ODE &ode, double intStep = 1e-2) : ODESolver(si, ode, intStep)
            {
            }

        protected:

            /// \brief Solve the ODE using boost::numeric::odeint.
            virtual void solve (StateType &state, const Control *control, const double duration) const
            {
                Solver solver;
                ODESolver::ODEFunctor odefunc (ode_, control);
                odeint::integrate_const (solver, odefunc, state, 0.0, duration, intStep_);
            }
        };

        /// \brief Solver for ordinary differential equations of the type q' = f(q, u),
        /// where q is the current state of the system and u is a control applied to the
        /// system.  StateType defines the container object describing the state of the system.
        /// Solver is the numerical integration method used to solve the equations.  The default
        /// is a fifth order Runge-Kutta Cash-Karp method with a fourth order error bound.
        /// This class wraps around the error stepper concept from boost::numeric::odeint.
        template <class Solver = odeint::runge_kutta_cash_karp54<ODESolver::StateType> >
        class ODEErrorSolver : public ODESolver
        {
        public:
            /// \brief Parameterized constructor.  Takes a reference to the SpaceInformation,
            /// an ODE to solve, and the integration step size - default is 0.01
            ODEErrorSolver (const SpaceInformationPtr &si, const ODESolver::ODE &ode, double intStep = 1e-2) : ODESolver(si, ode, intStep)
            {
            }

            /// \brief Retrieves the error values from the most recent integration
            ODESolver::StateType getError ()
            {
                return error_;
            }

        protected:
            /// \brief Solve the ODE using boost::numeric::odeint.  Save the resulting error values into error_.
            virtual void solve (StateType &state, const Control *control, const double duration) const
            {
                ODESolver::ODEFunctor odefunc (ode_, control);

                if (error_.size () != state.size ())
                    error_.assign (state.size (), 0.0);

                Solver solver;
                solver.adjust_size (state);

                double time = 0.0;
                while (time < duration + std::numeric_limits<float>::epsilon())
                {
                    solver.do_step (odefunc, state, time, intStep_, error_);
                    time += intStep_;
                }
            }

            /// \brief The error values calculated during numerical integration
            mutable ODESolver::StateType error_;
        };

        /// \brief Adaptive step size solver for ordinary differential equations of the type
        /// q' = f(q, u), where q is the current state of the system and u is a control applied
        /// to the system.  The maximum integration error is bounded in this approach.
        /// Solver is the numerical integration method used to solve the equations, and must implement
        /// the error stepper concept from boost::numeric::odeint.  The default
        /// is a fifth order Runge-Kutta Cash-Karp method with a fourth order error bound.
        template <class Solver = odeint::runge_kutta_cash_karp54<ODESolver::StateType> >
        class ODEAdaptiveSolver : public ODESolver
        {
        public:
            /// \brief Parameterized constructor.  Takes a reference to the SpaceInformation,
            /// an ODE to solve, and an optional integration step size - default is 0.01
            ODEAdaptiveSolver (const SpaceInformationPtr &si, const ODESolver::ODE &ode, double intStep = 1e-2) : ODESolver(si, ode, intStep), maxError_(1e-6), maxEpsilonError_(1e-7)
            {
            }

            /// \brief Retrieve the total error allowed during numerical integration
            double getMaximumError () const
            {
                return maxError_;
            }

            /// \brief Set the total error allowed during numerical integration
            void setMaximumError (double error)
            {
                maxError_ = error;
            }

            /// \brief Retrieve the error tolerance during one step of numerical integration (local truncation error)
            double getMaximumEpsilonError () const
            {
                return maxEpsilonError_;
            }

            /// \brief Set the error tolerance during one step of numerical integration (local truncation error)
            void setMaximumEpsilonError (double error)
            {
                maxEpsilonError_ = error;
            }

        protected:

            /// \brief Solve the ordinary differential equation given the input state
            /// of the system, a control to apply to the system, and the duration to
            /// apply the control.  The value of \e state will contain the final
            /// values for the system after integration.
            virtual void solve (StateType &state, const Control *control, const double duration) const
            {
                ODESolver::ODEFunctor odefunc (ode_, control);

#if BOOST_VERSION < 105600
                odeint::controlled_runge_kutta< Solver > solver (odeint::default_error_checker<double>(maxError_, maxEpsilonError_));
#else
                typename boost::numeric::odeint::result_of::make_controlled< Solver >::type solver = make_controlled( 1.0e-6 , 1.0e-6 , Solver() );
#endif
                odeint::integrate_adaptive (solver, odefunc, state, 0.0, duration, intStep_);
            }

            /// \brief The maximum error allowed when performing numerical integration
            double maxError_;

            /// \brief The maximum error allowed during one step of numerical integration
            double maxEpsilonError_;
        };
    }
}

#endif

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

#ifndef OMPL_CONTROL_PATH_CONTROL_
#define OMPL_CONTROL_PATH_CONTROL_

#include "ompl/control/SpaceInformation.h"
#include "ompl/base/Path.h"
#include "ompl/geometric/PathGeometric.h"
#include <vector>

namespace ompl
{
    namespace control
    {

        /** \brief Definition of a control path.

         This is the type of path produced when planning with
         differential constraints. */
        class PathControl : public base::Path
        {
        public:

            /** \brief Constructor */
            PathControl(const base::SpaceInformationPtr &si);

            /** \brief Copy constructor */
            PathControl(const PathControl &path);

            virtual ~PathControl()
            {
                freeMemory();
            }

            /** \brief Assignment operator */
            PathControl& operator=(const PathControl &other);

            /** \brief The path length (sum of control durations) */
            virtual double length() const;

            /** \brief Check if the path is valid */
            virtual bool check() const;

            /** \brief Print the path to a stream */
            virtual void print(std::ostream &out) const;
            /** \brief Print the path as a real-valued matrix where the
                i-th row represents the i-th state along the path, followed
                by the control and duration needed to reach this state. For
                the first state the control and duration are zeroes. The
                state components printed are those returned by
                ompl::base::StateSpace::copyToReals, while the control
                components printed are the discrete components (if any)
                followed by the real-valued ones as returned by
                ompl::control::ControlSpace::getValueAddressAtIndex. */
            virtual void printAsMatrix(std::ostream &out) const;

            /** \brief Convert this path into a geometric path (interpolation is performed and then states are copied) */
            geometric::PathGeometric asGeometric() const;

            /** @name Path operations
                @{ */

            /** \brief Append \e state to the end of the path; it is assumed \e state is the first state, so no control is applied.
                The memory for \e state is copied. There are no checks to make sure the number of controls and states make sense. */
            void append(const base::State *state);

            /** \brief Append \e state to the end of the path and assume \e control is applied for the duration \e duration.
                The memory for \e state and for \e control is copied. There are no checks to make sure the number of controls and states make sense. */
            void append(const base::State *state, const Control *control, double duration);

            /** \brief Make the path such that all controls are applied for a single time step (computes intermediate states) */
            void interpolate();

            /** \brief Set this path to a random segment */
            void random();

            /** \brief Set this path to a random valid segment. Sample \e attempts times for valid segments. Returns true on success.*/
            bool randomValid(unsigned int attempts);

            /** @} */

            /** @name Functionality for accessing states and controls
                @{ */

            /** \brief Get the states that make up the path (as a reference, so it can be modified, hence the function is not const) */
            std::vector<base::State*>& getStates()
            {
                return states_;
            }

            /** \brief Get the controls that make up the path (as a reference, so it can be modified, hence the function is not const) */
            std::vector<Control*>& getControls()
            {
                return controls_;
            }

            /** \brief Get the control durations used along the path (as a reference, so it can be modified, hence the function is not const) */
            std::vector<double>& getControlDurations()
            {
                return controlDurations_;
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

            /** \brief Get the control located at \e index along the path. This is the control that gets applied to the state located at \e index */
            Control* getControl(unsigned int index)
            {
                return controls_[index];
            }

            /** \brief Get the control located at \e index along the path. This is the control that gets applied to the state located at \e index */
            const Control* getControl(unsigned int index) const
            {
                return controls_[index];
            }

            /** \brief Get the duration of the control at \e index, which gets applied to the state at \e index. */
            double getControlDuration(unsigned int index) const
            {
                return controlDurations_[index];
            }

            /** \brief Get the number of states (way-points) that make up this path */
            std::size_t getStateCount() const
            {
                return states_.size();
            }

            /** \brief Get the number of controls applied along this path. This should be equal to getStateCount() - 1 unless there are 0 states, in which case the number of controls will also be 0. */
            std::size_t getControlCount() const
            {
                return controls_.size();
            }

            /** @} */

        protected:

            /** \brief The list of states that make up the path */
            std::vector<base::State*>   states_;

            /** \brief The control applied at each state. This array contains one element less than the list of states */
            std::vector<Control*>       controls_;

            /** \brief The duration of the control applied at each state. This array contains one element less than the list of states */
            std::vector<double>         controlDurations_;

            /** \brief Free the memory allocated by the path */
            void freeMemory();

            /** \brief Copy the content of a path to this one */
            void copyFrom(const PathControl &other);

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

#ifndef OMPL_CONTROL_SIMPLE_SETUP_
#define OMPL_CONTROL_SIMPLE_SETUP_

#include "ompl/base/Planner.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/PlannerData.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/control/PathControl.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

namespace ompl
{

    namespace control
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SimpleSetup);
        /// @endcond

        /** \class ompl::control::SimpleSetupPtr
            \brief A boost shared pointer wrapper for ompl::control::SimpleSetup */

        /** \brief Create the set of classes typically needed to solve a
            control problem */
        class SimpleSetup
        {
        public:

            /** \brief Constructor needs the control space used for planning. */
            explicit
            SimpleSetup(const SpaceInformationPtr &si);

            /** \brief Constructor needs the control space used for planning. */
            explicit
            SimpleSetup(const ControlSpacePtr &space);

            virtual ~SimpleSetup()
            {
            }

            /** \brief Get the current instance of the space information */
            const SpaceInformationPtr& getSpaceInformation() const
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

            /** \brief Get the current instance of the control space */
            const ControlSpacePtr& getControlSpace() const
            {
                return si_->getControlSpace();
            }

            /** \brief Get the current instance of the state validity checker */
            const base::StateValidityCheckerPtr& getStateValidityChecker() const
            {
                return si_->getStateValidityChecker();
            }

            /** \brief Get the instance of the state propagator being used */
            const StatePropagatorPtr& getStatePropagator() const
            {
                return si_->getStatePropagator();
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

            /** \brief Return true if a solution path is available (previous call to solve() was successful) and the solution is exact (not approximate) */
            bool haveExactSolutionPath() const;


            /** \brief Return true if a solution path is available (previous call to solve() was successful). The solution may be approximate. */
            bool haveSolutionPath() const
            {
                return pdef_->getSolutionPath().get();
            }

            /** \brief Get the solution path. Throw an exception if no solution is available */
            PathControl& getSolutionPath() const;

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

            /** \brief Set the function that performs state propagation */
            void setStatePropagator(const StatePropagatorFn &sp)
            {
                si_->setStatePropagator(sp);
            }

            /** \brief Set the instance of StatePropagator to perform state propagation */
            void setStatePropagator(const StatePropagatorPtr &sp)
            {
                si_->setStatePropagator(sp);
            }

            /** \brief Set the optimization objective to use */
            void setOptimizationObjective(const base::OptimizationObjectivePtr &optimizationObjective)
            {
                pdef_->setOptimizationObjective(optimizationObjective);
            }

            /** \brief Set the start and goal states to use. */
            void setStartAndGoalStates(const base::ScopedState<> &start, const base::ScopedState<> &goal, const double threshold = std::numeric_limits<double>::epsilon())
            {
                pdef_->setStartAndGoalStates(start, goal, threshold);
            }

            /** \brief A simple form of setGoal(). The goal will be an instance of ompl::base::GoalState */
            void setGoalState(const base::ScopedState<> &goal, const double threshold = std::numeric_limits<double>::epsilon())
            {
                pdef_->setGoalState(goal, threshold);
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

            /** \brief Run the planner for a specified amount of time (default is 1 second) */
            virtual base::PlannerStatus solve(double time = 1.0);

            /** \brief Run the planner until \e ptc becomes true (at most) */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Return the status of the last planning attempt */
            base::PlannerStatus getLastPlannerStatus() const
            {
                return last_status_;
            }

            /** \brief Get the amount of time (in seconds) spent during the last planning step */
            double getLastPlanComputationTime() const
            {
                return planTime_;
            }

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
            SpaceInformationPtr           si_;

            /// The created problem definition
            base::ProblemDefinitionPtr    pdef_;

            /// The maintained planner instance
            base::PlannerPtr              planner_;

            /// The optional planner allocator
            base::PlannerAllocator        pa_;

            /// Flag indicating whether the classes needed for planning are set up
            bool                          configured_;

            /// The amount of time the last planning step took
            double                        planTime_;

            /// The status of the last planning request
            base::PlannerStatus           last_status_;

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

#ifndef OMPL_CONTROL_SPACES_DISCRETE_CONTROL_SPACE_
#define OMPL_CONTROL_SPACES_DISCRETE_CONTROL_SPACE_

#include "ompl/control/ControlSpace.h"

namespace ompl
{
    namespace control
    {

        /** \brief Control space sampler for discrete controls */
        class DiscreteControlSampler : public ControlSampler
        {
        public:

            /** \brief Constructor */
            DiscreteControlSampler(const ControlSpace *space) : ControlSampler(space)
            {
            }

            virtual void sample(Control *control);
        };

        /** \brief A space representing discrete controls; i.e. there
            are a small number of discrete controls the system can react to.
            Controls are represented as integers [lowerBound, upperBound],
            where lowerBound and upperBound are inclusive. */
        class DiscreteControlSpace : public ControlSpace
        {
        public:

            /** \brief The definition of a discrete control */
            class ControlType : public Control
            {
            public:

                /** \brief The current control - an int in range [lowerBound, upperBound] */
                int value;
            };

            /** \brief Construct a discrete space in wich controls can take values in the set [\e lowerBound, \e upperBound] */
            DiscreteControlSpace(const base::StateSpacePtr &stateSpace, int lowerBound, int upperBound) :
                ControlSpace(stateSpace), lowerBound_(lowerBound), upperBound_(upperBound)
            {
                setName("Discrete" + getName());
                type_ = CONTROL_SPACE_DISCRETE;
            }

            virtual ~DiscreteControlSpace()
            {
            }

            virtual unsigned int getDimension() const;

            virtual void copyControl(Control *destination, const Control *source) const;

            virtual bool equalControls(const Control *control1, const Control *control2) const;

            virtual ControlSamplerPtr allocDefaultControlSampler() const;

            virtual Control* allocControl() const;

            virtual void freeControl(Control *control) const;

            /** \brief This sets the control value to \e lowerBound_ */
            virtual void nullControl(Control *control) const;

            virtual void printControl(const Control *control, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            /** \brief Returns the number of controls possible */
            unsigned int getControlCount() const
            {
                return upperBound_ - lowerBound_ + 1;
            }

            /** \brief Returns the lowest possible control value */
            int getLowerBound() const
            {
                return lowerBound_;
            }

            /** \brief Returns the highest possible control value */
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

            /** \brief Returns the serialization size for a single control in this space */
            virtual unsigned int getSerializationLength() const;

            /** \brief Serializes the given control into the serialization buffer. */
            virtual void serialize(void *serialization, const Control *ctrl) const;

            /** \brief Deserializes a control from the serialization buffer. */
            virtual void deserialize(Control *ctrl, const void *serialization) const;

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

#ifndef OMPL_CONTROL_SPACES_REAL_VECTOR_CONTROL_SPACE_
#define OMPL_CONTROL_SPACES_REAL_VECTOR_CONTROL_SPACE_

#include "ompl/control/ControlSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include <vector>

namespace ompl
{
    namespace control
    {

        /** \brief Uniform sampler for the R<sup>n</sup> state space */
        class RealVectorControlUniformSampler : public ControlSampler
        {
        public:

            /** \brief Constructor */
            RealVectorControlUniformSampler(const ControlSpace *space) : ControlSampler(space)
            {
            }

            virtual void sample(Control *control);
        };

        /** \brief A control space representing R<sup>n</sup>. */
        class RealVectorControlSpace : public ControlSpace
        {
        public:

            /** \brief The definition of a control in R<sup>n</sup> */
            class ControlType : public Control
            {
            public:

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

                /** \brief An array of length \e n, representing the value of the control */
                double *values;
            };

            /** \brief Constructor takes the state space the controls correspond to and the dimension of the space of controls, \e dim */
            RealVectorControlSpace(const base::StateSpacePtr &stateSpace, unsigned int dim) :
                ControlSpace(stateSpace), dimension_(dim), bounds_(dim), controlBytes_(dim * sizeof(double))
            {
                setName("RealVector" + getName());
                type_ = CONTROL_SPACE_REAL_VECTOR;
            }

            virtual ~RealVectorControlSpace()
            {
            }

            /** \brief Set the bounds (min max values for each dimension) for the control */
            void setBounds(const base::RealVectorBounds &bounds);

            /** \brief Get the bounds (min max values for each dimension) for the control */
            const base::RealVectorBounds& getBounds() const
            {
                return bounds_;
            }

            virtual unsigned int getDimension() const;

            virtual void copyControl(Control *destination, const Control *source) const;

            virtual bool equalControls(const Control *control1, const Control *control2) const;

            virtual ControlSamplerPtr allocDefaultControlSampler() const;

            virtual Control* allocControl() const;

            virtual void freeControl(Control *control) const;

            virtual void nullControl(Control *control) const;

            virtual void printControl(const Control *control, std::ostream &out) const;

            virtual double* getValueAddressAtIndex(Control *control, const unsigned int index) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void setup();

            /** \brief Returns the serialization size for a single control in this space */
            virtual unsigned int getSerializationLength() const;

            /** \brief Serializes the given control into the serialization buffer. */
            virtual void serialize(void *serialization, const Control *ctrl) const;

            /** \brief Deserializes a control from the serialization buffer. */
            virtual void deserialize(Control *ctrl, const void *serialization) const;

        protected:

            /** \brief The dimension of the state space */
            unsigned int           dimension_;

            /** \brief The bounds on controls */
            base::RealVectorBounds bounds_;

        private:
            std::size_t            controlBytes_;
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

/* Author: Mark Moll */

#ifndef OMPL_CONTROL_PLANNER_DATA_
#define OMPL_CONTROL_PLANNER_DATA_

#include "ompl/base/PlannerData.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/Control.h"
#include <boost/serialization/base_object.hpp>

namespace ompl
{
    namespace control
    {
        /// \brief Representation of an edge in PlannerData for planning with controls.
        /// This structure encodes a specific control and a duration to apply the control.
        /// \remarks If using PlannerDataEdgeControl in conjunction with PlannerDataStorage,
        /// (i.e., storing the PlannerData from a controls planner) you must export a GUID
        /// for PlannerDataEdgeControl so that the serializer can identify the derived
        /// edge class:
        /// \code
        /// #include <boost/serialization/export.hpp>
        /// ...
        /// BOOST_CLASS_EXPORT(ompl::control::PlannerDataEdgeControl);
        /// \endcode
        class PlannerDataEdgeControl : public base::PlannerDataEdge
        {
        public:
            /// \brief Constructor.  Accepts a control pointer and a duration.
            PlannerDataEdgeControl(const Control *c, double duration) : PlannerDataEdge(), c_(c), duration_(duration) {}
            /// \brief Copy constructor.
            PlannerDataEdgeControl(const PlannerDataEdgeControl &rhs) : PlannerDataEdge(), c_(rhs.c_), duration_(rhs.duration_) {}

            virtual ~PlannerDataEdgeControl() {}

            virtual base::PlannerDataEdge* clone() const
            {
                return static_cast<base::PlannerDataEdge*>(new PlannerDataEdgeControl(*this));
            }

            /// \brief Return the control associated with this edge.
            const Control* getControl() const { return c_; }
            /// \brief Return the duration associated with this edge.
            double getDuration() const { return duration_; }

            virtual bool operator==(const PlannerDataEdge &rhs) const
            {
                const PlannerDataEdgeControl *rhsc = static_cast<const PlannerDataEdgeControl*> (&rhs);
                if (c_ == rhsc->c_)
                    return static_cast<const PlannerDataEdge>(*this) == rhs;
                else
                    return false;
            }

        protected:
            friend class boost::serialization::access;
            friend class PlannerDataStorage;
            friend class PlannerData;

            PlannerDataEdgeControl() : PlannerDataEdge(), c_(NULL) {};

            template <class Archive>
            void serialize(Archive &ar, const unsigned int /*version*/)
            {
                ar & boost::serialization::base_object<base::PlannerDataEdge>(*this);
                ar & duration_;
                // Serializing the control is handled by control::PlannerDataStorage
            }

            const Control *c_;
            double duration_;
        };

        /// \copydoc ompl::base::PlannerData
        /// \brief This class assumes edges are derived from PlannerDataEdgeControl.
        /// If this is not the case, see base::PlannerData.
        class PlannerData : public base::PlannerData
        {
        public:

            /// \brief Constructor.  Accepts a SpaceInformationPtr for the space planned in.
            PlannerData(const SpaceInformationPtr &siC);
            /// \brief Destructor.
            virtual ~PlannerData();

            /// \brief Removes the vertex associated with the given data.  If the
            /// vertex does not exist, false is returned.
            /// This method has O(n) complexity in the number of vertices.
            virtual bool removeVertex(const base::PlannerDataVertex &st);
            /// \brief Removes the vertex with the given index.  If the index is
            /// out of range, false is returned.
            /// This method has O(n) complexity in the number of vertices.
            virtual bool removeVertex(unsigned int vIndex);

            /// \brief Removes the edge between vertex indexes \e v1 and \e v2.  Success is returned.
            virtual bool removeEdge(unsigned int v1, unsigned int v2);
            /// \brief Removes the edge between the vertices associated with the given vertex data.
            /// Success is returned.
            virtual bool removeEdge(const base::PlannerDataVertex &v1, const base::PlannerDataVertex &v2);

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

            /// \brief Return the instance of SpaceInformation used in this PlannerData
            const SpaceInformationPtr& getSpaceInformation() const;

            /// \brief Returns true if this PlannerData instance has controls associated with it
            virtual bool hasControls() const;

        protected:
            /// \brief The instance of control::SpaceInformation associated with this data
            SpaceInformationPtr  siC_;
            /// \brief A list of controls that are allocated during the decoupleFromPlanner method.
            /// These controls are freed by PlannerData in the destructor.
            std::set<Control*> decoupledControls_;

        private:
            void freeMemory();
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

#ifndef OMPL_CONTROL_PLANNER_DATA_STORAGE_
#define OMPL_CONTROL_PLANNER_DATA_STORAGE_

// PlannerDataStorage requires Boost version >= 1.44
#include <boost/version.hpp>
#if BOOST_VERSION < 104400
#warning Boost version >= 1.44 is required for PlannerDataStorage classes
#else

#include "ompl/base/PlannerDataStorage.h"
#include "ompl/control/PlannerData.h"
#include "ompl/control/SpaceInformation.h"

namespace ompl
{
    namespace control
    {
        /// \copydoc ompl::base::PlannerDataStorage
        /// \brief It is assumed that the edges in stored/loaded PlannerData can
        /// be cast to PlannerDataEdgeControl in this class.  If this is not the
        /// case, see ompl::base::PlannerDataStorage.
        class PlannerDataStorage : public base::PlannerDataStorage
        {
        public:
            /// \brief Default constructor
            PlannerDataStorage();
            /// \brief Destructor
            virtual ~PlannerDataStorage();

            /// \brief Load the PlannerData structure from the given filename.
            virtual void load(const char *filename, base::PlannerData &pd);

            /// \brief Deserializes the structure from the given stream.
            virtual void load(std::istream &in, base::PlannerData &pd);

            /// \brief Store (serialize) the structure to the given filename.
            /// The StateSpace and ControlSpace that was used to store the data
            /// must match those inside of the argument PlannerData.
            virtual void store(const base::PlannerData &pd, const char *filename);

            /// \brief Load the PlannerData structure from the given stream.
            /// The StateSpace and ControlSpace that was used to store the data
            /// must match those inside of the argument PlannerData.
            virtual void store(const base::PlannerData &pd, std::ostream &out);

        protected:

            /// @cond IGNORE
            // Information stored at the beginning of the PlannerData archive
            struct Header : base::PlannerDataStorage::Header
            {
                /// \brief Signature of control space that allocated the saved states in the vertices (see ompl::base::StateSpace::computeSignature()) */
                std::vector<int> control_signature;

                /// \brief boost::serialization routine
                template<typename Archive>
                void serialize(Archive & ar, const unsigned int /*version*/)
                {
                    ar & boost::serialization::base_object<base::PlannerDataStorage::Header>(*this);
                    ar & control_signature;
                }
            };

            // The object containing all control edge data that will be stored
            struct PlannerDataEdgeControlData : base::PlannerDataStorage::PlannerDataEdgeData
            {
                template<typename Archive>
                void serialize(Archive & ar, const unsigned int /*version*/)
                {
                    ar & boost::serialization::base_object<base::PlannerDataStorage::PlannerDataEdgeData>(*this);
                    ar & control_;
                }

                std::vector<unsigned char> control_;
            };
            /// \endcond

            /// \brief Read \e numEdges from the binary input \e ia and store them as PlannerData.  It is assumed
            /// that the edges can be cast to ompl::control::PlannerDataEdgeControl.
            virtual void loadEdges(base::PlannerData &pd, unsigned int numEdges, boost::archive::binary_iarchive &ia)
            {
                OMPL_DEBUG("Loading %d PlannerDataEdgeControl objects", numEdges);

                const ControlSpacePtr& space = static_cast<control::PlannerData&>(pd).getSpaceInformation()->getControlSpace();
                std::vector<Control*> controls;

                for (unsigned int i = 0; i < numEdges; ++i)
                {
                    PlannerDataEdgeControlData edgeData;
                    ia >> edgeData;

                    std::vector<unsigned char> ctrlBuf (space->getSerializationLength());
                    Control *ctrl = space->allocControl();
                    controls.push_back(ctrl);
                    space->deserialize(ctrl, &edgeData.control_[0]);
                    const_cast<PlannerDataEdgeControl*>(static_cast<const PlannerDataEdgeControl*>(edgeData.e_))->c_ = ctrl;

                    pd.addEdge(edgeData.endpoints_.first, edgeData.endpoints_.second, *edgeData.e_, base::Cost(edgeData.weight_));

                    // We deserialized the edge object pointer, and we own it.
                    // Since addEdge copies the object, it is safe to free here.
                    delete edgeData.e_;
                }

                // These edges are using control pointers allocated here.
                // To avoid a memory leak, we decouple planner data from the
                // 'planner', which will clone all controls and properly free the
                // memory when PlannerData goes out of scope.  Then it is safe
                // to free all memory allocated here.
                pd.decoupleFromPlanner();

                for (size_t i = 0; i < controls.size(); ++i)
                    space->freeControl(controls[i]);
            }

            /// \brief Serialize and store all edges in \e pd to the binary archive.  It is assumed
            /// that the edges can be cast to ompl::control::PlannerDataEdgeControl.
            virtual void storeEdges(const base::PlannerData &pd, boost::archive::binary_oarchive &oa)
            {
                OMPL_DEBUG("Storing %d PlannerDataEdgeControl objects", pd.numEdges());

                const ControlSpacePtr& space = static_cast<const control::PlannerData&>(pd).getSpaceInformation()->getControlSpace();
                std::vector<unsigned char> ctrl (space->getSerializationLength());

                for (unsigned int i = 0; i < pd.numVertices(); ++i)
                    for (unsigned int j = 0; j < pd.numVertices(); ++j)
                    {
                        if(pd.edgeExists(i, j))
                        {
                            PlannerDataEdgeControlData edgeData;
                            edgeData.e_ = &pd.getEdge(i, j);
                            edgeData.endpoints_.first = i;
                            edgeData.endpoints_.second = j;
                            base::Cost weight;
                            pd.getEdgeWeight(i, j, &weight);
                            edgeData.weight_ = weight.v;

                            space->serialize(&ctrl[0], static_cast<const PlannerDataEdgeControl*>(edgeData.e_)->getControl());
                            edgeData.control_ = ctrl;

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

/* Author: Ryan Luna */

#ifndef OMPL_CONTROL_PLANNERS_EST_EST_
#define OMPL_CONTROL_PLANNERS_EST_EST_

#include "ompl/datastructures/Grid.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/PDF.h"
#include <boost/unordered_map.hpp>
#include <vector>

namespace ompl
{

    namespace control
    {

        /**
           @anchor cEST
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
            EST(const SpaceInformationPtr &si);

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

            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:

                Motion() : state(NULL), control(NULL), steps(0), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The control contained by the motion */
                Control           *control;

                /** \brief The number of steps the control is applied for */
                unsigned int       steps;

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

            /** \brief Directed control sampler */
            DirectedControlSamplerPtr    controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation       *siC_;

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

#ifndef OMPL_CONTROL_PLANNERS_KPIECE_KPIECE1_
#define OMPL_CONTROL_PLANNERS_KPIECE_KPIECE1_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/GridB.h"
#include <vector>
#include <set>

namespace ompl
{

    namespace control
    {

        /**
           @anchor cKPIECE1
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
           This implementation is intended for systems with differential constraints.
           @par External documentation
           I.A. Şucan and L.E. Kavraki, Kinodynamic motion planning by interior-exterior cell exploration,
           in <em>Workshop on the Algorithmic Foundations of Robotics</em>, Dec. 2008.<br>
           <a href="http://ioan.sucan.ro/files/pubs/wafr2008.pdf">[PDF]</a>
        */

        /** \brief Kinodynamic Planning by Interior-Exterior Cell Exploration */
        class KPIECE1 : public base::Planner
        {
        public:

            /** \brief Constructor */
            KPIECE1(const SpaceInformationPtr &si);

            virtual ~KPIECE1();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            /** In the process of randomly selecting states in the state
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

            /** Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). This is the minimum fraction
                used to select cells that are exterior (minimum
                because if 95% of cells are on the border, they will
                be selected with 95% chance, even if this fraction is
                set to 90%)*/
            void setBorderFraction(double bp)
            {
                selectBorderFraction_ = bp;
            }

            /** \brief Get the fraction of time to focus exploration
                on boundary */
            double getBorderFraction() const
            {
                return selectBorderFraction_;
            }

            /** \brief When extending a motion from a cell, the
                extension can be successful or it can fail.  If the
                extension is successful, the score of the cell is
                multiplied by \e good. If the extension fails, the
                score of the cell is multiplied by \e bad. These
                numbers should be in the range (0, 1]. */
            void setCellScoreFactor(double good, double bad)
            {
                setGoodCellScoreFactor(good);
                setBadCellScoreFactor(bad);
            }

            /** \brief Set the factor that is to be applied to a cell's score when an expansion from that cell fails */
            void setBadCellScoreFactor(double bad)
            {
                badScoreFactor_ = bad;
            }

            /** \brief Set the factor that is to be applied to a cell's score when an expansion from that cell succeedes */
            void setGoodCellScoreFactor(double good)
            {
                goodScoreFactor_ = good;
            }

            /** \brief Get the factor that is multiplied to a cell's
                score if extending a motion from that cell succeeded. */
            double getGoodCellScoreFactor() const
            {
                return goodScoreFactor_;
            }

            /** \brief Get the factor that is multiplied to a cell's
                score if extending a motion from that cell failed. */
            double getBadCellScoreFactor() const
            {
                return badScoreFactor_;
            }

            /** \brief When motions reach close to the goal, they are stored in a separate queue
                to allow biasing towards the goal. This function sets the maximum size of that queue. */
            void setMaxCloseSamplesCount(unsigned int nCloseSamples)
            {
                nCloseSamples_ = nCloseSamples;
            }

            /** \brief Get the maximum number of samples to store in the queue of samples that are close to the goal */
            unsigned int getMaxCloseSamplesCount() const
            {
                return nCloseSamples_;
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
            struct Motion
            {
                Motion() : state(NULL), control(NULL), steps(0), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by this motion */
                base::State       *state;

                /** \brief The control contained by this motion */
                Control           *control;

                /** \brief The number of steps the control is applied for */
                unsigned int       steps;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;
            };

            /** \brief The data held by a cell in the grid of motions */
            struct CellData
            {
                CellData() : coverage(0.0), selections(1), score(1.0), iteration(0), importance(0.0)
                {
                }

                ~CellData()
                {
                }

                /** \brief The set of motions contained in this grid cell */
                std::vector<Motion*> motions;

                /** \brief A measure of coverage for this cell. For
                    this implementation, this is the sum of motion
                    durations */
                double               coverage;

                /** \brief The number of times this cell has been
                    selected for expansion */
                unsigned int         selections;

                /** \brief A heuristic score computed based on
                    distance to goal (if available), successes and
                    failures at expanding from this cell. */
                double               score;

                /** \brief The iteration at which this cell was created */
                unsigned int         iteration;

                /** \brief The computed importance (based on other class members) */
                double               importance;
            };

            /** \brief Definintion of an operator passed to the Grid
                structure, to order cells by importance */
            struct OrderCellsByImportance
            {
                bool operator()(const CellData * const a, const CellData * const b) const
                {
                    return a->importance > b->importance;
                }
            };

            /** \brief The datatype for the maintained grid datastructure */
            typedef GridB<CellData*, OrderCellsByImportance> Grid;

            /** \brief Information about a known good sample (closer to the goal than others) */
            struct CloseSample
            {
                /** \brief Constructor fully initializes the content of this structure */
                CloseSample(Grid::Cell *c, Motion *m, double d) : cell(c), motion(m), distance(d)
                {
                }

                /** \brief The cell of the motion that is close to the goal */
                Grid::Cell *cell;

                /** \brief The motion that is close to the goal */
                Motion     *motion;

                /** \brief The distance to the goal. This value is increased over time, as the number of selections for this sample increases */
                double      distance;

                /** \brief Sort samples in accordance to their distance to the goal */
                bool operator<(const CloseSample &other) const
                {
                    return distance < other.distance;
                }
            };

            /** \brief Bounded set of good samples */
            struct CloseSamples
            {
                /** \brief Construct an object to maintain a set of at most \e size samples */
                CloseSamples(unsigned int size) : maxSize(size)
                {
                }

                /** \brief Evaluate whether motion \e motion, part of
                    cell \e cell is good enough to be part of the set
                    of samples closest to the goal, given its distance
                    to the goal is \e distance. If so, add it to the
                    set and return true. Otherwise, return false.*/
                bool consider(Grid::Cell *cell, Motion *motion, double distance);

                /** \brief Select the top sample (closest to the goal)
                    and update its position in the set subsequently
                    (pretend the distance to the goal is
                    larger). Returns true if the sample selection is
                    successful. */
                bool selectMotion(Motion* &smotion, Grid::Cell* &scell);

                /** \brief Return true if samples can be selected from this set */
                bool canSample() const
                {
                    return samples.size() > 0;
                }

                /** \brief Maximum number of samples to maintain */
                unsigned int          maxSize;

                /** \brief The maintained samples */
                std::set<CloseSample> samples;
            };


            /** \brief The data defining a tree of motions for this algorithm */
            struct TreeData
            {
                TreeData() : grid(0), size(0), iteration(1)
                {
                }

                /** \brief A grid containing motions, imposed on a
                    projection of the state space */
                Grid         grid;

                /** \brief The total number of motions (there can be
                    multiple per cell) in the grid */
                unsigned int size;

                /** \brief The number of iterations performed on this tree */
                unsigned int iteration;
            };

            /** \brief This function is provided as a calback to the
                grid datastructure to update the importance of a
                cell */
            static void computeImportance(Grid::Cell *cell, void*)
            {
                CellData &cd = *(cell->data);
                cd.importance =  cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections);
            }

            /** \brief Free all the memory allocated by this planner */
            void freeMemory();

            /** \brief Free the memory for the motions contained in a grid */
            void freeGridMotions(Grid &grid);

            /** \brief Free the memory for the data contained in a grid cell */
            void freeCellData(CellData *cdata);

            /** \brief Free the memory for a motion */
            void freeMotion(Motion *motion);

            /** \brief Add a motion to the grid containing motions. As
                a hint, \e dist specifies the distance to the goal
                from the state of the motion being added. The function
                Returns the number of cells created to accommodate the
                new motion (0 or 1). */
                Grid::Cell* addMotion(Motion *motion, double dist);

            /** \brief Select a motion and the cell it is part of from
                the grid of motions. This is where preference is given
                to cells on the boundary of the grid.*/
            bool selectMotion(Motion* &smotion, Grid::Cell* &scell);

            /** \brief When generated motions are to be added to the tree of motions, they often need to be split, so they don't cross cell boundaries.
                Given that a motion starts out in the cell \e origin and it crosses the cells in \e coords[\e index] through \e coords[\e last] (inclusively),
                return the index of the state to be used in the next part of the motion (that is within a cell). This will be a value between \e index and \e last. */
            unsigned int findNextMotion(const std::vector<Grid::Coord> &coords, unsigned int index, unsigned int count);

            /** \brief A control sampler */
            ControlSamplerPtr             controlSampler_;

            /** \brief The tree datastructure */
            TreeData                      tree_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation       *siC_;

            /** \brief This algorithm uses a discretization (a grid)
                to guide the exploration. The exploration is imposed
                on a projection of the state space. */
            base::ProjectionEvaluatorPtr  projectionEvaluator_;

            /** \brief When extending a motion from a cell, the
                extension can be successful. If it is, the score of the
                cell is multiplied by this factor. */
            double                        goodScoreFactor_;

            /** \brief When extending a motion from a cell, the
                extension can fail. If it is, the score of the cell is
                multiplied by this factor. */
            double                        badScoreFactor_;

            /** \brief When motions reach close to the goal, they are stored in a separate queue
                to allow biasing towards the goal. This variable specifies the maximum number of samples
                to keep in that queue. */
            unsigned int                  nCloseSamples_;

            /** \brief The fraction of time to focus exploration on
                the border of the grid. */
            double                        selectBorderFraction_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                        goalBias_;

            /** \brief The random number generator */
            RNG                           rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                       *lastGoalMotion_;
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


#ifndef OMPL_CONTROL_PLANNERS_PDST_PDST_
#define OMPL_CONTROL_PLANNERS_PDST_PDST_

#include "ompl/base/Planner.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/control/PathControl.h"
#include "ompl/control/PlannerData.h"
#include "ompl/datastructures/BinaryHeap.h"


namespace ompl
{

    namespace control
    {

         /**
            @anchor cPDST
            @par Short description
            PDST is a tree-based motion planner that attempts to detect
            the less explored area of the space through the use of a
            binary space partition of a projection of the state space.
            Exploration is biased towards large cells with few path
            segments. Unlike most tree-based planners which expand from
            a randomly select endpoint of a path segment, PDST expands
            from a randomly selected point along a deterministically
            selected path segment. Because of this, it is recommended
            to increase the min. and max. control duration using
            ompl::control::SpaceInformation::setMinMaxControlDuration.
            It is important to set the projection
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

            PDST(const SpaceInformationPtr &si);

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
                bool operator() (Motion *p1, Motion *p2) const
                {
                    // lowest priority means highest score
                    return p1->score() < p2->score();
                }
            };

            /// Class representing the tree of motions exploring the state space
            struct Motion
            {
            public:
                Motion(base::State *startState, base::State *endState, Control *control,
                    unsigned int controlDuration, double priority, Motion *parent)
                    : startState_(startState), endState_(endState), control_(control),
                    controlDuration_(controlDuration), priority_(priority), parent_(parent),
                    cell_(NULL), heapElement_(NULL), isSplit_(false)
                {
                }
                /// constructor for start states
                Motion(base::State *state)
                    : startState_(state), endState_(state), control_(NULL), controlDuration_(0),
                    priority_(0.), parent_(NULL), cell_(NULL), heapElement_(NULL), isSplit_(false)
                {
                }
                /// The score is used to order motions in a priority queue.
                double score() const
                {
                    return priority_ / cell_->volume_;
                }
                void updatePriority()
                {
                    priority_ = priority_ * 2. + 1.;
                }

                /// The starting point of this motion
                base::State                     *startState_;
                /// The state reached by this motion
                base::State                     *endState_;
                /// The control that was applied to arrive at this state from the parent
                control::Control                *control_;
                /// The duration that the control was applied to arrive at this state from the parent
                unsigned int                     controlDuration_;
                /// Priority for selecting this path to extend from in the future
                double                           priority_;
                /// Parent motion from which this one started
                Motion *parent_;
                /// Pointer to the cell that contains this path
                Cell*                            cell_;
                /// Handle to the element of the priority queue for this Motion
                BinaryHeap<Motion*, MotionCompare>::Element *heapElement_;
                /// Whether this motion is the result of a split operation, in which case
                /// its endState_ and control_ should not be freed.
                bool                             isSplit_;
            };

            /// Cell is a Binary Space Partition
            struct Cell
            {
                Cell(double volume, const base::RealVectorBounds &bounds,
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
                Cell* stab(const base::EuclideanProjection& projection) const
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
                base::RealVectorBounds       bounds_;
                /// The motions contained in this cell. Motions are stored only in leaf nodes.
                std::vector<Motion*>         motions_;
            };


            /// \brief Inserts the motion into the appropriate cells, splitting the motion as necessary.
            /// \e motion is assumed to be fully contained within \e cell.
            void addMotion(Motion *motion, Cell *cell,
                base::State*, base::State*, base::EuclideanProjection&, base::EuclideanProjection&);
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
            /// \brief Find the max. duration that the control_ in motion can be applied s.t.
            /// the trajectory passes through state. This means that "ancestor" motions with
            /// the same control_ are also considered. A pointer to the oldest ancestor with
            /// the same control_ is returned. Upon return applying the control
            /// \e ancestor->control_ for duration steps starting from the state
            /// \e ancestor->startState_ should result in the state \e state.
            unsigned int findDurationAndAncestor(Motion *motion, base::State *state,
                base::State *scratch, Motion*& ancestor) const;

            void freeMemory();

            /// State sampler
            base::StateSamplerPtr                    sampler_;
            /// Directed control sampler
            DirectedControlSamplerPtr                controlSampler_;
            /// SpaceInformation convenience pointer
            const SpaceInformation                  *siC_;
            // Random number generator
            RNG                                      rng_;
            /// \brief Vector holding all of the start states supplied for the problem
            /// Each start motion is the root of its own tree of motions.
            std::vector<Motion*>                     startMotions_;
            /// Priority queue of motions
            BinaryHeap<Motion*, MotionCompare>       priorityQueue_;
            /// Binary Space Partition
            Cell*                                    bsp_;
            /// Projection evaluator for the problem
            base::ProjectionEvaluatorPtr             projectionEvaluator_;
            /// Number between 0 and 1 specifying the probability with which the goal should be sampled
            double                                   goalBias_;
            /// Objected used to sample the goal
            base::GoalSampleableRegion              *goalSampler_;
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

#ifndef OMPL_CONTROL_PLANNERS_RRT_RRT_
#define OMPL_CONTROL_PLANNERS_RRT_RRT_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{

    namespace control
    {

        /**
           @anchor cRRT
           @par Short description
           RRT is a tree-based motion planner that uses the following
           idea: RRT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           This implementation is intended for systems with differential constraints.
           @par External documentation
           S.M. LaValle and J.J. Kuffner, Randomized kinodynamic planning, <em>Intl. J. of Robotics Research</em>, vol. 20, pp. 378–400, May 2001. DOI: <a href="http://dx.doi.org/10.1177/02783640122067453">10.1177/02783640122067453</a><br>
           <a href="http://ijr.sagepub.com/content/20/5/378.full.pdf">[PDF]</a>
           <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">[more]</a>
        */

        /** \brief Rapidly-exploring Random Tree */
        class RRT : public base::Planner
        {
        public:

            /** \brief Constructor */
            RRT(const SpaceInformationPtr &si);

            virtual ~RRT();

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            virtual void clear();

            /** In the process of randomly selecting states in the state
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

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            virtual void getPlannerData(base::PlannerData &data) const;

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

                Motion() : state(NULL), control(NULL), steps(0), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The control contained by the motion */
                Control           *control;

                /** \brief The number of steps the control is applied for */
                unsigned int       steps;

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

            /** \brief Control sampler */
            DirectedControlSamplerPtr                      controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation                        *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool                                           addIntermediateStates_;

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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_DECOMPOSITION_
#define OMPL_CONTROL_PLANNERS_SYCLOP_DECOMPOSITION_

#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/State.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/RandomNumbers.h"

namespace ompl
{
    namespace control
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::Decomposition */
        OMPL_CLASS_FORWARD(Decomposition);
        /// @endcond

        /** \class ompl::control::DecompositionPtr
            \brief A boost shared pointer wrapper for ompl::control::Decomposition */

        /** \brief A Decomposition is a partition of a bounded Euclidean space into a fixed number of regions which are denoted by integers. */
        class Decomposition
        {
        public:

            /** \brief Constructor. Creates a Decomposition with a given dimension
                and a given set of bounds. Accepts as an optional argument a given
                number of regions. */
            Decomposition(int dim, const base::RealVectorBounds &b) : dimension_(dim), bounds_(b)
            {
                if (dim > static_cast<int>(b.low.size()))
                    throw Exception("Decomposition", "argument 'dim' exceeds dimension of given bounds");
                else if (dim < static_cast<int>(b.low.size()))
                    OMPL_WARN("Decomposition: dimension of given bounds exceeds argument 'dim'. Using the first 'dim' values of bounds");
            }

            virtual ~Decomposition()
            {
            }

            /** \brief Returns the number of regions in this Decomposition. */
            virtual int getNumRegions() const = 0;

            /** \brief Returns the dimension of this Decomposition. */
            virtual int getDimension() const
            {
                return dimension_;
            }

            /** \brief Returns the bounds of this Decomposition. */
            virtual const base::RealVectorBounds& getBounds() const
            {
                return bounds_;
            }

            /** \brief Returns the volume of a given region in this Decomposition. */
            virtual double getRegionVolume(int rid) = 0;

            /** \brief Returns the index of the region containing a given State.
             * Most often, this is obtained by first calling project().
             * Returns -1 if no region contains the State. */
            virtual int locateRegion(const base::State *s) const = 0;

            /** \brief Project a given State to a set of coordinates in R^k, where k is the dimension of this Decomposition. */
            virtual void project(const base::State *s, std::vector<double>& coord) const = 0;

            /** \brief Stores a given region's neighbors into a given vector. */
            virtual void getNeighbors(int rid, std::vector<int>& neighbors) const = 0;

            /** \brief Samples a projected coordinate from a given region. */
            virtual void sampleFromRegion(int rid, RNG &rng, std::vector<double>& coord) const = 0;

            /** \brief Samples a State using a projected coordinate and a StateSampler. */
            virtual void sampleFullState(const base::StateSamplerPtr& sampler, const std::vector<double>& coord, base::State *s) const = 0;

        protected:
            int dimension_;
            base::RealVectorBounds bounds_;
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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_GRIDDECOMPOSITION_
#define OMPL_CONTROL_PLANNERS_SYCLOP_GRIDDECOMPOSITION_

#include <cstdlib>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/util/RandomNumbers.h"

namespace ompl
{
    namespace control
    {
        /** \brief A GridDecomposition is a Decomposition implemented using a grid. */
        class GridDecomposition : public Decomposition
        {
        public:
            /** \brief Constructor. Creates a GridDecomposition as a hypercube with a given dimension, side length, and bounds.
                The cells of the hypercube are referenced by integer coordinates of the form
                \f$(r_1,\ldots,r_k)\f$, where \f$ 0 \leq r_i < \texttt{len}\f$. */
            GridDecomposition(int len, int dim, const base::RealVectorBounds &b);

            virtual ~GridDecomposition()
            {
            }

            virtual int getNumRegions(void) const { return numGridCells_; }

            virtual double getRegionVolume(int /*rid*/)
            {
                return cellVolume_;
            }

            virtual void getNeighbors(int rid, std::vector<int>& neighbors) const;

            virtual int locateRegion(const base::State *s) const;

            virtual void sampleFromRegion(int rid, RNG &rng, std::vector<double>& coord) const;

        protected:
            /** \brief Helper method to return the bounds of a given region. */
            virtual const base::RealVectorBounds& getRegionBounds(int rid) const;

            /** \brief Converts a given region to a coordinate in the grid. */
            void regionToGridCoord(int rid, std::vector<int>& coord) const;

            /** \brief Converts the given grid coordinate to its corresponding region ID. */
            int gridCoordToRegion (const std::vector<int> &coord) const;

            /** \brief Converts a decomposition space coordinate to the ID of the region that contains iit. */
            int coordToRegion(const std::vector<double>& coord) const;

            /** \brief Converts a decomposition space coordinate to a grid coordinate. */
            void coordToGridCoord(const std::vector<double>& coord, std::vector<int>& gridCoord) const;

            /** \brief Computes the neighbors of the given region in a n-dimensional grid */
            void computeGridNeighbors (int rid, std::vector <int> &neighbors) const;

            /** Recursive subroutine for grid neighbor computation */
            void computeGridNeighborsSub (const std::vector<int>& coord, std::vector<int>& neighbors,
                                          int dim, std::vector<int>& candidate) const;

            int length_;
            double cellVolume_;
            mutable boost::unordered_map<int, boost::shared_ptr<base::RealVectorBounds> > regToBounds_;

        private:
            const int numGridCells_;
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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOP_
#define OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOP_

#include <boost/graph/astar_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/unordered_map.hpp>
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/datastructures/PDF.h"
#include <map>
#include <vector>

namespace ompl
{
    namespace control
    {
        /**
           @anchor cSyclop
           @par Short description
           Syclop is a multi-layered planner that guides a low-level sampling-based tree planner
           through a sequence of sequence of discrete workspace regions from start to goal.
           Syclop is defined as an abstract base class whose pure virtual methods are defined
           by the chosen low-level sampling-based tree planner.
           @par External documentation
           E. Plaku, L.E. Kavraki, and M.Y. Vardi,
           Motion Planning with Dynamics by a Synergistic Combination of Layers of Planning,
           in <em>IEEE Transactions on Robotics</em>, 2010.
           DOI: <a href="http://dx.doi.org/10.1109/TRO.2010.2047820">10.1109/TRO.2010.2047820</a><br>
        */

        /** \brief Synergistic Combination of Layers of Planning. */
        class Syclop : public base::Planner
        {
        public:
            /** \brief Each edge weight between two adjacent regions in the Decomposition is defined
                as a product of edge cost factors. By default, given adjacent regions \f$r\f$ and \f$s\f$, Syclop uses the sole edge cost factor
                \f[
                    \frac{1 + \mbox{sel}^2(r,s)}{1 + \mbox{conn}^2(r,s)} \alpha(r) \alpha(s),
                \f]
                where for any region \f$t\f$,
                \f[
                    \alpha(t) = \frac{1}{\left(1 + \mbox{cov}(t)\right) \mbox{freeVol}^4(t)},
                \f]
                \f$\mbox{sel}(r,s)\f$ is the number of times \f$r\f$ and \f$s\f$ have been part of a lead or selected for exploration,
                \f$\mbox{conn}(r,s)\f$ estimates the progress made by the low-level planner in extending the tree from \f$r\f$ to \f$s\f$,
                \f$\mbox{cov}(t)\f$ estimates the tree coverage of the region \f$t\f$, and \f$\mbox{freeVol}(t)\f$ estimates the free volume
                of \f$t\f$.
                Additional edge cost factors can be added
                with the addEdgeCostFactor() function, and Syclop's list of edge cost factors can be cleared using clearEdgeCostFactors() . */
            typedef boost::function<double(int, int)> EdgeCostFactorFn;

            /** \brief Leads should consist of a path of adjacent regions in the decomposition that start with the start region and end at the end region.  Default is \f$A^\ast\f$ search. */
            typedef boost::function<void(int, int, std::vector<int>&)> LeadComputeFn;

            /** \brief Constructor. Requires a Decomposition, which Syclop uses to create high-level leads. */
            Syclop(const SpaceInformationPtr& si, const DecompositionPtr &d, const std::string& plannerName) : ompl::base::Planner(si, plannerName),
                numFreeVolSamples_(Defaults::NUM_FREEVOL_SAMPLES),
                probShortestPath_(Defaults::PROB_SHORTEST_PATH),
                probKeepAddingToAvail_(Defaults::PROB_KEEP_ADDING_TO_AVAIL),
                numRegionExpansions_(Defaults::NUM_REGION_EXPANSIONS),
                numTreeSelections_(Defaults::NUM_TREE_SELECTIONS),
                probAbandonLeadEarly_(Defaults::PROB_ABANDON_LEAD_EARLY),
                siC_(si.get()),
                decomp_(d),
                covGrid_(Defaults::COVGRID_LENGTH, decomp_),
                graphReady_(false),
                numMotions_(0)
            {
                specs_.approximateSolutions = true;

                Planner::declareParam<int>   ("free_volume_samples", this, &Syclop::setNumFreeVolumeSamples, &Syclop::getNumFreeVolumeSamples);
                Planner::declareParam<int>   ("num_region_expansions", this, &Syclop::setNumRegionExpansions, &Syclop::getNumRegionExpansions);
                Planner::declareParam<int>   ("num_tree_expansions", this, &Syclop::setNumTreeExpansions, &Syclop::getNumTreeExpansions);
                Planner::declareParam<double>("prob_abandon_lead_early", this, &Syclop::setProbAbandonLeadEarly, &Syclop::getProbAbandonLeadEarly);
                Planner::declareParam<double>("prob_add_available_regions", this, &Syclop::setProbAddingToAvailableRegions, &Syclop::getProbAddingToAvailableRegions);
                Planner::declareParam<double>("prob_shortest_path_lead", this, &Syclop::setProbShortestPathLead, &Syclop::getProbShortestPathLead);
            }

            virtual ~Syclop()
            {
            }

            /// @name ompl::base::Planner Interface
            /// @{

            virtual void setup();

            virtual void clear();

            /** \brief Continues solving until a solution is found or a given planner termination condition is met.
                Returns true if solution was found. */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
            /// @}

            /// @name Tunable parameters
            /// @{

            /** \brief Allows the user to override the lead computation function. */
            void setLeadComputeFn(const LeadComputeFn& compute);

            /** \brief Adds an edge cost factor to be used for edge weights between adjacent regions. */
            void addEdgeCostFactor(const EdgeCostFactorFn& factor);

            /** \brief Clears all edge cost factors, making all edge weights equivalent to 1. */
            void clearEdgeCostFactors();

            /// \brief Get the number of states to sample when estimating free volume in the Decomposition.
            int getNumFreeVolumeSamples () const
            {
                return numFreeVolSamples_;
            }

            /// \brief Set the number of states to sample when estimating free
            ///  volume in the Decomposition.
            void setNumFreeVolumeSamples (int numSamples)
            {
                numFreeVolSamples_ = numSamples;
            }

            /// \brief Get the probability [0,1] that a lead will be computed as
            ///  a shortest-path instead of a random-DFS.
            double getProbShortestPathLead () const
            {
                return probShortestPath_;
            }

            /// \brief Set the probability [0,1] that a lead will be computed as
            ///  a shortest-path instead of a random-DFS.
            void setProbShortestPathLead (double probability)
            {
                probShortestPath_ = probability;
            }

            /// \brief Get the probability [0,1] that the set of available
            ///  regions will be augmented.
            double getProbAddingToAvailableRegions () const
            {
                return probKeepAddingToAvail_;
            }

            /// \brief Set the probability [0,1] that the set of available
            ///  regions will be augmented.
            void setProbAddingToAvailableRegions (double probability)
            {
                probKeepAddingToAvail_ = probability;
            }

            /// \brief Get the number of times a new region will be chosen and
            ///  promoted for expansion from a given lead.
            int getNumRegionExpansions () const
            {
                return numRegionExpansions_;
            }

            /// \brief Set the number of times a new region will be chosen and
            ///  promoted for expansion from a given lead.
            void setNumRegionExpansions (int regionExpansions)
            {
                numRegionExpansions_ = regionExpansions;
            }

            /// \brief Get the number of calls to selectAndExtend() in the
            ///  low-level tree planner for a given lead and region.
            int getNumTreeExpansions () const
            {
                return numTreeSelections_;
            }

            /// \brief Set the number of calls to selectAndExtend() in the
            ///  low-level tree planner for a given lead and region.
            void setNumTreeExpansions (int treeExpansions)
            {
                numTreeSelections_ = treeExpansions;
            }

            /// \brief Get the probability [0,1] that a lead will be abandoned
            ///  early, before a new region is chosen for expansion.
            double getProbAbandonLeadEarly () const
            {
                return probAbandonLeadEarly_;
            }

            /// \brief The probability that a lead will be abandoned early,
            ///  before a new region is chosen for expansion.
            void setProbAbandonLeadEarly (double probability)
            {
                probAbandonLeadEarly_ = probability;
            }
            /// @}

            /** \brief Contains default values for Syclop parameters. */
            struct Defaults
            {
                static const int    NUM_FREEVOL_SAMPLES         = 100000;
                static const int    COVGRID_LENGTH              = 128;
                static const int    NUM_REGION_EXPANSIONS       = 100;
                static const int    NUM_TREE_SELECTIONS         = 1;
                // C++ standard prohibits non-integral static const member initialization
                // These constants are set in Syclop.cpp.  C++11 standard changes this
                // with the constexpr keyword, but for compatibility this is not done.
                static const double PROB_ABANDON_LEAD_EARLY     /*= 0.25*/;
                static const double PROB_KEEP_ADDING_TO_AVAIL   /*= 0.50*/;
                static const double PROB_SHORTEST_PATH          /*= 0.95*/;
            };

        protected:

            #pragma pack(push, 4)  // push default byte alignment to stack and align the following structure to 4 byte boundary
            /** \brief Representation of a motion

                A motion contains pointers to its state, its parent motion, and the control
                that was applied to get from its parent to its state. */
            class Motion
            {
            public:
                Motion() : state(NULL), control(NULL), parent(NULL), steps(0)
                {
                }
                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()), parent(NULL), steps(0)
                {
                }
                virtual ~Motion()
                {
                }
                /** \brief The state contained by the motion */
                base::State *state;
                /** \brief The control contained by the motion */
                Control *control;
                /** \brief The parent motion in the tree */
                const Motion *parent;
                /** \brief The number of steps for which the control is applied */
                unsigned int steps;
            };
            #pragma pack (pop)  // Restoring default byte alignment

            #pragma pack(push, 4) // push default byte alignment to stack and align the following structure to 4 byte boundary
            /** \brief Representation of a region in the Decomposition assigned to Syclop. */
            class Region
            {
            public:
                Region()
                {
                }
                virtual ~Region()
                {
                }
                /** \brief Clears motions and coverage information from this region. */
                void clear()
                {
                    motions.clear();
                    covGridCells.clear();
                    pdfElem = NULL;
                }

                /** \brief The cells of the underlying coverage grid that contain tree motions from this region */
                std::set<int> covGridCells;
                /** \brief The tree motions contained in this region */
                std::vector<Motion*> motions;
                /** \brief The volume of this region */
                double volume;
                /** \brief The free volume of this region */
                double freeVolume;
                /** \brief The percent of free volume of this region */
                double percentValidCells;
                /** \brief The probabilistic weight of this region, used when sampling from PDF */
                double weight;
                /** \brief The coefficient contributed by this region to edge weights in lead computations */
                double alpha;
                /** \brief The index of the graph node corresponding to this region */
                int index;
                /** \brief The number of times this region has been selected for expansion */
                unsigned int numSelections;
                /** \brief The Element corresponding to this region in the PDF of available regions. */
                PDF<int>::Element *pdfElem;
            };
            #pragma pack (pop)  // Restoring default byte alignment

            #pragma pack(push, 4)  // push default byte alignment to stack and align the following structure to 4 byte boundary
            /** \brief Representation of an adjacency (a directed edge) between two regions
                in the Decomposition assigned to Syclop. */
            class Adjacency
            {
            public:
                Adjacency()
                {
                }
                virtual ~Adjacency()
                {
                }
                /** \brief Clears coverage information from this adjacency. */
                void clear()
                {
                    covGridCells.clear();
                }
                /** \brief The cells of the underlying coverage grid that contain tree motions originating from
                    direct connections along this adjacency */
                std::set<int> covGridCells;
                /** \brief The source region of this adjacency edge */
                const Region *source;
                /** \brief The target region of this adjacency edge */
                const Region *target;
                /** \brief The cost of this adjacency edge, used in lead computations */
                double cost;
                /** \brief The number of times this adjacency has been included in a lead */
                int numLeadInclusions;
                /** \brief The number of times the low-level tree planner has selected motions from the source region
                    when attempting to extend the tree toward the target region. */
                int numSelections;
                /** \brief This value is true if and only if this adjacency's source and target regions both contain zero tree motions. */
                bool empty;
            };
            #pragma pack (pop) // Restoring default byte alignment

            /** \brief Add State s as a new root in the low-level tree, and return the Motion corresponding to s. */
            virtual Motion* addRoot(const base::State *s) = 0;

            /** \brief Select a Motion from the given Region, and extend the tree from the Motion.
                Add any new motions created to newMotions. */
            virtual void selectAndExtend(Region &region, std::vector<Motion*>& newMotions) = 0;

            /** \brief Returns a reference to the Region object with the given index. Assumes the index is valid. */
            inline const Region& getRegionFromIndex(const int rid) const
            {
                return graph_[boost::vertex(rid,graph_)];
            }

            /** \brief The number of states to sample to estimate free volume in the Decomposition. */
            int numFreeVolSamples_;

            /** \brief The probability that a lead will be computed as a shortest-path instead of a random-DFS. */
            double probShortestPath_;

            /** \brief The probability that the set of available regions will be augmented. */
            double probKeepAddingToAvail_;

            /** \brief The number of times a new region will be chosen and promoted for expansion from a given lead. */
            int numRegionExpansions_;

            /** \brief The number of calls to selectAndExtend() in the low-level tree planner for a given lead and region. */
            int numTreeSelections_;

            /** \brief The probability that a lead will be abandoned early, before a new region is chosen for expansion. */
            double probAbandonLeadEarly_;

            /** \brief Handle to the control::SpaceInformation object */
            const SpaceInformation *siC_;

            /** \brief The high level decomposition used to focus tree expansion */
            DecompositionPtr decomp_;

            /** \brief Random number generator */
            RNG rng_;

        private:
            /** \brief Syclop uses a CoverageGrid to estimate coverage in its assigned Decomposition.
                The CoverageGrid should have finer resolution than the Decomposition. */
            class CoverageGrid : public GridDecomposition
            {
            public:
                CoverageGrid(const int len, const DecompositionPtr& d) : GridDecomposition(len, d->getDimension(), d->getBounds()), decomp(d)
                {
                }

                virtual ~CoverageGrid()
                {
                }

                /** \brief Since the CoverageGrid is defined in the same space as the Decomposition,
                    it uses the Decomposition's projection function. */
                virtual void project(const base::State *s, std::vector<double>& coord) const
                {
                    decomp->project(s, coord);
                }

                /** \brief Syclop will not sample from the CoverageGrid. */
                virtual void sampleFullState(const base::StateSamplerPtr& /*sampler*/, const std::vector<double>& /*coord*/, base::State* /*s*/) const
                {
                }

            protected:
                const DecompositionPtr& decomp;
            };

            typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Region, Adjacency> RegionGraph;
            typedef boost::graph_traits<RegionGraph>::vertex_descriptor Vertex;
            typedef boost::graph_traits<RegionGraph>::vertex_iterator VertexIter;
            typedef boost::property_map<RegionGraph, boost::vertex_index_t>::type VertexIndexMap;
            typedef boost::graph_traits<RegionGraph>::edge_iterator EdgeIter;

            /// @cond IGNORE
            friend class DecompositionHeuristic;

            class DecompositionHeuristic : public boost::astar_heuristic<RegionGraph, double>
            {
            public:
                DecompositionHeuristic(const Syclop *s, const Region &goal) : syclop(s), goalRegion(goal)
                {
                }

                double operator()(Vertex v)
                {
                    const Region &region = syclop->getRegionFromIndex(v);
                    return region.alpha*goalRegion.alpha;
                }
            private:
                const Syclop *syclop;
                const Region &goalRegion;
            };

            struct found_goal {};

            class GoalVisitor : public boost::default_astar_visitor
            {
            public:
                GoalVisitor(const int goal) : goalRegion(goal)
                {
                }
                void examine_vertex(Vertex v, const RegionGraph& /*g*/)
                {
                    if (static_cast<int>(v) == goalRegion)
                        throw found_goal();
                }
            private:
                const int goalRegion;
            };
            /// @endcond

            /// @cond IGNORE
            class RegionSet
            {
            public:
                int sampleUniform()
                {
                    if (empty())
                        return -1;
                    return regions.sample(rng.uniform01());
                }
                void insert(const int r)
                {
                    if (regToElem.count(r) == 0)
                        regToElem[r] = regions.add(r, 1);
                    else
                    {
                        PDF<int>::Element *elem = regToElem[r];
                        regions.update(elem, regions.getWeight(elem)+1);
                    }
                }
                void clear()
                {
                    regions.clear();
                    regToElem.clear();
                }
                std::size_t size() const
                {
                    return regions.size();
                }
                bool empty() const
                {
                    return regions.empty();
                }
            private:
                RNG rng;
                PDF<int> regions;
                boost::unordered_map<const int, PDF<int>::Element*> regToElem;
            };
            /// @endcond

            /** \brief Initializes default values for a given Region. */
            void initRegion(Region &r);

            /** \brief Computes volume estimates for a given Region. */
            void setupRegionEstimates();

            /** \brief Recomputes coverage and selection estimates for a given Region. */
            void updateRegion(Region &r);

            /** \brief Initializes a given Adjacency between a source Region and a destination Region. */
            void initEdge(Adjacency &a, const Region *source, const Region *target);

            /** \brief Initializes default values for each Adjacency. */
            void setupEdgeEstimates();

            /** \brief Updates the edge cost for a given Adjacency according to Syclop's list of edge cost factors. */
            void updateEdge(Adjacency &a);

            /** \brief Given that a State s has been added to the tree,
                update the coverage estimate (if needed) for its corresponding Region. */
            bool updateCoverageEstimate(Region &r, const base::State *s);

            /** \brief Given that an edge has been added to the tree of motions from a state in Region c to
                the State s in Region d, update the corresponding Adjacency's cost and connection estimates. */
            bool updateConnectionEstimate(const Region &c, const Region &d, const base::State *s);

            /** \brief Build a RegionGraph according to the Decomposition assigned to Syclop,
                creating Region and Adjacency objects for each node and edge. */
            void buildGraph();

            /** \brief Clear all Region and Adjacency objects in the graph. */
            void clearGraphDetails();

            /** \brief Select a Region in which to promote expansion of the low-level tree. */
            int selectRegion();

            /** \brief Compute the set of Regions available for selection. */
            void computeAvailableRegions();

            /** \brief Default lead computation. A lead is a sequence of adjacent Regions from start to goal in the Decomposition. */
            void defaultComputeLead(int startRegion, int goalRegion, std::vector<int>& lead);

            /** \brief Default edge cost factor, which is used by Syclop for edge weights between adjacent Regions. */
            double defaultEdgeCost(int r, int s);

            /** \brief Lead computaton boost::function object */
            LeadComputeFn leadComputeFn;
            /** \brief The current computed lead */
            std::vector<int> lead_;
            /** \brief Used to sample regions in which to promote expansion */
            PDF<int> availDist_;
            /** \brief Stores all factor functions used to compute adjacency edge cost for lead computation */
            std::vector<EdgeCostFactorFn> edgeCostFactors_;
            /** \brief An underlying grid used to estimate coverage */
            CoverageGrid covGrid_;
            /** \brief A graph structure whose nodes and edges correspond to regions and adjacencies in the given Decomposition */
            RegionGraph graph_;
            /** \brief This value stores whether the graph structure has been built */
            bool graphReady_;
            /** \brief Maps pairs of regions to adjacency objects */
            boost::unordered_map<std::pair<int,int>, Adjacency*> regionsToEdge_;
            /** \brief The total number of motions in the low-level tree */
            unsigned int numMotions_;
            /** \brief The set of all regions that contain start states */
            RegionSet startRegions_;
            /** \brief The set of all regions that contain goal states */
            RegionSet goalRegions_;
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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOPEST_
#define OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOPEST_

#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"

namespace ompl
{
    namespace control
    {
        /** \brief SyclopEST is Syclop with EST as its low-level tree planner.
            \anchor cSyclopEST
        */
        class SyclopEST : public Syclop
        {
        public:
            /** \brief Constructor. Requires a Decomposition, which Syclop uses to create high-level leads. */
            SyclopEST(const SpaceInformationPtr& si, const DecompositionPtr &d) : Syclop(si,d,"SyclopEST")
            {
            }

            virtual ~SyclopEST()
            {
                freeMemory();
            }

            virtual void setup();
            virtual void clear();
            virtual void getPlannerData(base::PlannerData &data) const;

        protected:
            virtual Syclop::Motion* addRoot(const base::State *s);
            virtual void selectAndExtend(Region &region, std::vector<Motion*> &newMotions);

            /** \brief Free the memory allocated by this planner. */
            void freeMemory();

            base::StateSamplerPtr sampler_;
            ControlSamplerPtr controlSampler_;
            std::vector<Motion*> motions_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_;
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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOPRRT_
#define OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOPRRT_

#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
    namespace control
    {
        /** \brief SyclopRRT is Syclop with RRT as its low-level tree planner.
            \anchor cSyclopRRT
        */
        class SyclopRRT : public Syclop
        {
        public:
            /** \brief Constructor. Requires a Decomposition, which Syclop uses to create high-level leads. */
            SyclopRRT(const SpaceInformationPtr& si, const DecompositionPtr &d) : Syclop(si,d,"SyclopRRT"), regionalNN_(false)
            {
            }

            virtual ~SyclopRRT()
            {
                freeMemory();
            }

            virtual void setup();
            virtual void clear();
            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief If regionalNearestNeighbors is enabled, then when computing the closest Motion to a generated state
                in a given Region, SyclopRRT will perform a linear search over the current Region and its neighbors instead of
                querying a NearestNeighbors datastructure over the whole tree.
                This approach is enabled by default, and should be disabled if there exist Regions of the Decomposition that
                will be extremely densely populated with states - in such cases, querying a global NearestNeighbors datastructure will
                probably be faster. */
            void setRegionalNearestNeighbors(bool enabled)
            {
                regionalNN_ = enabled;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors()
            {
                regionalNN_ = false;
                nn_.reset(new NN<Motion*>());
            }

        protected:
            virtual Syclop::Motion* addRoot(const base::State *s);
            virtual void selectAndExtend(Region &region, std::vector<Motion*> &newMotions);

            /** \brief Free the memory allocated by this planner. */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            base::StateSamplerPtr sampler_;
            DirectedControlSamplerPtr controlSampler_;
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;
            bool regionalNN_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_;
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

#ifndef PY_BINDINGS_OMPL_PY_CONTROL_
#define PY_BINDINGS_OMPL_PY_CONTROL_

#include "ompl/control/ODESolver.h"
#include "py_boost_function.hpp"


namespace ompl
{
    namespace control
    {
// Boost.OdeInt needs Boost version >= 1.44
#if BOOST_VERSION >= 104400
        inline int dummyODESolverSize()
        {
            return sizeof(ODEBasicSolver<>) + sizeof(ODEErrorSolver<>) + sizeof(ODEAdaptiveSolver<>);
        }
#endif
    }
}

#endif
