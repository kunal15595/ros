/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPLAPP_GEOMETRY_RIGID_BODY_GEOMETRY_
#define OMPLAPP_GEOMETRY_RIGID_BODY_GEOMETRY_

#include "omplapp/config.h"
#include "omplapp/geometry/GeometrySpecification.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <boost/shared_ptr.hpp>
#if OMPL_HAS_ASSIMP3
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#else
#include <assimp/aiScene.h>
#include <assimp/assimp.hpp>
#endif
#include <string>
#include <vector>

namespace ompl
{

    /** \brief Namespace containing code specific to OMPL.app */
    namespace app
    {
        /** \brief Enumeration of the possible collision checker types */
        enum CollisionChecker
            { PQP, FCL };

        class RigidBodyGeometry
        {
        public:

            /** \brief Constructor expects a state space that can represent a rigid body */
            /// \param mtype The motion model (2D or 3D) for the rigid body.
            /// \param ctype The type of collision checker to use for rigid body planning.
            explicit
            RigidBodyGeometry(MotionModel mtype, CollisionChecker ctype) : mtype_(mtype), factor_(1.0), add_(0.0), ctype_(ctype)
            {
            }

            /// \brief Constructor expects a state space that can represent a rigid body
            /// \param mtype The motion model (2D or 3D) for the rigid body.
            /// \remarks This constructor defaults to a PQP state validity checker
            explicit
            RigidBodyGeometry(MotionModel mtype) : mtype_(mtype), factor_(1.0), add_(0.0), ctype_(FCL)
            {
            }

            virtual ~RigidBodyGeometry(void)
            {
            }

            MotionModel getMotionModel(void) const
            {
                return mtype_;
            }

            CollisionChecker getCollisionCheckerType(void) const
            {
                return ctype_;
            }

            bool hasEnvironment(void) const
            {
                return !importerEnv_.empty();
            }

            bool hasRobot(void) const
            {
                return !importerRobot_.empty();
            }

            unsigned int getLoadedRobotCount(void) const
            {
                return importerRobot_.size();
            }

            /** \brief Get the robot's center (average of all the vertices of all its parts) */
            aiVector3D getRobotCenter(unsigned int robotIndex) const;

            /** \brief This function specifies the name of the CAD
                file representing the environment (\e
                env). Returns 1 on success, 0 on failure. */
            virtual bool setEnvironmentMesh(const std::string &env);

            /** \brief This function specifies the name of the CAD
                file representing a part of the environment (\e
                env). Returns 1 on success, 0 on failure. */
            virtual bool addEnvironmentMesh(const std::string &env);

             /** \brief This function specifies the name of the CAD
                 file representing the robot (\e robot). Returns 1 on success, 0 on failure. */
            virtual bool setRobotMesh(const std::string &robot);

             /** \brief This function specifies the name of the CAD
                file representing a part of the robot (\e robot). Returns 1 on success, 0 on failure. */
            virtual bool addRobotMesh(const std::string &robot);

            /** \brief Change the type of collision checking for the rigid body */
            virtual void setStateValidityCheckerType (CollisionChecker ctype);

            /** \brief Allocate default state validity checker using PQP. */
            const base::StateValidityCheckerPtr& allocStateValidityChecker(const base::SpaceInformationPtr &si, const GeometricStateExtractor &se, bool selfCollision);

            const GeometrySpecification& getGeometrySpecification(void) const;

            /** \brief The bounds of the environment are inferred
                based on the axis-aligned bounding box for the objects
                in the environment. The inferred size is multiplied by
                \e factor. By default \e factor = 1, */
            void setBoundsFactor(double factor)
            {
                factor_ = factor;
            }

            /** \brief Get the data set by setBoundsFactor() */
            double getBoundsFactor(void) const
            {
                return factor_;
            }

            /** \brief The bounds of the environment are inferred
                based on the axis-aligned bounding box for the objects
                in the environment. \e add is added to the inferred
                size. By default \e add = 0, */
            void setBoundsAddition(double add)
            {
                add_ = add;
            }

            /** \brief Get the data set by setBoundsAddition() */
            double getBoundsAddition(void) const
            {
                return add_;
            }

            /** \brief Given the representation of an environment,
                infer its bounds. The bounds will be 2-dimensional
                when planning in 2D and 3-dimensional when planning in
                3D. */
            base::RealVectorBounds inferEnvironmentBounds(void) const;

        protected:

            void computeGeometrySpecification(void);

            MotionModel         mtype_;

            /** \brief The factor to multiply inferred environment bounds by (default 1) */
            double              factor_;

            /** \brief The value to add to inferred environment bounds (default 0) */
            double              add_;

            /** \brief Instance of assimp importer used to load environment */
            std::vector< boost::shared_ptr<Assimp::Importer> > importerEnv_;

            /** \brief Instance of assimp importer used to load robot */
            std::vector< boost::shared_ptr<Assimp::Importer> > importerRobot_;

            /** \brief Object containing mesh data for robot and environment */
            GeometrySpecification         geom_;

            /** \brief Instance of the state validity checker for collision checking */
            base::StateValidityCheckerPtr validitySvc_;

            /** \brief Value containing the type of collision checking to use */
            CollisionChecker              ctype_;

        };

    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPLAPP_GRAPHICS_RENDER_GEOMETRY_
#define OMPLAPP_GRAPHICS_RENDER_GEOMETRY_

#include "omplapp/geometry/RigidBodyGeometry.h"
#include <ompl/base/Planner.h>

namespace ompl
{
    namespace app
    {

        class RenderGeometry
        {
        public:

            /** \brief Constructor expects a state space that can represent a rigid body */
            RenderGeometry(const RigidBodyGeometry &rbg, const GeometricStateExtractor &se) : rbg_(rbg), se_(se)
            {
            }

            virtual ~RenderGeometry(void)
            {
            }

            int renderEnvironment(void) const;

            int renderRobot(void) const;

            int renderRobotPart(unsigned int index) const;

            int renderPlannerData(const base::PlannerData &pd) const;

        private:

            const RigidBodyGeometry &rbg_;
            GeometricStateExtractor  se_;

        };

    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPLAPP_APP_BASE_
#define OMPLAPP_APP_BASE_

#include "omplapp/geometry/RigidBodyGeometry.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include "omplapp/apps/detail/appUtil.h"

namespace ompl
{
    namespace app
    {

        enum AppType
            { GEOMETRIC, CONTROL };

        template<AppType T>
        struct AppTypeSelector
        {
            typedef geometric::SimpleSetup SimpleSetup;
            typedef base::StateSpacePtr    SpaceType;
        };

        template<>
        struct AppTypeSelector<CONTROL>
        {
            typedef control::SimpleSetup     SimpleSetup;
            typedef control::ControlSpacePtr SpaceType;
        };

        template<AppType T>
        class AppBase : public AppTypeSelector<T>::SimpleSetup,
                        public RigidBodyGeometry
        {
        public:
            AppBase(const typename AppTypeSelector<T>::SpaceType &space, MotionModel model) :
                AppTypeSelector<T>::SimpleSetup(space), RigidBodyGeometry(model)
            {
            }

            virtual ~AppBase(void)
            {
            }

            AppType getAppType()
            {
                return GEOMETRIC;
            }
            const std::string& getName()
            {
                return name_;
            }

            virtual bool isSelfCollisionEnabled(void) const = 0;

            virtual base::ScopedState<> getDefaultStartState(void) const = 0;

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const = 0;

            virtual base::ScopedState<> getGeometricComponentState(const base::ScopedState<> &state, unsigned int index) const
            {
                return base::ScopedState<>(getGeometricComponentStateSpace(), getGeometricComponentStateInternal(state.get(), index));
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const = 0;

            virtual unsigned int getRobotCount(void) const = 0;

            GeometricStateExtractor getGeometricStateExtractor(void) const
            {
                return boost::bind(&AppBase::getGeometricComponentStateInternal, this, _1, _2);
            }

            virtual void inferEnvironmentBounds(void)
            {
                InferEnvironmentBounds(getGeometricComponentStateSpace(), *static_cast<RigidBodyGeometry*>(this));
            }

            virtual void inferProblemDefinitionBounds(void)
            {
                InferProblemDefinitionBounds(AppTypeSelector<T>::SimpleSetup::getProblemDefinition(), getGeometricStateExtractor(), factor_, add_,
                                             getRobotCount(), getGeometricComponentStateSpace(), mtype_);
            }

            virtual void setup(void)
            {
                inferEnvironmentBounds();

                if (AppTypeSelector<T>::SimpleSetup::getProblemDefinition()->getStartStateCount() == 0)
                {
                    OMPL_INFORM("Adding default start state");
                    AppTypeSelector<T>::SimpleSetup::addStartState(getDefaultStartState());
                }

                inferProblemDefinitionBounds();

                const base::StateValidityCheckerPtr &svc = allocStateValidityChecker(AppTypeSelector<T>::SimpleSetup::si_,
                                                                                     getGeometricStateExtractor(), isSelfCollisionEnabled());
                if (AppTypeSelector<T>::SimpleSetup::si_->getStateValidityChecker() != svc)
                    AppTypeSelector<T>::SimpleSetup::si_->setStateValidityChecker(svc);

                AppTypeSelector<T>::SimpleSetup::getStateSpace()->setup();

                if (!AppTypeSelector<T>::SimpleSetup::getStateSpace()->hasDefaultProjection())
                    AppTypeSelector<T>::SimpleSetup::getStateSpace()->
                        registerDefaultProjection(allocGeometricStateProjector(AppTypeSelector<T>::SimpleSetup::getStateSpace(),
                                                                               mtype_, getGeometricComponentStateSpace(),
                                                                               getGeometricStateExtractor()));

                AppTypeSelector<T>::SimpleSetup::setup();
            }

            control::DecompositionPtr allocDecomposition()
            {
                return ompl::app::allocDecomposition(AppTypeSelector<T>::SimpleSetup::getStateSpace(),
                    mtype_, getGeometricComponentStateSpace());
            }

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const = 0;

            std::string name_;

        };

        template<>
        inline AppType AppBase<CONTROL>::getAppType(void)
        {
            return CONTROL;
        }

    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPLAPP_SE2_RIGID_BODY_PLANNING_
#define OMPLAPP_SE2_RIGID_BODY_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE2StateSpace.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class SE2RigidBodyPlanning : public AppBase<GEOMETRIC>
        {
        public:

            SE2RigidBodyPlanning(void) : AppBase<GEOMETRIC>(base::StateSpacePtr(new base::SE2StateSpace()), Motion_2D)
            {
                name_ = "Rigid body planning (2D)";
            }

            virtual ~SE2RigidBodyPlanning(void)
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }

            virtual base::ScopedState<> getDefaultStartState(void) const;

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                return state;
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                return getStateSpace();
            }

            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const
            {
                return state;
            }

        };

    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPLAPP_SE3_RIGID_BODY_PLANNING_
#define OMPLAPP_SE3_RIGID_BODY_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE3. */
        class SE3RigidBodyPlanning : public AppBase<GEOMETRIC>
        {
        public:

            SE3RigidBodyPlanning(void) : AppBase<GEOMETRIC>(base::StateSpacePtr(new base::SE3StateSpace()), Motion_3D)
            {
                name_ = "Rigid body planning (3D)";
            }

            virtual ~SE3RigidBodyPlanning(void)
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }

            virtual base::ScopedState<> getDefaultStartState(void) const;

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                return state;
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                return getStateSpace();
            }

            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const
            {
                return state;
            }

        };

    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2012, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ryan Luna */

#include "omplapp/config.h"
#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE2StateSpace.h>

namespace ompl
{
    namespace app
    {

        /// @brief Wrapper for ompl::app::RigidBodyPlanning that plans for multiple rigid bodies in SE2.
        class SE2MultiRigidBodyPlanning : public AppBase<GEOMETRIC>
        {
        public:
            /// @brief Constructs an instance of multiple rigid bodies for 2D geometric planning.  n is the number of independent bodies in SE(2)
            SE2MultiRigidBodyPlanning(unsigned int n);

            virtual ~SE2MultiRigidBodyPlanning(void) {}

            /// @brief Constructs the default start state where all robots begin at their geometric center.
            /// If robots are all using the same mesh, this state is not likely to be valid.
            virtual base::ScopedState<> getDefaultStartState(void) const;

            virtual void inferEnvironmentBounds(void);

            virtual void inferProblemDefinitionBounds(void);

            bool isSelfCollisionEnabled(void) const
            {
                // Make sure that self collision is enabled to avoid inter-rigid body collision
                return true;
            }

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                return state;
            }

            /// @brief Returns the state space corresponding for the indexth rigid body
            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(unsigned int index) const
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(index);
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                // Return the zeroth component.  All components are the same.
                return getGeometricComponentStateSpace(0);
            }

            virtual unsigned int getRobotCount(void) const
            {
                return n_;
            }

        protected:
            /// @brief Returns the state corresponding to the indexth rigid body in the compound state
            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const;

            /// @brief The number of independent rigid bodies to plan for
            unsigned int n_;
        };

    }
}

/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2012, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ryan Luna */

#include "omplapp/config.h"
#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ompl
{
    namespace app
    {

        /// @brief Wrapper for ompl::app::RigidBodyPlanning that plans for multiple rigid bodies in SE3.
        class SE3MultiRigidBodyPlanning : public AppBase<GEOMETRIC>
        {
        public:
            /// @brief Constructs an instance of multiple rigid bodies for 3D geometric planning.  n is the number of independent bodies in SE(3)
            SE3MultiRigidBodyPlanning(unsigned int n);

            virtual ~SE3MultiRigidBodyPlanning(void) {}

            /// @brief Constructs the default start state where all robots begin at their geometric center.
            /// If robots are all using the same mesh, this state is not likely to be valid.
            virtual base::ScopedState<> getDefaultStartState(void) const;

            virtual void inferEnvironmentBounds(void);

            virtual void inferProblemDefinitionBounds(void);

            bool isSelfCollisionEnabled(void) const
            {
                // Make sure that self collision is enabled to avoid inter-rigid body collision
                return true;
            }

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                return state;
            }

            /// @brief Returns the state space corresponding for the indexth rigid body
            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(unsigned int index) const
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(index);
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                // Return the zeroth component.  All components are the same.
                return getGeometricComponentStateSpace(0);
            }

            virtual unsigned int getRobotCount(void) const
            {
                return n_;
            }

        protected:
            /// @brief Returns the state corresponding to the indexth rigid body in the compound state
            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const;

            /// @brief The number of independent rigid bodies to plan for
            unsigned int n_;
        };

    }
}

/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_KINEMATIC_CAR_PLANNING_
#define OMPLAPP_KINEMATIC_CAR_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        /** \brief A class to facilitate planning for a generic kinematic car
            model

            The dynamics of the kinematic car are described by the following
            equations:
            \f{eqnarray*}{
            \dot x &=& u_0 \cos\theta,\\
            \dot y &=& u_0\sin\theta,\\
            \dot\theta &=& \frac{u_0}{L}\tan u_1,\f}
            where the control inputs \f$(u_0,u_1)\f$ are the translational
            velocity and the steering angle, respectively, and \f$L\f$ is the
            distance between the front and rear axle of the car (set to 1 by
            default).
        */
        class KinematicCarPlanning : public AppBase<CONTROL>
        {
        public:
            KinematicCarPlanning();
            KinematicCarPlanning(const control::ControlSpacePtr &controlSpace);
            ~KinematicCarPlanning()
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }
            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }
            virtual base::ScopedState<> getDefaultStartState(void) const;
            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                return state;
            }
            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                return getStateSpace();
            }
            double getVehicleLength()
            {
                return 1./lengthInv_;
            }
            void setVehicleLength(double length)
            {
                lengthInv_ = 1./length;
            }
            virtual void setDefaultControlBounds();

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const
            {
                return state;
            }

            virtual void ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot);

            virtual void postPropagate(const base::State* state, const control::Control* control, const double duration, base::State* result);

            static control::ControlSpacePtr constructControlSpace(void)
            {
                return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 2));
            }
            static base::StateSpacePtr constructStateSpace(void)
            {
                return base::StateSpacePtr(new base::SE2StateSpace());
            }

            double timeStep_;
            double lengthInv_;
            control::ODESolverPtr odeSolver;
        };
    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_DYNAMIC_CAR_PLANNING_
#define OMPLAPP_DYNAMIC_CAR_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        /** \brief A class to facilitate planning for a generic second-order
            car model

            The dynamics of the second-order car are described by the following
            equations:
            \f{eqnarray*}{
            \dot x &=& v\cos\theta,\\
            \dot y &=& v\sin\theta,\\
            \dot\theta &=& \frac{vm}{L}\tan \phi,\\
            \dot v &=& u_0,\\
            \dot\phi &=& u_1,\f}
            where \f$v\f$ is the speed, \f$\phi\f$ the steering angle, the
            controls \f$(u_0,u_1)\f$ control their rate of change, \f$m\f$ is
            the mass of the car, and \f$L\f$ is the distance between the front
            and rear axle of the car. Both \f$m\f$ and \f$L\f$ are set to 1 by
            default.
        */
        class DynamicCarPlanning : public AppBase<CONTROL>
        {
        public:
            DynamicCarPlanning()
                : AppBase<CONTROL>(constructControlSpace(), Motion_2D), timeStep_(1e-2), lengthInv_(1.), mass_(1.), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&DynamicCarPlanning::ode, this, _1, _2, _3)))
            {
                name_ = std::string("Dynamic car");
                setDefaultBounds();

                si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&DynamicCarPlanning::postPropagate, this, _1, _2, _3, _4)));
            }
            ~DynamicCarPlanning()
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }
            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }
            virtual base::ScopedState<> getDefaultStartState(void) const;
            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                base::ScopedState<> r(si_);
                r = 0.0;
                r[0] = state[0];
                r[1] = state[1];
                r[2] = state[2];
                return r;
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
            }

            double getVehicleLength()
            {
                return 1./lengthInv_;
            }
            void setVehicleLength(double length)
            {
                lengthInv_ = 1./length;
            }
            double getMass()
            {
                return mass_;
            }
            void setMass(double mass)
            {
                mass_ = mass;
            }
            virtual void setDefaultBounds();

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const
            {
                return state->as<base::CompoundState>()->components[0];
            }

            virtual void ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot);

            virtual void postPropagate(const base::State* state, const control::Control* control, const double duration, base::State* result);

            static control::ControlSpacePtr constructControlSpace(void)
            {
                return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 2));
            }
            static base::StateSpacePtr constructStateSpace(void)
            {
                base::StateSpacePtr stateSpace = base::StateSpacePtr(new base::CompoundStateSpace());
                stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::SE2StateSpace()), 1.);
                stateSpace->as<base::CompoundStateSpace>()->addSubspace(base::StateSpacePtr(new base::RealVectorStateSpace(2)), .3);
                stateSpace->as<base::CompoundStateSpace>()->lock();
                return stateSpace;
            }

            double timeStep_;
            double lengthInv_;
            double mass_;
            control::ODESolverPtr odeSolver;
        };

    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_BLIMP_PLANNING_
#define OMPLAPP_BLIMP_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        /** \brief A class to facilitate planning for a simple blimp model

            The dynamics of the blimp are described by the following equations:
            \f{eqnarray*}{
            \ddot x &=& u_f\cos\theta,\\
            \ddot y &=& u_f\sin\theta,\\
            \ddot z &=& u_z,\\
            \ddot\theta &=& u_\theta,\f}
            where \f$(x,y,z)\f$ is the position, \f$\theta\f$ the heading, and the
            controls \f$(u_f,u_z,u_\theta)\f$ control their rate of change.
        */
        class BlimpPlanning : public AppBase<CONTROL>
        {
        public:
            BlimpPlanning()
                : AppBase<CONTROL>(constructControlSpace(), Motion_3D), timeStep_(1e-2), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&BlimpPlanning::ode, this, _1, _2, _3)))
            {
                name_ = std::string("Blimp");
                setDefaultBounds();

                si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&BlimpPlanning::postPropagate, this, _1, _2, _3, _4)));
            }
            ~BlimpPlanning()
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }
            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }
            virtual base::ScopedState<> getDefaultStartState(void) const;
            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const;
            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
            }

            virtual void setDefaultBounds();

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const
            {
                return state->as<base::CompoundState>()->components[0];
            }

            void postPropagate(const base::State* state, const control::Control* control, const double duration, base::State* result);

            virtual void ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot);

            static control::ControlSpacePtr constructControlSpace(void)
            {
                return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 3));
            }
            static base::StateSpacePtr constructStateSpace(void);

            double timeStep_;
            control::ODESolverPtr odeSolver;
        };

    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_QUADROTOR_PLANNING_
#define OMPLAPP_QUADROTOR_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        /** \brief A class to facilitate planning for a simple quadrotor model

            The dynamics of the quadrotor are described by the following equations:
            \f{eqnarray*}{
            m\ddot \mathbf{p} &=& -u_0\mathbf{n}-\beta\dot\mathbf{p} -m\mathbf{g},\\
            \mathbf{\alpha} &=& (u_1,u_2,u_3)^T,\f}
            where \f$\mathbf{p}\f$ is the position, \f$\mathbf{n}\f$ is the Z-axis of
            the body frame in world coordinates, \f$\alpha\f$ is the angular
            acceleration, \f$m\f$ is the mass, and \f$\beta\f$ is a damping coefficient.
            The system is controlled through \f$u=(u_0,u_1,u_2,u_3)\f$.
        */
        class QuadrotorPlanning : public AppBase<CONTROL>
        {
        public:
            QuadrotorPlanning()
                : AppBase<CONTROL>(constructControlSpace(), Motion_3D), timeStep_(1e-2), massInv_(1.), beta_(1.), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&QuadrotorPlanning::ode, this, _1, _2, _3)))
            {
                name_ = std::string("Quadrotor");
                setDefaultBounds();

                si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&QuadrotorPlanning::postPropagate, this, _1, _2, _3, _4)));
            }
            ~QuadrotorPlanning()
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }
            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }
            virtual base::ScopedState<> getDefaultStartState(void) const;
            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const;
            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
            }
            double getMass()
            {
                return 1./massInv_;
            }
            void setMass(double mass)
            {
                massInv_ = 1./mass;
            }
            double getDampingCoefficient()
            {
                return beta_;
            }
            void setDampingCoefficient(double beta)
            {
                beta_ = beta;
            }
            virtual void setDefaultBounds();

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const
            {
                return state->as<base::CompoundState>()->components[0];
            }

            virtual void ode(const control::ODESolver::StateType& q, const control::Control* ctrl, control::ODESolver::StateType& qdot);

            virtual void postPropagate(const base::State* state, const control::Control* control, const double duration, base::State* result);

            static control::ControlSpacePtr constructControlSpace(void)
            {
                return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 4));
            }
            static base::StateSpacePtr constructStateSpace(void);

            double timeStep_;
            double massInv_;
            double beta_;
            control::ODESolverPtr odeSolver;
        };

    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPLAPP_G_SE2_RIGID_BODY_PLANNING_
#define OMPLAPP_G_SE2_RIGID_BODY_PLANNING_

#include "omplapp/apps/SE2RigidBodyPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GSE2RigidBodyPlanning : public SE2RigidBodyPlanning,
                                      public RenderGeometry
        {
        public:

            GSE2RigidBodyPlanning(void) : SE2RigidBodyPlanning(),
                                          RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GSE2RigidBodyPlanning(void)
            {
            }
        };

    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPLAPP_G_SE3_RIGID_BODY_PLANNING_
#define OMPLAPP_G_SE3_RIGID_BODY_PLANNING_

#include "omplapp/apps/SE3RigidBodyPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GSE3RigidBodyPlanning : public SE3RigidBodyPlanning,
                                      public RenderGeometry
        {
        public:

            GSE3RigidBodyPlanning(void) : SE3RigidBodyPlanning(),
                                          RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GSE3RigidBodyPlanning(void)
            {
            }
        };

    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_G_KINEMATIC_CAR_PLANNING_
#define OMPLAPP_G_KINEMATIC_CAR_PLANNING_

#include "omplapp/apps/KinematicCarPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GKinematicCarPlanning : public KinematicCarPlanning,
                                    public RenderGeometry
        {
        public:

            GKinematicCarPlanning(void) : KinematicCarPlanning(),
                                        RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GKinematicCarPlanning(void)
            {
            }
        };
    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_G_DYNAMIC_CAR_PLANNING_
#define OMPLAPP_G_DYNAMIC_CAR_PLANNING_

#include "omplapp/apps/DynamicCarPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GDynamicCarPlanning : public DynamicCarPlanning,
                                    public RenderGeometry
        {
        public:

            GDynamicCarPlanning(void) : DynamicCarPlanning(),
                                        RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GDynamicCarPlanning(void)
            {
            }
        };
    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_G_BLIMP_PLANNING_
#define OMPLAPP_G_BLIMP_PLANNING_

#include "omplapp/apps/BlimpPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GBlimpPlanning : public BlimpPlanning,
                               public RenderGeometry
        {
        public:

            GBlimpPlanning(void) : BlimpPlanning(),
                                   RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GBlimpPlanning(void)
            {
            }
        };
    }
}

#endif
/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_G_QUADROTOR_PLANNING_
#define OMPLAPP_G_QUADROTOR_PLANNING_

#include "omplapp/apps/QuadrotorPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GQuadrotorPlanning : public QuadrotorPlanning,
                                   public RenderGeometry
        {
        public:

            GQuadrotorPlanning(void) : QuadrotorPlanning(),
                                       RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GQuadrotorPlanning(void)
            {
            }
        };
    }
}

#endif
