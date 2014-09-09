// This file has been generated by Py++.

#include "boost/python.hpp"
#include "indexing_suite/container_suite.hpp"
#include "indexing_suite/vector.hpp"
#include "bindings/base.h"
#include "vectorPlannerSolution.pypp.hpp"

namespace bp = boost::python;

void register_vectorPlannerSolution_class(){

    { //::std::vector< ompl::base::PlannerSolution >
        typedef bp::class_< std::vector< ompl::base::PlannerSolution > > vectorPlannerSolution_exposer_t;
        vectorPlannerSolution_exposer_t vectorPlannerSolution_exposer = vectorPlannerSolution_exposer_t( "vectorPlannerSolution" );
        bp::scope vectorPlannerSolution_scope( vectorPlannerSolution_exposer );
        vectorPlannerSolution_exposer.def( bp::indexing::vector_suite< std::vector< ompl::base::PlannerSolution > >() );
    }

}