// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/app.h"
#include "_app_enumerations.pypp.hpp"

namespace bp = boost::python;

void register_enumerations(){

    bp::enum_< ompl::app::AppType>("AppType")
        .value("GEOMETRIC", ompl::app::GEOMETRIC)
        .value("CONTROL", ompl::app::CONTROL)
        .export_values()
        ;

    bp::enum_< ompl::app::CollisionChecker>("CollisionChecker")
        .value("PQP", ompl::app::PQP)
        .value("FCL", ompl::app::FCL)
        .export_values()
        ;

}
