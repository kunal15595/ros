// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/control.h"
#include "_control_enumerations.pypp.hpp"

namespace bp = boost::python;

void register_enumerations(){

    bp::enum_< ompl::control::ControlSpaceType>("ControlSpaceType")
        .value("CONTROL_SPACE_UNKNOWN", ompl::control::CONTROL_SPACE_UNKNOWN)
        .value("CONTROL_SPACE_REAL_VECTOR", ompl::control::CONTROL_SPACE_REAL_VECTOR)
        .value("CONTROL_SPACE_DISCRETE", ompl::control::CONTROL_SPACE_DISCRETE)
        .value("CONTROL_SPACE_TYPE_COUNT", ompl::control::CONTROL_SPACE_TYPE_COUNT)
        .export_values()
        ;

}