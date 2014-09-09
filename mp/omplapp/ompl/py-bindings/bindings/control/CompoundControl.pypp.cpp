// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/control.h"
#include "CompoundControl.pypp.hpp"

namespace bp = boost::python;

void register_CompoundControl_class(){

    { //::ompl::control::CompoundControl
        typedef bp::class_< ompl::control::CompoundControl, bp::bases< ompl::control::Control >, boost::noncopyable > CompoundControl_exposer_t;
        CompoundControl_exposer_t CompoundControl_exposer = CompoundControl_exposer_t( "CompoundControl", bp::init< >() );
        bp::scope CompoundControl_scope( CompoundControl_exposer );
    }

}