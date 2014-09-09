// This file has been generated by Py++.

#include "boost/python.hpp"
#include "boost/python/suite/indexing/map_indexing_suite.hpp"
#include "bindings/util.h"
#include "mapStringToString.pypp.hpp"

namespace bp = boost::python;

void register_mapStringToString_class(){

    { //::std::map< std::string, std::string >
        typedef bp::class_< std::map< std::string, std::string > > mapStringToString_exposer_t;
        mapStringToString_exposer_t mapStringToString_exposer = mapStringToString_exposer_t( "mapStringToString" );
        bp::scope mapStringToString_scope( mapStringToString_exposer );
        mapStringToString_exposer.def( bp::map_indexing_suite< ::std::map< std::string, std::string >, true >() );
    }

}