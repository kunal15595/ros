// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/geometric.h"
#include "PathHybridization.pypp.hpp"

namespace bp = boost::python;

std::string __str__(::ompl::geometric::PathHybridization* obj)
{
    std::ostringstream s;
    obj->print(s);
    return s.str();
}

void register_PathHybridization_class(){

    { //::ompl::geometric::PathHybridization
        typedef bp::class_< ompl::geometric::PathHybridization > PathHybridization_exposer_t;
        PathHybridization_exposer_t PathHybridization_exposer = PathHybridization_exposer_t( "PathHybridization", bp::init< ompl::base::SpaceInformationPtr const & >(( bp::arg("si") )) );
        bp::scope PathHybridization_scope( PathHybridization_exposer );
        bp::implicitly_convertible< ompl::base::SpaceInformationPtr const &, ompl::geometric::PathHybridization >();
        { //::ompl::geometric::PathHybridization::clear
        
            typedef void ( ::ompl::geometric::PathHybridization::*clear_function_type)(  ) ;
            
            PathHybridization_exposer.def( 
                "clear"
                , clear_function_type( &::ompl::geometric::PathHybridization::clear ) );
        
        }
        { //::ompl::geometric::PathHybridization::computeHybridPath
        
            typedef void ( ::ompl::geometric::PathHybridization::*computeHybridPath_function_type)(  ) ;
            
            PathHybridization_exposer.def( 
                "computeHybridPath"
                , computeHybridPath_function_type( &::ompl::geometric::PathHybridization::computeHybridPath ) );
        
        }
        { //::ompl::geometric::PathHybridization::getHybridPath
        
            typedef ::ompl::base::PathPtr const & ( ::ompl::geometric::PathHybridization::*getHybridPath_function_type)(  ) const;
            
            PathHybridization_exposer.def( 
                "getHybridPath"
                , getHybridPath_function_type( &::ompl::geometric::PathHybridization::getHybridPath )
                , bp::return_value_policy< bp::copy_const_reference >() );
        
        }
        { //::ompl::geometric::PathHybridization::getName
        
            typedef ::std::string const & ( ::ompl::geometric::PathHybridization::*getName_function_type)(  ) const;
            
            PathHybridization_exposer.def( 
                "getName"
                , getName_function_type( &::ompl::geometric::PathHybridization::getName )
                , bp::return_value_policy< bp::copy_const_reference >() );
        
        }
        { //::ompl::geometric::PathHybridization::matchPaths
        
            typedef void ( ::ompl::geometric::PathHybridization::*matchPaths_function_type)( ::ompl::geometric::PathGeometric const &,::ompl::geometric::PathGeometric const &,double,::std::vector< int > &,::std::vector< int > & ) const;
            
            PathHybridization_exposer.def( 
                "matchPaths"
                , matchPaths_function_type( &::ompl::geometric::PathHybridization::matchPaths )
                , ( bp::arg("p"), bp::arg("q"), bp::arg("gapCost"), bp::arg("indexP"), bp::arg("indexQ") ) );
        
        }
        { //::ompl::geometric::PathHybridization::pathCount
        
            typedef ::std::size_t ( ::ompl::geometric::PathHybridization::*pathCount_function_type)(  ) const;
            
            PathHybridization_exposer.def( 
                "pathCount"
                , pathCount_function_type( &::ompl::geometric::PathHybridization::pathCount ) );
        
        }
        { //::ompl::geometric::PathHybridization::recordPath
        
            typedef unsigned int ( ::ompl::geometric::PathHybridization::*recordPath_function_type)( ::ompl::base::PathPtr const &,bool ) ;
            
            PathHybridization_exposer.def( 
                "recordPath"
                , recordPath_function_type( &::ompl::geometric::PathHybridization::recordPath )
                , ( bp::arg("pp"), bp::arg("matchAcrossGaps") ) );
        
        }
        PathHybridization_exposer.def("__str__", &__str__);
    }

}
