// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/geometric.h"
#include "PathSimplifier.pypp.hpp"

namespace bp = boost::python;

void register_PathSimplifier_class(){

    { //::ompl::geometric::PathSimplifier
        typedef bp::class_< ompl::geometric::PathSimplifier > PathSimplifier_exposer_t;
        PathSimplifier_exposer_t PathSimplifier_exposer = PathSimplifier_exposer_t( "PathSimplifier", bp::init< ompl::base::SpaceInformationPtr const & >(( bp::arg("si") )) );
        bp::scope PathSimplifier_scope( PathSimplifier_exposer );
        bp::implicitly_convertible< ompl::base::SpaceInformationPtr const &, ompl::geometric::PathSimplifier >();
        { //::ompl::geometric::PathSimplifier::collapseCloseVertices
        
            typedef bool ( ::ompl::geometric::PathSimplifier::*collapseCloseVertices_function_type)( ::ompl::geometric::PathGeometric &,unsigned int,unsigned int ) ;
            
            PathSimplifier_exposer.def( 
                "collapseCloseVertices"
                , collapseCloseVertices_function_type( &::ompl::geometric::PathSimplifier::collapseCloseVertices )
                , ( bp::arg("path"), bp::arg("maxSteps")=(unsigned int)(0), bp::arg("maxEmptySteps")=(unsigned int)(0) ) );
        
        }
        { //::ompl::geometric::PathSimplifier::freeStates
        
            typedef void ( ::ompl::geometric::PathSimplifier::*freeStates_function_type)( bool ) ;
            
            PathSimplifier_exposer.def( 
                "freeStates"
                , freeStates_function_type( &::ompl::geometric::PathSimplifier::freeStates )
                , ( bp::arg("flag") ) );
        
        }
        { //::ompl::geometric::PathSimplifier::freeStates
        
            typedef bool ( ::ompl::geometric::PathSimplifier::*freeStates_function_type)(  ) const;
            
            PathSimplifier_exposer.def( 
                "freeStates"
                , freeStates_function_type( &::ompl::geometric::PathSimplifier::freeStates ) );
        
        }
        { //::ompl::geometric::PathSimplifier::reduceVertices
        
            typedef bool ( ::ompl::geometric::PathSimplifier::*reduceVertices_function_type)( ::ompl::geometric::PathGeometric &,unsigned int,unsigned int,double ) ;
            
            PathSimplifier_exposer.def( 
                "reduceVertices"
                , reduceVertices_function_type( &::ompl::geometric::PathSimplifier::reduceVertices )
                , ( bp::arg("path"), bp::arg("maxSteps")=(unsigned int)(0), bp::arg("maxEmptySteps")=(unsigned int)(0), bp::arg("rangeRatio")=3.30000000000000015543122344752191565930843353271484375e-1 ) );
        
        }
        { //::ompl::geometric::PathSimplifier::shortcutPath
        
            typedef bool ( ::ompl::geometric::PathSimplifier::*shortcutPath_function_type)( ::ompl::geometric::PathGeometric &,unsigned int,unsigned int,double,double ) ;
            
            PathSimplifier_exposer.def( 
                "shortcutPath"
                , shortcutPath_function_type( &::ompl::geometric::PathSimplifier::shortcutPath )
                , ( bp::arg("path"), bp::arg("maxSteps")=(unsigned int)(0), bp::arg("maxEmptySteps")=(unsigned int)(0), bp::arg("rangeRatio")=3.30000000000000015543122344752191565930843353271484375e-1, bp::arg("snapToVertex")=5.00000000000000010408340855860842566471546888351440429688e-3 ) );
        
        }
        { //::ompl::geometric::PathSimplifier::simplify
        
            typedef void ( ::ompl::geometric::PathSimplifier::*simplify_function_type)( ::ompl::geometric::PathGeometric &,double ) ;
            
            PathSimplifier_exposer.def( 
                "simplify"
                , simplify_function_type( &::ompl::geometric::PathSimplifier::simplify )
                , ( bp::arg("path"), bp::arg("maxTime") ) );
        
        }
        { //::ompl::geometric::PathSimplifier::simplify
        
            typedef void ( ::ompl::geometric::PathSimplifier::*simplify_function_type)( ::ompl::geometric::PathGeometric &,::ompl::base::PlannerTerminationCondition const & ) ;
            
            PathSimplifier_exposer.def( 
                "simplify"
                , simplify_function_type( &::ompl::geometric::PathSimplifier::simplify )
                , ( bp::arg("path"), bp::arg("ptc") ) );
        
        }
        { //::ompl::geometric::PathSimplifier::simplifyMax
        
            typedef void ( ::ompl::geometric::PathSimplifier::*simplifyMax_function_type)( ::ompl::geometric::PathGeometric & ) ;
            
            PathSimplifier_exposer.def( 
                "simplifyMax"
                , simplifyMax_function_type( &::ompl::geometric::PathSimplifier::simplifyMax )
                , ( bp::arg("path") ) );
        
        }
        { //::ompl::geometric::PathSimplifier::smoothBSpline
        
            typedef void ( ::ompl::geometric::PathSimplifier::*smoothBSpline_function_type)( ::ompl::geometric::PathGeometric &,unsigned int,double ) ;
            
            PathSimplifier_exposer.def( 
                "smoothBSpline"
                , smoothBSpline_function_type( &::ompl::geometric::PathSimplifier::smoothBSpline )
                , ( bp::arg("path"), bp::arg("maxSteps")=(unsigned int)(5), bp::arg("minChange")=std::numeric_limits<double>::epsilon() ) );
        
        }
        bp::register_ptr_to_python< boost::shared_ptr< ompl::geometric::PathSimplifier > >();
    }

}
