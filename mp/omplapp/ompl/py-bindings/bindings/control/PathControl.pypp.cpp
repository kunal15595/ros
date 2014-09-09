// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/control.h"
#include "PathControl.pypp.hpp"

namespace bp = boost::python;

struct PathControl_wrapper : ompl::control::PathControl, bp::wrapper< ompl::control::PathControl > {

    PathControl_wrapper(::ompl::base::SpaceInformationPtr const & si )
    : ompl::control::PathControl( si )
      , bp::wrapper< ompl::control::PathControl >(){
        // constructor
    
    }

    PathControl_wrapper(::ompl::control::PathControl const & path )
    : ompl::control::PathControl( boost::ref(path) )
      , bp::wrapper< ompl::control::PathControl >(){
        // copy constructor
    
    }

    virtual bool check(  ) const  {
        if( bp::override func_check = this->get_override( "check" ) )
            return func_check(  );
        else{
            return this->ompl::control::PathControl::check(  );
        }
    }
    
    bool default_check(  ) const  {
        return ompl::control::PathControl::check( );
    }

    void copyFrom( ::ompl::control::PathControl const & other ){
        ompl::control::PathControl::copyFrom( boost::ref(other) );
    }

    void freeMemory(  ){
        ompl::control::PathControl::freeMemory(  );
    }

    virtual double length(  ) const  {
        if( bp::override func_length = this->get_override( "length" ) )
            return func_length(  );
        else{
            return this->ompl::control::PathControl::length(  );
        }
    }
    
    double default_length(  ) const  {
        return ompl::control::PathControl::length( );
    }

    virtual ::ompl::base::Cost cost( ::ompl::base::OptimizationObjectivePtr const & obj ) const  {
        if( bp::override func_cost = this->get_override( "cost" ) )
            return func_cost( obj );
        else{
            return this->ompl::base::Path::cost( obj );
        }
    }
    
    ::ompl::base::Cost default_cost( ::ompl::base::OptimizationObjectivePtr const & obj ) const  {
        return ompl::base::Path::cost( obj );
    }

};

std::string __str__(::ompl::control::PathControl* obj)
{
    std::ostringstream s;
    obj->print(s);
    return s.str();
}

std::string __printAsMatrix(::ompl::control::PathControl* path)
        {
            std::ostringstream s;
            path->printAsMatrix(s);
            return s.str();
        }

void register_PathControl_class(){

    { //::ompl::control::PathControl
        typedef bp::class_< PathControl_wrapper, bp::bases< ::ompl::base::Path > > PathControl_exposer_t;
        PathControl_exposer_t PathControl_exposer = PathControl_exposer_t( "PathControl", bp::init< ompl::base::SpaceInformationPtr const & >(( bp::arg("si") )) );
        bp::scope PathControl_scope( PathControl_exposer );
        bp::implicitly_convertible< ompl::base::SpaceInformationPtr const &, ompl::control::PathControl >();
        PathControl_exposer.def( bp::init< ompl::control::PathControl const & >(( bp::arg("path") )) );
        { //::ompl::control::PathControl::append
        
            typedef void ( ::ompl::control::PathControl::*append_function_type)( ::ompl::base::State const * ) ;
            
            PathControl_exposer.def( 
                "append"
                , append_function_type( &::ompl::control::PathControl::append )
                , ( bp::arg("state") ) );
        
        }
        { //::ompl::control::PathControl::append
        
            typedef void ( ::ompl::control::PathControl::*append_function_type)( ::ompl::base::State const *,::ompl::control::Control const *,double ) ;
            
            PathControl_exposer.def( 
                "append"
                , append_function_type( &::ompl::control::PathControl::append )
                , ( bp::arg("state"), bp::arg("control"), bp::arg("duration") ) );
        
        }
        { //::ompl::control::PathControl::asGeometric
        
            typedef ::ompl::geometric::PathGeometric ( ::ompl::control::PathControl::*asGeometric_function_type)(  ) const;
            
            PathControl_exposer.def( 
                "asGeometric"
                , asGeometric_function_type( &::ompl::control::PathControl::asGeometric ) );
        
        }
        { //::ompl::control::PathControl::check
        
            typedef bool ( ::ompl::control::PathControl::*check_function_type)(  ) const;
            typedef bool ( PathControl_wrapper::*default_check_function_type)(  ) const;
            
            PathControl_exposer.def( 
                "check"
                , check_function_type(&::ompl::control::PathControl::check)
                , default_check_function_type(&PathControl_wrapper::default_check) );
        
        }
        { //::ompl::control::PathControl::copyFrom
        
            typedef void ( PathControl_wrapper::*copyFrom_function_type)( ::ompl::control::PathControl const & ) ;
            
            PathControl_exposer.def( 
                "copyFrom"
                , copyFrom_function_type( &PathControl_wrapper::copyFrom )
                , ( bp::arg("other") ) );
        
        }
        { //::ompl::control::PathControl::freeMemory
        
            typedef void ( PathControl_wrapper::*freeMemory_function_type)(  ) ;
            
            PathControl_exposer.def( 
                "freeMemory"
                , freeMemory_function_type( &PathControl_wrapper::freeMemory ) );
        
        }
        { //::ompl::control::PathControl::getControl
        
            typedef ::ompl::control::Control * ( ::ompl::control::PathControl::*getControl_function_type)( unsigned int ) ;
            
            PathControl_exposer.def( 
                "getControl"
                , getControl_function_type( &::ompl::control::PathControl::getControl )
                , ( bp::arg("index") )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::control::PathControl::getControl
        
            typedef ::ompl::control::Control const * ( ::ompl::control::PathControl::*getControl_function_type)( unsigned int ) const;
            
            PathControl_exposer.def( 
                "getControl"
                , getControl_function_type( &::ompl::control::PathControl::getControl )
                , ( bp::arg("index") )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::control::PathControl::getControlCount
        
            typedef ::std::size_t ( ::ompl::control::PathControl::*getControlCount_function_type)(  ) const;
            
            PathControl_exposer.def( 
                "getControlCount"
                , getControlCount_function_type( &::ompl::control::PathControl::getControlCount ) );
        
        }
        { //::ompl::control::PathControl::getControlDuration
        
            typedef double ( ::ompl::control::PathControl::*getControlDuration_function_type)( unsigned int ) const;
            
            PathControl_exposer.def( 
                "getControlDuration"
                , getControlDuration_function_type( &::ompl::control::PathControl::getControlDuration )
                , ( bp::arg("index") ) );
        
        }
        { //::ompl::control::PathControl::getControlDurations
        
            typedef ::std::vector< double > & ( ::ompl::control::PathControl::*getControlDurations_function_type)(  ) ;
            
            PathControl_exposer.def( 
                "getControlDurations"
                , getControlDurations_function_type( &::ompl::control::PathControl::getControlDurations )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::control::PathControl::getControls
        
            typedef ::std::vector< ompl::control::Control* > & ( ::ompl::control::PathControl::*getControls_function_type)(  ) ;
            
            PathControl_exposer.def( 
                "getControls"
                , getControls_function_type( &::ompl::control::PathControl::getControls )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::control::PathControl::getState
        
            typedef ::ompl::base::State * ( ::ompl::control::PathControl::*getState_function_type)( unsigned int ) ;
            
            PathControl_exposer.def( 
                "getState"
                , getState_function_type( &::ompl::control::PathControl::getState )
                , ( bp::arg("index") )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::control::PathControl::getState
        
            typedef ::ompl::base::State const * ( ::ompl::control::PathControl::*getState_function_type)( unsigned int ) const;
            
            PathControl_exposer.def( 
                "getState"
                , getState_function_type( &::ompl::control::PathControl::getState )
                , ( bp::arg("index") )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::control::PathControl::getStateCount
        
            typedef ::std::size_t ( ::ompl::control::PathControl::*getStateCount_function_type)(  ) const;
            
            PathControl_exposer.def( 
                "getStateCount"
                , getStateCount_function_type( &::ompl::control::PathControl::getStateCount ) );
        
        }
        { //::ompl::control::PathControl::getStates
        
            typedef ::std::vector< ompl::base::State* > & ( ::ompl::control::PathControl::*getStates_function_type)(  ) ;
            
            PathControl_exposer.def( 
                "getStates"
                , getStates_function_type( &::ompl::control::PathControl::getStates )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::control::PathControl::interpolate
        
            typedef void ( ::ompl::control::PathControl::*interpolate_function_type)(  ) ;
            
            PathControl_exposer.def( 
                "interpolate"
                , interpolate_function_type( &::ompl::control::PathControl::interpolate ) );
        
        }
        { //::ompl::control::PathControl::length
        
            typedef double ( ::ompl::control::PathControl::*length_function_type)(  ) const;
            typedef double ( PathControl_wrapper::*default_length_function_type)(  ) const;
            
            PathControl_exposer.def( 
                "length"
                , length_function_type(&::ompl::control::PathControl::length)
                , default_length_function_type(&PathControl_wrapper::default_length) );
        
        }
        { //::ompl::control::PathControl::operator=
        
            typedef ::ompl::control::PathControl & ( ::ompl::control::PathControl::*assign_function_type)( ::ompl::control::PathControl const & ) ;
            
            PathControl_exposer.def( 
                "assign"
                , assign_function_type( &::ompl::control::PathControl::operator= )
                , ( bp::arg("other") )
                , bp::return_value_policy< bp::reference_existing_object >() );
        
        }
        { //::ompl::control::PathControl::random
        
            typedef void ( ::ompl::control::PathControl::*random_function_type)(  ) ;
            
            PathControl_exposer.def( 
                "random"
                , random_function_type( &::ompl::control::PathControl::random ) );
        
        }
        { //::ompl::control::PathControl::randomValid
        
            typedef bool ( ::ompl::control::PathControl::*randomValid_function_type)( unsigned int ) ;
            
            PathControl_exposer.def( 
                "randomValid"
                , randomValid_function_type( &::ompl::control::PathControl::randomValid )
                , ( bp::arg("attempts") ) );
        
        }
        PathControl_exposer.def("__str__", &__str__);
        PathControl_exposer.def("printAsMatrix", &__printAsMatrix);
    }

}