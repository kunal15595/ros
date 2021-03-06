// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/base.h"
#include "MotionValidator.pypp.hpp"

namespace bp = boost::python;

struct MotionValidator_wrapper : ompl::base::MotionValidator, bp::wrapper< ompl::base::MotionValidator > {

    MotionValidator_wrapper(::ompl::base::SpaceInformation * si )
    : ompl::base::MotionValidator( boost::python::ptr(si) )
      , bp::wrapper< ompl::base::MotionValidator >(){
        // constructor
    
    }

    MotionValidator_wrapper(::ompl::base::SpaceInformationPtr const & si )
    : ompl::base::MotionValidator( si )
      , bp::wrapper< ompl::base::MotionValidator >(){
        // constructor
    
    }

    virtual bool checkMotion( ::ompl::base::State const * s1, ::ompl::base::State const * s2 ) const {
        bp::override func_checkMotion = this->get_override( "checkMotion" );
        return func_checkMotion( boost::python::ptr(s1), boost::python::ptr(s2) );
    }

    virtual bool checkMotion( ::ompl::base::State const * s1, ::ompl::base::State const * s2, ::std::pair< ompl::base::State*, double > & lastValid ) const {
        bp::override func_checkMotion = this->get_override( "checkMotion" );
        return func_checkMotion( boost::python::ptr(s1), boost::python::ptr(s2), boost::ref(lastValid) );
    }

};

void register_MotionValidator_class(){

    { //::ompl::base::MotionValidator
        typedef bp::class_< MotionValidator_wrapper, boost::noncopyable > MotionValidator_exposer_t;
        MotionValidator_exposer_t MotionValidator_exposer = MotionValidator_exposer_t( "MotionValidator", bp::init< ompl::base::SpaceInformation * >(( bp::arg("si") )) );
        bp::scope MotionValidator_scope( MotionValidator_exposer );
        MotionValidator_exposer.def( bp::init< ompl::base::SpaceInformationPtr const & >(( bp::arg("si") )) );
        { //::ompl::base::MotionValidator::checkMotion
        
            typedef bool ( ::ompl::base::MotionValidator::*checkMotion_function_type)( ::ompl::base::State const *,::ompl::base::State const * ) const;
            
            MotionValidator_exposer.def( 
                "checkMotion"
                , bp::pure_virtual( checkMotion_function_type(&::ompl::base::MotionValidator::checkMotion) )
                , ( bp::arg("s1"), bp::arg("s2") ) );
        
        }
        { //::ompl::base::MotionValidator::checkMotion
        
            typedef bool ( ::ompl::base::MotionValidator::*checkMotion_function_type)( ::ompl::base::State const *,::ompl::base::State const *,::std::pair<ompl::base::State*,double> & ) const;
            
            MotionValidator_exposer.def( 
                "checkMotion"
                , bp::pure_virtual( checkMotion_function_type(&::ompl::base::MotionValidator::checkMotion) )
                , ( bp::arg("s1"), bp::arg("s2"), bp::arg("lastValid") ) );
        
        }
        { //::ompl::base::MotionValidator::getInvalidMotionCount
        
            typedef unsigned int ( ::ompl::base::MotionValidator::*getInvalidMotionCount_function_type)(  ) const;
            
            MotionValidator_exposer.def( 
                "getInvalidMotionCount"
                , getInvalidMotionCount_function_type( &::ompl::base::MotionValidator::getInvalidMotionCount ) );
        
        }
        { //::ompl::base::MotionValidator::getValidMotionCount
        
            typedef unsigned int ( ::ompl::base::MotionValidator::*getValidMotionCount_function_type)(  ) const;
            
            MotionValidator_exposer.def( 
                "getValidMotionCount"
                , getValidMotionCount_function_type( &::ompl::base::MotionValidator::getValidMotionCount ) );
        
        }
        { //::ompl::base::MotionValidator::getValidMotionFraction
        
            typedef double ( ::ompl::base::MotionValidator::*getValidMotionFraction_function_type)(  ) const;
            
            MotionValidator_exposer.def( 
                "getValidMotionFraction"
                , getValidMotionFraction_function_type( &::ompl::base::MotionValidator::getValidMotionFraction ) );
        
        }
        { //::ompl::base::MotionValidator::resetMotionCounter
        
            typedef void ( ::ompl::base::MotionValidator::*resetMotionCounter_function_type)(  ) ;
            
            MotionValidator_exposer.def( 
                "resetMotionCounter"
                , resetMotionCounter_function_type( &::ompl::base::MotionValidator::resetMotionCounter ) );
        
        }
        bp::register_ptr_to_python< boost::shared_ptr< ompl::base::MotionValidator > >();
    }

}
