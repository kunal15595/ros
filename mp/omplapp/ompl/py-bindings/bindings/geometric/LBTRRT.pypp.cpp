// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/geometric.h"
#include "LBTRRT.pypp.hpp"

namespace bp = boost::python;

struct LBTRRT_wrapper : ompl::geometric::LBTRRT, bp::wrapper< ompl::geometric::LBTRRT > {

    LBTRRT_wrapper(::ompl::base::SpaceInformationPtr const & si )
    : ompl::geometric::LBTRRT( si )
      , bp::wrapper< ompl::geometric::LBTRRT >(){
        // constructor
    
    }

    virtual void clear(  ) {
        if( bp::override func_clear = this->get_override( "clear" ) )
            func_clear(  );
        else{
            this->ompl::geometric::LBTRRT::clear(  );
        }
    }
    
    void default_clear(  ) {
        ompl::geometric::LBTRRT::clear( );
    }

    void freeMemory(  ){
        ompl::geometric::LBTRRT::freeMemory(  );
    }

    virtual void getPlannerData( ::ompl::base::PlannerData & data ) const  {
        if( bp::override func_getPlannerData = this->get_override( "getPlannerData" ) )
            func_getPlannerData( boost::ref(data) );
        else{
            this->ompl::geometric::LBTRRT::getPlannerData( boost::ref(data) );
        }
    }
    
    void default_getPlannerData( ::ompl::base::PlannerData & data ) const  {
        ompl::geometric::LBTRRT::getPlannerData( boost::ref(data) );
    }

    virtual void setup(  ) {
        if( bp::override func_setup = this->get_override( "setup" ) )
            func_setup(  );
        else{
            this->ompl::geometric::LBTRRT::setup(  );
        }
    }
    
    void default_setup(  ) {
        ompl::geometric::LBTRRT::setup( );
    }

    virtual ::ompl::base::PlannerStatus solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
        if( bp::override func_solve = this->get_override( "solve" ) )
            return func_solve( boost::ref(ptc) );
        else{
            return this->ompl::geometric::LBTRRT::solve( boost::ref(ptc) );
        }
    }
    
    ::ompl::base::PlannerStatus default_solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
        return ompl::geometric::LBTRRT::solve( boost::ref(ptc) );
    }

    virtual void checkValidity(  ) {
        if( bp::override func_checkValidity = this->get_override( "checkValidity" ) )
            func_checkValidity(  );
        else{
            this->ompl::base::Planner::checkValidity(  );
        }
    }
    
    void default_checkValidity(  ) {
        ompl::base::Planner::checkValidity( );
    }

    virtual void setProblemDefinition( ::ompl::base::ProblemDefinitionPtr const & pdef ) {
        if( bp::override func_setProblemDefinition = this->get_override( "setProblemDefinition" ) )
            func_setProblemDefinition( pdef );
        else{
            this->ompl::base::Planner::setProblemDefinition( pdef );
        }
    }
    
    void default_setProblemDefinition( ::ompl::base::ProblemDefinitionPtr const & pdef ) {
        ompl::base::Planner::setProblemDefinition( pdef );
    }

};

void register_LBTRRT_class(){

    { //::ompl::geometric::LBTRRT
        typedef bp::class_< LBTRRT_wrapper, bp::bases< ::ompl::base::Planner >, boost::noncopyable > LBTRRT_exposer_t;
        LBTRRT_exposer_t LBTRRT_exposer = LBTRRT_exposer_t( "LBTRRT", bp::init< ompl::base::SpaceInformationPtr const & >(( bp::arg("si") )) );
        bp::scope LBTRRT_scope( LBTRRT_exposer );
        bp::implicitly_convertible< ompl::base::SpaceInformationPtr const &, ompl::geometric::LBTRRT >();
        { //::ompl::geometric::LBTRRT::clear
        
            typedef void ( ::ompl::geometric::LBTRRT::*clear_function_type)(  ) ;
            typedef void ( LBTRRT_wrapper::*default_clear_function_type)(  ) ;
            
            LBTRRT_exposer.def( 
                "clear"
                , clear_function_type(&::ompl::geometric::LBTRRT::clear)
                , default_clear_function_type(&LBTRRT_wrapper::default_clear) );
        
        }
        { //::ompl::geometric::LBTRRT::freeMemory
        
            typedef void ( LBTRRT_wrapper::*freeMemory_function_type)(  ) ;
            
            LBTRRT_exposer.def( 
                "freeMemory"
                , freeMemory_function_type( &LBTRRT_wrapper::freeMemory ) );
        
        }
        { //::ompl::geometric::LBTRRT::getApproximationFactor
        
            typedef double ( ::ompl::geometric::LBTRRT::*getApproximationFactor_function_type)(  ) const;
            
            LBTRRT_exposer.def( 
                "getApproximationFactor"
                , getApproximationFactor_function_type( &::ompl::geometric::LBTRRT::getApproximationFactor ) );
        
        }
        { //::ompl::geometric::LBTRRT::getGoalBias
        
            typedef double ( ::ompl::geometric::LBTRRT::*getGoalBias_function_type)(  ) const;
            
            LBTRRT_exposer.def( 
                "getGoalBias"
                , getGoalBias_function_type( &::ompl::geometric::LBTRRT::getGoalBias ) );
        
        }
        { //::ompl::geometric::LBTRRT::getPlannerData
        
            typedef void ( ::ompl::geometric::LBTRRT::*getPlannerData_function_type)( ::ompl::base::PlannerData & ) const;
            typedef void ( LBTRRT_wrapper::*default_getPlannerData_function_type)( ::ompl::base::PlannerData & ) const;
            
            LBTRRT_exposer.def( 
                "getPlannerData"
                , getPlannerData_function_type(&::ompl::geometric::LBTRRT::getPlannerData)
                , default_getPlannerData_function_type(&LBTRRT_wrapper::default_getPlannerData)
                , ( bp::arg("data") ) );
        
        }
        { //::ompl::geometric::LBTRRT::getRange
        
            typedef double ( ::ompl::geometric::LBTRRT::*getRange_function_type)(  ) const;
            
            LBTRRT_exposer.def( 
                "getRange"
                , getRange_function_type( &::ompl::geometric::LBTRRT::getRange ) );
        
        }
        { //::ompl::geometric::LBTRRT::setApproximationFactor
        
            typedef void ( ::ompl::geometric::LBTRRT::*setApproximationFactor_function_type)( double ) ;
            
            LBTRRT_exposer.def( 
                "setApproximationFactor"
                , setApproximationFactor_function_type( &::ompl::geometric::LBTRRT::setApproximationFactor )
                , ( bp::arg("epsilon") ) );
        
        }
        { //::ompl::geometric::LBTRRT::setGoalBias
        
            typedef void ( ::ompl::geometric::LBTRRT::*setGoalBias_function_type)( double ) ;
            
            LBTRRT_exposer.def( 
                "setGoalBias"
                , setGoalBias_function_type( &::ompl::geometric::LBTRRT::setGoalBias )
                , ( bp::arg("goalBias") ) );
        
        }
        { //::ompl::geometric::LBTRRT::setRange
        
            typedef void ( ::ompl::geometric::LBTRRT::*setRange_function_type)( double ) ;
            
            LBTRRT_exposer.def( 
                "setRange"
                , setRange_function_type( &::ompl::geometric::LBTRRT::setRange )
                , ( bp::arg("distance") ) );
        
        }
        { //::ompl::geometric::LBTRRT::setup
        
            typedef void ( ::ompl::geometric::LBTRRT::*setup_function_type)(  ) ;
            typedef void ( LBTRRT_wrapper::*default_setup_function_type)(  ) ;
            
            LBTRRT_exposer.def( 
                "setup"
                , setup_function_type(&::ompl::geometric::LBTRRT::setup)
                , default_setup_function_type(&LBTRRT_wrapper::default_setup) );
        
        }
        { //::ompl::geometric::LBTRRT::solve
        
            typedef ::ompl::base::PlannerStatus ( ::ompl::geometric::LBTRRT::*solve_function_type)( ::ompl::base::PlannerTerminationCondition const & ) ;
            typedef ::ompl::base::PlannerStatus ( LBTRRT_wrapper::*default_solve_function_type)( ::ompl::base::PlannerTerminationCondition const & ) ;
            
            LBTRRT_exposer.def( 
                "solve"
                , solve_function_type(&::ompl::geometric::LBTRRT::solve)
                , default_solve_function_type(&LBTRRT_wrapper::default_solve)
                , ( bp::arg("ptc") ) );
        
        }
        LBTRRT_exposer.def("solve", (::ompl::base::PlannerStatus(::ompl::base::Planner::*)( double ))(&::ompl::base::Planner::solve), (bp::arg("solveTime")) );
        LBTRRT_exposer.def("setProblemDefinition",&::ompl::base::Planner::setProblemDefinition,
                            &LBTRRT_wrapper::default_setProblemDefinition, (bp::arg("pdef")) );
        LBTRRT_exposer.def("checkValidity",&::ompl::base::Planner::checkValidity,
                        &LBTRRT_wrapper::default_checkValidity );
    }

}