// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/control.h"
#include "EST.pypp.hpp"

namespace bp = boost::python;

struct ControlEST_wrapper : ompl::control::EST, bp::wrapper< ompl::control::EST > {

    ControlEST_wrapper(::ompl::control::SpaceInformationPtr const & si )
    : ompl::control::EST( si )
      , bp::wrapper< ompl::control::EST >(){
        // constructor
    
    }

    virtual void clear(  ) {
        if( bp::override func_clear = this->get_override( "clear" ) )
            func_clear(  );
        else{
            this->ompl::control::EST::clear(  );
        }
    }
    
    void default_clear(  ) {
        ompl::control::EST::clear( );
    }

    void freeMemory(  ){
        ompl::control::EST::freeMemory(  );
    }

    virtual void getPlannerData( ::ompl::base::PlannerData & data ) const  {
        if( bp::override func_getPlannerData = this->get_override( "getPlannerData" ) )
            func_getPlannerData( boost::ref(data) );
        else{
            this->ompl::control::EST::getPlannerData( boost::ref(data) );
        }
    }
    
    void default_getPlannerData( ::ompl::base::PlannerData & data ) const  {
        ompl::control::EST::getPlannerData( boost::ref(data) );
    }

    virtual void setup(  ) {
        if( bp::override func_setup = this->get_override( "setup" ) )
            func_setup(  );
        else{
            this->ompl::control::EST::setup(  );
        }
    }
    
    void default_setup(  ) {
        ompl::control::EST::setup( );
    }

    virtual ::ompl::base::PlannerStatus solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
        if( bp::override func_solve = this->get_override( "solve" ) )
            return func_solve( boost::ref(ptc) );
        else{
            return this->ompl::control::EST::solve( boost::ref(ptc) );
        }
    }
    
    ::ompl::base::PlannerStatus default_solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
        return ompl::control::EST::solve( boost::ref(ptc) );
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

void register_EST_class(){

    { //::ompl::control::EST
        typedef bp::class_< ControlEST_wrapper, bp::bases< ::ompl::base::Planner >, boost::noncopyable > EST_exposer_t;
        EST_exposer_t EST_exposer = EST_exposer_t( "EST", bp::init< ompl::control::SpaceInformationPtr const & >(( bp::arg("si") )) );
        bp::scope EST_scope( EST_exposer );
        bp::implicitly_convertible< ompl::control::SpaceInformationPtr const &, ompl::control::EST >();
        { //::ompl::control::EST::clear
        
            typedef void ( ::ompl::control::EST::*clear_function_type)(  ) ;
            typedef void ( ControlEST_wrapper::*default_clear_function_type)(  ) ;
            
            EST_exposer.def( 
                "clear"
                , clear_function_type(&::ompl::control::EST::clear)
                , default_clear_function_type(&ControlEST_wrapper::default_clear) );
        
        }
        { //::ompl::control::EST::freeMemory
        
            typedef void ( ControlEST_wrapper::*freeMemory_function_type)(  ) ;
            
            EST_exposer.def( 
                "freeMemory"
                , freeMemory_function_type( &ControlEST_wrapper::freeMemory ) );
        
        }
        { //::ompl::control::EST::getGoalBias
        
            typedef double ( ::ompl::control::EST::*getGoalBias_function_type)(  ) const;
            
            EST_exposer.def( 
                "getGoalBias"
                , getGoalBias_function_type( &::ompl::control::EST::getGoalBias ) );
        
        }
        { //::ompl::control::EST::getPlannerData
        
            typedef void ( ::ompl::control::EST::*getPlannerData_function_type)( ::ompl::base::PlannerData & ) const;
            typedef void ( ControlEST_wrapper::*default_getPlannerData_function_type)( ::ompl::base::PlannerData & ) const;
            
            EST_exposer.def( 
                "getPlannerData"
                , getPlannerData_function_type(&::ompl::control::EST::getPlannerData)
                , default_getPlannerData_function_type(&ControlEST_wrapper::default_getPlannerData)
                , ( bp::arg("data") ) );
        
        }
        { //::ompl::control::EST::getProjectionEvaluator
        
            typedef ::ompl::base::ProjectionEvaluatorPtr const & ( ::ompl::control::EST::*getProjectionEvaluator_function_type)(  ) const;
            
            EST_exposer.def( 
                "getProjectionEvaluator"
                , getProjectionEvaluator_function_type( &::ompl::control::EST::getProjectionEvaluator )
                , bp::return_value_policy< bp::copy_const_reference >() );
        
        }
        { //::ompl::control::EST::getRange
        
            typedef double ( ::ompl::control::EST::*getRange_function_type)(  ) const;
            
            EST_exposer.def( 
                "getRange"
                , getRange_function_type( &::ompl::control::EST::getRange ) );
        
        }
        { //::ompl::control::EST::setGoalBias
        
            typedef void ( ::ompl::control::EST::*setGoalBias_function_type)( double ) ;
            
            EST_exposer.def( 
                "setGoalBias"
                , setGoalBias_function_type( &::ompl::control::EST::setGoalBias )
                , ( bp::arg("goalBias") ) );
        
        }
        { //::ompl::control::EST::setProjectionEvaluator
        
            typedef void ( ::ompl::control::EST::*setProjectionEvaluator_function_type)( ::ompl::base::ProjectionEvaluatorPtr const & ) ;
            
            EST_exposer.def( 
                "setProjectionEvaluator"
                , setProjectionEvaluator_function_type( &::ompl::control::EST::setProjectionEvaluator )
                , ( bp::arg("projectionEvaluator") ) );
        
        }
        { //::ompl::control::EST::setProjectionEvaluator
        
            typedef void ( ::ompl::control::EST::*setProjectionEvaluator_function_type)( ::std::string const & ) ;
            
            EST_exposer.def( 
                "setProjectionEvaluator"
                , setProjectionEvaluator_function_type( &::ompl::control::EST::setProjectionEvaluator )
                , ( bp::arg("name") ) );
        
        }
        { //::ompl::control::EST::setRange
        
            typedef void ( ::ompl::control::EST::*setRange_function_type)( double ) ;
            
            EST_exposer.def( 
                "setRange"
                , setRange_function_type( &::ompl::control::EST::setRange )
                , ( bp::arg("distance") ) );
        
        }
        { //::ompl::control::EST::setup
        
            typedef void ( ::ompl::control::EST::*setup_function_type)(  ) ;
            typedef void ( ControlEST_wrapper::*default_setup_function_type)(  ) ;
            
            EST_exposer.def( 
                "setup"
                , setup_function_type(&::ompl::control::EST::setup)
                , default_setup_function_type(&ControlEST_wrapper::default_setup) );
        
        }
        { //::ompl::control::EST::solve
        
            typedef ::ompl::base::PlannerStatus ( ::ompl::control::EST::*solve_function_type)( ::ompl::base::PlannerTerminationCondition const & ) ;
            typedef ::ompl::base::PlannerStatus ( ControlEST_wrapper::*default_solve_function_type)( ::ompl::base::PlannerTerminationCondition const & ) ;
            
            EST_exposer.def( 
                "solve"
                , solve_function_type(&::ompl::control::EST::solve)
                , default_solve_function_type(&ControlEST_wrapper::default_solve)
                , ( bp::arg("ptc") ) );
        
        }
        EST_exposer.def("solve", (::ompl::base::PlannerStatus(::ompl::base::Planner::*)( double ))(&::ompl::base::Planner::solve), (bp::arg("solveTime")) );
        EST_exposer.def("setProblemDefinition",&::ompl::base::Planner::setProblemDefinition,
                            &ControlEST_wrapper::default_setProblemDefinition, (bp::arg("pdef")) );
        EST_exposer.def("checkValidity",&::ompl::base::Planner::checkValidity,
                            &ControlEST_wrapper::default_checkValidity );
    }

}
