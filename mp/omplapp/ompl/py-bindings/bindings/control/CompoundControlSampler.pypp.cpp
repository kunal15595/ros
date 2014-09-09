// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/control.h"
#include "CompoundControlSampler.pypp.hpp"

namespace bp = boost::python;

struct CompoundControlSampler_wrapper : ompl::control::CompoundControlSampler, bp::wrapper< ompl::control::CompoundControlSampler > {

    CompoundControlSampler_wrapper(::ompl::control::ControlSpace const * space )
    : ompl::control::CompoundControlSampler( boost::python::ptr(space) )
      , bp::wrapper< ompl::control::CompoundControlSampler >(){
        // constructor
    
    }

    virtual void addSampler( ::ompl::control::ControlSamplerPtr const & sampler ) {
        if( bp::override func_addSampler = this->get_override( "addSampler" ) )
            func_addSampler( sampler );
        else{
            this->ompl::control::CompoundControlSampler::addSampler( sampler );
        }
    }
    
    void default_addSampler( ::ompl::control::ControlSamplerPtr const & sampler ) {
        ompl::control::CompoundControlSampler::addSampler( sampler );
    }

    virtual void sample( ::ompl::control::Control * control ) {
        if( bp::override func_sample = this->get_override( "sample" ) )
            func_sample( boost::python::ptr(control) );
        else{
            this->ompl::control::CompoundControlSampler::sample( boost::python::ptr(control) );
        }
    }
    
    void default_sample( ::ompl::control::Control * control ) {
        ompl::control::CompoundControlSampler::sample( boost::python::ptr(control) );
    }

    virtual void sample( ::ompl::control::Control * control, ::ompl::base::State const * state ) {
        if( bp::override func_sample = this->get_override( "sample" ) )
            func_sample( boost::python::ptr(control), boost::python::ptr(state) );
        else{
            this->ompl::control::CompoundControlSampler::sample( boost::python::ptr(control), boost::python::ptr(state) );
        }
    }
    
    void default_sample( ::ompl::control::Control * control, ::ompl::base::State const * state ) {
        ompl::control::CompoundControlSampler::sample( boost::python::ptr(control), boost::python::ptr(state) );
    }

    virtual void sampleNext( ::ompl::control::Control * control, ::ompl::control::Control const * previous ) {
        if( bp::override func_sampleNext = this->get_override( "sampleNext" ) )
            func_sampleNext( boost::python::ptr(control), boost::python::ptr(previous) );
        else{
            this->ompl::control::CompoundControlSampler::sampleNext( boost::python::ptr(control), boost::python::ptr(previous) );
        }
    }
    
    void default_sampleNext( ::ompl::control::Control * control, ::ompl::control::Control const * previous ) {
        ompl::control::CompoundControlSampler::sampleNext( boost::python::ptr(control), boost::python::ptr(previous) );
    }

    virtual void sampleNext( ::ompl::control::Control * control, ::ompl::control::Control const * previous, ::ompl::base::State const * state ) {
        if( bp::override func_sampleNext = this->get_override( "sampleNext" ) )
            func_sampleNext( boost::python::ptr(control), boost::python::ptr(previous), boost::python::ptr(state) );
        else{
            this->ompl::control::CompoundControlSampler::sampleNext( boost::python::ptr(control), boost::python::ptr(previous), boost::python::ptr(state) );
        }
    }
    
    void default_sampleNext( ::ompl::control::Control * control, ::ompl::control::Control const * previous, ::ompl::base::State const * state ) {
        ompl::control::CompoundControlSampler::sampleNext( boost::python::ptr(control), boost::python::ptr(previous), boost::python::ptr(state) );
    }

    virtual unsigned int sampleStepCount( unsigned int minSteps, unsigned int maxSteps ) {
        if( bp::override func_sampleStepCount = this->get_override( "sampleStepCount" ) )
            return func_sampleStepCount( minSteps, maxSteps );
        else{
            return this->ompl::control::ControlSampler::sampleStepCount( minSteps, maxSteps );
        }
    }
    
    unsigned int default_sampleStepCount( unsigned int minSteps, unsigned int maxSteps ) {
        return ompl::control::ControlSampler::sampleStepCount( minSteps, maxSteps );
    }

};

void register_CompoundControlSampler_class(){

    { //::ompl::control::CompoundControlSampler
        typedef bp::class_< CompoundControlSampler_wrapper, bp::bases< ompl::control::ControlSampler >, boost::noncopyable > CompoundControlSampler_exposer_t;
        CompoundControlSampler_exposer_t CompoundControlSampler_exposer = CompoundControlSampler_exposer_t( "CompoundControlSampler", bp::init< ompl::control::ControlSpace const * >(( bp::arg("space") )) );
        bp::scope CompoundControlSampler_scope( CompoundControlSampler_exposer );
        bp::implicitly_convertible< ompl::control::ControlSpace const *, ompl::control::CompoundControlSampler >();
        { //::ompl::control::CompoundControlSampler::addSampler
        
            typedef void ( ::ompl::control::CompoundControlSampler::*addSampler_function_type)( ::ompl::control::ControlSamplerPtr const & ) ;
            typedef void ( CompoundControlSampler_wrapper::*default_addSampler_function_type)( ::ompl::control::ControlSamplerPtr const & ) ;
            
            CompoundControlSampler_exposer.def( 
                "addSampler"
                , addSampler_function_type(&::ompl::control::CompoundControlSampler::addSampler)
                , default_addSampler_function_type(&CompoundControlSampler_wrapper::default_addSampler)
                , ( bp::arg("sampler") ) );
        
        }
        { //::ompl::control::CompoundControlSampler::sample
        
            typedef void ( ::ompl::control::CompoundControlSampler::*sample_function_type)( ::ompl::control::Control * ) ;
            typedef void ( CompoundControlSampler_wrapper::*default_sample_function_type)( ::ompl::control::Control * ) ;
            
            CompoundControlSampler_exposer.def( 
                "sample"
                , sample_function_type(&::ompl::control::CompoundControlSampler::sample)
                , default_sample_function_type(&CompoundControlSampler_wrapper::default_sample)
                , ( bp::arg("control") ) );
        
        }
        { //::ompl::control::CompoundControlSampler::sample
        
            typedef void ( ::ompl::control::CompoundControlSampler::*sample_function_type)( ::ompl::control::Control *,::ompl::base::State const * ) ;
            typedef void ( CompoundControlSampler_wrapper::*default_sample_function_type)( ::ompl::control::Control *,::ompl::base::State const * ) ;
            
            CompoundControlSampler_exposer.def( 
                "sample"
                , sample_function_type(&::ompl::control::CompoundControlSampler::sample)
                , default_sample_function_type(&CompoundControlSampler_wrapper::default_sample)
                , ( bp::arg("control"), bp::arg("state") ) );
        
        }
        { //::ompl::control::CompoundControlSampler::sampleNext
        
            typedef void ( ::ompl::control::CompoundControlSampler::*sampleNext_function_type)( ::ompl::control::Control *,::ompl::control::Control const * ) ;
            typedef void ( CompoundControlSampler_wrapper::*default_sampleNext_function_type)( ::ompl::control::Control *,::ompl::control::Control const * ) ;
            
            CompoundControlSampler_exposer.def( 
                "sampleNext"
                , sampleNext_function_type(&::ompl::control::CompoundControlSampler::sampleNext)
                , default_sampleNext_function_type(&CompoundControlSampler_wrapper::default_sampleNext)
                , ( bp::arg("control"), bp::arg("previous") ) );
        
        }
        { //::ompl::control::CompoundControlSampler::sampleNext
        
            typedef void ( ::ompl::control::CompoundControlSampler::*sampleNext_function_type)( ::ompl::control::Control *,::ompl::control::Control const *,::ompl::base::State const * ) ;
            typedef void ( CompoundControlSampler_wrapper::*default_sampleNext_function_type)( ::ompl::control::Control *,::ompl::control::Control const *,::ompl::base::State const * ) ;
            
            CompoundControlSampler_exposer.def( 
                "sampleNext"
                , sampleNext_function_type(&::ompl::control::CompoundControlSampler::sampleNext)
                , default_sampleNext_function_type(&CompoundControlSampler_wrapper::default_sampleNext)
                , ( bp::arg("control"), bp::arg("previous"), bp::arg("state") ) );
        
        }
        { //::ompl::control::ControlSampler::sampleStepCount
        
            typedef unsigned int ( ::ompl::control::ControlSampler::*sampleStepCount_function_type)( unsigned int,unsigned int ) ;
            typedef unsigned int ( CompoundControlSampler_wrapper::*default_sampleStepCount_function_type)( unsigned int,unsigned int ) ;
            
            CompoundControlSampler_exposer.def( 
                "sampleStepCount"
                , sampleStepCount_function_type(&::ompl::control::ControlSampler::sampleStepCount)
                , default_sampleStepCount_function_type(&CompoundControlSampler_wrapper::default_sampleStepCount)
                , ( bp::arg("minSteps"), bp::arg("maxSteps") ) );
        
        }
    }

}