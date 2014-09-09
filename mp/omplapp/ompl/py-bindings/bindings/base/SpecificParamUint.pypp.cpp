// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/base.h"
#include "SpecificParamUint.pypp.hpp"

namespace bp = boost::python;

struct SpecificParam_less__unsigned_int__greater__wrapper : ompl::base::SpecificParam< unsigned int >, bp::wrapper< ompl::base::SpecificParam< unsigned int > > {

    SpecificParam_less__unsigned_int__greater__wrapper(ompl::base::SpecificParam<unsigned int> const & arg )
    : ompl::base::SpecificParam<unsigned int>( arg )
      , bp::wrapper< ompl::base::SpecificParam< unsigned int > >(){
        // copy constructor
        
    }

    virtual ::std::string getValue(  ) const  {
        if( bp::override func_getValue = this->get_override( "getValue" ) )
            return func_getValue(  );
        else{
            return this->ompl::base::SpecificParam< unsigned int >::getValue(  );
        }
    }
    
    ::std::string default_getValue(  ) const  {
        return ompl::base::SpecificParam< unsigned int >::getValue( );
    }

    virtual bool setValue( ::std::string const & value ) {
        if( bp::override func_setValue = this->get_override( "setValue" ) )
            return func_setValue( value );
        else{
            return this->ompl::base::SpecificParam< unsigned int >::setValue( value );
        }
    }
    
    bool default_setValue( ::std::string const & value ) {
        return ompl::base::SpecificParam< unsigned int >::setValue( value );
    }

};

void register_SpecificParamUint_class(){

    { //::ompl::base::SpecificParam< unsigned int >
        typedef bp::class_< SpecificParam_less__unsigned_int__greater__wrapper, bp::bases< ompl::base::GenericParam > > SpecificParamUint_exposer_t;
        SpecificParamUint_exposer_t SpecificParamUint_exposer = SpecificParamUint_exposer_t( "SpecificParamUint", bp::no_init );
        bp::scope SpecificParamUint_scope( SpecificParamUint_exposer );
        { //::ompl::base::SpecificParam< unsigned int >::getValue
        
            typedef ompl::base::SpecificParam< unsigned int > exported_class_t;
            typedef ::std::string ( exported_class_t::*getValue_function_type)(  ) const;
            typedef ::std::string ( SpecificParam_less__unsigned_int__greater__wrapper::*default_getValue_function_type)(  ) const;
            
            SpecificParamUint_exposer.def( 
                "getValue"
                , getValue_function_type(&::ompl::base::SpecificParam< unsigned int >::getValue)
                , default_getValue_function_type(&SpecificParam_less__unsigned_int__greater__wrapper::default_getValue) );
        
        }
        { //::ompl::base::SpecificParam< unsigned int >::setValue
        
            typedef ompl::base::SpecificParam< unsigned int > exported_class_t;
            typedef bool ( exported_class_t::*setValue_function_type)( ::std::string const & ) ;
            typedef bool ( SpecificParam_less__unsigned_int__greater__wrapper::*default_setValue_function_type)( ::std::string const & ) ;
            
            SpecificParamUint_exposer.def( 
                "setValue"
                , setValue_function_type(&::ompl::base::SpecificParam< unsigned int >::setValue)
                , default_setValue_function_type(&SpecificParam_less__unsigned_int__greater__wrapper::default_setValue)
                , ( bp::arg("value") ) );
        
        }
    }

}