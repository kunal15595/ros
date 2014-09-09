// This file has been generated by Py++.

#include "boost/python.hpp"
#include "bindings/geometric.h"
#include "NearestNeighbors.pypp.hpp"

namespace bp = boost::python;

struct NearestNeighbors_less__unsigned_long__greater__wrapper : ompl::NearestNeighbors< unsigned long >, bp::wrapper< ompl::NearestNeighbors< unsigned long > > {

    NearestNeighbors_less__unsigned_long__greater__wrapper( )
    : ompl::NearestNeighbors<unsigned long>( )
      , bp::wrapper< ompl::NearestNeighbors< unsigned long > >(){
        // null constructor
    
    }

    virtual void add( long unsigned int const & data ){
        bp::override func_add = this->get_override( "add" );
        func_add( data );
    }

    virtual void add( ::std::vector< unsigned long > const & data ) {
        if( bp::override func_add = this->get_override( "add" ) )
            func_add( boost::ref(data) );
        else{
            this->ompl::NearestNeighbors< unsigned long >::add( boost::ref(data) );
        }
    }
    
    void default_add( ::std::vector< unsigned long > const & data ) {
        ompl::NearestNeighbors< unsigned long >::add( boost::ref(data) );
    }

    virtual void clear(  ){
        bp::override func_clear = this->get_override( "clear" );
        func_clear(  );
    }

    virtual void list( ::std::vector< unsigned long > & data ) const {
        bp::override func_list = this->get_override( "list" );
        func_list( boost::ref(data) );
    }

    virtual long unsigned int nearest( long unsigned int const & data ) const {
        bp::override func_nearest = this->get_override( "nearest" );
        return func_nearest( data );
    }

    virtual void nearestK( long unsigned int const & data, ::std::size_t k, ::std::vector< unsigned long > & nbh ) const {
        bp::override func_nearestK = this->get_override( "nearestK" );
        func_nearestK( data, k, boost::ref(nbh) );
    }

    virtual void nearestR( long unsigned int const & data, double radius, ::std::vector< unsigned long > & nbh ) const {
        bp::override func_nearestR = this->get_override( "nearestR" );
        func_nearestR( data, radius, boost::ref(nbh) );
    }

    virtual bool remove( long unsigned int const & data ){
        bp::override func_remove = this->get_override( "remove" );
        return func_remove( data );
    }

    virtual void setDistanceFunction( ::boost::function< double (unsigned long const&, unsigned long const&) > const & distFun ) {
        if( bp::override func_setDistanceFunction = this->get_override( "setDistanceFunction" ) )
            func_setDistanceFunction( boost::ref(distFun) );
        else{
            this->ompl::NearestNeighbors< unsigned long >::setDistanceFunction( boost::ref(distFun) );
        }
    }
    
    void default_setDistanceFunction( ::boost::function< double (unsigned long const&, unsigned long const&) > const & distFun ) {
        ompl::NearestNeighbors< unsigned long >::setDistanceFunction( boost::ref(distFun) );
    }

    virtual ::std::size_t size(  ) const {
        bp::override func_size = this->get_override( "size" );
        return func_size(  );
    }

};

void register_NearestNeighbors_class(){

    { //::ompl::NearestNeighbors< unsigned long >
        typedef bp::class_< NearestNeighbors_less__unsigned_long__greater__wrapper, boost::noncopyable > NearestNeighbors_exposer_t;
        NearestNeighbors_exposer_t NearestNeighbors_exposer = NearestNeighbors_exposer_t( "NearestNeighbors", bp::init< >() );
        bp::scope NearestNeighbors_scope( NearestNeighbors_exposer );
        { //::ompl::NearestNeighbors< unsigned long >::add
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef void ( exported_class_t::*add_function_type)( long unsigned int const & ) ;
            
            NearestNeighbors_exposer.def( 
                "add"
                , bp::pure_virtual( add_function_type(&::ompl::NearestNeighbors< unsigned long >::add) )
                , ( bp::arg("data") ) );
        
        }
        { //::ompl::NearestNeighbors< unsigned long >::add
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef void ( exported_class_t::*add_function_type)( ::std::vector< unsigned long > const & ) ;
            typedef void ( NearestNeighbors_less__unsigned_long__greater__wrapper::*default_add_function_type)( ::std::vector< unsigned long > const & ) ;
            
            NearestNeighbors_exposer.def( 
                "add"
                , add_function_type(&::ompl::NearestNeighbors< unsigned long >::add)
                , default_add_function_type(&NearestNeighbors_less__unsigned_long__greater__wrapper::default_add)
                , ( bp::arg("data") ) );
        
        }
        { //::ompl::NearestNeighbors< unsigned long >::clear
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef void ( exported_class_t::*clear_function_type)(  ) ;
            
            NearestNeighbors_exposer.def( 
                "clear"
                , bp::pure_virtual( clear_function_type(&::ompl::NearestNeighbors< unsigned long >::clear) ) );
        
        }
        { //::ompl::NearestNeighbors< unsigned long >::getDistanceFunction
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef ::boost::function< double (unsigned long const&, unsigned long const&) > const & ( exported_class_t::*getDistanceFunction_function_type)(  ) const;
            
            NearestNeighbors_exposer.def( 
                "getDistanceFunction"
                , getDistanceFunction_function_type( &::ompl::NearestNeighbors< unsigned long >::getDistanceFunction )
                , bp::return_value_policy< bp::copy_const_reference >() );
        
        }
        { //::ompl::NearestNeighbors< unsigned long >::list
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef void ( exported_class_t::*list_function_type)( ::std::vector<unsigned long, std::allocator<unsigned long> > & ) const;
            
            NearestNeighbors_exposer.def( 
                "list"
                , bp::pure_virtual( list_function_type(&::ompl::NearestNeighbors< unsigned long >::list) )
                , ( bp::arg("data") ) );
        
        }
        { //::ompl::NearestNeighbors< unsigned long >::nearest
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef long unsigned int ( exported_class_t::*nearest_function_type)( long unsigned int const & ) const;
            
            NearestNeighbors_exposer.def( 
                "nearest"
                , bp::pure_virtual( nearest_function_type(&::ompl::NearestNeighbors< unsigned long >::nearest) )
                , ( bp::arg("data") ) );
        
        }
        { //::ompl::NearestNeighbors< unsigned long >::nearestK
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef void ( exported_class_t::*nearestK_function_type)( long unsigned int const &,::std::size_t,::std::vector<unsigned long, std::allocator<unsigned long> > & ) const;
            
            NearestNeighbors_exposer.def( 
                "nearestK"
                , bp::pure_virtual( nearestK_function_type(&::ompl::NearestNeighbors< unsigned long >::nearestK) )
                , ( bp::arg("data"), bp::arg("k"), bp::arg("nbh") ) );
        
        }
        { //::ompl::NearestNeighbors< unsigned long >::nearestR
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef void ( exported_class_t::*nearestR_function_type)( long unsigned int const &,double,::std::vector<unsigned long, std::allocator<unsigned long> > & ) const;
            
            NearestNeighbors_exposer.def( 
                "nearestR"
                , bp::pure_virtual( nearestR_function_type(&::ompl::NearestNeighbors< unsigned long >::nearestR) )
                , ( bp::arg("data"), bp::arg("radius"), bp::arg("nbh") ) );
        
        }
        { //::ompl::NearestNeighbors< unsigned long >::remove
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef bool ( exported_class_t::*remove_function_type)( long unsigned int const & ) ;
            
            NearestNeighbors_exposer.def( 
                "remove"
                , bp::pure_virtual( remove_function_type(&::ompl::NearestNeighbors< unsigned long >::remove) )
                , ( bp::arg("data") ) );
        
        }
        { //::ompl::NearestNeighbors< unsigned long >::setDistanceFunction
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef void ( exported_class_t::*setDistanceFunction_function_type)( ::boost::function< double (unsigned long const&, unsigned long const&) > const & ) ;
            typedef void ( NearestNeighbors_less__unsigned_long__greater__wrapper::*default_setDistanceFunction_function_type)( ::boost::function< double (unsigned long const&, unsigned long const&) > const & ) ;
            
            NearestNeighbors_exposer.def( 
                "setDistanceFunction"
                , setDistanceFunction_function_type(&::ompl::NearestNeighbors< unsigned long >::setDistanceFunction)
                , default_setDistanceFunction_function_type(&NearestNeighbors_less__unsigned_long__greater__wrapper::default_setDistanceFunction)
                , ( bp::arg("distFun") ) );
        
        }
        { //::ompl::NearestNeighbors< unsigned long >::size
        
            typedef ompl::NearestNeighbors< unsigned long > exported_class_t;
            typedef ::std::size_t ( exported_class_t::*size_function_type)(  ) const;
            
            NearestNeighbors_exposer.def( 
                "size"
                , bp::pure_virtual( size_function_type(&::ompl::NearestNeighbors< unsigned long >::size) ) );
        
        }
        bp::register_ptr_to_python< boost::shared_ptr< ompl::NearestNeighbors<unsigned long> > >();
    }

}