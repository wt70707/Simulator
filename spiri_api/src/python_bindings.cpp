#include <staterobot.h>
#include <iostream>


#include <boost/python.hpp>
using namespace boost::python;

BOOST_PYTHON_MODULE(Staterobot)
{
    class_<Staterobot,boost::noncopyable>("Staterobot")


    ;
}
