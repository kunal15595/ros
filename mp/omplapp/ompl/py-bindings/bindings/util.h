/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_UTIL_EXCEPTION_
#define OMPL_UTIL_EXCEPTION_

#include <stdexcept>
#include <string>

namespace ompl
{

    /** \brief The exception type for ompl */
    class Exception : public std::runtime_error
    {
    public:

        /** \brief This is just a wrapper on std::runtime_error */
        explicit
        Exception(const std::string& what) : std::runtime_error(what)
        {
        }

        /** \brief This is just a wrapper on std::runtime_error with a
            prefix added */
        Exception(const std::string &prefix, const std::string& what) : std::runtime_error(prefix + ": " + what)
        {
        }

        virtual ~Exception() throw()
        {
        }

    };

}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_UTIL_RANDOM_NUMBERS_
#define OMPL_UTIL_RANDOM_NUMBERS_

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <cassert>

namespace ompl
{
    /** \brief Random number generation. An instance of this class
        cannot be used by multiple threads at once (member functions
        are not const). However, the constructor is thread safe and
        different instances can be used safely in any number of
        threads. It is also guaranteed that all created instances will
        have a different random seed. */
    class RNG
    {
    public:

        /** \brief Constructor. Always sets a different random seed */
        RNG();

        /** \brief Generate a random real between 0 and 1 */
        double uniform01()
        {
            return uni_();
        }

        /** \brief Generate a random real within given bounds: [\e lower_bound, \e upper_bound) */
        double uniformReal(double lower_bound, double upper_bound)
        {
            assert(lower_bound <= upper_bound);
            return (upper_bound - lower_bound) * uni_() + lower_bound;
        }

        /** \brief Generate a random integer within given bounds: [\e lower_bound, \e upper_bound] */
        int uniformInt(int lower_bound, int upper_bound)
        {
            int r = (int)floor(uniformReal((double)lower_bound, (double)(upper_bound) + 1.0));
            return (r > upper_bound) ? upper_bound : r;
        }

        /** \brief Generate a random boolean */
        bool uniformBool()
        {
            return uni_() <= 0.5;
        }

        /** \brief Generate a random real using a normal distribution with mean 0 and variance 1 */
        double gaussian01()
        {
            return normal_();
        }

        /** \brief Generate a random real using a normal distribution with given mean and variance */
        double gaussian(double mean, double stddev)
        {
            return normal_() * stddev + mean;
        }

        /** \brief Generate a random real using a half-normal distribution. The value is within specified bounds [\e
            r_min, \e r_max], but with a bias towards \e r_max. The function is implemended using a Gaussian distribution with
            mean at \e r_max - \e r_min. The distribution is 'folded' around \e r_max axis towards \e r_min.
            The variance of the distribution is (\e r_max - \e r_min) / \e focus. The higher the focus,
            the more probable it is that generated numbers are close to \e r_max. */
        double halfNormalReal(double r_min, double r_max, double focus = 3.0);

        /** \brief Generate a random integer using a half-normal
            distribution. The value is within specified bounds ([\e r_min, \e r_max]), but
            with a bias towards \e r_max. The function is implemented on top of halfNormalReal() */
        int    halfNormalInt(int r_min, int r_max, double focus = 3.0);

        /** \brief Uniform random unit quaternion sampling. The computed value has the order (x,y,z,w) */
        void   quaternion(double value[4]);

        /** \brief Uniform random sampling of Euler roll-pitch-yaw angles, each in the range (-pi, pi]. The computed value has the order (roll, pitch, yaw) */
        void   eulerRPY(double value[3]);

        /** \brief Set the seed for random number generation. Use this
            function to ensure the same sequence of random numbers is
            generated. */
        static void setSeed(boost::uint32_t seed);

        /** \brief Get the seed used for random number
            generation. Passing the returned value to setSeed() at a
            subsequent execution of the code will ensure deterministic
            (repeatable) behaviour. Useful for debugging. */
        static boost::uint32_t getSeed();

    private:

        boost::mt19937                                                           generator_;
        boost::uniform_real<>                                                    uniDist_;
        boost::normal_distribution<>                                             normalDist_;
        boost::variate_generator<boost::mt19937&, boost::uniform_real<> >        uni_;
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > normal_;

    };

}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_UTIL_CONSOLE_
#define OMPL_UTIL_CONSOLE_

#include <string>

/** \file Console.h
    \defgroup logging Logging Macros
    \{

    \def OMPL_ERROR(fmt, ...)
    \brief Log a formatted error string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \def OMPL_WARN(fmt, ...)
    \brief Log a formatted warning string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \def OMPL_INFORM(fmt, ...)
    \brief Log a formatted information string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \def OMPL_DEBUG(fmt, ...)
    \brief Log a formatted debugging string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \}
*/
#define OMPL_ERROR(fmt, ...)  ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_ERROR, fmt, ##__VA_ARGS__)

#define OMPL_WARN(fmt, ...)   ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_WARN,  fmt, ##__VA_ARGS__)

#define OMPL_INFORM(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_INFO,  fmt, ##__VA_ARGS__)

#define OMPL_DEBUG(fmt, ...)  ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_DEBUG, fmt, ##__VA_ARGS__)

namespace ompl
{

    /** \brief Message namespace. This contains classes needed to
        output error messages (or logging) from within the library.
        Message logging can be performed with \ref logging "logging macros" */
    namespace msg
    {
        /** \brief The set of priorities for message logging */
        enum LogLevel
        {
            LOG_DEBUG = 0,
            LOG_INFO,
            LOG_WARN,
            LOG_ERROR,
            LOG_NONE
        };

        /** \brief Generic class to handle output from a piece of
            code.

            In order to handle output from the library in different
            ways, an implementation of this class needs to be
            provided. This instance can be set with the useOutputHandler
            function. */
        class OutputHandler
        {
        public:

            OutputHandler()
            {
            }

            virtual ~OutputHandler()
            {
            }

            /** \brief log a message to the output handler with the given text
                and logging level from a specific file and line number */
            virtual void log(const std::string &text, LogLevel level, const char *filename, int line) = 0;
        };

        /** \brief Default implementation of OutputHandler. This sends
            the information to the console. */
        class OutputHandlerSTD : public OutputHandler
        {
        public:

            OutputHandlerSTD() : OutputHandler()
            {
            }

            virtual void log(const std::string &text, LogLevel level, const char *filename, int line);

        };

        /** \brief Implementation of OutputHandler that saves messages in a file. */
        class OutputHandlerFile : public OutputHandler
        {
        public:

            /** \brief The name of the file in which to save the message data */
            OutputHandlerFile(const char *filename);

            virtual ~OutputHandlerFile();

            virtual void log(const std::string &text, LogLevel level, const char *filename, int line);

        private:

            /** \brief The file to save to */
            FILE *file_;

        };

        /** \brief This function instructs ompl that no messages should be outputted. Equivalent to useOutputHandler(NULL) */
        void noOutputHandler();

        /** \brief Restore the output handler that was previously in use (if any) */
        void restorePreviousOutputHandler();

        /** \brief Specify the instance of the OutputHandler to use. By default, this is OutputHandlerSTD */
        void useOutputHandler(OutputHandler *oh);

        /** \brief Get the instance of the OutputHandler currently used. This is NULL in case there is no output handler. */
        OutputHandler* getOutputHandler();

        /** \brief Set the minimum level of logging data to output.  Messages
            with lower logging levels will not be recorded. */
        void setLogLevel(LogLevel level);

        /** \brief Retrieve the current level of logging data.  Messages
            with lower logging levels will not be recorded. */
        LogLevel getLogLevel();

        /** \brief Root level logging function.  This should not be invoked directly,
            but rather used via a \ref logging "logging macro".  Formats the message
            string given the arguments and forwards the string to the output handler */
        void log(const char *file, int line, LogLevel level, const char *m, ...);
    }

}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_UTIL_PPM_
#define OMPL_UTIL_PPM_

#include <vector>

namespace ompl
{

    /** \brief Load and save .ppm files. */
    class PPM
    {
    public:
        struct Color
        {
            unsigned char red, green, blue;

            // needed for Python bindings
            bool operator==(const Color c)
            {
                return red == c.red && green == c.green && blue == c.blue;
            }
        };

        PPM();

        /** \brief Load a .ppm file. Throw an exception in case of an error. */
        void loadFile(const char *filename);

        /** \brief Save image data to a .ppm file. Throw an exception in case of an error. */
        void saveFile(const char *filename);

        /** \brief Get the width of the loaded image. */
        unsigned int getWidth() const
        {
            return width_;
        }

        /** \brief Get the height of the loaded image. */
        unsigned int getHeight() const
        {
            return height_;
        }

        /** \brief Set the width for the loaded image. This must
        eventually match the number of pixels, if saveFile() gets called. */
        void setWidth(unsigned int width)
        {
            width_ = width;
        }

        /** \brief Set the height for the loaded image. This must
        eventually match the number of pixels, if saveFile() gets called. */
        void setHeight(unsigned int height)
        {
            height_ = height;
        }

        /** \brief Get read-only access to the pixels in the image. To access a pixel at coordinate (row,col),
        use getPixels()[row * getWidth() + col]. */
        const std::vector<Color> &getPixels() const
        {
            return pixels_;
        }
        /** \brief Get write access to the pixels in the image. To access a pixel at coordinate (row,col),
        use getPixels()[row * getWidth() + col].  This must eventually match the width & height set
        by setWidth() and setHeight(). */
        std::vector<Color> &getPixels()
        {
            return pixels_;
        }

        /** \brief Directly access a pixel in the image */
        const Color& getPixel(const int row, const int col) const
        {
            return pixels_[row * width_ + col];
        }

        /** \brief Directly access a pixel in the image */
        Color& getPixel(const int row, const int col)
        {
            return pixels_[row * width_ + col];
        }
    private:

        std::vector<Color> pixels_;
        unsigned int       width_;
        unsigned int       height_;
    };
}

#endif
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Mark Moll */

#ifndef PY_BINDINGS_OMPL_PY_UTIL_
#define PY_BINDINGS_OMPL_PY_UTIL_

#include <vector>
#include <map>

namespace ompl
{
    namespace util
    {
        inline int dummySTLContainerSize()
        {
            return sizeof(std::vector<std::size_t>) +
                sizeof(std::vector<bool>) +
                sizeof(std::vector<int>) +
                sizeof(std::vector<unsigned long>) +
                sizeof(std::vector<double>) +
                sizeof(std::vector<unsigned int>) +
                sizeof(std::vector<std::string>) +
                sizeof(std::vector< std::vector<int> >) +
                sizeof(std::vector< std::vector<unsigned int> >) +
                sizeof(std::vector< std::vector<double> >) +
                sizeof(std::vector< std::map< std::string, std::string > >) +
                sizeof(std::map< std::string, std::string >);
        }
    }
}

#endif
