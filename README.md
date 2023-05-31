# papi
## Pino's Application Programming Interface - PAPI
A collection of C++ namespaces, classes, and functions designed for the automatic operation of services on a drone.
### Installation:
  Clone this repository in your local project.
```
git clone https://github.com/acsii-63/papi.git
```
### Usage:
<!--   - Utilizing this API in a similar manner to a C++ header file.
  - In the include path of your C++ program, add the following statement:
```
#include "/path-to-the-repository/papi/PAPI.h"
``` -->
  If you are going to use this API in a ROS package (in a catkin workspace), you may want to add these things in the CMakeLists.txt file:
```
## Add JSONCPP dependency
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(${JSONCPP_INCLUDE_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})

## Add pthread library
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
```
  And add these things in the target_link_libraries() of an add_executable():
```
${JSONCPP_LIBRARIES} Threads::Threads
```
Example:
![Image Alt Text](docs/cmake_img1.png)
