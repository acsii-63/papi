#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cstdbool>

#include <string>
#include <vector>
#include <list>
// #include <algorithm>
// #include <iostream>

#define vector3 std::vector<double> // Vector3 contains 3 double variables

enum class Peripheral
{

};

enum class Controller
{
    PX4_VELO_FB
};

enum class Terminator
{
    STD,
    AUTO
};

enum class Exit
{
    SUCCESS
};

enum class Planner
{
    EGO_PLANNER
};

enum class BOOLEAN
{
    TRUE,
    FALSE
};

class Init
{
public:
    std::vector<int8_t> peripheral; // Vector contains peripherals that need to be turned on
    int8_t controller;              // Controller using in the flight process
    int8_t terminator;              // Terminator
    std::vector<int8_t> exit;       // Vector contains exit codes
};

class Travel
{
public:
    int8_t planner;                  // Planner using in the travel process
    std::vector<vector3> waypoint;   // Vector contains waypoints in travel process in Lat, Long, Alt
    std::vector<vector3> constraint; // Vector contains one or more constants including: maximum velocity, maximum acceleration, geofence in 3 directions
    int8_t terminator;               // Terminator
    std::vector<int8_t> exit;        // Vector contains exit codes
};

class Action
{
public:
    int land_maxv;               // Maximum velocity when landing
    std::vector<int8_t> release; // Vector contains package list.
    int takeoff_height;          // Takeoff height
    int8_t disarm;               // Disarm status. If 0, take action. Other value, do nothing
    int8_t self_check;           // Self-check status. If 0, take action. Other value, do nothing
    int8_t terminator;           // Terminator
    std::vector<int8_t> exit;    // Vector contains exit codes
};