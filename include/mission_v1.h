#ifndef MISSION_V1_H
#define MISSION_V1_H

#include <cstdlib>
#include <cstdbool>
#include <cstring>
#include <cstdint>

#include <iostream>
#include <string>
#include <vector>

#define vector3 std::vector<double> // Vector3 contains 3 double variables

enum Peripheral : int8_t
{
    PERIPHERAL_UNSPECIFIED,
    PERIPHERAL_CAM_FORWARD,
    PERIPHERAL_CAM_DOWNWARD,
    PERIPHERAL_LIDAR,
    PERIPHERAL_CAM_ODOM,
    PERIPHERAL_PCU
};

enum Controller : int8_t
{
    CONTROLLER_UNSPECIFIED,
    CONTROLLER_PX4_VELO_FB,
    CONTROLLER_A_FB,
    CONTROLLER_A_FW,
    CONTROLLER_A_ADRJ
};

enum Planner : int8_t
{
    PLANNER_UNSPECIFIED,
    PLANNER_EGO,
    PLANNER_FAST,
    PLANNER_MARKER,
    PLANNER_SAFELAND
};

enum Terminator : int8_t
{
    TERMINATION_UNSPECIFIED,
    TERMINATION_AUTO,
    TERMINATION_STD
};

enum Exit : int8_t
{
    EXIT_UNSPECIFIED = -1,
    EXIT_PASSED = EXIT_SUCCESS, // 0
    EXIT_FAILED = EXIT_FAILURE  // 1
};

enum Action : int8_t
{
    ACTION_UNSPECIFIED,
    ACTION_TAKEOFF,
    ACTION_DISARM,
    ACTION_SELFCHECK,
    ACTION_RELEASE,
    ACTION_RTLHOME,
    ACTION_HOLD,
    ACTION_AUTOLAND
};

enum Response : int8_t
{
    RESPONSE_UNSPECIFIED = -1,
    RESPONSE_SUCCESS,
    RESPONSE_SYNTAX_ERROR,
    RESPONSE_MISSING_INSTRUCTION,
    RESPONSE_ENCODING_ERROR,
};

/**************************************************** DEFINES ****************************************************/

// Instruction Template
class SingleInstruction
{
public: // Base functions
    virtual ~SingleInstruction();

    // Get Terminator
    virtual int8_t getTerminator();

    // Return a pointer to exit code vector
    virtual int8_t *getPointer_Exit();

public: // InitInstruction base functions
    // Return a pointer to peripherals vector in InitInstruction
    virtual int8_t *Init_getPointer_Peripherals();

    // Get Controller in InitInstruction
    virtual int8_t Init_getController();

public: // TravelInstruction base functions
    // Get Planner
    virtual int8_t Travel_getPlanner();

    // Return a pointer to waypoints vector in TravelInstruction
    virtual double *Travel_getPointer_Waypoints();

    // Retuen a vector to constraints vector in TravelInstruction
    virtual double *Travel_getPointer_Constraints();

public: // ActionInstruction base functions
    // Get Param
    virtual double Action_getParam();

public:
    std::string name; // Name of the instruction.
};

// Sequence contains Intructions
class SequenceItems
{
public:
    std::vector<SingleInstruction> instructions; // Vector contains Instructions.

public:
    SequenceItems();
    ~SequenceItems();

    // Add the instruction to the sequence.
    void addInstruction(SingleInstruction *_instruction);
};

// Init Instruction, child class of the SingleInstruction class.
class InitInstruction : public SingleInstruction
{
public:
    ~InitInstruction() override;

public:
    std::vector<int8_t> peripherals; // Vector contains peripherals that need to be turned on
    int8_t controller;               // Controller using in the flight process
    int8_t terminator;               // Terminator
    std::vector<int8_t> exit;        // Vector contains exit codes
};

// Travel Instruction, child class of the SingleInstruction class
class TravelInstruction : public SingleInstruction
{
public:
    ~TravelInstruction() override;

public:
    int8_t planner;                   // Planner using in the travel process
    std::vector<vector3> waypoints;   // Vector contains waypoints in travel process in Lat, Long, Alt
    std::vector<vector3> constraints; // Vector contains one or more constants including: maximum velocity, maximum acceleration, geofence in 3 directions
    int8_t terminator;                // Terminator
    std::vector<int8_t> exit;         // Vector contains exit codes
};

// Action Instruction, child class of the SingleInstruction class
class ActionInstruction : public SingleInstruction
{
public:
    ~ActionInstruction() override;

public:
    double param;             // Parameter in current action
    int8_t terminator;        // Terminator
    std::vector<int8_t> exit; // Vector contains exit codes
};

// Mission received from GCS
class MissionRequest
{
public:
    std::string id;               // ID of the mission
    int8_t number_sequence_items; // Number of items in the sequence
    std::string description;      // Mission description
    SequenceItems sequence_items; // Sequence contains intructions.
};

// Response message when received mission from GCS
class MissionResponse
{
public:
    MissionResponse();
    ~MissionResponse();

public:
    int8_t responseCode; // Response Code
};

/*************************************************** IMPLEMENTS ***************************************************/

SequenceItems::SequenceItems() {}

SequenceItems::~SequenceItems() {}

void SequenceItems::addInstruction(SingleInstruction *_instruction)
{
    SingleInstruction *temp_instruction = new SingleInstruction;
    temp_instruction = _instruction;
    instructions.push_back(*temp_instruction);
}

/*******************************************/

SingleInstruction::~SingleInstruction() {}

/*******************************************/

InitInstruction::~InitInstruction() {}

/*******************************************/

TravelInstruction::~TravelInstruction() {}

/*******************************************/

ActionInstruction::~ActionInstruction() {}

#endif