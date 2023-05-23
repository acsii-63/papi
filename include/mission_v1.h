#ifndef MISSION_V1_H
#define MISSION_V1_H

#include <cstdlib>
#include <cstdbool>
#include <cstring>
#include <cstdint>

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <fstream>

#include <jsoncpp/json/json.h>

#define vector3 std::vector<double> // Vector3 contains 3 double variables

#define DOUBLE_MIN (double)-999999999 // Min value of double type

#define DOUBLE_MAX (double)999999999 // Max value of double type

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

/************** ENUM THINGS **************/

namespace enumConvert
{
    int8_t stringToPeripheral(const std::string inputString);
    int8_t stringToController(const std::string inputString);
    int8_t stringToPlanner(const std::string inputString);
    int8_t stringToTerminator(const std::string inputString);
    int8_t stringToExit(const std::string inputString);
    int8_t stringToAction(const std::string inputString);
    int8_t stringToResponse(const std::string inputString);
};

/*****************************************/

// Instruction Template
class SingleInstruction
{
public: // Base functions
    virtual ~SingleInstruction();

public: // InitInstruction base functions
    // Return a pointer to peripherals vector in InitInstruction
    virtual int8_t *Init_getPointer_Peripherals();

    // Get Controller in InitInstruction
    virtual int8_t Init_getController();

    // Get Terminator in InitInstruction
    virtual int8_t Init_getTerminator();

    // Return a pointer to exit code vector in InitInstruction
    virtual int8_t *Init_getPointer_Exit();

public: // TravelInstruction base functions
    // Get Planner
    virtual int8_t Travel_getPlanner();

    // Return a pointer to waypoints vector in TravelInstruction
    virtual double *Travel_getPointer_Waypoints();

    // Retuen a vector to constraints vector in TravelInstruction
    virtual double *Travel_getPointer_Constraints();

    // Get Terminator in TravelInstruction
    virtual int8_t Travel_getTerminator();

    // Return a pointer to exit code vector in TravelInstruction
    virtual int8_t *Travel_getPointer_Exit();

public: // ActionInstruction base functions
    // Get Action in ActionInstruction
    virtual int8_t Action_getAction();

    // Get Param in ActionInstruction
    virtual double Action_getParam();

    // Get Terminator in ActionInstruction
    virtual int8_t Action_getTerminator();

    // Return a pointer to exit code vector in ActionInstruction
    virtual int8_t *Action_getPointer_Exit();

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

    int8_t Init_getController() override;

    int8_t *Init_getPointer_Peripherals() override;

    int8_t Init_getTerminator() override;

    int8_t *Init_getPointer_Exit() override;

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

    int8_t Travel_getPlanner() override;

    double *Travel_getPointer_Waypoints() override;

    double *Travel_getPointer_Constraints() override;

    int8_t Travel_getTerminator() override;

    int8_t *Travel_getPointer_Exit() override;

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

    int8_t Action_getAction() override;

    double Action_getParam() override;

    int8_t Action_getTerminator() override;

    int8_t *Action_getPointer_Exit() override;

public:
    int8_t action;            // Action
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

/********************* JSON PARSING NAMESPACE *********************/

namespace jsonParsing
{
    // Function to handle init_sequence
    bool handleInitSequence(const Json::Value &_sequence, InitInstruction &_init_instruction);

    // Function to handle action_sequence
    bool handleActionSequence(const Json::Value &_sequence, ActionInstruction &_action_instruction);

    // Function to handle travel_sequence
    bool handleTravelSequence(const Json::Value &_sequence, TravelInstruction &_travel_instruction);

    // Parsing from JSON file to MissionRequest class object
    bool parsing(const std::ifstream &_file, MissionRequest &_mission);
};

/*************************************************** IMPLEMENTS ***************************************************/

/************** ENUM THINGS **************/

int8_t enumConvert::stringToPeripheral(const std::string inputString)
{
    if (inputString == "PERIPHERAL_UNSPECIFIED")
    {
        return PERIPHERAL_UNSPECIFIED;
    }
    else if (inputString == "PERIPHERAL_CAM_DOWNWARD")
    {
        return PERIPHERAL_CAM_DOWNWARD;
    }
    else if (inputString == "PERIPHERAL_CAM_FORWARD")
    {
        return PERIPHERAL_CAM_FORWARD;
    }
    else if (inputString == "PERIPHERAL_LIDAR")
    {
        return PERIPHERAL_LIDAR;
    }
    else if (inputString == "PERIPHERAL_PCU")
    {
        return PERIPHERAL_PCU;
    }
    else if (inputString == "PERIPHERAL_CAM_ODOM")
    {
        return PERIPHERAL_CAM_ODOM;
    }
    else
    {
        return INT8_MIN;
    }
}

int8_t enumConvert::stringToController(const std::string inputString)
{
    if (inputString == "CONTROLLER_UNSPECIFIED")
    {
        return CONTROLLER_UNSPECIFIED;
    }
    else if (inputString == "CONTROLLER_PX4_VELO_FB")
    {
        return CONTROLLER_PX4_VELO_FB;
    }
    else if (inputString == "CONTROLLER_A_FB")
    {
        return CONTROLLER_A_FB;
    }
    else if (inputString == "CONTROLLER_A_FW")
    {
        return CONTROLLER_A_FW;
    }
    else if (inputString == "CONTROLLER_A_ADRJ")
    {
        return CONTROLLER_A_ADRJ;
    }
    else
    {
        return INT8_MIN;
    }
}

int8_t enumConvert::stringToPlanner(const std::string inputString)
{
    if (inputString == "PLANNER_UNSPECIFIED")
    {
        return PLANNER_UNSPECIFIED;
    }
    else if (inputString == "PLANNER_EGO")
    {
        return PLANNER_EGO;
    }
    else if (inputString == "PLANNER_FAST")
    {
        return PLANNER_FAST;
    }
    else if (inputString == "PLANNER_MARKER")
    {
        return PLANNER_MARKER;
    }
    else if (inputString == "PLANNER_SAFELAND")
    {
        return PLANNER_SAFELAND;
    }
    else
    {
        return INT8_MIN;
    }
}

int8_t enumConvert::stringToTerminator(const std::string inputString)
{
    if (inputString == "TERMINATION_UNSPECIFIED")
    {
        return TERMINATION_UNSPECIFIED;
    }
    else if (inputString == "TERMINATION_AUTO")
    {
        return TERMINATION_AUTO;
    }
    else if (inputString == "TERMINATION_STD")
    {
        return TERMINATION_STD;
    }
    else
    {
        return INT8_MIN;
    }
}

int8_t enumConvert::stringToExit(const std::string inputString)
{
    if (inputString == "EXIT_UNSPECIFIED")
    {
        return EXIT_UNSPECIFIED;
    }
    else if (inputString == "EXIT_PASSED")
    {
        return EXIT_PASSED;
    }
    else if (inputString == "EXIT_FAILED")
    {
        return EXIT_FAILED;
    }
    else
    {
        return INT8_MIN;
    }
}

int8_t enumConvert::stringToAction(const std::string inputString)
{
    if (inputString == "ACTION_UNSPECIFIED")
    {
        return ACTION_UNSPECIFIED;
    }
    else if (inputString == "ACTION_TAKEOFF")
    {
        return ACTION_TAKEOFF;
    }
    else if (inputString == "ACTION_DISARM")
    {
        return ACTION_DISARM;
    }
    else if (inputString == "ACTION_SELFCHECK")
    {
        return ACTION_SELFCHECK;
    }
    else if (inputString == "ACTION_RELEASE")
    {
        return ACTION_RELEASE;
    }
    else if (inputString == "ACTION_RTLHOME")
    {
        return ACTION_RTLHOME;
    }
    else if (inputString == "ACTION_HOLD")
    {
        return ACTION_HOLD;
    }
    else if (inputString == "ACTION_AUTOLAND")
    {
        return ACTION_AUTOLAND;
    }
    else
    {
        return INT8_MIN;
    }
}

int8_t enumConvert::stringToResponse(const std::string inputString)
{
    if (inputString == "RESPONSE_UNSPECIFIED")
    {
        return RESPONSE_UNSPECIFIED;
    }
    else if (inputString == "RESPONSE_SUCCESS")
    {
        return RESPONSE_SUCCESS;
    }
    else if (inputString == "RESPONSE_SYNTAX_ERROR")
    {
        return RESPONSE_SYNTAX_ERROR;
    }
    else if (inputString == "RESPONSE_MISSING_INSTRUCTION")
    {
        return RESPONSE_MISSING_INSTRUCTION;
    }
    else if (inputString == "RESPONSE_ENCODING_ERROR")
    {
        return RESPONSE_ENCODING_ERROR;
    }
    else
    {
        return INT8_MIN;
    }
}

/*****************************************/

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

int8_t *SingleInstruction::Init_getPointer_Peripherals()
{
    return nullptr;
}

int8_t SingleInstruction::Init_getController()
{
    return INT8_MIN;
}

int8_t SingleInstruction::Init_getTerminator()
{
    return INT8_MIN;
}

int8_t *SingleInstruction::Init_getPointer_Exit()
{
    return nullptr;
}

int8_t SingleInstruction::Travel_getPlanner()
{
    return INT8_MIN;
}

double *SingleInstruction::Travel_getPointer_Waypoints()
{
    return nullptr;
}

double *SingleInstruction::Travel_getPointer_Constraints()
{
    return nullptr;
}

int8_t SingleInstruction::Travel_getTerminator()
{
    return INT8_MIN;
}

int8_t *SingleInstruction::Travel_getPointer_Exit()
{
    return nullptr;
}

int8_t SingleInstruction::Action_getAction()
{
    return INT8_MIN;
}

double SingleInstruction::Action_getParam()
{
    return DOUBLE_MIN;
}

int8_t SingleInstruction::Action_getTerminator()
{
    return INT8_MIN;
}

int8_t *SingleInstruction::Action_getPointer_Exit()
{
    return nullptr;
}

/*******************************************/

InitInstruction::~InitInstruction() {}

int8_t InitInstruction::Init_getController()
{
    return controller;
}

int8_t *InitInstruction::Init_getPointer_Peripherals()
{
    return peripherals.data();
}

int8_t InitInstruction::Init_getTerminator()
{
    return terminator;
}

int8_t *InitInstruction::Init_getPointer_Exit()
{
    return exit.data();
}

/*******************************************/

TravelInstruction::~TravelInstruction() {}

int8_t TravelInstruction::Travel_getPlanner()
{
    return planner;
}

double *TravelInstruction::Travel_getPointer_Waypoints()
{
    return waypoints.data()->data();
}

double *TravelInstruction::Travel_getPointer_Constraints()
{
    return constraints.data()->data();
}

int8_t TravelInstruction::Travel_getTerminator()
{
    return terminator;
}

int8_t *TravelInstruction::Travel_getPointer_Exit()
{
    return exit.data();
}

/*******************************************/

ActionInstruction::~ActionInstruction() {}

int8_t ActionInstruction::Action_getAction()
{
    return action;
}

double ActionInstruction::Action_getParam()
{
    return param;
}

int8_t ActionInstruction::Action_getTerminator()
{
    return terminator;
}

int8_t *ActionInstruction::Action_getPointer_Exit()
{
    return exit.data();
}

/********************* JSON PARSING NAMESPACE *********************/

bool jsonParsing::handleInitSequence(const Json::Value &_sequence, InitInstruction &_init_instruction)
{
    const Json::Value &peripheral = _sequence["peripheral"];

    if (!peripheral.isArray())
    {
        std::cerr << "The format of the peripheral array is incorrect." << std::endl;
        return false;
    }
    std::vector<int8_t> this_peripheral;
    for (const auto &value : peripheral)
    {
        int8_t peripheralValue = value.asInt();
        this_peripheral.push_back(peripheralValue);
    }
    _init_instruction.peripherals = this_peripheral;

    std::string this_controller = _sequence["controller"].asString();
    _init_instruction.controller = enumConvert::stringToController(this_controller);

    std::string this_terminator = _sequence["terminator"].asString();
    _init_instruction.terminator = enumConvert::stringToTerminator(this_terminator);

    return true;
}

bool jsonParsing::handleActionSequence(const Json::Value &_sequence, ActionInstruction &_action_instruction)
{
    std::string this_action = _sequence["action"].asString();
    _action_instruction.action = enumConvert::stringToAction(this_action);

    double this_param = _sequence["param"].asDouble();
    _action_instruction.param = this_param;

    std::string this_terminator = _sequence["terminator"].asString();
    _action_instruction.terminator = enumConvert::stringToTerminator(this_terminator);

    return true;
}

bool jsonParsing::handleTravelSequence(const Json::Value &_sequence, TravelInstruction &_travel_instruction)
{
    std::string this_planner = _sequence["planner"].asString();
    _travel_instruction.planner = enumConvert::stringToPlanner(this_planner);

    const Json::Value &waypoints = _sequence["waypoint"];
    if (!waypoints.isArray())
    {
        std::cerr << "The format of the waypoints array is incorrect." << std::endl;
        return false;
    }
    std::vector<vector3> this_waypoints;
    for (const auto &point : waypoints)
    {
        std::vector<double> temp_vector3;
        temp_vector3.push_back(point[0].asDouble());
        temp_vector3.push_back(point[1].asDouble());
        temp_vector3.push_back(point[2].asDouble());

        this_waypoints.push_back(temp_vector3);
    }
    _travel_instruction.waypoints = this_waypoints;

    const Json::Value &constraints = _sequence["constraint"];
    if (!constraints.isArray())
    {
        std::cerr << "The format of the constraints array is incorrect." << std::endl;
        return false;
    }
    std::vector<vector3> this_constraints;
    for (const auto &constr : constraints)
    {
        std::vector<double> temp_vector3;
        temp_vector3.push_back(constr[0].asDouble());
        temp_vector3.push_back(constr[1].asDouble());
        temp_vector3.push_back(constr[2].asDouble());

        this_constraints.push_back(temp_vector3);
    }
    _travel_instruction.constraints = this_constraints;

    std::string this_terminator = _sequence["terminator"].asString();
    _travel_instruction.terminator = enumConvert::stringToTerminator(this_terminator);

    return true;
}

bool jsonParsing::parsing(const std::ifstream &_file, MissionRequest &_mission)
{
    // Json file
    std::ifstream file("../sample/sample.json");
    if (!file.is_open())
    {
        std::cerr << "Unable to open json file" << std::endl;
        return false;
    }

    // Read the file content into a string
    std::string jsonContent((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();

    // Parse the JSON string using a Json::Reader object
    Json::Value jsonData;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonContent, jsonData))
    {
        std::cerr << "Cannot Parse jsonData using jsonReader" << std::endl;
        return false;
    }

    _mission.id = jsonData["id"].asString();
    _mission.number_sequence_items = jsonData["number_sequence_items"].asInt();
    _mission.description = jsonData["description"].asString();

    // Access the sequence_items array
    const Json::Value &sequence_items = jsonData["sequence_items"];
    if (!sequence_items.isArray())
    {
        std::cerr << "The format of the sequence items is incorrect." << std::endl;
        return false;
    }

    int sequence_index = 0;
    for (auto const &item : sequence_items.getMemberNames())
    {
        if (item == "init_sequence")
        {
            const Json::Value &init_sequence = sequence_items[sequence_index];
            InitInstruction init_instruction;
            if (!handleInitSequence(init_sequence, init_instruction))
            {
                std::cerr << "Failed to parse the no." << sequence_index << " sequence item: init_sequence." << std::endl;
                return false;
            }
            _mission.sequence_items.addInstruction(&init_instruction);
        }

        else if (item == "action_sequence")
        {
            const Json::Value &action_sequence = sequence_items[sequence_index];
            ActionInstruction action_instruction;
            if (!handleActionSequence(action_sequence, action_instruction))
            {
                std::cerr << "Failed to parse the no." << sequence_index << " sequence item: action_instruction." << std::endl;
                return false;
            }
            _mission.sequence_items.addInstruction(&action_instruction);
        }

        else if (item == "travel_sequence")
        {
            const Json::Value &travel_sequence = sequence_items[sequence_index];
            TravelInstruction travel_instruction;
            if (!handleTravelSequence(travel_sequence, travel_instruction))
            {
                std::cerr << "Failed to parse the no." << sequence_index << " sequence item: travel_instruction." << std::endl;
                return false;
            }
            _mission.sequence_items.addInstruction(&travel_instruction);
        }

        sequence_index++;
    }

    return true;
}

#endif