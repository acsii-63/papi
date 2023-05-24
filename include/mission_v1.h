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

enum Peripheral : int
{
    PERIPHERAL_UNSPECIFIED,
    PERIPHERAL_CAM_FORWARD,
    PERIPHERAL_CAM_DOWNWARD,
    PERIPHERAL_LIDAR,
    PERIPHERAL_CAM_ODOM,
    PERIPHERAL_PCU
};

enum Controller : int
{
    CONTROLLER_UNSPECIFIED,
    CONTROLLER_PX4_VELO_FB,
    CONTROLLER_A_FB,
    CONTROLLER_A_FW,
    CONTROLLER_A_ADRJ
};

enum Planner : int
{
    PLANNER_UNSPECIFIED,
    PLANNER_EGO,
    PLANNER_FAST,
    PLANNER_MARKER,
    PLANNER_SAFELAND
};

enum Terminator : int
{
    TERMINATION_UNSPECIFIED,
    TERMINATION_AUTO,
    TERMINATION_STD
};

enum Exit : int
{
    EXIT_UNSPECIFIED = -1,
    EXIT_PASSED = EXIT_SUCCESS, // 0
    EXIT_FAILED = EXIT_FAILURE  // 1
};

enum Action : int
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

enum Response : int
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
    int stringToPeripheral(const std::string inputString);
    int stringToController(const std::string inputString);
    int stringToPlanner(const std::string inputString);
    int stringToTerminator(const std::string inputString);
    int stringToExit(const std::string inputString);
    int stringToAction(const std::string inputString);
    int stringToResponse(const std::string inputString);
};

/*****************************************/

// Instruction Template
class SingleInstruction
{
public: // Base functions
    virtual ~SingleInstruction();

public: // InitInstruction base functions
    virtual void Init_getPeripherals(std::vector<int> &_peripherals_vector);

    // Get Controller in InitInstruction
    virtual int Init_getController();

    // Get Terminator in InitInstruction
    virtual int Init_getTerminator();

    virtual void Init_getExit(std::vector<int> &_exit_vector);

public: // TravelInstruction base functions
    // Get Planner
    virtual int Travel_getPlanner();

    virtual void Travel_getWaypoints(std::vector<vector3> &_waypoints_vector);

    virtual void Travel_getConstraints(std::vector<vector3> &_const_vector);

    // Get Terminator in TravelInstruction
    virtual int Travel_getTerminator();

    virtual void Travel_getExit(std::vector<int> &_exit_vector);

public: // ActionInstruction base functions
    // Get Action in ActionInstruction
    virtual int Action_getAction();

    // Get Param in ActionInstruction
    virtual double Action_getParam();

    // Get Terminator in ActionInstruction
    virtual int Action_getTerminator();

    virtual void Action_getExit(std::vector<int> &_exit_vector);

public:
    std::string name; // Name of the instruction.
};

// Init Instruction, child class of the SingleInstruction class.
class InitInstruction : public SingleInstruction
{
public:
    ~InitInstruction() override;

    int Init_getController() override;

    int Init_getTerminator() override;

    void Init_getPeripherals(std::vector<int> &_peripherals_vector) override;

    void Init_getExit(std::vector<int> &_exit_vector) override;

public:
    std::vector<int> peripherals; // Vector contains peripherals that need to be turned on
    int controller;               // Controller using in the flight process
    int terminator;               // Terminator
    std::vector<int> exit;        // Vector contains exit codes
};

// Travel Instruction, child class of the SingleInstruction class
class TravelInstruction : public SingleInstruction
{
public:
    ~TravelInstruction() override;

    int Travel_getPlanner() override;

    int Travel_getTerminator() override;

    void Travel_getWaypoints(std::vector<vector3> &_waypoints_vector) override;

    void Travel_getConstraints(std::vector<vector3> &_const_vector) override;

    void Travel_getExit(std::vector<int> &_exit_vector) override;

public:
    int planner;                      // Planner using in the travel process
    std::vector<vector3> waypoints;   // Vector contains waypoints in travel process in Lat, Long, Alt
    std::vector<vector3> constraints; // Vector contains one or more constants including: maximum velocity, maximum acceleration, geofence in 3 directions
    int terminator;                   // Terminator
    std::vector<int> exit;            // Vector contains exit codes
};

// Action Instruction, child class of the SingleInstruction class
class ActionInstruction : public SingleInstruction
{
public:
    ~ActionInstruction() override;

    int Action_getAction() override;

    double Action_getParam() override;

    int Action_getTerminator() override;

    void Action_getExit(std::vector<int> &_exit_vector) override;

public:
    int action;            // Action
    double param;          // Parameter in current action
    int terminator;        // Terminator
    std::vector<int> exit; // Vector contains exit codes
};

// Mission received from GCS
class MissionRequest
{
public:
    std::string id;                                        // ID of the mission
    int number_sequence_items;                             // Number of items in the sequence
    std::string description;                               // Mission description
    std::vector<SingleInstruction *> sequence_istructions; // Vector of instruction pointers
    std::vector<std::string> sequence_names;               // Vector of instruction name in order.
};

// Response message when received mission from GCS
class MissionResponse
{
public:
    MissionResponse();
    ~MissionResponse();

public:
    int responseCode; // Response Code
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
    bool parsing(const std::string _path_to_json_file, MissionRequest &_mission);
};

/*************************************************** IMPLEMENTS ***************************************************/

/************** ENUM THINGS **************/

int enumConvert::stringToPeripheral(const std::string inputString)
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

int enumConvert::stringToController(const std::string inputString)
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

int enumConvert::stringToPlanner(const std::string inputString)
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

int enumConvert::stringToTerminator(const std::string inputString)
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

int enumConvert::stringToExit(const std::string inputString)
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

int enumConvert::stringToAction(const std::string inputString)
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

int enumConvert::stringToResponse(const std::string inputString)
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

/*******************************************/

SingleInstruction::~SingleInstruction() {}

int SingleInstruction::Init_getController()
{
    return INT8_MIN;
}

int SingleInstruction::Init_getTerminator()
{
    return INT8_MIN;
}

void SingleInstruction::Init_getPeripherals(std::vector<int> &_peripherals_vector)
{
    return;
}

void SingleInstruction::Init_getExit(std::vector<int> &_exit_vector)
{
    return;
}

int SingleInstruction::Travel_getPlanner()
{
    return INT8_MIN;
}

int SingleInstruction::Travel_getTerminator()
{
    return INT8_MIN;
}

void SingleInstruction::Travel_getExit(std::vector<int> &_exit_vector)
{
    return;
}

void SingleInstruction::Travel_getWaypoints(std::vector<vector3> &_waypoints_vector)
{
    return;
}

void SingleInstruction::Travel_getConstraints(std::vector<vector3> &_const_vector)
{
    return;
}

int SingleInstruction::Action_getAction()
{
    return INT8_MIN;
}

double SingleInstruction::Action_getParam()
{
    return DOUBLE_MIN;
}

int SingleInstruction::Action_getTerminator()
{
    return INT8_MIN;
}

void SingleInstruction::Action_getExit(std::vector<int> &_exit_vector)
{
    return;
}

/*******************************************/

InitInstruction::~InitInstruction() {}

int InitInstruction::Init_getController()
{
    return controller;
}

int InitInstruction::Init_getTerminator()
{
    return terminator;
}

void InitInstruction::Init_getPeripherals(std::vector<int> &_peripherals_vector)
{
    _peripherals_vector = peripherals;
}

void InitInstruction::Init_getExit(std::vector<int> &_exit_vector)
{
    _exit_vector = exit;
}

/*******************************************/

TravelInstruction::~TravelInstruction() {}

int TravelInstruction::Travel_getPlanner()
{
    return planner;
}

int TravelInstruction::Travel_getTerminator()
{
    return terminator;
}

void TravelInstruction::Travel_getWaypoints(std::vector<vector3> &_waypoints_vector)
{
    _waypoints_vector = waypoints;
}

void TravelInstruction::Travel_getConstraints(std::vector<vector3> &_const_vector)
{
    _const_vector = constraints;
}

void TravelInstruction::Travel_getExit(std::vector<int> &_exit_vector)
{
    _exit_vector = exit;
}

/*******************************************/

ActionInstruction::~ActionInstruction() {}

int ActionInstruction::Action_getAction()
{
    return action;
}

double ActionInstruction::Action_getParam()
{
    return param;
}

int ActionInstruction::Action_getTerminator()
{
    return terminator;
}

void ActionInstruction::Action_getExit(std::vector<int> &_exit_vector)
{
    _exit_vector = exit;
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
    std::vector<int> this_peripherals;
    for (const auto &value : peripheral)
    {
        int peripheralValue = value.asInt();
        this_peripherals.push_back(peripheralValue);
    }
    _init_instruction.peripherals = this_peripherals;

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

bool jsonParsing::parsing(const std::string _path_to_json_file, MissionRequest &_mission)
{
    // Json file
    std::ifstream file(_path_to_json_file);
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

    for (auto const &item : sequence_items)
    {
        const std::vector<std::string> &member_names = item.getMemberNames();
        if (member_names.empty())
        {
            std::cerr << "Can not get the object name of no." << sequence_index << " sequence item." << std::endl;
            return false;
        }
        std::string current_item_name = member_names[0];

        if (current_item_name == "init_sequence")
        {
            const Json::Value &init_sequence = sequence_items[sequence_index]["init_sequence"];
            InitInstruction *init_instruction = new InitInstruction();
            if (!handleInitSequence(init_sequence, *init_instruction))
            {
                std::cerr << "Failed to parse the no." << sequence_index << " sequence item: init_sequence." << std::endl;
                return false;
            }
            _mission.sequence_istructions.push_back(init_instruction);
            _mission.sequence_names.push_back("init_sequence");
        }

        else if (current_item_name == "action_sequence")
        {
            const Json::Value &action_sequence = sequence_items[sequence_index]["action_sequence"];
            ActionInstruction *action_instruction = new ActionInstruction();
            if (!handleActionSequence(action_sequence, *action_instruction))
            {
                std::cerr << "Failed to parse the no." << sequence_index << " sequence item: action_instruction." << std::endl;
                return false;
            }
            _mission.sequence_istructions.push_back(action_instruction);
            _mission.sequence_names.push_back("action_sequence");
        }

        else if (current_item_name == "travel_sequence")
        {
            const Json::Value &travel_sequence = sequence_items[sequence_index]["travel_sequence"];
            TravelInstruction *travel_instruction = new TravelInstruction();
            if (!handleTravelSequence(travel_sequence, *travel_instruction))
            {
                std::cerr << "Failed to parse the no." << sequence_index << " sequence item: travel_instruction." << std::endl;
                return false;
            }
            _mission.sequence_istructions.push_back(travel_instruction);
            _mission.sequence_names.push_back("travel_sequence");
        }

        sequence_index++;
    }

    return true;
}

#endif