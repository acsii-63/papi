#ifndef PAPI_H
#define PAPI_H

#include <cstdlib>
#include <cstdbool>
#include <ctime>
#include <cstring>
#include <unistd.h>
#include <errno.h>
#include <csignal>
#include <fcntl.h>

#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/un.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>

#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <iomanip>
#include <algorithm>

#include "./include/mission_v1.h"

#define vector3 std::vector<double> // Vector3 contains 3 double variables

#define LOCAL_HOST "127.0.0.1"

// #define DEFAULT_CAM_FORWARD_NODE_PORT 24001  // D455
// #define DEFAULT_CAM_DOWNWARD_NODE_PORT 24002 // Flir
// #define DEFAULT_CAM_ODOM_NODE_PORT 24003     // T265
// #define DEFAULT_LIDAR_NODE_PORT 24004        // Lidar
// #define DEFAULT_FCU_NODE_PORT 24005          // FCU?

#define DEFAULT_PERIPHERALS_STATUS_CONTROL_PORT 24001
#define DEFAULT_PERIPHERALS_STATUS_NODE_PORT 24101

#define DEFAULT_COMM_IMAGE_PORT 5000

#define DEFAULT_LOG_DIR "/home/pino/logs"
#define DEFAULT_JSON_FILE_PATH "/home/pino/pino_ws/papi/sample/sample_takeoff_land.json"
#define DEFAULT_IMAGE_DIR_PATH "/home/pino/image"

#define DEFAULT_CONNECTION_TIMEOUT 60
#define DEFALUT_IMAGE_CONFIRM_TIMEOUT 120
#define TIME_WAIT_FOR_ACTIVE 10

/*
WAITING_FOR_HOME_POSE = 0
TAKE_OFF = 1
HOLD = 2
MISSION_EXECUTION = 3
AUTO_LAND = 4
ONGROUND = 5
*/
enum UAV_STATE : int8_t
{
    WAITING_FOR_HOME_POSE,
    TAKE_OFF,
    HOLD,
    MISSION_EXECUTION,
    AUTO_LAND,
    ONGROUND
};

enum UAV_STATUS : int8_t
{
    READY,
    BUSY
};

enum PERIPHERAL_STATUS : int
{
    UNSPECIFIED = -1,   // The peripheral remains unused.
    ACTIVE,             // Obtain the ACTIVE status once the message has been present for a continuous duration of 5 seconds.
    INACTIVE,           // Retrieve the INACTIVE status when the message has been absent for a consecutive duration of 1 second.
    WAITING_FOR_ACTIVE, // Acquire the WAITING_FOR_ACTIVE status if the message persists for less than 5 consecutive seconds.
    NOT_FOUND           // The peripheral is required but cannot be located
};

enum DEVICE : int
{
    FLIR = 0,  // FLIR
    D455,      // D455
    T265,      // T265
    LIDAR,     // LIDAR
    TERABEE,   // Range Finder
    RTK,       // RTK
    FCU_STATE, // MAV State
    FCU_IMU,   // MAV IMU
    FCU_ODOM,  // MAV Odometry
    FCU_MAG,   // MAV Magnetometer
    FCU_PRES,  // MAV Absolute Pressure
    FCU_BAT,   // MAV Battery
    FCU_MOTOR, // MAV Motor outputs, control
    FCU_AHRS,  // MAV Accelerometer
    FCU_TELE,  // MAV Telemetry (P900?)
    FCU_GPS    // MAV GPS
};

/**************************************************** DEFINES ****************************************************/

namespace PAPI
{
    // Logs file
    std::ofstream log_file;

    // PAPI::system
    namespace system
    {
        // Run a command with given argv in std::vector<std::string> type, using execvp().
        void runCommand_execvp(const std::string &_command, const std::vector<std::string> _argv);

        // Execute the command in a separate thread.
        void command_sys(const std::string &_command);

        // Run a command with given argv in std::vector<std::string> type (use for rosservice), using std::system().
        void runCommand_system(const std::string &_command, const std::vector<std::string> _argv);

        // Create logs file.
        void createLogsFile(const std::string &_path_to_logs_dir);

        // Close logs file.
        void closeLogsFile();

        // Get the PID of a node using pgrep.
        int getPID_pgrep(const std::string &_node_name);

        // Get the PID of a node.
        int getPID(const std::string &_node_name);

        /* Check the status of a node.
         * If its PID exists, return true, if not, return false. */
        bool checkStatus(const std::string &_node_name);

        // Kill a node by killing its process.
        void killNode_system(const std::string &_node_name);

        // Kill a node based on its name
        void killNode_name(const std::string &_node_name);

        // Get list of active nodes, return in std::vector of std::string type.
        std::vector<std::string> getNodeList();

        // Get list of active topics, return in std::vector of std::string type.
        std::vector<std::string> getTopicList();

        // Get list of active services, return in std::vector of std::string type.
        std::vector<std::string> getServiceList();

        // Sleep current thread.
        void threadSleeper(const int _time);

        // Get vector of PID with given name
        std::vector<std::string> getPIDList(const std::string _command_name);

        // Run rosservice call setpoint with 3 params: mode, sub, timeout
        void serviceCall_setPoint(const int _mode, const double _sub, const int8_t _timeout);

        // Run rostopic pub /controller/flatsetpoint
        bool topicPub_flatSetPoint(std::vector<vector3> _setpoint, int _mark);

        // Parsing the json file into mission object.
        bool jsonParsing(const std::string _path_to_json_file, MissionRequest &_mission);

        // Parsing peripherals status string into vector.
        std::vector<int> getPeripheralsStatus_fromString_toVector(const std::string &_str);

        // Send image to communication service
        void sendImage(const int _device);
    }

    // PAPI::communication
    namespace communication
    {
        // Server class
        class Server
        {
        public:
            Server();
            Server(int _port);

            // Start the server
            int serverStart();

            // Send message using TCP
            void sendMsg(const std::string &_str);

            // Close the server
            void serverClose();

            // Return server socket
            int getServerSocket();

            // Return the client socket of the client connecting to the server
            int getClientSocket();

            // Return the sending port of the server
            int getPort();

        private:
            int port;          // Sending port of the server
            int server_socket; // Server socket
            int client_socket; // Client socket
        };

        // Client class
        class Client
        {
        public:
            Client();
            Client(const std::string _server_ip, const int _server_port);
            Client(const int _server_socket);

            // Start the client
            int clientStart();

            // Close the client
            void clientClose();

            // Recive message then return a std::string
            std::string reciveMessage();

            // Return the server IP
            std::string getServerIP();

            // Return the Server Port
            int getServerPort();

            // Return the Server Socket
            int getServerSocket();

            // Return the Client Socket
            int getClientSocket();

        private:
            std::string server_ip; // IP of the server connecting
            int server_port;       // Port of the server connecting
            int server_socket;     // Server socket
            int client_socket;     // Client socket
        };

        class UnixSocket
        {
        public:
            UnixSocket(const std::string &path);

            ~UnixSocket();

            // Send the message string to the Unix socket
            void sendString(const std::string &message);

            // Receive the message string from the Unix socket
            std::string receiveString();

        private:
            int sockfd;                 // Unix socket file descriptor
            struct sockaddr_un address; // Unix domain socket address
            std::string system_path;    // The file system path of the Unix domain socket to connect to
        };

        // Get the current system time, return in std::string
        std::string getCurrentTime();

        // Write log to .log file
        void writeLogFile(const std::string &_log, std::ofstream &_logFile);

        // Write log to .log file and also send it using TCP
        void transferLog(const std::string &_log, std::ofstream &_logFile, const PAPI::communication::Server &_server);
    }

    // PAPI::drone
    namespace drone
    {
        // Return current state of the drone (UAV_STATE)
        int getState();

        // Take off and hold at _height meters.
        bool takeOffAndHold(double _height, int8_t _timeout);

        // Hold the drone
        bool hold(int8_t _timeout);

        // Mission Execute with setpoint and mask [TESTING ONLY]
        bool missionExecute(std::vector<vector3> _setpoint, int _mask, int8_t _timeout);

        // Auto Land with maximum velocity.
        bool autoLand(double _max_v, int8_t _timeout);

        // Launch driver for 3 cams.
        void turnOnEverything();

        // Check every peripherals
        bool peripheralsCheck();

        // Make Init Instruction
        bool makeInitInstruction(SingleInstruction *_init_instruction);

        // Make Action Instruction
        bool makeActionInstruction(SingleInstruction *_action_instruction);

        // Make Travel Instructon
        bool makeTravelInstruction(SingleInstruction *_travel_instruction);

        // Make Overall Instruction
        bool makeInstruction(SingleInstruction *_instruction);

        // Turn on peripheral by it ID in enum Peripheral
        bool turnOnPeripheral(const int _peripheral);

        namespace CheckStatus
        {
            int8_t afterTakeOff();

            int8_t afterRelease();

            int8_t afterReturnHome();

            int8_t afterAutoLand();

            int8_t afterTravel();

            bool getReady(std::string _type, int _sub);
        }
    }
}

/*************************************************** IMPLEMENTS ***************************************************/

/************************ drone ************************/

int8_t PAPI::drone::CheckStatus::afterTakeOff()
{
    if (PAPI::drone::getState() == UAV_STATE::HOLD)
        return UAV_STATUS::READY;
    return UAV_STATUS::BUSY;
}

int8_t PAPI::drone::CheckStatus::afterRelease()
{
    if (PAPI::drone::getState() == UAV_STATE::ONGROUND)
        return UAV_STATUS::READY;
    return UAV_STATUS::BUSY;
}

int8_t PAPI::drone::CheckStatus::afterReturnHome()
{
    if (PAPI::drone::getState() == UAV_STATE::ONGROUND)
        return UAV_STATUS::READY;
    return UAV_STATUS::BUSY;
}

int8_t PAPI::drone::CheckStatus::afterAutoLand()
{
    if (PAPI::drone::getState() == UAV_STATE::ONGROUND)
        return UAV_STATUS::READY;
    return UAV_STATUS::BUSY;
}

int8_t PAPI::drone::CheckStatus::afterTravel()
{
    if (PAPI::drone::getState() == UAV_STATE::HOLD)
        return UAV_STATUS::READY;
    return UAV_STATUS::BUSY;
}

bool PAPI::drone::CheckStatus::getReady(std::string _type, int _sub)
{
    if (_type == "travel_sequence")
        return (PAPI::drone::CheckStatus::afterTravel() == UAV_STATUS::READY) ? true : false;

    else if (_type == "init_sequence")
        return true;

    else if (_sub == Action::ACTION_AUTOLAND)
        return (PAPI::drone::CheckStatus::afterAutoLand() == UAV_STATUS::READY) ? true : false;

    else if (_sub == Action::ACTION_RELEASE)
        return (PAPI::drone::CheckStatus::afterRelease() == UAV_STATUS::READY) ? true : false;

    else if (_sub == Action::ACTION_RTLHOME)
        return (PAPI::drone::CheckStatus::afterReturnHome() == UAV_STATUS::READY) ? true : false;

    else if (_sub == Action::ACTION_TAKEOFF)
        return (PAPI::drone::CheckStatus::afterTakeOff() == UAV_STATUS::READY) ? true : false;

    return true;
}

int PAPI::drone::getState()
{
    std::string command = "rosservice call /controller/get_mode";
    FILE *pipe = popen(command.c_str(), "r");

    if (!pipe)
    {
        return -1;
    }

    char buffer[16];
    fgets(buffer, 16, pipe);
    std::string state = std::string(buffer);

    return std::stoi(state.substr(8, 1));
}

bool PAPI::drone::takeOffAndHold(const double _height, int8_t _timeout = 50)
{
    if (PAPI::drone::getState() != UAV_STATE::ONGROUND)
    {
        std::cerr << "\n Takeoff failed: Drone state is not ONGROUND.\n";
        return false;
    }

    PAPI::system::serviceCall_setPoint(UAV_STATE::TAKE_OFF, _height, _timeout);

    return true;
}

bool PAPI::drone::hold(int8_t _timeout = 50)
{
    int8_t current_state = PAPI::drone::getState();
    if (current_state != UAV_STATE::TAKE_OFF && current_state != UAV_STATE::MISSION_EXECUTION && current_state != UAV_STATE::AUTO_LAND)
    {
        std::cerr << std::endl
                  << "Hold failed: Drone state is invalid." << std::endl;
        return false;
    }

    PAPI::system::serviceCall_setPoint(UAV_STATE::HOLD, 0, _timeout);

    return true;
}

bool PAPI::drone::missionExecute(std::vector<vector3> _setpoint, int _mask, int8_t _timeout)
{
    int8_t current_state = PAPI::drone::getState();
    if (current_state != UAV_STATE::TAKE_OFF && current_state != UAV_STATE::HOLD)
    {
        std::cerr << std::endl
                  << "Mission Execute failed: Drone state is invalid." << std::endl;
        return false;
    }

    PAPI::system::serviceCall_setPoint(UAV_STATE::MISSION_EXECUTION, 0, _timeout);

    if (!PAPI::system::topicPub_flatSetPoint(_setpoint, _mask))
        return false;

    return true;
}

bool PAPI::drone::autoLand(const double _max_v, int8_t _timeout = 50)
{
    int8_t current_state = PAPI::drone::getState();
    if (current_state != UAV_STATE::TAKE_OFF && current_state != UAV_STATE::MISSION_EXECUTION && current_state != UAV_STATE::HOLD)
    {
        std::cerr << std::endl
                  << "Landing failed: Drone state is invalid." << std::endl;
        return false;
    }

    PAPI::system::serviceCall_setPoint(UAV_STATE::AUTO_LAND, _max_v, _timeout);

    return true;
}

bool PAPI::drone::makeInitInstruction(SingleInstruction *_init_instruction)
{
    // SingleInstruction init_instruction = *_init_instruction;
    std::cout << std::endl
              << "=======================================" << std::endl
              << _init_instruction->name << ":" << std::endl;

    std::vector<int> peripheral_list;
    _init_instruction->Init_getPeripherals(peripheral_list);
    bool peripheral_status = PAPI::drone::peripheralsCheck();

    // for (int index = 0; index < peripheral_list.size(); index++)
    // {
    //     if (!PAPI::drone::turnOnPeripheral(peripheral_list[index]))
    //     {
    //         peripheral_status = false;
    //         std::cerr << "Fail to turn on no." << index << "peripheral." << std::endl;
    //     }
    // }

    if (!peripheral_status)
        return false;
    else
        std::cout << "All Peripherals turn on successful." << std::endl;

    std::cout << "Controller: " << _init_instruction->Init_getController() << std::endl;
    std::cout << "Terminator: " << _init_instruction->Init_getTerminator() << std::endl;

    return true;
}

bool PAPI::drone::makeActionInstruction(SingleInstruction *_action_instruction)
{
    int action = _action_instruction->Action_getAction();

    std::cout << std::endl
              << "=======================================" << std::endl
              << _action_instruction->name << ": " << action << std::endl;

    switch (action)
    {
    case Action::ACTION_UNSPECIFIED:
        std::cerr << "Action Unspecified." << std::endl;
        return false;

    case Action::ACTION_TAKEOFF:
        if (!PAPI::drone::takeOffAndHold(_action_instruction->Action_getParam()))
        {
            std::cerr << "Fail to Take Off." << std::endl;
            return false;
        }
        break;

    case Action::ACTION_DISARM:
        break;

    case Action::ACTION_SELFCHECK:
        break;

    case Action::ACTION_RELEASE:
        break;

    case Action::ACTION_RTLHOME:
        break;

    case Action::ACTION_HOLD:
        // if (!PAPI::drone::hold())
        // {
        //     std::cerr << "Fail to Hold." << std::endl;
        //     return false;
        // }
        if (PAPI::drone::getState() != UAV_STATE::HOLD)
        {
            std::cerr << "Fail to Hold." << std::endl;
            return false;
        }
        // sleep(static_cast<int>(_action_instruction->Action_getParam()));

        for (int time = 0; time < static_cast<int>(_action_instruction->Action_getParam()); time++)
        {
            sleep(1);
            std::cerr << "Hover time: " << time + 1 << " second(s)." << std::endl;
        }

        break;

    case Action::ACTION_AUTOLAND:
        if (!PAPI::drone::autoLand(_action_instruction->Action_getParam()))
        {
            std::cerr << "Fail to Land." << std::endl;
            return false;
        }
        break;

    default:
        std::cerr << "Action unknow." << std::endl;
        return false;
        break;
    }

    return true;
}

bool PAPI::drone::makeTravelInstruction(SingleInstruction *_travel_instruction)
{
    int planner = _travel_instruction->Travel_getPlanner();
    std::cout << std::endl
              << "=======================================" << std::endl
              << _travel_instruction->name << ": "
              << "Planner: " << planner << std::endl;

    std::vector<vector3> waypoints;
    _travel_instruction->Travel_getWaypoints(waypoints);

    if (!PAPI::drone::missionExecute(waypoints, waypoints.size() - 1, 0))
    {
        std::cerr << "The attempt to move to the requested location was unsuccessful." << std::endl;
        return false;
    }

    return true;
}

bool PAPI::drone::turnOnPeripheral(const int _peripheral)
{
    switch (_peripheral)
    {
    case Peripheral::PERIPHERAL_CAM_DOWNWARD:
        /* code */
        break;

    case Peripheral::PERIPHERAL_CAM_FORWARD:
        /* code */
        break;

    case Peripheral::PERIPHERAL_CAM_ODOM:
        /* code */
        break;

    default:
        break;
    }

    return true;
}

bool PAPI::drone::makeInstruction(SingleInstruction *_instruction)
{
    if (_instruction->name == "init_sequence")
    {
        if (!PAPI::drone::makeInitInstruction(_instruction))
        {
            std::cerr << "Fail to make Init Instruction" << std::endl;
            return false;
        }

        bool endOfInstruction = false;
        while (!endOfInstruction)
        {
            endOfInstruction = PAPI::drone::CheckStatus::getReady("init_sequence", INT8_MIN);
        }

        return true;
    }
    else if (_instruction->name == "action_sequence")
    {
        if (!PAPI::drone::makeActionInstruction(_instruction))
        {
            std::cerr << "Fail to make Action Instruction" << std::endl;
            return false;
        }

        bool endOfInstruction = false;
        while (!endOfInstruction)
        {
            endOfInstruction = PAPI::drone::CheckStatus::getReady("action_sequence", _instruction->Action_getAction());
        }

        return true;
    }
    else if (_instruction->name == "travel_sequence")
    {
        if (!PAPI::drone::makeTravelInstruction(_instruction))
        {
            std::cerr << "Fail to make Travel Instruction" << std::endl;
            return false;
        }

        bool endOfInstruction = false;
        while (!endOfInstruction)
        {
            endOfInstruction = PAPI::drone::CheckStatus::getReady("travel_sequence", INT8_MIN);
        }

        return true;
    }

    std::cerr << "Invalid Instruction." << std::endl;
    return false;
}

bool PAPI::drone::peripheralsCheck()
{

    return false;
}

/*********************** system ************************/

void PAPI::system::runCommand_execvp(const std::string &_command, const std::vector<std::string> _argv)
{
    std::string logs = "";
    pid_t pid = fork();
    if (pid == 0)
    {
        /* Child process */
        std::string command = _command;
        char *cmd = const_cast<char *>(command.c_str()); // const char *__file

        char **argv = new char *[_argv.size() + 2]; // char *const *__argv
        argv[0] = const_cast<char *>(command.c_str());
        for (int i = 0; i < _argv.size(); i++)
            argv[i + 1] = const_cast<char *>(_argv[i].c_str());
        argv[_argv.size() + 1] = NULL;

        execvp(cmd, argv);

        logs = "Failed to execute " + std::string(cmd) + ".\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);

        exit(1);
    }
    else if (pid > 0)
    {
        /* Parent process */
        logs = "Command started in process " + std::to_string(pid) + ".\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);
    }
    else
    {
        // Fork failed
        logs = "Failed to fork.\n";

        exit(1);
    }

    /* The code after fork() call goes here */
}

void PAPI::system::runCommand_system(const std::string &_command, const std::vector<std::string> _argv)
{
    std::string logs = "";
    std::string command_with_args;

    std::stringstream ss;
    ss << _command << " ";
    for (std::string arg : _argv)
        ss << arg << " ";

    command_with_args = ss.str();
    command_with_args.pop_back();

    std::thread thr(command_sys, command_with_args); // Create a new thread to run the command

    /* Do other stuff in the main thread */
    std::thread::id thr_id = thr.get_id();
    ss.clear();
    ss << "Command started in thread " << thr_id << ".";
    PAPI::communication::writeLogFile(ss.str(), PAPI::log_file);

    // thr.join(); // Wait for the thread to complete
    thr.detach(); // Detach the thread to run independently
}

void PAPI::system::command_sys(const std::string &_command)
{
    std::system(_command.c_str());
}

void PAPI::system::createLogsFile(const std::string &_path_to_logs_dir)
{
    std::string command = "touch ";

    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm *timeinfo = std::localtime(&time);

    // Convert the tm struct to a formatted string
    std::stringstream ss;
    ss << std::put_time(timeinfo, "%Y_%m_%d_%H_%M_%S");
    // command = command + _path_to_logs_dir + "/" + ss.str() + ".log";
    std::string logsFile = _path_to_logs_dir + "/" + ss.str() + ".log";
    command = command + logsFile;

    std::system(command.c_str());

    PAPI::log_file.open(logsFile, std::fstream::out);
}

void PAPI::system::closeLogsFile()
{
    PAPI::log_file.close();
}

int PAPI::system::getPID_pgrep(const std::string &_node_name)
{
    std::string command_system = "pgrep -f " + _node_name; // Contruct the command to run without ROS master
    int pid_system;

    FILE *pipe;
    std::string pid_str = "";
    char buffer[128];
    std::string result = "";
    std::string logs = "";

    // Open a pipe to run the command and read the output
    pipe = popen(command_system.c_str(), "r");
    if (!pipe)
    {
        logs = "Failed to run command " + command_system + ".\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);
        return -1;
    }

    while (!feof(pipe))
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;

    if (result.empty())
    {
        PAPI::communication::writeLogFile("Not found, maybe the node is not active.\n", PAPI::log_file);
        return -1;
    }

    // Extract the system PID from the output
    pid_str = result.substr(0, result.find_first_of("\r\n"));
    pid_system = std::stoi(pid_str);

    pclose(pipe);

    return pid_system;
}

int PAPI::system::getPID(const std::string &_node_name)
{
    std::string node_name = _node_name; // The name of the node to get the PID of

    std::string command_rosnode = "rosnode info " + node_name + " | grep Pid"; // Construct the command to run with ROS master

    char buffer[128];
    std::string result = "";
    std::string logs = "";

    std::string pid_str = "";
    int pid_master;
    FILE *pipe;

    // Open a pipe to run the command and read the output
    pipe = popen(command_rosnode.c_str(), "r");

    if (!pipe)
    {
        logs = "Failed to run command " + command_rosnode + ".\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);

        return getPID_pgrep(_node_name);
    }

    while (!feof(pipe))
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;

    // Extract the ERROR from the output
    size_t error = result.find("ERROR");
    if (error != std::string::npos)
        return getPID_pgrep(_node_name);

    // Extract the system PID from the output
    pid_str = result.substr(result.find(":") + 2, result.length() - 1);
    pid_master = std::stoi(pid_str);

    return pid_master;
}

bool PAPI::system::checkStatus(const std::string &_node_name)
{
    return getPID(_node_name) > 0 ? true : false;
}

void PAPI::system::killNode_system(const std::string &_node_name)
{
    std::string pid_str = std::to_string(getPID(_node_name));
    std::string kill_command = "kill -9 " + pid_str;

    std::system(kill_command.c_str());
}

void PAPI::system::killNode_name(const std::string &_node_name)
{
    std::string kill_command_rosnode = "rosnode kill /" + _node_name;

    FILE *pipe = popen(kill_command_rosnode.c_str(), "r");
    if (!pipe)
    {
        // If the rosnode command dont run, kill node by kill command
        killNode_system(_node_name);
        return;
    }

    char buffer[128];
    std::string result = "";
    while (!feof(pipe))
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;

    // Extract the ERROR from the output, if get ERROR, kill node by kill command
    size_t error = result.find("ERROR");
    if (error != std::string::npos)
        killNode_system(_node_name);

    pclose(pipe);
}

std::vector<std::string> PAPI::system::getNodeList()
{
    std::vector<std::string> result;
    std::string logs = "";

    std::string command = "rosnode list";
    FILE *pipe = popen(command.c_str(), "r");

    if (!pipe)
    {
        logs = "Failed to execute " + command + ".\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);

        return result;
    }

    char buffer[2048];
    while (fgets(buffer, 2048, pipe))
    {
        std::string temp_string = std::string(buffer);
        result.push_back(temp_string.substr(1, temp_string.size() - 2));
    }

    return result;
}

std::vector<std::string> PAPI::system::getTopicList()
{
    std::vector<std::string> result;
    std::string logs = "";

    std::string command = "rostopic list";
    FILE *pipe = popen(command.c_str(), "r");

    if (!pipe)
    {
        logs = "Failed to execute " + command + ".\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);

        return result;
    }

    char buffer[2048];
    while (fgets(buffer, 2048, pipe))
    {
        std::string temp_string = std::string(buffer);
        result.push_back(temp_string.substr(1, temp_string.size() - 2));
    }

    return result;
}

std::vector<std::string> PAPI::system::getServiceList()
{
    std::vector<std::string> result;
    std::string logs = "";

    std::string command = "rosservice list";
    FILE *pipe = popen(command.c_str(), "r");

    if (!pipe)
    {
        logs = "Failed to execute " + command + ".\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);

        return result;
    }

    char buffer[2048];
    while (fgets(buffer, 2048, pipe))
    {
        std::string temp_string = std::string(buffer);
        result.push_back(temp_string.substr(1, temp_string.size() - 2));
    }

    return result;
}

void PAPI::system::threadSleeper(const int _time)
{
    std::chrono::seconds pauseTime(_time);
    std::this_thread::sleep_for(pauseTime);
}

std::vector<std::string> PAPI::system::getPIDList(const std::string _command_name)
{
    std::vector<std::string> result;
    std::string logs = "";

    std::string command = "pgrep -f " + _command_name;
    FILE *pipe = popen(command.c_str(), "r");

    if (!pipe)
    {
        logs = "Failed to execute " + command + ".\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);

        return result;
    }

    char buffer[2048];
    while (fgets(buffer, 2048, pipe))
    {
        std::string temp_string = std::string(buffer);
        temp_string.pop_back();
        result.push_back(temp_string);
    }

    // The last PID is the PID of "pgrep" command.
    result.pop_back();

    return result;
}

void PAPI::system::serviceCall_setPoint(const int _mode, const double _sub, const int8_t _timeout)
{
    std::string service_cmd;               // rosservice command.
    std::vector<std::string> service_argv; // rosservice arguments vector.

    service_cmd = "rosservice";
    service_argv.push_back("call");
    service_argv.push_back("/controller/set_mode");
    std::stringstream ss;
    ss << "\"mode: " << _mode << "\nsub : " << _sub << "\ntimeout: " << _timeout << "\"";
    service_argv.push_back(ss.str());

    PAPI::system::runCommand_system(service_cmd, service_argv);

    service_cmd.clear();
    service_argv.clear();
}

bool PAPI::system::topicPub_flatSetPoint(std::vector<vector3> _setpoint, int _mask)
{
    if (_mask < 0 || _setpoint.size() < 1)
    {
        std::cerr << "Input setpoint is empty." << std::endl;
        return false;
    }

    std::string topic_cmd;               // rostopic command.
    std::vector<std::string> topic_argv; // rostopic arguments vector.

    topic_cmd = "rostopic";
    topic_argv.push_back("pub");
    topic_argv.push_back("/controller/flatsetpoint");
    topic_argv.push_back("controller_msgs/FlatTarget");

    int count_mask = 0;
    std::stringstream ss;

    // ss << "\"header:\n  seq: 0\n  stamp: {secs: 0, nsecs: 0} \n  frame_id: \'\'" << std::endl
    ss << "\"header:" << std::endl
       << " seq: 0" << std::endl
       << " stamp: {secs: 0, nsecs: 0}" << std::endl
       << " frame_id: \'\'" << std::endl
       << "type_mask: " << _mask << std::endl
       << "position: {x: " << _setpoint[count_mask][0] << ", y: " << _setpoint[count_mask][1] << ", z: " << _setpoint[count_mask][2] << "}" << std::endl;
    ++count_mask;

    if (count_mask <= _mask && count_mask == 1)
    {
        ss << "velocity: {x: " << _setpoint[count_mask][0] << ", y: " << _setpoint[count_mask][1] << ", z: " << _setpoint[count_mask][2] << "}" << std::endl;
        ++count_mask;
    }
    else
    {
        ss << "velocity: {x: 0.0, y: 0.0, z: 0.0}" << std::endl;
    }

    if (count_mask <= _mask && count_mask == 2)
    {
        ss << "acceleration: {x: " << _setpoint[count_mask][0] << ", y: " << _setpoint[count_mask][1] << ", z: " << _setpoint[count_mask][2] << "}" << std::endl;
        ++count_mask;
    }
    else
    {
        ss << "acceleration: {x: 0.0, y: 0.0, z: 0.0}" << std::endl;
    }

    if (count_mask <= _mask && count_mask == 3)
    {
        ss << "jerk: {x: " << _setpoint[count_mask][0] << ", y: " << _setpoint[count_mask][1] << ", z: " << _setpoint[count_mask][2] << "}" << std::endl;
        ++count_mask;
    }
    else
    {
        ss << "jerk: {x: 0.0, y: 0.0, z: 0.0}" << std::endl;
    }

    if (count_mask <= _mask && count_mask == 4)
    {
        ss << "snap: {x: " << _setpoint[count_mask][0] << ", y: " << _setpoint[count_mask][1] << ", z: " << _setpoint[count_mask][2] << "}\"" << std::endl;
        ++count_mask;
    }
    else
    {
        ss << "snap: {x: 0.0, y: 0.0, z: 0.0}\"";
    }

    //    << "velocity: {x: 0.0, y: 0.0, z: 0.0}" << std::endl
    //    << "acceleration: {x: 0.0, y: 0.0, z: 0.0}" << std::endl
    //    << "jerk: {x: 0.0, y: 0.0, z: 0.0}" << std::endl
    //    << "snap: {x: 0.0, y: 0.0, z: 0.0}\"";
    topic_argv.push_back(ss.str());

    // std::cout << std::endl
    //           << ss.str() << std::endl;

    PAPI::system::runCommand_system(topic_cmd, topic_argv);

    topic_cmd.clear();
    topic_argv.clear();

    return true;
}

bool PAPI::system::jsonParsing(const std::string _path_to_json_file, MissionRequest &_mission)
{
    if (jsonParsing::parsing(_path_to_json_file, _mission))
    {
        std::cout << "*****************************" << std::endl
                  << "*     PARSING SUCCESSFUL    *" << std::endl
                  << "*****************************" << std::endl;
        return true;
    }
    return false;
}

void PAPI::system::sendImage(const int _device)
{
    std::string curl_cmd = "curl";
    std::vector<std::string> curl_argv;
    curl_argv.push_back("-X POST -F");

    std::stringstream ss;
    ss << "\"image=@" << DEFAULT_IMAGE_DIR_PATH << "/";
    switch (_device)
    {
    case Peripheral::PERIPHERAL_CAM_DOWNWARD:
        ss << "flir_image.png\"";
        break;

    case Peripheral::PERIPHERAL_CAM_FORWARD:
        ss << "d455_image.png\"";
        break;

    default:
        break;
    }
    curl_argv.push_back(ss.str());

    // ss.clear();
    std::stringstream ss1;
    ss1 << "http://localhost:5000/upload";
    curl_argv.push_back(ss1.str());

    curl_argv.push_back("http://localhost:5000/upload");

    PAPI::system::runCommand_system(curl_cmd, curl_argv);
}

/******************** communication ********************/

PAPI::communication::Server::Server() {}

PAPI::communication::Server::Server(int _port) : port(_port) {}

int PAPI::communication::Server::serverStart()
{
    socklen_t len;
    struct sockaddr_in server_address, client_address;

    // create socket
    server_socket = socket(AF_INET, SOCK_STREAM, 0);

    // configure server address
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(port);

    // bind socket to address
    bind(server_socket, (struct sockaddr *)&server_address, sizeof(server_address));

    // listen for connections
    listen(server_socket, 5);

    PAPI::communication::writeLogFile("Waiting for client connection...\n", PAPI::log_file);

    // accept client connection
    len = sizeof(client_address);

    // Set server socket to non-blocking mode
    int flags = fcntl(server_socket, F_GETFL, 0);
    fcntl(server_socket, F_SETFL, flags | O_NONBLOCK);

    // Record the starting time
    auto startTime = std::chrono::high_resolution_clock::now();

    // Set the timeout duration
    auto timeoutDuration = std::chrono::seconds(DEFAULT_CONNECTION_TIMEOUT);

    // Loop until a client connects or the timeout occurs
    while (true)
    {
        // Check if the timeout has occurred
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedDuration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
        if (elapsedDuration >= timeoutDuration)
        {
            std::cerr << "Timeout: No client connected within 60 seconds." << std::endl;
            PAPI::communication::writeLogFile("Server timeout: no client connected.\n", PAPI::log_file);
            close(server_socket);
            return -1;
        }

        // Try to accept a client connection
        client_socket = accept(server_socket, (struct sockaddr *)&client_address, &len);

        // Check if a client connected successfully
        if (client_socket != -1)
        {
            // Restore server socket to blocking mode
            fcntl(server_socket, F_SETFL, flags);

            std::cout << "Connected to client.\n";

            break; // Exit the loop if a client connected
        }
    }

    PAPI::communication::writeLogFile("Connected to client.\n", PAPI::log_file);
    return 0;
}

void PAPI::communication::Server::sendMsg(const std::string &_str)
{
    std::string message = _str;
    char buffer[1024];

    strncpy(buffer, message.c_str(), sizeof(buffer));
    send(client_socket, buffer, strlen(buffer), 0);
}

void PAPI::communication::Server::serverClose()
{
    // Close socket
    close(server_socket);
    PAPI::communication::writeLogFile("Server closed.\n", PAPI::log_file);
}

int PAPI::communication::Server::getServerSocket()
{
    return server_socket;
}

int PAPI::communication::Server::getClientSocket()
{
    return client_socket;
}

int PAPI::communication::Server::getPort()
{
    return port;
}

/*****************/

PAPI::communication::Client::Client() {}

PAPI::communication::Client::Client(const std::string _server_ip, const int _server_port) : server_ip(_server_ip),
                                                                                            server_port(_server_port)
{
}

PAPI::communication::Client::Client(const int _server_socket) : server_socket(_server_socket) {}

int PAPI::communication::Client::clientStart()
{
    struct sockaddr_in server_address;

    // create socket
    client_socket = socket(AF_INET, SOCK_STREAM, 0);

    // configure server address
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(server_port);
    inet_pton(AF_INET, server_ip.c_str(), &server_address.sin_addr);

    // Record the starting time
    auto startTime = std::chrono::high_resolution_clock::now();
    // Set the timeout duration
    auto timeoutDuration = std::chrono::seconds(DEFAULT_CONNECTION_TIMEOUT);

    // try to connect to server

    int connect_result = connect(client_socket, (struct sockaddr *)&server_address, sizeof(server_address));
    // Check if the timeout has occurred
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto elapsedDuration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);

    while (connect_result != 0 && elapsedDuration <= timeoutDuration)
    {
        connect_result = connect(client_socket, (struct sockaddr *)&server_address, sizeof(server_address));
        currentTime = std::chrono::high_resolution_clock::now();
        elapsedDuration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
    }

    return (connect_result == 0) ? 0 : -1;
}

void PAPI::communication::Client::clientClose()
{
    close(client_socket);
}

std::string PAPI::communication::Client::reciveMessage()
{
    char buffer[4096] = {0};

    // Recive message
    recv(client_socket, buffer, sizeof(buffer), 0);

    std::string result(buffer, sizeof(buffer));
    return result;
}

std::string PAPI::communication::Client::getServerIP()
{
    return server_ip;
}

int PAPI::communication::Client::getServerPort()
{
    return server_port;
}

int PAPI::communication::Client::getServerSocket()
{
    return server_socket;
}

int PAPI::communication::Client::getClientSocket()
{
    return client_socket;
}

/*****************/

PAPI::communication::UnixSocket::UnixSocket(const std::string &path) : system_path(path)
{
    std::string logs;

    // Create a Unix domain socket
    sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sockfd == -1)
    {
        logs = "Failed to create socket.\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);
        logs.clear();

        throw std::runtime_error("Failed to create socket.\n");
    }

    // Set the socket address
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1);

    // Connect to the socket
    int result = connect(sockfd, (struct sockaddr *)&address, sizeof(address));
    if (result == -1)
    {
        logs = "Failed to connect to socket.\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);
        logs.clear();

        throw std::runtime_error("Failed to connect to socket.\n");
    }

    // Set the receive timeout to 1 second
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    result = setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));
    if (result == -1)
    {
        logs = "Failed to set receive timeout.\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);
        logs.clear();

        throw std::runtime_error("Failed to set receive timeout.\n");
    }
}

PAPI::communication::UnixSocket::~UnixSocket()
{
    // Close the socket
    close(sockfd);
}

void PAPI::communication::UnixSocket::sendString(const std::string &message)
{
    // Send the message
    int result = send(sockfd, message.c_str(), message.length(), 0);
    if (result == -1)
    {
        throw std::runtime_error("Failed to send message.\n");
    }
}

std::string PAPI::communication::UnixSocket::receiveString()
{
    std::string logs;
    const int MAX_LENGTH = 1024;
    char buffer[MAX_LENGTH];
    // Receive the message
    int result = recv(sockfd, buffer, MAX_LENGTH - 1, 0);

    if (result == -1 && errno == EAGAIN)
    {
        logs = "Receive timed out.\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);
        logs.clear();

        throw std::runtime_error("Receive timed out.\n");
    }
    else if (result == -1)
    {
        throw std::runtime_error("Failed to receive message.\n");
    }

    buffer[result] = '\0';
    return std::string(buffer);
}

/*****************/
void PAPI::communication::writeLogFile(const std::string &_log, std::ofstream &_logFile)
{
    std::string add_time = "[" + getCurrentTime() + "] " + _log;

    // Create a stringstream object to store the logs
    std::stringstream ss(add_time);

    // Process each log entry
    std::string logEntry;

    while (std::getline(ss, logEntry))
        // Write the log entry to the file
        _logFile << logEntry << std::endl;
}

std::string PAPI::communication::getCurrentTime()
{
    // Get the current time
    auto currentTime = std::chrono::system_clock::now();

    // Convert the time to a timepoint
    std::time_t currentTime_t = std::chrono::system_clock::to_time_t(currentTime);

    // Convert the timepoint to a tm struct
    std::tm *currentTime_tm = std::localtime(&currentTime_t);

    // Extract the milliseconds
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime.time_since_epoch()) % 1000;

    std::string result = std::to_string(currentTime_tm->tm_hour) + ":" + std::to_string(currentTime_tm->tm_min) + ":" + std::to_string(currentTime_tm->tm_sec) + ":" + std::to_string(ms.count());

    return result;
}

void PAPI::communication::transferLog(const std::string &_log, std::ofstream &_logFile, const PAPI::communication::Server &_server)
{
    PAPI::communication::writeLogFile(_log, _logFile);

    PAPI::communication::Server server(_server);
    server.sendMsg(_log);
}

#endif