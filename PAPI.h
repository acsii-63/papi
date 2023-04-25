#ifndef CONTROL_API_H
#define CONTROL_API_H

#include <cstdlib>
#include <cstdbool>
#include <ctime>
#include <cstring>
#include <unistd.h>

#include <sys/socket.h>
#include <sys/wait.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <iostream>
#include <sstream>
#include <fstream>

#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <iomanip>

// #include <algorithm>

/**************************************************** DEFINES ****************************************************/

namespace PAPI
{
    // Logs file
    std::ofstream log_file;

    // PAPI::drone
    namespace drone
    {
    }

    // PAPI::system
    namespace system
    {
        // Run a command with given argv in std::vector<std::string> type.
        void runCommand(const std::string &_command, const std::vector<std::string> _argv);

        // Create logs file.
        void createLogsFile(const std::string &_path_to_logs_dir);

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
            void serverStart();

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
            int port;          // Sending port of the serve
            int server_socket; // Server socket
            int client_socket; // Client socket
        };

        // Get the current system time, return in std::string
        std::string getCurrentTime();

        // Write log to .log file
        void writeLogFile(const std::string &_log, std::ofstream &_logFile);
    }
}

/*************************************************** IMPLEMENTS ***************************************************/

/************************ drone ************************/

/*********************** system ************************/

void PAPI::system::runCommand(const std::string &_command, const std::vector<std::string> _argv)
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

        logs = "Failed to execute" + std::string(cmd) + ".\n";
        PAPI::communication::writeLogFile(logs, PAPI::log_file);

        exit(1);
    }
    else if (pid > 0)
    {
        /* Parent process */
        logs = "Command started in process" + std::to_string(pid) + ".\n";
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

void PAPI::system::createLogsFile(const std::string &_path_to_logs_dir)
{
    std::string command = "touch ";

    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm *timeinfo = std::localtime(&time);

    // Convert the tm struct to a formatted string
    std::stringstream ss;
    ss << std::put_time(timeinfo, "%Y_%m_%d_%H_%M_%S");
    command = command + _path_to_logs_dir + "/" + ss.str() + ".log";

    std::system(command.c_str());
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
    while (fgets(buffer, 8192, pipe))
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
    while (fgets(buffer, 8192, pipe))
    {
        std::string temp_string = std::string(buffer);
        result.push_back(temp_string.substr(1, temp_string.size() - 2));
    }

    return result;
}

/******************** communication ********************/

PAPI::communication::Server::Server() {}

PAPI::communication::Server::Server(int _port) : port(_port) {}

void PAPI::communication::Server::serverStart()
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
    client_socket = accept(server_socket, (struct sockaddr *)&client_address, &len);

    PAPI::communication::writeLogFile("Client connected.\n", PAPI::log_file);
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

#endif