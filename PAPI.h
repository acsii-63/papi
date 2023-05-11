#ifndef CONTROL_API_H
#define CONTROL_API_H

#include <cstdlib>
#include <cstdbool>
#include <ctime>
#include <cstring>
#include <unistd.h>
#include <errno.h>

#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/un.h>

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
#include <algorithm>

#define vector3 std::vector<double> // Vector3 contains 3 double variables

/*
WAITING_FOR_HOME_POSE = 0
TAKE_OFF = 1
HOLD = 2
MISSION_EXECUTION = 3
AUTO_LAND = 4
ONGROUND = 5
*/
enum class UAV_STATE
{
    WAITING_FOR_HOME_POSE,
    TAKE_OFF,
    HOLD,
    MISSION_EXECUTION,
    AUTO_LAND,
    ONGROUND
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

        //
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
            void clientStart();

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
    }
}

/*************************************************** IMPLEMENTS ***************************************************/

/************************ drone ************************/

int PAPI::drone::getState()
{
    std::string command = "rosservice call /controller/get_mode";
    FILE *pipe = popen(command.c_str(), "r");

    if (!pipe)
    {
        return -1;
    }

    char buffer[8];
    fgets(buffer, 8, pipe);
    std::string state = std::string(buffer);

    return std::stoi(state);
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

PAPI::communication::Client::Client() {}

PAPI::communication::Client::Client(const std::string _server_ip, const int _server_port) : server_ip(_server_ip),
                                                                                            server_port(_server_port)
{
}

PAPI::communication::Client::Client(const int _server_socket) : server_socket(_server_socket) {}

void PAPI::communication::Client::clientStart()
{
    struct sockaddr_in server_address;

    // create socket
    client_socket = socket(AF_INET, SOCK_STREAM, 0);

    // configure server address
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(server_port);
    inet_pton(AF_INET, server_ip.c_str(), &server_address.sin_addr);

    // connect to server
    connect(client_socket, (struct sockaddr *)&server_address, sizeof(server_address));
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