/* Compile:
    g++ test.cpp -o test -pthread -ljsoncpp
*/

#include "../PAPI.h"

// auto parseInstruction(SingleInstruction *_instruction)
// {
//     if (_instruction->name == "init_instruction")
//         {
//             InitInstruction result = _instruction;
//         }
// }

// Init the system, contain px4, mavros and geometric_controller
bool initWorldAndVehicle()
{
    std::string px4_cmd = "roslaunch";
    std::vector<std::string> px4_argv;
    px4_argv.push_back("px4");
    px4_argv.push_back("posix_sitl.launch");
    px4_argv.push_back("> /home/pino/logs/roslaunch_logs/px4_log.log");
    // px4_argv.push_back("2>&1 &");

    std::string mavros_cmd = "roslaunch";
    std::vector<std::string> mavros_argv;
    mavros_argv.push_back("mavros");
    mavros_argv.push_back("px4.launch");
    mavros_argv.push_back("fcu_url:=\"udp://:14540@localhost:14557\"");
    mavros_argv.push_back("> /home/pino/logs/roslaunch_logs/mavros_log.log");
    // mavros_argv.push_back("2>&1 &");

    std::string controller_cmd = "roslaunch";
    std::vector<std::string> controller_argv;
    controller_argv.push_back("geometric_controller");
    controller_argv.push_back("automatic.launch");
    controller_argv.push_back("> /home/pino/logs/roslaunch_logs/controller_log.log");
    // controller_argv.push_back("2>&1 &");

    /*************************************************/

    PAPI::system::runCommand_system(px4_cmd, px4_argv);
    sleep(15); // Low-end System :)

    PAPI::system::runCommand_system(mavros_cmd, mavros_argv);
    sleep(20); // Low-end System :)

    PAPI::system::runCommand_system(controller_cmd, controller_argv);
    sleep(5); // Low-end System :)

    return true;
}

MissionRequest mission;

int main()
{
    // SequenceItems sequence;
    // InitInstruction init_instruction;
    // TravelInstruction travel_instruction;
    // ActionInstruction action_instruction;

    // init_instruction.name = "init_sequence";
    // init_instruction.controller = Controller::CONTROLLER_A_ADRJ;

    // travel_instruction.name = "travel_sequence";
    // travel_instruction.planner = Planner::PLANNER_EGO;

    // action_instruction.name = "action_sequence";
    // action_instruction.param = 5;

    // sequence.addInstruction(&init_instruction);
    // sequence.addInstruction(&travel_instruction);
    // sequence.addInstruction(&action_instruction);

    // std::cout << std::endl
    //           << sequence.instructions[0].name << std::endl
    //           << sequence.instructions[1].name << std::endl
    //           << sequence.instructions[2].name << std::endl;

    /**********************************************************************************************/

    // // Open the JSON file
    // std::ifstream file("../sample/sample.json");
    // if (!file.is_open())
    // {
    //     // Handle file open error
    //     return 1;
    // }

    // // Read the file content into a string
    // std::string jsonContent((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // // Close the file
    // file.close();

    // // Parse the JSON string using a Json::Reader object
    // Json::Value jsonData;
    // Json::Reader jsonReader;
    // if (!jsonReader.parse(jsonContent, jsonData))
    // {
    //     // Handle JSON parsing error
    //     return 1;
    // }

    // // Access the JSON data

    // std::string id = jsonData["id"].asString();
    // int number_sequence_items = jsonData["number_sequence_items"].asInt();
    // std::string description = jsonData["description"].asString();

    // std::cout << id << std::endl
    //           << number_sequence_items << std::endl
    //           << description << std::endl;

    // Json::Value sequence_items = jsonData["sequence_items"];

    // if (sequence_items.isArray())
    // {
    //     // Iterate over the elements in the array
    //     for (unsigned int i = 0; i < sequence_items.size(); i++)
    //     {
    //         Json::Value instruction = sequence_items[i];
    //     }
    // }

    /**********************************************************************************************/

    // MissionRequest mission;

    // if (jsonParsing::parsing("/home/pino/pino_ws/papi/sample/sample.json", mission))
    // {
    //     std::cout << "*****************************" << std::endl
    //               << "*     PARSING SUCCESSFUL    *" << std::endl
    //               << "*****************************" << std::endl;
    // }

    // std::cout << mission.sequence_istructions[0]->Init_getController() << std::endl;

    // int *ptr_peripherals = mission.sequence_istructions[0]->Init_getPointer_Peripherals();
    // std::vector<int> peripheral_list;
    // while (*ptr_peripherals != '\0')
    // {
    //     peripheral_list.push_back(*ptr_peripherals);
    //     std::cout << *ptr_peripherals << " ";
    //     ++ptr_peripherals;
    // }

    // std::cout << std::endl
    //           << peripheral_list.size() << std::endl;

    // std::cout << mission.sequence_istructions[1]->Action_getAction() << std::endl
    //           << mission.sequence_istructions[3]->Action_getAction() << std::endl;

    // std::vector<int> this_peri;
    // std::cout << this_peri.size() << std::endl;

    // mission.sequence_istructions[0]->Init_getPeripherals(this_peri);
    // std::cout << this_peri.size() << std::endl;
    // std::cout << this_peri[1] << std::endl;

    // std::vector<vector3> this_waypoints;
    // mission.sequence_istructions[2]->Travel_getWaypoints(this_waypoints);
    // for (int i = 0; i < this_waypoints.size(); i++)
    // {
    //     for (int j = 0; j < this_waypoints[i].size(); j++)
    //         std::cout << this_waypoints[i][j] << " ";
    //     std::cout << std::endl;
    // }

    // std::cout << mission.sequence_names.size() << " | " << mission.number_sequence_items << std::endl;

    // if (!PAPI::system::jsonParsing("/home/pino/pino_ws/papi/sample/sample.json", mission))
    //     return -1;

    // SingleInstruction* init_instruction = mission.sequence_istructions[0];

    // if (!PAPI::drone::makeInstruction(init_instruction))
    // {
    //     std::cerr << "FAILED.\n";
    // }

    // PAPI::system::createLogsFile("./logs");

    // if (!initWorldAndVehicle())
    // {
    //     std::cerr << "Init World and Vehicle unsuccessful." << std::endl;
    //     return -1;
    // }
    // std::cout << "Init World and Vehicle successful." << std::endl;

    // bool check = PAPI::drone::takeOffAndHold(5);
    // do
    // {
    //     std::cout << "state: " << PAPI::drone::getState() << std::endl;
    // } while (PAPI::drone::getState() != UAV_STATE::HOLD);

    // std::cout << "state: " << PAPI::drone::getState() << std::endl;

    std::vector<vector3> setpoints;
    vector3 position;
    position.push_back(0);
    position.push_back(2);
    position.push_back(2);
    setpoints.push_back(position);

    // PAPI::drone::missionExecute(setpoints, 0, 0);
    PAPI::system::topicPub_flatSetPoint(setpoints, 0);
    
    return 0;
}