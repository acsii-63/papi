/* Compile with jsoncpp lib:
    g++ test.cpp -o test -ljsoncpp
*/

#include "../include/mission_v1.h"

// auto parseInstruction(SingleInstruction *_instruction)
// {
//     if (_instruction->name == "init_instruction")
//         {
//             InitInstruction result = _instruction;
//         }
// }

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

    MissionRequest mission;

    if (jsonParsing::parsing("/home/pino/pino_ws/papi/sample/sample.json", mission))
    {
        std::cout << std::endl
                  << "SUCCESSFUL" << std::endl;
    }

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

    return 0;
}