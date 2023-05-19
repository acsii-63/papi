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
    SequenceItems sequence;
    InitInstruction init_instruction;
    TravelInstruction travel_instruction;
    ActionInstruction action_instruction;

    init_instruction.name = "init_sequence";
    init_instruction.controller = Controller::CONTROLLER_A_ADRJ;

    travel_instruction.name = "travel_sequence";
    travel_instruction.planner = Planner::PLANNER_EGO;

    action_instruction.name = "action_sequence";
    action_instruction.param = 5;

    sequence.addInstruction(&init_instruction);
    sequence.addInstruction(&travel_instruction);
    sequence.addInstruction(&action_instruction);

    std::cout << std::endl
              << sequence.instructions[0].name << std::endl
              << sequence.instructions[1].name << std::endl
              << sequence.instructions[2].name << std::endl;

    return 0;
}