#include "System_header.hpp"

void Init_MoveCommand(string& move_instruction, Orientation& mov_condition, Action_Mode& Robot_Condition);
void Scan_Surrounding(bool& flagMOV_ready, Orientation& move_label);
void Construct_Command(string& move_instruction, Orientation& move_label);
pair<uint16_t, float> calculate_path(pair<uint16_t, float> path, Orientation& move_label);

