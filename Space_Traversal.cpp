#include "System_header.hpp"

string movement_command, move_instruction;

bool flagMOV_ready = false;
bool SeeK_n_Destroy = false;
class Coordinates{
    public:
    set<pair<uint16_t, float>> Directions;
        
    pair<uint16_t, float> Get_Path(){
        set<pair<uint16_t, float>>::iterator itr = Directions.end();
        return make_pair((*itr).first, (*itr).second);
    }

};



void reCallibrate(Orientation& move_label){
    //move 40* to the right
    move_label = reCalirbate;
    unique_lock<mutex> ul1(global_mx1);
    
    Construct_Command(move_instruction, move_label);

    ul1.unlock();
    gl_Conditonal1.notify_one();
    ul1.lock();
    
    gl_Conditonal1.wait(ul1, [](){return flagMOV_ready == false;});
    move_label = Scan_Area;
}



void Init_MoveCommand(string& move_instruction, Orientation& move_label, Action_Mode& Robot_Condition){
    if(move_label == Scan_Area && Robot_Condition == Lost_in_Space)
    {
        reCallibrate(move_label);
        Scan_Surrounding(flagMOV_ready, move_label);
        Robot_Condition = Beacon_Aprox;
    }

    else if(move_label == Intermission && Robot_Condition == Beacon_Aprox){
        unique_lock<mutex> ul1(Head_mx);
        move_label = Flip;
        Construct_Command(move_instruction, move_label);
        SeeK_n_Destroy = true;

        ul1.unlock();
        Head_cv.notify_one();
        ul1.lock();
        
        Head_cv.wait(ul1, [&](){return flagMOV_ready == false;});

        //---------DELETE THE FOWLLOWING------//
        move_label = Scan_Area;
        Robot_Condition = Lost_in_Space;
        
    }

    else if(ROBOT_OPERATIONAL == false || Robot_Condition == Something_went_Wrong){
        Trivial_error_txt("[System_Core] :: Space Treversal Failed");
        return;
    }
    // I feel like I forgot something here
}
 



void Scan_Surrounding(bool& flagMOV_ready, Orientation& move_label){
    Coordinates compas;
    float current_angle = 0;
    unique_lock<mutex>ul1(global_mx1);
        
            for(uint16_t j=0; j<(Angles_to_scan * 2); ++j)
            {
                // if the robot does collisions... the problem is here
                compas.Directions.insert(make_pair(current_angle,Global_Ultrasonic.load()));
                
                Construct_Command(move_instruction, move_label);
                ul1.unlock();
                gl_Conditonal1.notify_one();
                //why am I locking this?
                ul1.lock();
                gl_Conditonal1.wait(ul1, [&](){return flagMOV_ready == false;});

                current_angle += (SSA/2);
            }
        
        ul1.~unique_lock();
        calculate_path(compas.Get_Path(), move_label);
}



pair<uint16_t, float> calculate_path(pair<uint16_t, float> path, Orientation& move_label){
    unique_lock<mutex> ul1(global_mx1);
    
        angle_of_attack = path.first;
        move_label = Left;
        Construct_Command(move_instruction, move_label);

        ul1.unlock();
        gl_Conditonal1.notify_one();
        ul1.lock();
        gl_Conditonal1.wait(ul1, [&](){return flagMOV_ready == false;});

        angle_of_attack.store(static_cast<int>(path.second));
        move_label = Forward;
  
        Construct_Command(move_instruction, move_label);

        ul1.unlock();
        gl_Conditonal1.notify_one();
        ul1.lock();
        gl_Conditonal1.wait(ul1, [&](){return flagMOV_ready == false;});
        
        angle_of_attack = 0.0;
        move_label = Intermission;

        ul1.~unique_lock();
}



void Construct_Command(string& move_instruction, Orientation& move_label){
    move_instruction.clear();
    switch (move_label)
    {
        case Scan_Area:
            move_instruction.append(to_string(DC_motor_SPEED) + " NO right " + to_string(Turn_Radious)+ " " + to_string(SSA/2));
            break;
        case reCalirbate:
            move_instruction.append(to_string(DC_motor_SPEED) + " NO left " + to_string(Turn_Radious) + " " + to_string(SSA));
            break;
        case Flip:
            move_instruction.append(to_string(DC_motor_SPEED) + " NO right "+ to_string(Turn_Radious)+  " " +to_string(time_for_spin/2));
            break;

        case Right:
            move_instruction.append(to_string(DC_motor_SPEED) + " NO right "+ to_string(Turn_Radious)+  " " +to_string(time_for_spin/2));
            break;
        case Left:
            move_instruction.append(to_string(DC_motor_SPEED) + " NO left "+ to_string(Turn_Radious)+ " " +to_string(time_for_spin/angle_of_attack));
            break;
        case Backward:
            move_instruction.append(to_string(DC_motor_SPEED) + " backward NO "+ to_string(Turn_Radious)+ " how much distance? ");
            break;
        case Forward:
            // move_instruction.append(to_string(DC_motor_SPEED) + " forward NO "+ to_string(Turn_Radious)+ " " +to_string(angle_of_attack.load() * time_for_movement));
            //currently bugged
            move_instruction.append(to_string(DC_motor_SPEED) + " forward NO "+ to_string(Turn_Radious)+ " " +to_string(2.0));
            break;
        case Forward_right:
            move_instruction.append(to_string(DC_motor_SPEED) + " forward right "+ to_string(Turn_Radious)+ " how much distance? ");
            break;
        case Forward_Left:
            move_instruction.append(to_string(DC_motor_SPEED) + " forward left "+ to_string(Turn_Radious)+ " how much distance? ");
            break;
        case Backward_Right:
            move_instruction.append(to_string(DC_motor_SPEED) + " backward right "+ to_string(Turn_Radious)+ " how much distance? ");
            break;
        case Backward_left:
            move_instruction.append(to_string(DC_motor_SPEED) + " backward left "+ to_string(Turn_Radious)+ " how much distance? ");
            break;

        default:
            break;
    }

    flagMOV_ready = true;
}