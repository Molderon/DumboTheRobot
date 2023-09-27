#include "System_header.hpp"
#include "Robot.hpp"

atomic <float> Global_Ultrasonic(0);
atomic <bool> ROBOT_OPERATIONAL(true);
atomic <float> angle_of_attack(0.0);

unordered_map<string,pair<string, uint16_t>>Port_map{
                {"Ultra_Sonic", make_pair("*:",5556)},
                {"Space-Traversal",make_pair("*:",5567)},
                {"General_Instruction_Set",make_pair("127.0.1.1",1111)},
                {"System_LED", make_pair("*:",5555)},
                {"Head_Module", make_pair("*:",5558)}
        };

mutex global_mx1, Error_mx;
condition_variable gl_Conditonal1, Error_cv;


class Vortal
{
    private:
         deque<pair<thread*, bool>>::iterator thread_itr;
         deque<pair<thread*, bool>> Thread_Handler;
         struct System_Report{
            bool System_Intermission, 
                 Sufficient_Memory,
                 CPU_load_sufficient;
                 string date, geolocation;
         };
         
        bool is_Full_Launch;

    public:  
        static void Launch(Robot* robot)
        {
        //run a threads that calibrates the camera, loads aruco data, open a channel that oversiers Python and C++
          switch (robot->get_secondary_systems())
              {
              case false:
                  ROBOT_OPERATIONAL = true;
                  robot->Behavour_Nominal(robot->get_ref_rn_naviation());
                  break;
              case true:
                  ROBOT_OPERATIONAL = true;
                  robot->Behavour_FULL();
                  break;
              }
        }


        Vortal(Robot* robot){
            this->is_Full_Launch = robot->get_secondary_systems();
            System_Report Report;
            /*
            Generate Report for host utilizations
            -> CPU
            -> Memory
            -> Operating System
            -> date & geolocation (if possible)
            */
        }

         ~Vortal(){
            ROBOT_OPERATIONAL = false;
            this->thread_itr = this->Thread_Handler.begin();

            for(thread_itr = Thread_Handler.begin(); thread_itr != Thread_Handler.end(); thread_itr++){
                if((*thread_itr).second == false){
                    if((*thread_itr).first->joinable()){(*thread_itr).first->join();}
                }
                else{
                    (*thread_itr).first->~thread();
                    //ahh verry elegan'te <3
                }
                Thread_Handler.erase(thread_itr);
            }
        }
};



void Dusk_and_Her_Embrace(shared_ptr<Vortal>& System_Shock, Robot* robot){
    // Implement exeption handeling, and function backtracking
    // Clean-up Properly, use system Signals   
    (*System_Shock).~Vortal();
    robot->~Robot();
}



void Port_Scan(unordered_map<string, pair<string, uint16_t>>& Port_map)
{
    unordered_map<string,pair<string,uint16_t>>::iterator itr = Port_map.begin();
    vector<pair<string, uint16_t>> Busy_ports;

        while(itr != Port_map.end()){
            zsock_t* responder = zsock_new(ZMQ_REP);
            string port = "tcp://" + (*itr).second.first+ ":" + to_string((*itr).second.second);
            int open = zsock_bind(responder, port.c_str());

            if(open != (*itr).second.second ) Busy_ports.push_back(make_pair((*itr).second.first,(*itr).second.second));
            zsock_destroy(&responder);

            ++itr; 
        }

        if(!Busy_ports.empty())
        {
            for(uint16_t i  =0; i<Busy_ports.size(); ++i)
            {
                string kill_command =  "fuser -k -n tcp " + to_string(Busy_ports[i].second);
                system(kill_command.c_str());
            }
        }
}



int main(int argc, const char* argv[]){  
    
    Port_Scan(Port_map);
    Robot* robot = new Robot(argc, argv);
    robot->Set_SelfRef0(robot);   

    shared_ptr<Vortal> System_Shock = make_shared<Vortal>(robot);
    (*System_Shock).Launch(robot);

    Dusk_and_Her_Embrace(System_Shock, robot);
    return 0;
}