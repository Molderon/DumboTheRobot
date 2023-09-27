#include "System_header.hpp"
#include "Robot.hpp"

mutex Head_mx;
condition_variable Head_cv;

Robot_Navigation::~Robot_Navigation(){
    cout<<"Navigation Droped\n";
}

void Awake_Python(){    
    if(fork() == 0) system("python3 Driver.py");
}    

void read_Ultra_Sonic(){
    
    sleep(1);// I do not know how long is python's Start-up time
    vector<double> vUltra_Sonic;
    zsock_t* responder = zsock_new(ZMQ_REP);
    int err = zsock_bind(responder, "tcp://*:5556");
    
    if (err != 5556)    
    {
        zsock_destroy(&responder);
        Trivial_error_txt("[System_Core]:: Bad port with UltraSonic Interface\n");
        ROBOT_OPERATIONAL = false;
        return;
    }

    while (ROBOT_OPERATIONAL) {
        zstr_send(responder, "UltraSonic -r");
        string buffer = zstr_recv(responder);
        
            if (buffer.empty())
             {
                zsock_destroy(&responder);
                Trivial_error_txt("[System]::Corrupted Data from UltraSonic Sensor\n\n");
                break;
            }
            if (stod(buffer) > 2.00) { vUltra_Sonic.push_back(2.00);}
            else if(stod(buffer) <= 0.70){vUltra_Sonic.push_back(stod(buffer)/2);}
            else { vUltra_Sonic.push_back(stod(buffer)); }
                if (vUltra_Sonic.size() >= 5) {
                    float avg = 0;
                    for (int i = 0; i < vUltra_Sonic.size(); ++i)
                        { avg += vUltra_Sonic[i]; }
                    
                    Global_Ultrasonic.store(avg /5);
            
                    vUltra_Sonic.clear();
                }
        
    }
    zsock_destroy(&responder);
};


    void Robot_Movement(Orientation& move_label){
        zsock_t* responder = zsock_new(ZMQ_REQ);
        int err = zsock_bind(responder, "tcp://*:5567");
        if (err != 5567) { Trivial_error_txt("[System_Core]::Bad port bind with Driver Interface\n\n"); return; }
        
        string reciver;
        
        while(ROBOT_OPERATIONAL){
            unique_lock<mutex>ul1(global_mx1);
            gl_Conditonal1.wait(ul1, [](){ return flagMOV_ready;});
    
        
            zstr_send(responder, move_instruction.c_str());
            cout<<"C++:: Move instrunction sent!\n" <<move_instruction<<endl;
            reciver = zstr_recv(responder);

                if(reciver != "GG" || 
                         ROBOT_OPERATIONAL != true)
                {
                         zsock_destroy(&responder);
                         ROBOT_OPERATIONAL = false;
                         Trivial_error_txt("[System-Core]::DC motor command -> Bad formating\n\n");
                         return;
                }

            flagMOV_ready = false;
            ul1.unlock();
            gl_Conditonal1.notify_one();
            ul1.lock();
        }
        zsock_destroy(&responder);
    }



void Head_Control(){
   zsock_t* request = zsock_new(ZMQ_REQ);
   uint16_t err = zsock_bind(request, "tcp://*:5558");
   string observe_local_scene = "Observe!";

   if(err != 5558){
    zsock_destroy(&request);
    ROBOT_OPERATIONAL = false;
    Trivial_error_txt("Broken Port -> Head Servo");
    return;
   }

   while(ROBOT_OPERATIONAL){
    unique_lock<mutex>ul1(Head_mx);
    Head_cv.wait(ul1, [](){ return SeeK_n_Destroy;});

    zstr_send(request, observe_local_scene.c_str()); 
    observe_local_scene = zstr_recv(request);
    
    if(observe_local_scene != "Seek Done!")
    {
        zsock_destroy(&request);
        ROBOT_OPERATIONAL = false;
        Trivial_error_txt("Python Head_Module is damaged\n");
        ul1.~unique_lock();
        
        return;
    }

    SeeK_n_Destroy = false;
    ul1.unlock();
    Head_cv.notify_one();
    ul1.lock();
   }
   zsock_destroy(&request);
}



 void Robot_Navigation::start(){
    thread t1(Awake_Python);
    thread t2(read_Ultra_Sonic);
    thread t3(Head_Control);
        if(t1.joinable() && t2.joinable() && t3.joinable())
        {
            t1.join();
            t2.join();
            t3.join();
        }
    };



Robot_Navigation::Robot_Navigation(){};

      Robot_Navigation* Robot::get_ref_rn_naviation()
            {return self_ref0->rn_naviation;}




      Robot::Robot(int& argc, const char** argv)
      {
            sys_def(argc, argv, Operational_set);
            Robot::Robot_Condition = Initializing;
      }



    void Robot::sys_def(const int& argc, const char* argv[],
                          pair<bool,bool>& Operatial_set){
        if(argv[argc-1] == "FULL")
        {
            Operational_set.first = true;
            Operational_set.second = true;
        }

        else if(argv[argc-1] == "Nominal")
        {
            Operational_set.first = true;
            Operational_set.second = false;
        }

        else
        {
            Robot_Condition = Something_went_Wrong;
            ROBOT_OPERATIONAL = false;

            Operatial_set.first = false;
            Operatial_set.second = false;
        }
    }


      void Robot::Set_SelfRef0(Robot* Mind_ref) 
      {
        this->self_ref0 = Mind_ref;
      }


      bool Robot::get_secondary_systems()
      {
        if(Operational_set.first == true && Operational_set.second == true)
        { return true; }
       
        else if(Operational_set.first == true && Operational_set.second == false)
        { return false; }
       
        else{return NULL;} 
      }
      


      static void Launch(Robot_Navigation* rn_naviation)
      {
        rn_naviation->start();
      }

      
      void Robot::Behavour_Nominal(Robot_Navigation* rn_naviation){     
 
 
        thread Run_Driver(Launch, rn_naviation);
        while(ROBOT_OPERATIONAL)
        {

            if(Robot_Condition == Initializing){
                //initializing complete
                Robot_Condition = static_cast<Action_Mode>(Action_Mode::Lost_in_Space);

                thread send_MOV(Robot_Movement, ref(rn_naviation->move_label));
                thread space_awarness(Init_MoveCommand, ref(move_instruction),
                         ref(rn_naviation->move_label), ref(Robot_Condition));

                  if(space_awarness.joinable() )
                    {space_awarness.join();
                    send_MOV.detach();}          
            }


            else if(Robot_Condition == Lost_in_Space){
                Robot_Condition = Initializing;
            }    

            else if(Robot_Condition == Beacon_Aprox){
                
                thread send_MOV(Robot_Movement, ref(rn_naviation->move_label));
                thread space_awarness(Init_MoveCommand, ref(move_instruction),
                         ref(rn_naviation->move_label), ref(Robot_Condition));

                  if(space_awarness.joinable())
                    {space_awarness.join(); send_MOV.detach();}   
                //----------DELETE THE FOLLOWING CODE---------//

                Robot_Condition = Lost_in_Space;
            } 

            else if (Robot_Condition == Something_went_Wrong){
                system("shutdown -P now");
            } 

            else{
                system("shutdown -P now");
                // Programming language probably have failed...
            }

        }
        if(Run_Driver.joinable()){Run_Driver.join();}
        else{Run_Driver.detach();}
    }



      void Robot::Behavour_FULL(){   

            if(Robot_Condition == Initializing){
                Robot_Condition = static_cast<Action_Mode>(Action_Mode::Lost_in_Space);

                thread send_MOV(Robot_Movement, ref(rn_naviation->move_label));
                thread space_awarness(Init_MoveCommand, ref(move_instruction),
                                    ref(rn_naviation->move_label), ref(Robot_Condition));
                  if(space_awarness.joinable() && send_MOV.joinable())
                  {  space_awarness.join(); send_MOV.join();}          
            }


            else if(Robot_Condition == Lost_in_Space){
                
            }    


            else if(Robot_Condition == Beacon_Aprox){

            } 


            else if(Robot_Condition == Be_Cute){
                // Evoke new Thread + Python script 
            }


            else if(Robot_Condition == Kill_Human){
                // Evoke new Thread + Python script 
            }  


            else if (Robot_Condition == Something_went_Wrong){
                 // Evoke new Thread + Python script 
            } 
            

            else if(Robot_Condition == Browse_Internet){
                // Evoke new Thread + Python script
            }


            else{
                // big dudu
            }
    }


    void Robot::_temp() {
        float systemp, millideg;
        int n;
        FILE *thermal;

        thermal = fopen("/sys/class/thermal/thermal_zone0/temp","r");
        n = fscanf(thermal,"%f",&millideg);
        fclose(thermal);
        systemp = millideg / 1000;
        
        if(systemp > 75)
        {
            Robot_Condition = Something_went_Wrong;
            ROBOT_OPERATIONAL = false;
            Trivial_error_txt("[Hardware]::CPU is overheating\n\n");
            //Initiate safe shutdown sequence
            
            this_thread::sleep_for(chrono::seconds(2));
            
            system("sudo shutdown -P now");
        }
        else{std::this_thread::sleep_for(std::chrono::seconds(45));}

    }
      

    Robot::~Robot(){
        if(CPU_routine.joinable()) CPU_routine.join();
        else{CPU_routine.~thread();}
        delete rn_naviation;
    };