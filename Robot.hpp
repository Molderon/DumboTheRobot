#include "System_header.hpp"


    class Robot_Navigation
    {
        protected:
          double Ultra_Sonic;
          pair<float, uint16_t> Direction;
          float time_displacement;


        public:
          Orientation move_label;
          friend void Awake_Python();
          friend void read_Ultra_Sonic();
          void start();
           Robot_Navigation();
          ~Robot_Navigation();
    };


    class Robot{
        protected:
          pair<bool,bool> Operational_set = make_pair(false, false);
          Action_Mode Robot_Condition;
          float CPU_temp = 0.0;
          thread CPU_routine;

        public:
          Robot_Navigation* rn_naviation = new Robot_Navigation(); 
          Robot_Navigation* get_ref_rn_naviation();
          Robot* self_ref0;

        Robot(int& argc, const char** argv);
        ~Robot();

        void sys_def(const int& argc, const char* argv[],
                     pair<bool,bool>& Operatial_set);

        void Set_SelfRef0(Robot* Mind_ref);
        
        bool get_secondary_systems();
        friend void Robot_Movement (Orientation& move_label);
        void Behavour_Nominal(Robot_Navigation* rn_naviation);
        void Behavour_FULL();


        private:
          void _temp();
    };