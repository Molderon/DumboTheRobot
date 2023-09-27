#include<iostream>
#include<unordered_map>
#include<cmath>
#include<utility>
#include<fstream>
#include<sstream>
#include<array>
#include<iterator>
#include<vector>
#include<mutex>
#include<future>
#include<thread>
#include<condition_variable>

#include<opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/calib3d.hpp>
#include<opencv4/opencv2/core/core.hpp>

#include "Augmented_Reality/aruco/include/aruco/aruco.h"

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/string_util.h"
#include "tensorflow/lite/examples/label_image/get_top_n.h"
#include "tensorflow/lite/model.h"

using namespace std;
using namespace cv;


bool BE_ANNOYING = true, Loc_rdy = false;
bool my_vision_is_augmented = true;;

condition_variable Loc_CV;
mutex Loc_MX;


struct common_util{
 //default metrics for distance estimation
  uint16_t Readings_range = 20;
  float Distance_Coef = 6.388f,
  additional_coef = 0.32f,
  Markers_Size = 3.0f;

  const float MAX_X = 300.0f, MAX_Y = 595.9f,
  magic_triang = 0.9f, magic_trilat = 0.0f;

  string Beacon_Type = "4x4", Data_path = "Beacon_DB/Data_Base.txt",
  calibration_path = "../../Beacon_DB/Camera_Calibration.yaml",
  Model_FileName = "Aruco_Model.tensorflow";

  const char* get_Model(){return Model_FileName.c_str();}
};

common_util utilities;



class Beacon{
   pair<float,float> beacon_location;
   uint16_t embedded_index;
   bool orientation;
   
   public:
   Beacon(float loc_X, float loc_Y, uint16_t index, bool orientation)
   {
      this->beacon_location.first = loc_X;
      this->beacon_location.second = loc_Y;
      this->embedded_index = index;
      this->orientation = orientation;   
   };

   uint16_t get_index(){return this->embedded_index;}
   float get_location_X(){return this->beacon_location.first;}
   float get_location_Y(){return this->beacon_location.second;}
};



class Camera{
        public:
        cv::VideoCapture Cam_Input;
        cv::Matx33d cam_matrix;
        cv::Mat dist_coeff, Object_Points;
        
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::DetectorParameters Detector = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
       
        
        void Set_cam_Param(Matx33d& cam_matrix, Mat& dist_coeff){

            if (Is_File_Gone(utilities.calibration_path.c_str())){
                cv::FileStorage file(utilities.calibration_path.c_str(), FileStorage::READ);
                file["camera_matrix"] >> cam_matrix;
                file["distortion_coefficient"] >> dist_coeff;
                file.release();
             }
               else
               {
                  //Default Logitech c920 camera params
                  cam_matrix = {632.29863082751251, 0, 319.5, 0, 632.29863082751251, 239.5, 0, 0, 1};
                  dist_coeff = (Mat_<double>(5,1) << 0.070528331223347215, 0.26247385180956367, 0, 0, -1.0640942232949715);
               }
        }



        static const char* resolve_distance(uint16_t i, vector<Vec3d> t_Vecs)
        {
            string Z_axis;
            
            Z_axis = to_string(static_cast<uint16_t>(t_Vecs[i][2]*utilities.Distance_Coef < 600.0f ? 
            t_Vecs[i][2] * utilities.Distance_Coef : 
            (t_Vecs[i][2] * (utilities.Distance_Coef - utilities.additional_coef))));

            return Z_axis.c_str();
        }


        
        bool Is_File_Gone(const char* file){
             ifstream test(file);
             return test.good();
        }



        void init_obj(Mat& Object_Points){

            cv::Mat base(4,1,CV_32FC3);
            this->Object_Points = base;

            this->Object_Points.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-utilities.Markers_Size/2.f, utilities.Markers_Size/2.f, 0);
            this->Object_Points.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(utilities.Markers_Size/2.f, utilities.Markers_Size/2.f, 0);
            this->Object_Points.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(utilities.Markers_Size/2.f, -utilities.Markers_Size/2.f, 0);
            this->Object_Points.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-utilities.Markers_Size/2.f, -utilities.Markers_Size/2.f, 0);
        }



        public:
         Camera(){
             init_obj(Object_Points);
             Set_cam_Param(cam_matrix, dist_coeff);
         }
};



class Environment{
   public:

      float map_height = 595.9f, map_width = 300.0f;
      unordered_map<uint16_t,Beacon*> local_deck;
      bool my_vision_is_augmented;
      
      Environment()
      {
         load_deck();
         set_map_size(utilities);
      };

     
      ~Environment()
      {
         unordered_map<uint16_t,Beacon*>:: iterator itr;
         for(itr = local_deck.begin(); itr != local_deck.end(); itr++)
         {
             free((*itr).second);
         }
      };

      unordered_map<uint16_t, Beacon*>* get_deck()
      {
         return &this->local_deck;
      }

      void set_map_size(common_util& utilities)
      {
         this->map_width = utilities.MAX_X;
         this->map_height = utilities.MAX_Y;
      }

      pair<uint16_t, uint16_t> get_map_size()
      {
         return make_pair(this->map_width, this->map_height);
      }
      

   protected:
      void load_deck()
      {
         ifstream file; string current_line;
         float cords[2] = {0,0};
         uint16_t data[2] = {0,0}; 

         file.open(utilities.Data_path, ios::in);
         if(!file.is_open() || file.peek() == ifstream::traits_type::eof())
         {
            this->my_vision_is_augmented = false;
            cout<<"Nope, No DataBase found\n";
            //Trivial_error_txt("[Augmented_Reality]::Data_Base is Broken\n");
            file.close();
         }

         while(getline(file, current_line))
         {
            istringstream in_line(current_line);
            in_line >> cords[0];
            in_line >> cords[1];
            in_line >> data[0];
            in_line >> data[1];          //непротивоконституциоснователствувайте, yep that constructor is too long :/
            local_deck.insert(make_pair(data[0], new Beacon(cords[0], cords[1], data[0], static_cast<bool>(data[1]))));
            //cout<<"Beacon Loaded is :"<<data[0]<<" at:"<<cords[0]<<", "<<cords[1]<<endl;
         }
         
            file.close();
         }
      
      
      void display_deck()
      {
         unordered_map<uint16_t,Beacon*>::iterator itr;
         for(itr = local_deck.begin(); itr != local_deck.end(); itr++)
         {
            cout<<"Beacon at: (X:"<<(*itr).second->get_location_X()<<" -  Y:"<<(*itr).second->get_location_Y()<<") with index:"<<(*itr).second->get_index()<<"\n";
         }
      }      
};




Environment* Generate_Map(){
   return new Environment();
}

Camera* Init_Camera(){
   return new Camera();
}



class Self_Localization{
   private:
   float Triangulation_solution[2] = {0,0};
   float Trilateration_solution[2] = {0,0};
   
   bool tri_ang = false, tri_lat = false;
   Self_Localization* self_ref;

   pair<float,float> Absolute_Location;
   future<Camera*> Cam_ptr;
   future<Environment*> Env_ptr;

   

   public:
   Self_Localization()
   {
     this->Cam_ptr = async(Init_Camera);
     this->Env_ptr = async(Generate_Map);
   }

   ~Self_Localization()
   {
     delete this->Cam_ptr.get();
     delete this->Env_ptr.get(); 
   }
   unordered_map<uint16_t,Vec3f> Found_Beacons;
   
   void set_selfref(Self_Localization* WhereAmI){this->self_ref = WhereAmI;}
   Camera* get_camptr(){return this->Cam_ptr.get();}

   unordered_map<uint16_t, Vec3f> get_Beacons(){return this->Found_Beacons;}
   void Log(){
      if(tri_ang == true){cout<<"[Triangulated Localization]::"<<"X-at("<<this->Triangulation_solution[0]<<"), Y-at("<<this->Triangulation_solution[1]<<")\n";}
      else if(tri_lat == true){cout<<"[Trilaterated Localization]::"<<"X-at("<<this->Trilateration_solution[0]<<"), Y-at("<<this->Trilateration_solution[1]<<")\n";}
      else{/*cerr<<"[Localization]::Failed\n\n";*/}

      this->tri_ang = false;
      this->tri_lat = false;
      this->Found_Beacons.clear();
   }

   
   public:

   // Cuently Works for onesided markers
   // To be continued
   void Triangulation(Self_Localization* WhereAmI){
      pair<float,float> Results;
      unordered_map<uint16_t,Vec3f>::iterator itr = WhereAmI->Found_Beacons.begin();
      float Side_X = 0.0f, Side_Y = 0.0f;
      unordered_map<uint16_t, Beacon*>::iterator current_mark;

      for(itr; itr != WhereAmI->Found_Beacons.end(); ++itr)
      {  
         (*itr).second[1] >=1  ? (*itr).second[1]+=0 : (*itr).second[1]*=-1;

         ((*itr).second[2] * utilities.Distance_Coef) > 620 ? 
         (*itr).second[2] *= utilities.Distance_Coef :
         (*itr).second[2] *= (utilities.Distance_Coef - utilities.additional_coef);

         (*itr).second[2] /= 100; 

         if((*itr).second[1] < 45){
            Side_X = ((tan((*itr).second[1]* 3.14 / 180.0)) * (*itr).second[2])/ utilities.magic_triang;
                        
            Side_Y = ((*itr).second[2] * sin(((90 - (*itr).second[1])* 3.14 / 180.0)));
         }

         else{
            Side_X = (sin((*itr).second[1]*3.14 / 180.0) * (*itr).second[2]);
            Side_Y = ((tan(90 - (*itr).second[1])*3.14 / 180.0) * (*itr).second[2]);
         }
            //Buggy Here
            try
            {
               current_mark = Env_ptr.get()->local_deck.find((*itr).first);
            }
            catch(const exception& e){
               cout<<"This shound be re-written \n";
            }
            
            Results.first +=  ((*current_mark).second->get_location_X()) - Side_X;
            Results.second += ((*current_mark).second->get_location_Y()) - Side_Y;
      }

      WhereAmI->Triangulation_solution[0] = (Results.first / WhereAmI->Found_Beacons.size());
      WhereAmI->Triangulation_solution[1] = (Results.second / WhereAmI->Found_Beacons.size());
      Results.first = 0.0f;
      Results.second = 0.0f;
      
      WhereAmI->tri_ang = true;
   }



   void Trilateration(Self_Localization* WhereAmI){
      
         bool simple_algo = false;
         vector<pair<uint16_t, float>>Beacons;
         array<pair<uint16_t,pair<float,float>>,3> Metrics;

         unordered_map<uint16_t, Vec3f>::iterator itr = WhereAmI->Found_Beacons.begin();
         unordered_map<uint16_t, Beacon*>* deck = WhereAmI->Env_ptr.get()->get_deck();
         unordered_map<uint16_t, Beacon*>::iterator deck_itr;

         float frac_a, frac_b, frac_c, frac_d, frac_e, frac_f; 

         for(uint16_t i = 0; i< 3; ++i)
         {
            Beacons.push_back(make_pair(
                    (*itr).first,((*itr).second[2])*utilities.Distance_Coef > 620 ?
                    (*itr).second[2] * utilities.Distance_Coef :
                    (*itr).second[2] * (utilities.Distance_Coef - utilities.additional_coef)));

            deck_itr = (*deck).find((*itr).first);
            
            if(deck_itr == (*deck).end())
            {
               WhereAmI->tri_lat = false;
               break;
            }

            Metrics.at(i).first = (*deck_itr).first;
            Metrics.at(i).second.first = (*deck_itr).second->get_location_X();
            Metrics.at(i).second.second = (*deck_itr).second->get_location_Y();

            itr++;
         }

         frac_a = (Metrics.at(1).second.first * 2) - (Metrics.at(0).second.first * 2);
         frac_b = (Metrics.at(1).second.second * 2) - (Metrics.at(0).second.second * 2);

         frac_c = sqrt(Beacons[0].second) - sqrt(Beacons[1].second) - sqrt(Metrics.at(0).second.first) 
         + sqrt(Metrics.at(1).second.first) - sqrt(Metrics.at(0).second.second) + sqrt(Metrics.at(1).second.second);
         frac_d = (Metrics.at(2).second.first * 2) - (Metrics.at(1).second.first * 2);
         frac_e = (Metrics.at(2).second.second * 2) - (Metrics.at(1).second.second * 2);

         frac_f = sqrt(Beacons[1].second) - sqrt(Beacons[2].second) - sqrt(Metrics.at(1).second.first) 
         + sqrt(Metrics.at(2).second.first) - sqrt(Metrics.at(1).second.second) + sqrt(Metrics.at(2).second.second);

         WhereAmI->Trilateration_solution[0] = 
         ((((frac_c * frac_e)-(frac_f * frac_b)) / ((frac_e * frac_a) - (frac_b * frac_d)))) * utilities.MAX_X;
         WhereAmI->Trilateration_solution[1] =
          ((((frac_c * frac_d)-(frac_a * frac_f)) / ((frac_b * frac_d) - (frac_a * frac_e)))) * utilities.MAX_Y; 
         
         if(WhereAmI->Trilateration_solution[0] < 0){WhereAmI->Trilateration_solution[0] *= -1;}
         if(WhereAmI->Trilateration_solution[1] < 0){WhereAmI->Trilateration_solution[1] *= -1;}
         
         cout<<WhereAmI->Trilateration_solution[0]<<endl<<WhereAmI->Trilateration_solution[1]<<endl;

         WhereAmI->tri_lat = true;
   }
};



void Branch(Self_Localization* WhereAmI){
   unique_lock<mutex> ul1(Loc_MX);

   while(my_vision_is_augmented){
      Loc_CV.wait(ul1, [](){return Loc_rdy;});
      if(WhereAmI->Found_Beacons.size() >= 3)
      {
         //WhereAmI->Triangulation(WhereAmI);
         //WhereAmI->Trilateration(WhereAmI);
         //WhereAmI->Fix_Error();
      }  
      else
      {  
         //WhereAmI->Triangulation(WhereAmI);
      }

      WhereAmI->Found_Beacons.clear();
      WhereAmI->Log();
      Loc_rdy = false;

      ul1.unlock();
      Loc_CV.notify_one();
      ul1.lock();
   }
}





Vec3f Euler_Transform(Vec3d& T_vec, Vec3d& R_vec)
{
   Vec3f solution;
   Mat Empty_Mat, Rotation_Matrix,
   translation_vec = (Mat_<double>(3,1)<<T_vec[0],T_vec[1],T_vec[2]),
   rotation_vec = (Mat_<double>(3,1)<<R_vec[0],R_vec[1],R_vec[2]);
   double roll_degrees, pitch_degrees;


   cv::Rodrigues(rotation_vec, Rotation_Matrix);
   RQDecomp3x3(Rotation_Matrix, Empty_Mat, Empty_Mat);
   
   roll_degrees = (atan2(Rotation_Matrix.at<double>(2, 1), Rotation_Matrix.at<double>(2, 2))) * 180.0f / CV_PI;
   pitch_degrees = (atan2(-Rotation_Matrix.at<double>(2, 0), 
   sqrt(pow(Rotation_Matrix.at<double>(2, 1), 2) + pow(Rotation_Matrix.at<double>(2, 2), 2)))) * 180.0f / CV_PI;


   solution[0] = roll_degrees;
   solution[1] = pitch_degrees;
   solution[2] = T_vec[2];

   return solution;
}




static void Augmented_Reallity(Self_Localization* WhereAmI){
        vector<int> ids;
        vector<cv::Vec3d>r_Vecs, t_Vecs;
        std::vector<std::vector<cv::Point2f>> corners;
        Mat image, image_cpy; int n_beacons = 0;

        Camera* Camera_Object = WhereAmI->get_camptr();
        cv::aruco::ArucoDetector Detective(Camera_Object->dict, Camera_Object->Detector);

        unique_lock<mutex> ulock(Loc_MX);
        Camera_Object->Cam_Input.open(2);
        while(Camera_Object->Cam_Input.grab())
        {
            if(!Camera_Object->Cam_Input.retrieve(image))
            {
               //Trivial_error_txt("[System_Core]::Vision Depracated\n");
               //ROBOT_OPERATIONAL = false;
               delete Camera_Object;
               my_vision_is_augmented = false;
            }
            
            image.copyTo(image_cpy);
            Detective.detectMarkers(image, corners, ids);
            
            if(ids.size()>0) cv::aruco::drawDetectedMarkers(image, corners, ids);
            else{continue;} 


            n_beacons = corners.size();
            
            t_Vecs.resize(n_beacons);
            r_Vecs.resize(n_beacons);

            for(uint16_t i = 0; i< n_beacons; i++){
                solvePnP(Camera_Object->Object_Points, corners.at(i), Camera_Object->cam_matrix, Camera_Object->dist_coeff, r_Vecs.at(i), t_Vecs.at(i));
                WhereAmI->Found_Beacons.insert(make_pair(ids[i], Euler_Transform(t_Vecs[i], r_Vecs[i])));
            }            

            Loc_rdy = true;
            ulock.unlock();
            
            Loc_CV.notify_one();
            ulock.lock();


            Loc_CV.wait(ulock, [](){return Loc_rdy == false;});

            if(BE_ANNOYING == true){
                for(uint16_t i = 0; i<ids.size(); ++i)
                {
                    cv::drawFrameAxes(image_cpy, Camera_Object->cam_matrix, Camera_Object->dist_coeff, r_Vecs[i], t_Vecs[i], 3);
                    cv::putText(image_cpy, Camera_Object->resolve_distance(i,t_Vecs), cv::Point(40,40), cv::FONT_HERSHEY_DUPLEX, 2, Scalar(30,255,130),2,false);
                }
                cv::aruco::drawDetectedMarkers(image_cpy,corners,ids);
                cv::imshow("[Distance and Pose - estimation]",image_cpy);
            }
               char key = static_cast<char>(cv::waitKey(5));
                if(key == 27){break;}
            // No IE diminished due to computing power
        }
};



void Run(Self_Localization* WhereAmI){
   thread thr_1(Augmented_Reallity, ref(WhereAmI));
   thread thr_2(Branch, ref(WhereAmI));
   
   if (thr_1.joinable() && thr_2.joinable())
   {
      thr_1.join();
      thr_2.join();
   }
}


bool Beacon_TSF_Lite(){ 
    bool Beacon_Detected = false;
    unique_ptr<tflite::Interpreter> interpreter;
    tflite::ops::builtin::BuiltinOpResolver resolver;
    unique_ptr<tflite::FlatBufferModel> model = tflite::FlatBufferModel::BuildFromFile(utilities.get_Model());

   
   if (model == nullptr)
    {
        fprintf(stderr, "failed to load model\n");
        exit(-1);
    }

    tflite::InterpreterBuilder(*model.get(), resolver)(&interpreter);
   
   
    if (interpreter == nullptr)
    {
        fprintf(stderr, "Failed to initiate the interpreter\n");
        exit(-1);
    }

    if (interpreter->AllocateTensors() != kTfLiteOk)
    {
        fprintf(stderr, "Failed to allocate tensor\n");
        exit(-1);
    }


    interpreter->SetAllowFp16PrecisionForFp32(true);
    interpreter->SetNumThreads(1);
    // Get Input Tensor Dimensions
    int input = interpreter->inputs()[0];
    auto height = interpreter->tensor(input)->dims->data[1];
    auto width = interpreter->tensor(input)->dims->data[2];
    auto channels = interpreter->tensor(input)->dims->data[3];

   cv::VideoCapture Cam_Stream;
   Mat Frame, image;
   Cam_Stream.open(2);
   
   while(Cam_Stream.grab())
   {
      if(!Cam_Stream.retrieve(Frame)){
         cerr<<"[System]::Bad Frame Stream\n";
         Cam_Stream.release();
         my_vision_is_augmented = false;
         return false;
      }

      cv::resize(Frame, image, cv::Size(width, height), cv::INTER_NEAREST);
      memcpy(interpreter->typed_input_tensor<unsigned char>(0), image.data, image.total() * image.elemSize());
      
      interpreter->Invoke();


    int output = interpreter->outputs()[0];
    TfLiteIntArray *output_dims = interpreter->tensor(output)->dims;
    auto output_size = output_dims->data[output_dims->size - 1];
    std::vector<std::pair<float, int>> Found;
    float threshold = 0.01f;



    switch (interpreter->tensor(output)->type)
    {
    case kTfLiteInt32:
        tflite::label_image::get_top_n<float>(interpreter->typed_output_tensor<float>(0), output_size, 1, threshold, &Found, kTfLiteFloat32);
        break;
    case kTfLiteUInt8:
        tflite::label_image::get_top_n<uint8_t>(interpreter->typed_output_tensor<uint8_t>(0), output_size, 1, threshold, &Found, kTfLiteUInt8);
        break;
    default:
        fprintf(stderr, "cannot handle output type\n");
        exit(-1);
    }

     if(Found.size() > 0)
     {
       for (const auto &result : Found){
         if(result.first >= 0.60f)
         {
            Beacon_Detected = true;
            Cam_Stream.release();
            break;
         }
       }
     }
   }

   return Beacon_Detected == true ?  false : Beacon_Detected;
}

