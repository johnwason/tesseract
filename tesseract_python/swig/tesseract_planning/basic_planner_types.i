%{
#include <tesseract_planning/basic_planner_types.h>
%}

%template(StatusCodeMap) std::unordered_map<int, std::string>; 

namespace tesseract
{
namespace tesseract_planning
{
typedef std::unordered_map<int, std::string> StatusCodeMap;

struct PlannerRequest
{
  std::string name;                       
  tesseract_ros::ROSBasicEnvConstPtr env; 
  EnvStateConstPtr start_state;           
  std::string config;                     
  std::string config_format;              
};

struct PlannerResponse
{
  std::vector<std::string> joint_names; 
  TrajArray trajectory;                 
  int status_code; 
  std::string status_description;
};


}
}