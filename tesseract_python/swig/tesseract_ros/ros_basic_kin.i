%{
#include <tesseract_ros/ros_basic_kin.h>	
%}

%shared_ptr(tesseract::tesseract_ros::ROSBasicKin);

namespace tesseract
{
namespace tesseract_ros
{
%nodefaultctor ROSBasicKin;
class ROSBasicKin : public BasicKin
{
	
};

typedef std::shared_ptr<ROSBasicEnv> ROSBasicEnvPtr;
typedef std::shared_ptr<const ROSBasicEnv> ROSBasicEnvConstPtr;

}
}