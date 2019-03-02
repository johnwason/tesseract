%{
#include <tesseract_ros/kdl/kdl_env.h>	
%}

%shared_ptr(tesseract::tesseract_ros::KDLEnv);

namespace tesseract
{
namespace tesseract_ros
{
class KDLEnv : public ROSBasicEnv
{
public:
	KDLEnv();	
};

typedef std::shared_ptr<KDLEnv> KDLEnvPtr;
typedef std::shared_ptr<const KDLEnv> KDLEnvConstPtr;

}
}
