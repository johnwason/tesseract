%{
#include <tesseract_ros/ros_basic_plotting.h>
%}

%shared_ptr(tesseract::tesseract_ros::ROSBasicPlotting);

namespace tesseract
{
namespace tesseract_ros
{

class ROSBasicPlotting : public BasicPlotting
{
public:
  ROSBasicPlotting(std::shared_ptr<const ROSBasicEnv> env);
  
  void plotScene() const;
  
};

}
}