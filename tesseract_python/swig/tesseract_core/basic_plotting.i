%{
#include <tesseract_core/basic_plotting.h>
%}

%shared_ptr(tesseract::BasicPlotting);

namespace tesseract
{
%nodefaultctor BasicPlotting;
class BasicPlotting
{
public:
	virtual ~BasicPlotting();
	
	virtual void plotTrajectory(const std::vector<std::string>& joint_names, const Eigen::Ref<const TrajArray>& traj);
	
	virtual void plotContactResults(const std::vector<std::string>& link_names,
	                                  const ContactResultVector& dist_results,
	                                  const Eigen::Ref<const Eigen::VectorXd>& safety_distances);
	
	virtual void plotArrow(const Eigen::Ref<const Eigen::Vector3d>& pt1,
	                         const Eigen::Ref<const Eigen::Vector3d>& pt2,
	                         const Eigen::Ref<const Eigen::Vector4d>& rgba,
	                         double scale);
	
	virtual void plotAxis(const Eigen::Isometry3d& axis, double scale);
	
	virtual void clear();
	
};

typedef std::shared_ptr<BasicPlotting> BasicPlottingPtr;
typedef std::shared_ptr<const BasicPlotting> BasicPlottingConstPtr;

}
		
		