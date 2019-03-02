%{
#include <tesseract_ros/kdl/kdl_joint_kin.h>	
%}

%shared_ptr(tesseract::tesseract_ros::KDLJointKin);

namespace tesseract
{
namespace tesseract_ros
{
class KDLJointKin : public ROSBasicKin
{
public:
	
	KDLJointKin();
	
	bool checkInitialized() const;
	//const std::string& getTipLinkName() const;
	
	%extend
	{
		void init(const std::string& urdf_xml_string, const std::vector<std::string>& joint_names, const std::string& name)
		{
			urdf::ModelInterfaceSharedPtr urdf_model;		
			urdf_model = urdf::parseURDF(urdf_xml_string);
			if (!urdf_model)
			{
				throw std::runtime_error("parseURDF failed");
			}
			if (!$self->init(urdf_model, joint_names, name))
			{
				throw std::runtime_error("KDLJointKin::init");
			}
		}
		
	}
	
};

typedef std::shared_ptr<KDLJointKin> KDLJointKinPtr;
typedef std::shared_ptr<const KDLJointKin> KDLJointKinConstPtr;

}
}
		
		