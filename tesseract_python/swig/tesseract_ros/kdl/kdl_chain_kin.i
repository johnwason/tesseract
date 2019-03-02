%{
#include <tesseract_ros/kdl/kdl_chain_kin.h>	
%}

%shared_ptr(tesseract::tesseract_ros::KDLChainKin);

namespace tesseract
{
namespace tesseract_ros
{
class KDLChainKin : public ROSBasicKin
{
public:
	
	KDLChainKin();
	
	bool checkInitialized() const;
	const std::string& getTipLinkName() const;
	
	%extend
	{
		void init(const std::string& urdf_xml_string, const std::string& base_link,
            const std::string& tip_link, const std::string& name)
		{
			urdf::ModelInterfaceSharedPtr urdf_model;		
			urdf_model = urdf::parseURDF(urdf_xml_string);
			if (!urdf_model)
			{
				throw std::runtime_error("parseURDF failed");
			}
			if (!$self->init(urdf_model, base_link, tip_link, name))
			{
				throw std::runtime_error("KDLChainKin::init");
			}
		}
		
	}
	
};

typedef std::shared_ptr<KDLChainKin> KDLChainKinPtr;
typedef std::shared_ptr<const KDLChainKin> KDLChainKinConstPtr;

}
}
		
		