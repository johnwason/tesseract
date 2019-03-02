%{
#include <tesseract_ros/ros_basic_env.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
%}

%shared_ptr(tesseract::tesseract_ros::ROSBasicEnv);
namespace tesseract
{
namespace tesseract_ros
{
%nodefaultctor ROSBasicEnv;
class ROSBasicEnv : public BasicEnv
{
public:
%extend
{
	void init(const std::string& urdf_xml_string)
	{
		urdf::ModelInterfaceSharedPtr urdf_model;		
		urdf_model = urdf::parseURDF(urdf_xml_string);
		if (!urdf_model)
		{
			throw std::runtime_error("parseURDF failed");
		}
		if (!$self->init(urdf_model))
		{
			throw std::runtime_error("BasicEnv::init failed");	
		}
	}
	
	void init(const std::string& urdf_xml_string, const std::string& srdf_xml_string )
	{
		urdf::ModelInterfaceSharedPtr urdf_model;
		srdf::ModelSharedPtr srdf_model;
		urdf_model = urdf::parseURDF(urdf_xml_string);
		if (!urdf_model)
		{
			throw std::runtime_error("parseURDF failed");
		}
		srdf_model = srdf::ModelSharedPtr(new srdf::Model);
		srdf_model->initString(*urdf_model, srdf_xml_string);
		if (!$self->init(urdf_model, srdf_model))
		{
			throw std::runtime_error("BasicEnv::init failed");	
		}
	}	
}
	
virtual void loadDiscreteContactManagerPlugin(const std::string& plugin);

virtual void loadContinuousContactManagerPlugin(const std::string& plugin);

};

}
}