%{
#include <tesseract_ros/ros_tesseract_utils.h>	
%}

%rosmsg_typemaps(tesseract_msgs::AttachableObject);
%rosmsg_typemaps(tesseract_msgs::AttachedBodyInfo);
%rosmsg_typemaps(sensor_msgs::JointState);
%rosmsg_typemaps(tesseract_msgs::TesseractState);
%rosmsg_typemaps(trajectory_msgs::JointTrajectory);
%rosmsg_typemaps(tesseract_msgs::ContactResult);

namespace tesseract
{
namespace tesseract_ros
{
void attachableObjectToAttachableObjectMsg(tesseract_msgs::AttachableObject& ao_msg, const AttachableObject& ao);

void attachedBodyInfoToAttachedBodyInfoMsg(tesseract_msgs::AttachedBodyInfo& ab_info_msg, const AttachedBodyInfo& ab_info);

void tesseractEnvStateToJointStateMsg(sensor_msgs::JointState& joint_state, const EnvState& state, const ros::Time& stamp);

void tesseractToTesseractStateMsg(tesseract_msgs::TesseractState& state_msg, const tesseract_ros::ROSBasicEnv& env, const ros::Time& stamp);

void tesseractTrajectoryToJointTrajectoryMsg(trajectory_msgs::JointTrajectory& traj_msg, const tesseract::EnvState& start_state,
                                             const std::vector<std::string>& joint_names, const Eigen::Ref<const TrajArray>& traj);

void tesseractTrajectoryToJointTrajectoryMsg(trajectory_msgs::JointTrajectory& traj_msg,
                                                           const std::vector<std::string>& joint_names,
                                                           const Eigen::Ref<const TrajArray>& traj);

void tesseractTrajectoryToJointTrajectoryMsg(trajectory_msgs::JointTrajectoryPtr traj_msg,
                                                           const std::vector<std::string>& joint_names,
                                                           const Eigen::Ref<const TrajArray>& traj);

void tesseractContactResultToContactResultMsg(tesseract_msgs::ContactResult& contact_result_msg,
                                                            const tesseract::ContactResult& contact_result,
                                                            const ros::Time& stamp);

}
}

%inline {
	std::shared_ptr<tesseract::AttachableObject> attachableObjectMsgToAttachableObject( const tesseract_msgs::AttachableObject& ao_msg)
	{
		std::shared_ptr<tesseract::AttachableObject> ao=std::make_shared<tesseract::AttachableObject>();
		tesseract::tesseract_ros::attachableObjectMsgToAttachableObject(ao,ao_msg);
		return ao;
	}
	
	std::shared_ptr<tesseract::AttachedBodyInfo> attachedBodyInfoMsgToAttachedBodyInfo(const tesseract_msgs::AttachedBodyInfo& ab_msg)
	{
		std::shared_ptr<tesseract::AttachedBodyInfo> ab=std::make_shared<tesseract::AttachedBodyInfo>();
		tesseract::tesseract_ros::attachedBodyInfoMsgToAttachedBodyInfo(*ab,ab_msg);
		return ab;	
	}
}


%rename(processJointStateMsg) processJointStateMsg_swig;
%rename(processTesseractStateMsg) processTesseractStateMsg_swig;
%rename(processAttachableObjectMsg) processAttachableObjectMsg_swig;
%rename(processAttachedBodyInfoMsg) processAttachedBodyInfoMsg_swig;
%inline
{

	void processAttachableObjectMsg_swig(std::shared_ptr<tesseract::tesseract_ros::ROSBasicEnv> env, const tesseract_msgs::AttachableObject& ao_msg)
	{
		if (!tesseract::tesseract_ros::processAttachableObjectMsg(*env,ao_msg))
		{
			throw std::runtime_error("processAttachedObjectMsg failed");
		}
	}
	
	void processAttachedBodyInfoMsg_swig(std::shared_ptr<tesseract::tesseract_ros::ROSBasicEnv> env, const tesseract_msgs::AttachedBodyInfo& ab_msg)
	{
		if(!tesseract::tesseract_ros::processAttachedBodyInfoMsg(*env, ab_msg))
		{
			throw std::runtime_error("processAttachedBodyInfoMsg failed");
		}
	}
		
	void processJointStateMsg_swig(std::shared_ptr<tesseract::tesseract_ros::ROSBasicEnv> env, const sensor_msgs::JointState& joint_state_msg)
	{
		if(!tesseract::tesseract_ros::processJointStateMsg(*env, joint_state_msg))
		{
			throw std::runtime_error("processJointStateMsg failed");
		}
	}
		
	void processTesseractStateMsg_swig(std::shared_ptr<tesseract::tesseract_ros::ROSBasicEnv> env, const tesseract_msgs::TesseractState& state_msg)
	{
		if (!tesseract::tesseract_ros::processTesseractStateMsg(env, state_msg))
		{
			throw std::runtime_error("processTesseractStateMsg failed");
		}
	}
	
	
}