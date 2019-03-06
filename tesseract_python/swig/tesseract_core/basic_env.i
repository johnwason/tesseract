%{
#include <tesseract_core/basic_env.h>
%}

%shared_ptr(tesseract::BasicEnv);

namespace tesseract
{

%nodefaultctor BasicEnv;
class BasicEnv
{
public:
	
	virtual ~BasicEnv();
	
	virtual void setName(const std::string& name);
	virtual const std::string& getName() const;
	
	virtual void setState(const std::unordered_map<std::string, double>& joints);
	virtual void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values);
	virtual void setState(const std::vector<std::string>& joint_names,
	                      const Eigen::Ref<const Eigen::VectorXd>& joint_values);
	
	virtual EnvStateConstPtr getState() const;
	
	virtual EnvStatePtr getState(const std::unordered_map<std::string, double>& joints) const ;
	virtual EnvStatePtr getState(const std::vector<std::string>& joint_names,
							     const std::vector<double>& joint_values) const;
	virtual EnvStatePtr getState(const std::vector<std::string>& joint_names,
	                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;
	
	virtual BasicKinConstPtr getManipulator(const std::string& manipulator_name) const;
	
	virtual bool hasManipulator(const std::string& manipulator_name) const;
		
%extend
{
	virtual void addManipulator(const std::string& base_link,
	                            const std::string& tip_link,
	                            const std::string& manipulator_name)
	{
		if(!$self->addManipulator(base_link, tip_link, manipulator_name))
		{
			throw std::runtime_error("addManipular failed");
		}
	}
	
	
	virtual void addManipulator(const std::vector<std::string>& joint_names, const std::string& manipulator_name)
	{
		if (!$self->addManipulator(joint_names,manipulator_name))
		{
			throw std::runtime_error("addManipular failed");
		}
	}
}
	
	virtual void addAttachableObject(const AttachableObjectConstPtr attachable_object);
	
	virtual void removeAttachableObject(const std::string& name);
	
	virtual const AttachableObjectConstPtrMap getAttachableObjects() const;
	
	virtual void clearAttachableObjects();
	
	virtual const AttachedBodyInfo getAttachedBody(const std::string& name) const;
	
	virtual const AttachedBodyInfoMap getAttachedBodies() const;
	
	virtual void attachBody(const AttachedBodyInfo& attached_body_info);
	
	virtual void detachBody(const std::string& name);
	
	virtual void clearAttachedBodies();
	
	virtual ObjectColorMapConstPtr getKnownObjectColors() const;
	
	virtual void clearKnownObjectColors();
	
	virtual std::vector<std::string> getJointNames() const;
	
	virtual Eigen::VectorXd getCurrentJointValues() const;
	
	virtual Eigen::VectorXd getCurrentJointValues(const std::string& manipulator_name) const;
	
	virtual const std::string& getRootLinkName() const;
	
	virtual std::vector<std::string> getLinkNames() const;
	
	virtual std::vector<std::string> getActiveLinkNames() const;
	
	virtual VectorIsometry3d getLinkTransforms() const;
	
	virtual const Eigen::Isometry3d getLinkTransform(const std::string& link_name) const;
	
	virtual AllowedCollisionMatrixConstPtr getAllowedCollisionMatrix() const;
	
	virtual AllowedCollisionMatrixPtr getAllowedCollisionMatrixNonConst();
	
	virtual IsContactAllowedFn getIsContactAllowedFn() const;
	
	virtual void setIsContactAllowedFn(IsContactAllowedFn fn);
	
	virtual DiscreteContactManagerBasePtr getDiscreteContactManager() const;
	
	virtual ContinuousContactManagerBasePtr getContinuousContactManager() const;
	
	typedef std::shared_ptr<BasicEnv> BasicEnvPtr;
	typedef std::shared_ptr<const BasicEnv> BasicEnvConstPtr;
	
};


}

%inline
%{
	std::vector<tesseract::ContactResultMap> continuousCollisionCheckTrajectory(tesseract::ContinuousContactManagerBase& manager,
	                                               const tesseract::BasicEnv& env,
	                                               const tesseract::BasicKin& kin,
	                                               const Eigen::Ref<const tesseract::TrajArray>& traj,
	                                               bool first_only = true)
	{
		std::vector<tesseract::ContactResultMap> contacts;
		if(!tesseract::continuousCollisionCheckTrajectory(manager, env, kin, traj, contacts, first_only))
		{
			throw std::runtime_error("continuousCollisionCheckTrajectory failed");			
		}
		
		return contacts;	
	}
%}