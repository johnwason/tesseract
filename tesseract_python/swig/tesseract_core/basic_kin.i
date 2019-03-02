%{
#include <tesseract_core/basic_kin.h>
%}

%shared_ptr(tesseract::BasicKin);

namespace tesseract
{
%nodefaultctor BasicKin;
class BasicKin
{
public:
	/*virtual bool calcFwdKin(Eigen::Isometry3d& pose,
                          const Eigen::Isometry3d& change_base,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;
                          
    virtual bool calcFwdKin(Eigen::Isometry3d& pose,
                          const Eigen::Isometry3d& change_base,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                          const std::string& link_name,
                          const EnvState& state) const = 0; */
                          
%extend {
    
    Eigen::Isometry3d calcFwdKin(const Eigen::Isometry3d& change_base,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
    {
    	Eigen::Isometry3d pose;
    	if (!$self->calcFwdKin(pose, change_base, joint_angles))
    	{
    		throw std::runtime_error("calcFwdKin failed");
    	}
    	return pose;
    }
    
    Eigen::Isometry3d calcFwdKin(const Eigen::Isometry3d& change_base,
            const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
            const std::string& link_name,
            const EnvState& state) const
	{
		Eigen::Isometry3d pose;
		if (!$self->calcFwdKin(pose, change_base, joint_angles, link_name, state))
		{
			throw std::runtime_error("calcFwdKin failed");
		}
		return pose;
	}    
    

}


    /*virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                            const Eigen::Isometry3d& change_base,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;
                            
    virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                            const Eigen::Isometry3d& change_base,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                            const std::string& link_name,
                            const EnvState& state) const = 0;
                            
    virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                            const Eigen::Isometry3d& change_base,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                            const std::string& link_name,
                            const EnvState& state,
                            const Eigen::Ref<const Eigen::Vector3d>& link_point) const = 0;
    */

	%extend
	{
		Eigen::MatrixXd calcJacobian(const Eigen::Isometry3d& change_base,
			const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
		{
			Eigen::MatrixXd jacobian(6, joint_angles.rows());
			if(!$self->calcJacobian(jacobian, change_base, joint_angles))
			{
				throw std::runtime_error("calcJacobian failed");
			}
			return jacobian;
		}
		
		Eigen::MatrixXd calcJacobian(const Eigen::Isometry3d& change_base,
			const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
			const std::string& link_name,
			const EnvState& state) const
		{
			Eigen::MatrixXd jacobian(6, joint_angles.rows());
			if(!$self->calcJacobian(jacobian, change_base, joint_angles, link_name, state))
			{
				throw std::runtime_error("calcJacobian failed");
			}
			return jacobian;
		}
		
		Eigen::MatrixXd calcJacobian(const Eigen::Isometry3d& change_base,
			const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
			const std::string& link_name,
			const EnvState& state,
			const Eigen::Ref<const Eigen::Vector3d>& link_point) const
		{
			Eigen::MatrixXd jacobian(6, joint_angles.rows());
			if(!$self->calcJacobian(jacobian, change_base, joint_angles, link_name, state, link_point))
			{
				throw std::runtime_error("calcJacobian failed");
			}
			return jacobian;
		}
		
		
		
	}

                            
    virtual bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const = 0;
    
    virtual const std::vector<std::string> getJointNames() const;
    
    virtual const std::vector<std::string> getLinkNames() const;
    
    virtual const Eigen::MatrixX2d getLimits() const;
    
    virtual unsigned int numJoints() const;
    
    virtual const std::string getBaseLinkName() const;
    
    virtual const std::string getName() const;
    
    virtual void addAttachedLink(const std::string& link_name, const std::string& parent_link_name);
    
    virtual void removeAttachedLink(const std::string& link_name);
    
    virtual void clearAttachedLinks();
    
    /*static bool solvePInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
                        const Eigen::Ref<const Eigen::VectorXd>& b,
                        Eigen::Ref<Eigen::VectorXd> x);
                        
    static bool dampedPInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
                         Eigen::Ref<Eigen::MatrixXd> P,
                         const double eps = 0.011,
                         const double lambda = 0.01);
    */
%extend
{
    static Eigen::VectorXd solvePInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
		const Eigen::Ref<const Eigen::VectorXd>& b)
    {
    	Eigen::VectorXd x;
    	if (!tesseract::BasicKin::solvePInv(A, b, x))
    	{
    		throw std::runtime_error("solvePInv failed");
    	}
    	return x;    	
    }
    
    static Eigen::MatrixXd dampedPInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
		 const double eps = 0.011,
		 const double lambda = 0.01)
    {
    	Eigen::MatrixXd P;
    	if (!tesseract::BasicKin::dampedPInv(A,P,eps,lambda))
    	{
    		throw std::runtime_error("dampedPInv failed");
    	}
    	return P;
    }
}
                       
    virtual ~BasicKin();
};

typedef std::shared_ptr<BasicKin> BasicKinPtr;
typedef std::shared_ptr<const BasicKin> BasicKinConstPtr;

}
