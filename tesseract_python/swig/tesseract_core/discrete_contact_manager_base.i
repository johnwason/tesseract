%{
#include <tesseract_core/discrete_contact_manager_base.h>
%}

%shared_ptr(tesseract::DiscreteContactManagerBase)

namespace tesseract
{
	
%nodefaultctor DiscreteContactManagerBase;
	
class DiscreteContactManagerBase
{
public:
	virtual ~DiscreteContactManagerBase();
	
	virtual std::shared_ptr<DiscreteContactManagerBase> clone() const;
	
%extend
{
	
	void addCollisionObject(const std::string& name,
                                    const int& mask_id,
                                    const std::vector<shapes::ShapeConstPtr>& shapes,
                                    const VectorIsometry3d& shape_poses,
                                    const CollisionObjectTypeVector& collision_object_types,
                                    bool enabled = true)
	{
		if (!$self->addCollisionObject(name, mask_id, shapes, shape_poses,
				collision_object_types, enabled))
		{
			throw std::runtime_error("addCollisionObject failed");
		}
	}	

}

	virtual bool hasCollisionObject(const std::string& name) const;
	
%extend
{	
	void removeCollisionObject(const std::string& name)
	{
		if (!$self->removeCollisionObject(name))
		{
			throw std::runtime_error("removeCollisionObject failed");
		}
	}
	
	void enableCollisionObject(const std::string& name)
	{
		if (!$self->enableCollisionObject(name))
		{
			throw std::runtime_error("enableCollisionObject failed");
		}
	}
	
	void disableCollisionObject(const std::string& name)
	{
		if (!$self->disableCollisionObject(name))
		{
			throw std::runtime_error("disableCollisionObject failed");
		}
	}	
}

	virtual void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose);
	
	virtual void setCollisionObjectsTransform(const std::vector<std::string>& names, const VectorIsometry3d& poses);
	
	virtual void setCollisionObjectsTransform(const TransformMap& transforms);
	
	virtual void setActiveCollisionObjects(const std::vector<std::string>& names);
	
	virtual const std::vector<std::string> getActiveCollisionObjects() const;
	
	virtual void setContactDistanceThreshold(double contact_distance);
	
	virtual double getContactDistanceThreshold();
	
	virtual void setIsContactAllowedFn(IsContactAllowedFn fn);
	
	virtual IsContactAllowedFn getIsContactAllowedFn();

%extend
{
	ContactResultMap contactTest(const ContactTestType& type)
	{
		tesseract::ContactResultMap collisions;
		$self->contactTest(collisions, type);
		return collisions;
	}
	
}

};

typedef std::shared_ptr<DiscreteContactManagerBase> DiscreteContactManagerBasePtr;
typedef std::shared_ptr<const DiscreteContactManagerBase> DiscreteContactManagerBaseConstPtr;

}