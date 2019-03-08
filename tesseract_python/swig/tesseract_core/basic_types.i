%{
#include <tesseract_core/basic_types.h>
%}

%shared_ptr(tesseract::AllowedCollisionMatrix);
%shared_ptr(tesseract::EnvState);
%shared_ptr(tesseract::AttachableObject);
%shared_ptr(tesseract::ObjectColorMap);

namespace tesseract
{
namespace CollisionObjectTypes
{
enum CollisionObjectType
{
  UseShapeType = 0,
  ConvexHull = 1,            
  MultiSphere = 2, 
  SDF = 3
};
}
typedef CollisionObjectTypes::CollisionObjectType CollisionObjectType;
typedef std::vector<CollisionObjectType> CollisionObjectTypeVector;

namespace BodyTypes
{
enum BodyType
{
  ROBOT_LINK = 0,    
  ROBOT_ATTACHED = 1 
};
}
typedef BodyTypes::BodyType BodyType;

namespace ContinouseCollisionTypes
{
enum ContinouseCollisionType
{
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};
}
typedef ContinouseCollisionTypes::ContinouseCollisionType ContinouseCollisionType;

namespace ContactTestTypes
{
enum ContactTestType
{
  FIRST = 0,   
  CLOSEST = 1, 
  ALL = 2,     
  LIMITED = 3  
};
}
typedef ContactTestTypes::ContactTestType ContactTestType;

}

%define tesseract_aligned_vector(name,T)
%template(name) std::vector<T , Eigen::aligned_allocator<T >>;
%enddef

%define tesseract_aligned_map(name,Key,Value)
%template(name) std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%define tesseract_aligned_unordered_map(name,Key,Value)
%template(name) std::unordered_map<Key,Value,std::hash<Key>,std::equal_to<Key>,Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

tesseract_aligned_vector(VectorIsometry3d, Eigen::Isometry3d);
tesseract_aligned_vector(VectorVector3d, Eigen::Vector3d);
tesseract_aligned_vector(VectorVector4d, Eigen::Vector4d);

tesseract_aligned_map(TransformMap, std::string, Eigen::Isometry3d);


tesseract_aligned_vector(ContactResultVector, tesseract::ContactResult);
%template(string_pair) std::pair<std::string,std::string>;
%template(ContactResultMap) std::map<std::pair<std::string,std::string>,std::vector<tesseract::ContactResult , Eigen::aligned_allocator<tesseract::ContactResult >>,std::less<std::pair<std::string,std::string>>,Eigen::aligned_allocator<std::pair<const std::pair<std::string,std::string>,std::vector<tesseract::ContactResult , Eigen::aligned_allocator<tesseract::ContactResult >>>>>;
		

//%tesseract_aligned_map(ContactResultMap, %arg(std::pair<std::string,std::string>), std::vector<tesseract::ContactResult , Eigen::aligned_allocator<tesseract::ContactResult >>);
//%template(ContactResultMap_vector) std::vector<std::map<std::pair<std::string,std::string>, tesseract::ContactResult, std::less<std::pair<std::string,std::string>>, Eigen::aligned_allocator<std::pair<const std::pair<std::string,std::string>, tesseract::ContactResult>>>>;

%template(CollisionObjectTypeVector) std::vector<tesseract::CollisionObjectTypes::CollisionObjectType>;

//tesseract_aligned_unordered_map(ObjectColorMap,std::string,tesseract::ObjectColor);
tesseract_aligned_unordered_map(AttachedBodyInfoMap,std::string,tesseract::AttachedBodyInfo);
//%template(AttachableObjectConstPtrMap)  std::unordered_map<std::string, tesseract::AttachableObjectConstPtr, std::hash<std::string>, std::allocator<std::pair< const std::string, tesseract::AttachableObjectConstPtr> > >;

%template(joints_map) std::unordered_map<std::string, double>;

%template(shapes_vector) std::vector<std::shared_ptr<const shapes::Shape> >;

namespace tesseract
{

class AllowedCollisionEntries;

/*template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename Key, typename Value>
using AlignedMap = std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;

template <typename Key, typename Value>
using AlignedUnorderedMap = std::unordered_map<Key,
                                               Value,
                                               std::hash<Key>,
                                               std::equal_to<Key>,
                                               Eigen::aligned_allocator<std::pair<const Key, Value>>>;

using VectorIsometry3d = AlignedVector<Eigen::Isometry3d>;
using VectorVector4d = AlignedVector<Eigen::Vector4d>;
using VectorVector3d = std::vector<Eigen::Vector3d>;
using TransformMap = AlignedMap<std::string, Eigen::Isometry3d>;
*/

typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> VectorIsometry3d;
typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> VectorVector4d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VectorVector3d;
typedef std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>> TransformMap;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;

struct AllowedCollisionMatrix
{
  
  virtual ~AllowedCollisionMatrix() = default;
  
  virtual void addAllowedCollision(const std::string& link_name1,
                                   const std::string& link_name2,
                                   const std::string& reason);

  virtual void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2);
  
  virtual bool isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const;
  
  void clearAllowedCollisions();
  
};
typedef std::shared_ptr<AllowedCollisionMatrix> AllowedCollisionMatrixPtr;
typedef std::shared_ptr<const AllowedCollisionMatrix> AllowedCollisionMatrixConstPtr;

typedef std::function<bool(const std::string&, const std::string&)> IsContactAllowedFn;

struct ContactResult
{  
  double distance;
  int type_id[2];
  std::string link_names[2];
  Eigen::Vector3d nearest_points[2];
  Eigen::Vector3d normal;
  Eigen::Vector3d cc_nearest_points[2];
  double cc_time;
  ContinouseCollisionType cc_type;

  ContactResult();  
  void clear();
};
typedef std::vector<tesseract::ContactResult, Eigen::aligned_allocator<tesseract::ContactResult>> ContactResultVector;
typedef std::map<std::pair<std::string,std::string>, std::vector<tesseract::ContactResult, Eigen::aligned_allocator<tesseract::ContactResult>>, std::less<std::pair<std::string,std::string>>, Eigen::aligned_allocator<std::pair<const std::pair<std::string,std::string>, std::vector<tesseract::ContactResult, Eigen::aligned_allocator<tesseract::ContactResult>>>>> ContactResultMap;

struct ContactTestData
{

  ContactTestData(const std::vector<std::string>& active,
                  const double& contact_distance,
                  const IsContactAllowedFn& fn,
                  const ContactTestType& type,
                  ContactResultMap& res);

  const std::vector<std::string>& active;
  const double& contact_distance;
  const IsContactAllowedFn& fn;
  const ContactTestType& type;

  ContactResultMap& res;

  bool done;

};

inline void moveContactResultsMapToContactResultsVector(ContactResultMap& contact_map,
                                                               ContactResultVector& contact_vector);
                                                              
struct EnvState
{
  std::unordered_map<std::string, double> joints;
  TransformMap transforms;
};
typedef std::shared_ptr<EnvState> EnvStatePtr;
typedef std::shared_ptr<const EnvState> EnvStateConstPtr;

struct AttachedBodyInfo
{
  
  AttachedBodyInfo();
  std::string object_name;
  std::string parent_link_name;
  Eigen::Isometry3d transform;
  std::vector<std::string> touch_links;
};

struct VisualObjectGeometry
{  
  std::vector<std::shared_ptr<const shapes::Shape>> shapes;
  VectorIsometry3d shape_poses;
  VectorVector4d shape_colors;
};
 
struct CollisionObjectGeometry : public VisualObjectGeometry
{  
  std::vector<tesseract::CollisionObjectTypes::CollisionObjectType>
  collision_object_types; 
};

struct AttachableObject
{  
  std::string name;
  VisualObjectGeometry visual;
  CollisionObjectGeometry collision;
};
typedef std::shared_ptr<AttachableObject> AttachableObjectPtr;
typedef std::shared_ptr<const AttachableObject> AttachableObjectConstPtr;

struct ObjectColor
{  
  VectorVector4d visual;
  VectorVector4d collision;
};

typedef AlignedUnorderedMap<std::string, ObjectColor> ObjectColorMap;
typedef std::shared_ptr<ObjectColorMap> ObjectColorMapPtr;
typedef std::shared_ptr<const ObjectColorMap> ObjectColorMapConstPtr;
typedef tesseract::AlignedUnorderedMap<std::string, AttachedBodyInfo> AttachedBodyInfoMap;
typedef std::unordered_map<std::string, AttachableObjectConstPtr, std::hash<std::string> > AttachableObjectConstPtrMap;                 

}