/**
 * @file types.h
 * @brief Tesseracts Collision Common Types
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_COLLISION_TYPES_H
#define TESSERACT_COLLISION_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
#include <functional>
#include <boost/bind.hpp>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/visibility_control.h>

namespace tesseract_collision
{
using CollisionShapesConst = std::vector<tesseract_geometry::Geometry::ConstPtr>;
using CollisionShapeConstPtr = tesseract_geometry::Geometry::ConstPtr;
using CollisionShapePtr = tesseract_geometry::Geometry::Ptr;

/**
 * @brief Should return true if contact allowed, otherwise false.
 *
 * Also the order of strings should not matter, the function should handled by the function.
 */
using IsContactAllowedFn = std::function<bool(const std::string&, const std::string&)>;

enum class ContinuousCollisionType
{
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};

enum class ContactTestType
{
  FIRST = 0,   /**< Return at first contact for any pair of objects */
  CLOSEST = 1, /**< Return the global minimum for a pair of objects */
  ALL = 2,     /**< Return all contacts for a pair of objects */
  LIMITED = 3  /**< Return limited set of contacts for a pair of objects */
};

static const std::vector<std::string> ContactTestTypeStrings = {
  "FIRST",
  "CLOSEST",
  "ALL",
  "LIMITED",
};

struct TESSERACT_COLLISION_CORE_PUBLIC ContactResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief The distance between two links */
  double distance;
  /** @brief A user defined type id that is added to the contact shapes */
  std::array<int,2> type_id;
  /** @brief The two links that are in contact */
  std::array<std::string,2> link_names;
  /** @brief The two shapes that are in contact. Each link can be made up of multiple shapes */
  std::array<int,2> shape_id;
  /** @brief Some shapes like octomap and mesh have subshape (boxes and triangles) */
  std::array<int,2> subshape_id;
  /** @brief The nearest point on both links in world coordinates */
  std::array<Eigen::Vector3d,2> nearest_points;
  /** @brief The nearest point on both links in local(link) coordinates */
  std::array<Eigen::Vector3d,2> nearest_points_local;
  /** @brief The transform of link in world coordinates */
  std::array<Eigen::Isometry3d,2> transform;
  /**
   * @brief The normal vector to move the two objects out of contact in world coordinates
   *
   * @note This points from link_name[0] to link_name[1], so it shows the direction to move link_name[1] to avoid or get
   *       out of collision with link_name[0].
   */
  Eigen::Vector3d normal;
  /** @brief This is between 0 and 1 indicating the point of contact */
  std::array<double,2> cc_time;
  /** @brief The type of continuous contact */
  std::array<ContinuousCollisionType, 2> cc_type;
  /** @brief The transform of link in world coordinates at its desired final location.
   * Note: This is not the location of the link at the point of contact but the final location the link when performing
   *       continuous collision checking. If you desire the location of contact use cc_time and interpolate between
   *       transform and cc_transform;
   */
  std::array<Eigen::Isometry3d,2> cc_transform;

  /** @brief Some collision checkers only provide a single contact point for a given pair. This is used to indicate
   * if only one contact point is provided which means nearest_points[0] must equal nearest_points[1].
   */
  bool single_contact_point = false;

  ContactResult() { clear(); }

  /** @brief reset to default values */
  void clear()
  {
    distance = std::numeric_limits<double>::max();
    nearest_points[0].setZero();
    nearest_points[1].setZero();
    nearest_points_local[0].setZero();
    nearest_points_local[1].setZero();
    transform[0] = Eigen::Isometry3d::Identity();
    transform[1] = Eigen::Isometry3d::Identity();
    link_names[0] = "";
    link_names[1] = "";
    shape_id[0] = -1;
    shape_id[1] = -1;
    subshape_id[0] = -1;
    subshape_id[1] = -1;
    type_id[0] = 0;
    type_id[1] = 0;
    normal.setZero();
    cc_time[0] = -1;
    cc_time[1] = -1;
    cc_type[0] = ContinuousCollisionType::CCType_None;
    cc_type[1] = ContinuousCollisionType::CCType_None;
    cc_transform[0] = Eigen::Isometry3d::Identity();
    cc_transform[1] = Eigen::Isometry3d::Identity();
    single_contact_point = false;
  }
};

#ifndef SWIG
using ContactResultVector = tesseract_common::AlignedVector<ContactResult>;
using ContactResultMap = tesseract_common::AlignedMap<std::pair<std::string, std::string>, ContactResultVector>;
#else
using ContactResultVector = std::vector<ContactResult, Eigen::aligned_allocator<ContactResult>>;
using ContactResultMap = std::map<std::pair<std::string, std::string>, std::vector<ContactResult, Eigen::aligned_allocator<ContactResult>>, std::less<std::pair<std::string, std::string>>, Eigen::aligned_allocator<std::pair<const std::pair<std::string, std::string>, std::vector<ContactResult, Eigen::aligned_allocator<ContactResult>>>>>;
#endif
/**
 * @brief Should return true if contact results are valid, otherwise false.
 *
 * This is used so users may provide a callback to reject/approve collision results in various algorithms.
 */
using IsContactResultValidFn = std::function<bool(const ContactResult&)>;

/** @brief The ContactRequest struct */
struct TESSERACT_COLLISION_CORE_PUBLIC ContactRequest
{
  /** @brief This controls the exit condition for the contact test type */
  ContactTestType type = ContactTestType::ALL ;

  /** @brief This enables the calculation of penetration contact data if two objects are in collision */
  bool calculate_penetration = true ;

  /** @brief This enables the calculation of distance data if two objects are within the contact threshold */
  bool calculate_distance = true;

  /** @brief This is used if the ContactTestType is set to LIMITED, where the test will exit when number of contacts
   * reach this limit */
  long contact_limit = 0;

  /** @brief This provides a user defined function approve/reject contact results */
  IsContactResultValidFn is_valid = nullptr;

  ContactRequest(ContactTestType type = ContactTestType::ALL) : type(type) {}
};

#ifndef SWIG

inline std::size_t flattenMoveResults(ContactResultMap&& m, ContactResultVector& v)
{
  v.clear();
  v.reserve(m.size());
  for (const auto& mv : m)
    std::move(mv.second.begin(), mv.second.end(), std::back_inserter(v));

  return v.size();
}

inline std::size_t flattenCopyResults(const ContactResultMap& m, ContactResultVector& v)
{
  v.clear();
  v.reserve(m.size());
  for (const auto& mv : m)
    std::copy(mv.second.begin(), mv.second.end(), std::back_inserter(v));

  return v.size();
}

// Need to mark depricated
inline std::size_t flattenResults(ContactResultMap&& m, ContactResultVector& v)
{
  return flattenMoveResults(std::move(m), v);
}

#endif // SWIG

#ifndef SWIG
/**
 * @brief This data is intended only to be used internal to the collision checkers as a container and should not
 *        be externally used by other libraries or packages.
 */
struct TESSERACT_COLLISION_CORE_LOCAL ContactTestData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContactTestData() = default;
  ContactTestData(const std::vector<std::string>& active,
                  double contact_distance,
                  IsContactAllowedFn fn,
                  ContactRequest req,
                  ContactResultMap& res)
    : active(&active), contact_distance(contact_distance), fn(std::move(fn)), req(std::move(req)), res(&res)
  {
  }

  /** @brief A vector of active links */
  const std::vector<std::string>* active = nullptr;

  /** @brief The current contact_distance threshold */
  double contact_distance = 0;

  /** @brief The allowed collision function used to check if two links should be excluded from collision checking */
  IsContactAllowedFn fn = nullptr;

  /** @brief The type of contact request data */
  ContactRequest req;

  /** @brief Destance query results information */
  ContactResultMap* res = nullptr;

  /** @brief Indicate if search is finished */
  bool done = false;
};
#endif // SWIG
}  // namespace tesseract_collision

#ifdef SWIG
tesseract_aligned_vector(ContactResultVector, tesseract_collision::ContactResult);
tesseract_aligned_map_of_aligned_vector(ContactResultMap, %arg(std::pair<std::string,std::string>), tesseract_collision::ContactResult);

%inline
{
//TODO: this function has lousy performance
tesseract_collision::ContactResultVector flattenResults(tesseract_collision::ContactResultMap m)
{
    tesseract_collision::ContactResultVector v;
    tesseract_collision::flattenResults(std::move(m),v);
    return v;
}
}

#endif // SWIG

#endif  // TESSERACT_COLLISION_TYPES_H
