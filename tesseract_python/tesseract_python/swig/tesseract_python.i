/**
 * @file tesseract_python.i
 * @brief The tesseract_python SWIG master file.
 *
 * @author John Wason
 * @date November 26, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%module(directors="1", package="tesseract") tesseract_python

#pragma SWIG nowarn=473

%{
//#define SWIG_PYTHON_EXTRA_NATIVE_CONTAINERS
%}

%include <std_shared_ptr.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_pair.i>
%include <std_map.i>
%include <std_unordered_map.i>
%include <std_array.i>
%include <stdint.i>
%include <attribute.i>
%include <exception.i>
%include <pybuffer.i>

%{
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>

// tesseract_common
#include <tesseract_common/types.h>
#include <tesseract_common/status_code.h>
#include <tesseract_common/resource.h>

// tesseract_geometry
#include <tesseract_geometry/geometry.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/utils.h>

// tesseract_scene_graph
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/allowed_collision_matrix.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_scene_graph/srdf/types.h>
#include <tesseract_scene_graph/srdf_model.h>

// tesseract_kinematics
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/forward_kinematics_factory.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics_factory.h>
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>
#include <tesseract_kinematics/ikfast/ikfast_inv_kin.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain_factory.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr_factory.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>

// tesseract_collision
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/discrete_contact_manager_factory.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager_factory.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>

// tesseract_environment
#include <tesseract_environment/core/types.h>
#include <tesseract_environment/core/commands.h>
#include <tesseract_environment/core/state_solver.h>
#include <tesseract_environment/core/environment.h>

// tesseract_command_language
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/serialize.h>
#include <tesseract_command_language/deserialize.h>

// tesseract
#include <tesseract/tesseract.h>

%}

%exception {
  try {
    $action
  }
  SWIG_CATCH_STDEXCEPT
}

%feature("director:except") {
    if ($error != NULL) {
        throw Swig::DirectorMethodException();
    }
}

%include "eigen.i"
%include "shared_factory.i"
%include "json_typemaps.i"
%include "eigen_types.i"

%template(vector_string) std::vector<std::string>;
%template(pair_string) std::pair<std::string, std::string>;
%template(vector_pair_string) std::vector<std::pair<std::string, std::string> >;
%template(map_string_vector_pair_string) std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>;

%template(vector_double) std::vector<double>;
%template(map_string_vector_double) std::unordered_map<std::string, std::vector<double> >;
%template(map_string_double) std::unordered_map<std::string, double>;
%template(map_string_map_string_double) std::unordered_map<std::string, std::unordered_map<std::string, double> >;
%template(map_string_map_string_map_string_double) std::unordered_map<std::string, std::unordered_map<std::string, std::unordered_map<std::string, double> > >;

%template(array2_int) std::array<int,2>;
%template(array2_string) std::array<std::string,2>;
%template(array2_Vector3d) std::array<Eigen::Vector3d,2>;
%template(array2_Isometry3d) std::array<Eigen::Isometry3d,2>;
%template(array2_double) std::array<double,2>;

%define tesseract_aligned_vector(name,T)
%template(name) std::vector<T , Eigen::aligned_allocator<T >>;
%enddef

%define tesseract_aligned_map(name,Key,Value)
%template(name) std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%define tesseract_aligned_map_of_aligned_vector(name,Key,Value)
tesseract_aligned_map(name, %arg(Key), %arg(std::vector<Value , Eigen::aligned_allocator<Value >>));
%enddef

%define tesseract_aligned_unordered_map(name,Key,Value)
%template(name) std::unordered_map<Key,Value,std::hash<Key>,std::equal_to<Key>,Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

tesseract_aligned_vector(VectorIsometry3d, Eigen::Isometry3d);
tesseract_aligned_vector(VectorVector4d, Eigen::Vector4d);
tesseract_aligned_map(TransformMap, std::string, Eigen::Isometry3d);

#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#define TESSERACT_COMMON_IGNORE_WARNINGS_POP

// tesseract_common
#define TESSERACT_COMMON_PUBLIC
%include "tesseract_common/types.h"
%include "tesseract_common/status_code.h"
%include "tesseract_common/resource.h"

// tesseract_geometry
#define TESSERACT_GEOMETRY_PUBLIC
%include "tesseract_geometry/geometry.h"
%include "tesseract_geometry/geometries.h"
%include "tesseract_geometry/utils.h"

// tesseract_scene_graph
#define TESSERACT_SCENE_GRAPH_PUBLIC
%include "tesseract_scene_graph/joint.h"
%include "tesseract_scene_graph/link.h"
%include "tesseract_scene_graph/allowed_collision_matrix.h"
%include "tesseract_scene_graph/graph.h"
%include "tesseract_scene_graph/resource_locator.h"
%include "tesseract_scene_graph/srdf/types.h"
%include "tesseract_scene_graph/srdf_model.h"

// tesseract_kinematics
#define TESSERACT_KINEMATICS_CORE_PUBLIC
#define TESSERACT_KINEMATICS_IKFAST_PUBLIC
#define TESSERACT_KINEMATICS_KDL_PUBLIC
#define TESSERACT_KINEMATICS_OPW_PUBLIC
%include "tesseract_kinematics/core/forward_kinematics.h"
%include "tesseract_kinematics/core/forward_kinematics_factory.h"
%include "tesseract_kinematics/core/inverse_kinematics.h"
%include "tesseract_kinematics/core/inverse_kinematics_factory.h"
%include "tesseract_kinematics/core/rop_inverse_kinematics.h"
%include "tesseract_kinematics/core/rep_inverse_kinematics.h"
%include "tesseract_kinematics/ikfast/ikfast_inv_kin.h"
%include "tesseract_kinematics/kdl/kdl_fwd_kin_chain.h"
%include "tesseract_kinematics/kdl/kdl_fwd_kin_chain_factory.h"
%include "tesseract_kinematics/kdl/kdl_fwd_kin_tree.h"
%include "tesseract_kinematics/kdl/kdl_fwd_kin_tree_factory.h"
%include "tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h"
%include "tesseract_kinematics/kdl/kdl_inv_kin_chain_lma_factory.h"
%include "tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h"
%include "tesseract_kinematics/kdl/kdl_inv_kin_chain_nr_factory.h"
%include "tesseract_kinematics/opw/opw_inv_kin.h"

// tesseract_collision
#define TESSERACT_COLLISION_CORE_PUBLIC
#define TESSERACT_COLLISION_FCL_PUBLIC
#define TESSERACT_COLLISION_BULLET_PUBLIC
%include "tesseract_collision/core/types.h"
%include "tesseract_collision/core/discrete_contact_manager.h"
%include "tesseract_collision/core/discrete_contact_manager_factory.h"
%include "tesseract_collision/core/continuous_contact_manager.h"
%include "tesseract_collision/core/continuous_contact_manager_factory.h"
%include "tesseract_collision/fcl/fcl_discrete_managers.h"
%include "tesseract_collision/bullet/bullet_cast_bvh_manager.h"
%include "tesseract_collision/bullet/bullet_cast_simple_manager.h"
%include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
%include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"

// tesseract_environment
#define TESSERACT_ENVIRONMENT_CORE_PUBLIC
%include "tesseract_environment/core/types.h"
%include "tesseract_environment/core/commands.h"
%include "tesseract_environment/core/state_solver.h"
%include "tesseract_environment/core/environment.h"

// tesseract_command_language
#define TESSERACT_COMMAND_LANGUAGE_PUBLIC
%{
namespace std
{
  template<typename T> struct remove_reference<swig::SwigPySequence_Ref<T>>
  {
    typedef T type;
  };

  template<typename T> struct remove_reference<const swig::SwigPySequence_Ref<T>>
  {
    typedef const T type;
  };

  template<typename T> struct remove_reference<SwigValueWrapper<T>>
  {
    typedef T type;
  };

  template<typename T> struct remove_reference<const SwigValueWrapper<T>>
  {
    typedef const T type;
  };

  template<typename T> struct remove_reference<SwigValueWrapper<T>&>
  {
    typedef T type;
  };

  template<typename T> struct remove_reference<const SwigValueWrapper<T>&>
  {
    typedef const T type;
  };

}
%}


%include "tesseract_command_language/core/waypoint.h"
%include "tesseract_command_language/core/instruction.h"
%include "tesseract_command_language/command_language.h"
%include "tesseract_command_language/serialize.h"
%include "tesseract_command_language/deserialize.h"

// tesseract
#define TESSERACT_PUBLIC
%include "tesseract/manipulator_manager.h"
%include "tesseract/tesseract_init_info.h"
%include "tesseract/tesseract.h"


/*
%include "tesseract/forward_kinematics_manager.i"
%include "tesseract/inverse_kinematics_manager.i"
%include "tesseract/tesseract.i"
%include "tesseract_visualization/visualization.i"
%include "tesseract_planning/tesseract_motion_planners/core/waypoint.i"
%include "tesseract_planning/tesseract_motion_planners/core/utils.i"
%include "tesseract_planning/tesseract_motion_planners/core/types.i"
%include "tesseract_planning/tesseract_motion_planners/core/trajectory_validator.i"
%include "tesseract_planning/tesseract_motion_planners/core/planner.i"
%include "trajopt/problem_description.i"
%include "tesseract_planning/tesseract_motion_planners/trajopt/config/trajopt_planner_config.i"
%include "tesseract_planning/tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.i"
%include "tesseract_planning/tesseract_motion_planners/trajopt/trajopt_motion_planner.i"*/

