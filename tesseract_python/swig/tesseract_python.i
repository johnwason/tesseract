/**
 * @file tesseract_python.1
 * @brief The tesseract_python SWIG master file.
 *
 * @author John Wason
 * @date February 22, 2019
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

%module(directors="1") tesseract_python

#pragma SWIG nowarn=473

%{ 
#define SWIG_PYTHON_EXTRA_NATIVE_CONTAINERS 
%} 

%include <std_shared_ptr.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_pair.i>
%include <std_map.i>
%include <std_unordered_map.i>
%include <stdint.i>
%include <attribute.i>
%include <exception.i>

%exception {
  try {
    $action
  } 
  SWIG_CATCH_STDEXCEPT  
}


%include "eigen.i"
%include "shared_factory.i"
%include "json_typemaps.i"
%include "eigen_types.i"
%include "rosmsg_typemaps.i"

%template(vector_string) std::vector<std::string>;

%{
#include <ros/ros.h>
%}

%inline %{
void tesseract_python_ros_init(const std::vector<std::string>& args, const std::string& nodename)
{
	std::vector<char*> cargs;
	for (size_t i=0; i<args.size(); i++)
		cargs.push_back((char*)&args[i]);	
	
	int nargs=cargs.size();
	ros::init(nargs, &cargs[0], nodename, ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
}
%}

%pythoncode %{
tesseract_python_ros_init([],"tesseract_python_module")
%}

%include "geometric_shapes/shapes.i"
%include "tesseract_core/basic_types.i"
%include "tesseract_core/basic_kin.i"
%include "tesseract_core/discrete_contact_manager_base.i"
%include "tesseract_core/continuous_contact_manager_base.i"
%include "tesseract_core/basic_env.i"
%include "tesseract_core/basic_plotting.i"

%include "tesseract_ros/ros_basic_kin.i"
%include "tesseract_ros/ros_basic_env.i"
%include "tesseract_ros/ros_basic_plotting.i"
%include "tesseract_ros/kdl/kdl_chain_kin.i"
%include "tesseract_ros/kdl/kdl_joint_kin.i"
%include "tesseract_ros/kdl/kdl_env.i"
%include "tesseract_ros/ros_tesseract_utils.i"

%include "tesseract_planning/basic_planner_types.i"
%include "tesseract_planning/basic_planner.i"

%include "trajopt/problem_description.i"
%include "tesseract_planning/trajopt/trajopt_planner.i"

