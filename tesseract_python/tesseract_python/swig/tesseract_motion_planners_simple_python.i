/**
 * @file tesseract_motion_planners_simple_python.i
 * @brief The tesseract_motion_planners_simple_python SWIG master file.
 *
 * @author John Wason
 * @date December 8, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Wason Technology, LLC
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

%module(directors="1", package="tesseract_motion_planners_simple") tesseract_motion_planners_simple_python

#pragma SWIG nowarn=473

%import "tesseract_common_python.i"
%import "tesseract_geometry_python.i"
%import "tesseract_scene_graph_python.i"
%import "tesseract_kinematics_python.i"
%import "tesseract_collision_python.i"
%import "tesseract_environment_python.i"
%import "tesseract_command_language_python.i"
%import "tesseract_python.i"
%import "tesseract_motion_planners_python.i"

%{
// tesseract_motion_planners_simple
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_lvs_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_interpolation_plan_profile.h>
#include <tesseract_motion_planners/simple/step_generators/fixed_size_assign_position.h>
#include <tesseract_motion_planners/simple/step_generators/fixed_size_interpolation.h>
#include <tesseract_motion_planners/simple/step_generators/lvs_interpolation.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
%}


// tesseract_motion_planners_simple
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC
%include "tesseract_motion_planners/simple/profile/simple_planner_profile.h"
%include "tesseract_motion_planners/simple/profile/simple_planner_default_lvs_plan_profile.h"
%include "tesseract_motion_planners/simple/profile/simple_planner_interpolation_plan_profile.h"
%include "tesseract_motion_planners/simple/step_generators/fixed_size_assign_position.h"
%include "tesseract_motion_planners/simple/step_generators/fixed_size_interpolation.h"
%include "tesseract_motion_planners/simple/step_generators/lvs_interpolation.h"
%include "tesseract_motion_planners/simple/simple_motion_planner.h"
