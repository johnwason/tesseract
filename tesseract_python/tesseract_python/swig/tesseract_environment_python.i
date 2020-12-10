/**
 * @file tesseract_environment_python.i
 * @brief The tesseract_environment_python SWIG master file.
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

%module(directors="1", package="tesseract_environment") tesseract_environment_python

#pragma SWIG nowarn=473

%import "tesseract_common_python.i"
%import "tesseract_geometry_python.i"
%import "tesseract_scene_graph_python.i"
%import "tesseract_kinematics_python.i"
%import "tesseract_collision_python.i"

%{
// tesseract_environment
#include <tesseract_environment/core/types.h>
#include <tesseract_environment/core/commands.h>
#include <tesseract_environment/core/state_solver.h>
#include <tesseract_environment/core/environment.h>
%}

// tesseract_environment
#define TESSERACT_ENVIRONMENT_CORE_PUBLIC
%include "tesseract_environment/core/types.h"
%include "tesseract_environment/core/commands.h"
%include "tesseract_environment/core/manipulator_manager.h"
%include "tesseract_environment/core/state_solver.h"
%include "tesseract_environment/core/environment.h"