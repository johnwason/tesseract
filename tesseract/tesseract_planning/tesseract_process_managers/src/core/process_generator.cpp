/**
 * @file process_generator.cpp
 * @brief Process generator
 *
 * @author Matthew Powelson
 * @date July 15. 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_process_managers/core/process_generator.h>

namespace tesseract_planning
{
ProcessGenerator::ProcessGenerator(std::string name) : name_(std::move(name)) {}

const std::string& ProcessGenerator::getName() const { return name_; }

tf::Task ProcessGenerator::generateTask(ProcessInput input, tf::Taskflow& taskflow)
{
  tf::Task task = taskflow.placeholder();
  std::size_t unique_id = task.hash_value();
  task.work([=]() { process(input, unique_id); });
  task.name(getName());
  return task;
}

void ProcessGenerator::assignTask(ProcessInput input, tf::Task& task)
{
  std::size_t unique_id = task.hash_value();
  task.work([=]() { process(input, unique_id); });
  task.name(getName());
}

tf::Task ProcessGenerator::generateConditionalTask(ProcessInput input, tf::Taskflow& taskflow)
{
  tf::Task task = taskflow.placeholder();
  std::size_t unique_id = task.hash_value();
  task.work([=]() { return conditionalProcess(input, unique_id); });
  task.name(getName());
  return task;
}

void ProcessGenerator::assignConditionalTask(ProcessInput input, tf::Task& task)
{
  std::size_t unique_id = task.hash_value();
  task.work([=]() { return conditionalProcess(input, unique_id); });
  task.name(getName());
}
}  // namespace tesseract_planning
