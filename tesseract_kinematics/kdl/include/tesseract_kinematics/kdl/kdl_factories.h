/**
 * @file kdl_factories.h
 * @brief Tesseract KDL Factories.
 *
 * @author Levi Armstrong
 * @date Aug 27, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_KDL_FACTORIES_H
#define TESSERACT_KINEMATICS_KDL_FACTORIES_H

#include <tesseract_kinematics/core/kinematics_plugin_factory.h>

namespace tesseract_kinematics
{
class KDLFwdKinChainFactory : public FwdKinFactory
{
  std::unique_ptr<ForwardKinematics> create(const std::string& solver_name,
                                            const tesseract_scene_graph::SceneGraph& scene_graph,
                                            const tesseract_scene_graph::SceneState& scene_state,
                                            const KinematicsPluginFactory& plugin_factory,
                                            const YAML::Node& config) const override final;
};

class KDLInvKinChainLMAFactory : public InvKinFactory
{
  std::unique_ptr<InverseKinematics> create(const std::string& solver_name,
                                            const tesseract_scene_graph::SceneGraph& scene_graph,
                                            const tesseract_scene_graph::SceneState& scene_state,
                                            const KinematicsPluginFactory& plugin_factory,
                                            const YAML::Node& config) const override final;
};

class KDLInvKinChainNRFactory : public InvKinFactory
{
  std::unique_ptr<InverseKinematics> create(const std::string& solver_name,
                                            const tesseract_scene_graph::SceneGraph& scene_graph,
                                            const tesseract_scene_graph::SceneState& scene_state,
                                            const KinematicsPluginFactory& plugin_factory,
                                            const YAML::Node& config) const override final;
};

class KDLInvKinChainNR_JLFactory : public InvKinFactory
{
  std::unique_ptr<InverseKinematics> create(const std::string& solver_name,
                                            const tesseract_scene_graph::SceneGraph& scene_graph,
                                            const tesseract_scene_graph::SceneState& scene_state,
                                            const KinematicsPluginFactory& plugin_factory,
                                            const YAML::Node& config) const override final;
};

TESSERACT_PLUGIN_ANCHOR_DECL(KDLFactoriesAnchor)

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_KDL_FACTORIES_H
