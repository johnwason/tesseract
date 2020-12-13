/**
 * @file kdl_fwd_kin_chain_nr.h
 * @brief Tesseract KDL inverse kinematics chain Newton-Raphson implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_NR_H
#define TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_NR_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <unordered_map>
#include <console_bridge/console.h>

#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/kdl/kdl_utils.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::KDLInvKinChainNR)
#endif // SWIG

namespace tesseract_kinematics
{
/**
 * @brief KDL Inverse kinematic chain implementation.
 */
class KDLInvKinChainNR : public InverseKinematics
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<KDLInvKinChainNR>;
  using ConstPtr = std::shared_ptr<const KDLInvKinChainNR>;

  KDLInvKinChainNR() = default;
  ~KDLInvKinChainNR() override = default;
  KDLInvKinChainNR(const KDLInvKinChainNR&) = delete;
  KDLInvKinChainNR& operator=(const KDLInvKinChainNR&) = delete;
  KDLInvKinChainNR(KDLInvKinChainNR&&) = delete;
  KDLInvKinChainNR& operator=(KDLInvKinChainNR&&) = delete;

  InverseKinematics::Ptr clone() const override;

  bool update() override;

  bool calcInvKin(Eigen::VectorXd& solutions,
                  const Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& seed) const override;

  bool calcInvKin(Eigen::VectorXd& solutions,
                  const Eigen::Isometry3d& pose,
                  const Eigen::Ref<const Eigen::VectorXd>& seed,
                  const std::string& link_name) const override;

  bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const override;

  const std::vector<std::string>& getJointNames() const override;

  const std::vector<std::string>& getLinkNames() const override;

  const std::vector<std::string>& getActiveLinkNames() const override;

  const tesseract_common::KinematicLimits& getLimits() const override;

  void setLimits(tesseract_common::KinematicLimits limits) override;

  unsigned int numJoints() const override;

  const std::string& getBaseLinkName() const override;

  const std::string& getTipLinkName() const override;

  const std::string& getName() const override;

  const std::string& getSolverName() const override;

  tesseract_scene_graph::SceneGraph::ConstPtr getSceneGraph() const;

  /**
   * @brief Initializes KDL Forward Kinematics
   * Creates KDL::Chain from tesseract scene graph
   * @param scene_graph The Tesseract Scene Graph
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @param name The name of the kinematic chain
   * @return True if init() completes successfully
   */
  bool init(const tesseract_scene_graph::SceneGraph::ConstPtr& scene_graph,
            const std::string& base_link,
            const std::string& tip_link,
            const std::string& name);

  /**
   * @brief Initializes Inverse Kinematics as chain
   * Creates a inverse kinematic chain object from sequential chains
   * @param scene_graph The Tesseract Scene Graph
   * @param chains A vector of kinematics chains <base_link, tip_link> that get concatenated
   * @param name The name of the kinematic chain
   * @return True if init() completes successfully
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            const std::vector<std::pair<std::string, std::string> >& chains,
            std::string name);

  /**
   * @brief Checks if kinematics has been initialized
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const;

private:
  bool initialized_{ false };                                  /**< Identifies if the object has been initialized */
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_;    /**< Tesseract Scene Graph */
  KDLChainData kdl_data_;                                      /**< KDL data parsed from Scene Graph */
  std::string name_;                                           /**< Name of the kinematic chain */
  std::string solver_name_{ "KDLInvKinChainNR" };              /**< Name of this solver */
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_; /**< KDL Forward Kinematic Solver */
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;  /**< KDL Inverse kinematic velocity solver */
  std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_solver_;        /**< KDL Inverse kinematic solver */

  /**
   * @brief This used by the clone method
   * @return True if init() completes successfully
   */
  bool init(const KDLInvKinChainNR& kin);

  /** @brief calcFwdKin helper function */
  bool calcInvKinHelper(Eigen::VectorXd& solutions,
                        const Eigen::Isometry3d& pose,
                        const Eigen::Ref<const Eigen::VectorXd>& seed,
                        int segment_num = -1) const;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KDL_INV_KIN_CHAIN_NR_H
