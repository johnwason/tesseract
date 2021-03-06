/**
 * @file utils.h
 * @brief Planner utility functions.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
tesseract_common::VectorIsometry3d interpolate(const Eigen::Isometry3d& start, const Eigen::Isometry3d& stop, int steps)
{
  // Required position change
  Eigen::Vector3d delta_translation = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();
  Eigen::Affine3d stop_prime = start.inverse() * stop;
  Eigen::AngleAxisd delta_rotation(stop_prime.rotation());

  // Step size
  Eigen::Vector3d step = delta_translation / steps;

  // Orientation interpolation
  Eigen::Quaterniond start_q(start.rotation());
  Eigen::Quaterniond stop_q(stop.rotation());
  double slerp_ratio = 1.0 / steps;

  tesseract_common::VectorIsometry3d result;
  Eigen::Vector3d trans;
  Eigen::Quaterniond q;
  Eigen::Isometry3d pose;
  result.reserve(static_cast<size_t>(steps) + 1);
  for (unsigned i = 0; i <= static_cast<unsigned>(steps); ++i)
  {
    trans = start_pos + step * i;
    q = start_q.slerp(slerp_ratio * i, stop_q);
    pose = (Eigen::Translation3d(trans) * q);
    result.push_back(pose);
  }
  return result;
}

Eigen::MatrixXd interpolate(const Eigen::Ref<const Eigen::VectorXd>& start,
                            const Eigen::Ref<const Eigen::VectorXd>& stop,
                            int steps)
{
  assert(start.size() == stop.size());

  Eigen::MatrixXd result(start.size(), steps + 1);

  for (int i = 0; i < start.size(); ++i)
    result.row(i) = Eigen::VectorXd::LinSpaced(steps + 1, start(i), stop(i));

  return result;
}

std::vector<Waypoint> interpolate_waypoint(const Waypoint& start, const Waypoint& stop, int steps)
{
  switch (start.getType())
  {
    case static_cast<int>(WaypointType::CARTESIAN_WAYPOINT):
    {
      const auto* w1 = start.cast_const<Eigen::Isometry3d>();
      const auto* w2 = stop.cast_const<Eigen::Isometry3d>();
      tesseract_common::VectorIsometry3d eigen_poses = interpolate(*w1, *w2, steps);

      std::vector<Waypoint> result;
      result.reserve(eigen_poses.size());
      for (auto& eigen_pose : eigen_poses)
        result.emplace_back(CartesianWaypoint(eigen_pose));

      return result;
    }
    case static_cast<int>(WaypointType::JOINT_WAYPOINT):
    {
      const auto* jwp1 = start.cast_const<JointWaypoint>();
      //      const auto* jwp2 = stop.cast_const<JointWaypoint>();

      // TODO: Should check joint names are in the same order

      const auto* w1 = start.cast_const<Eigen::VectorXd>();
      const auto* w2 = stop.cast_const<Eigen::VectorXd>();
      Eigen::MatrixXd joint_poses = interpolate(*w1, *w2, steps);

      std::vector<Waypoint> result;
      result.reserve(static_cast<std::size_t>(joint_poses.cols()));
      for (int i = 0; i < joint_poses.cols(); ++i)
        result.emplace_back(JointWaypoint(jwp1->joint_names, joint_poses.col(i)));

      return result;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("Interpolator for Waypoint type %d is currently not support!", start.getType());
      return std::vector<Waypoint>();
    }
  }
}

bool programFlattenFilter(const Instruction& instruction,
                          const CompositeInstruction& /*composite*/,
                          bool parent_is_first_composite)
{
  if (isMoveInstruction(instruction))
  {
    if (instruction.cast_const<MoveInstruction>()->isStart())
      return (parent_is_first_composite);
  }
  else if (isPlanInstruction(instruction))
  {
    if (instruction.cast_const<PlanInstruction>()->isStart())
      return (parent_is_first_composite);
  }
  else if (isCompositeInstruction(instruction))
  {
    return false;
  }

  return true;
};

std::vector<std::reference_wrapper<Instruction>> flattenProgram(CompositeInstruction& composite_instruction)
{
  return flatten(composite_instruction, programFlattenFilter);
}

std::vector<std::reference_wrapper<const Instruction>> flattenProgram(const CompositeInstruction& composite_instruction)
{
  return flatten(composite_instruction, programFlattenFilter);
}

std::vector<std::reference_wrapper<Instruction>> flattenProgramToPattern(CompositeInstruction& composite_instruction,
                                                                         const CompositeInstruction& pattern)
{
  return flattenToPattern(composite_instruction, pattern, programFlattenFilter);
}

std::vector<std::reference_wrapper<const Instruction>>
flattenProgramToPattern(const CompositeInstruction& composite_instruction, const CompositeInstruction& pattern)
{
  return flattenToPattern(composite_instruction, pattern, programFlattenFilter);
}

bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::ContinuousContactManager& manager,
                         const tesseract_environment::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         const tesseract_collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract_collision::CollisionEvaluatorType::CONTINUOUS &&
      config.type != tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
    throw std::runtime_error("contactCheckProgram was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type (Continuous)");
  bool found = false;

  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    assert(config.longest_valid_segment_length > 0);

    // Flatten results
    std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

    contacts.reserve(mi.size() - 1);
    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      const auto* swp0 = mi.at(iStep).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
      const auto* swp1 =
          mi.at(iStep + 1).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();

      // TODO: Should check joint names and make sure they are in the same order
      double dist = (swp1->position - swp0->position).norm();
      if (dist > config.longest_valid_segment_length)
      {
        long cnt = static_cast<long>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, swp0->position.size());
        for (long iVar = 0; iVar < swp0->position.size(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, swp0->position(iVar), swp1->position(iVar));

        for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
        {
          tesseract_environment::EnvState::Ptr state0 = state_solver.getState(swp0->joint_names, subtraj.row(iSubStep));
          tesseract_environment::EnvState::Ptr state1 =
              state_solver.getState(swp0->joint_names, subtraj.row(iSubStep + 1));
          if (checkTrajectorySegment(contacts, manager, state0, state1, config))
          {
            found = true;
            if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
            {
              std::stringstream ss;
              ss << "Continuous collision detected at step: " << iStep << " of " << (mi.size() - 1)
                 << " substep: " << iSubStep << std::endl;

              ss << "     Names:";
              for (const auto& name : swp0->joint_names)
                ss << " " << name;

              ss << std::endl
                 << "    State0: " << subtraj.row(iSubStep) << std::endl
                 << "    State1: " << subtraj.row(iSubStep + 1) << std::endl;

              CONSOLE_BRIDGE_logError(ss.str().c_str());
            }
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            break;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
      else
      {
        // Flatten results
        std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

        contacts.reserve(mi.size() - 1);
        for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
        {
          const auto* swp0 =
              mi.at(iStep).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
          const auto* swp1 =
              mi.at(iStep + 1).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();

          // TODO: Should check joint names and make sure they are in the same order
          double dist = (swp1->position - swp0->position).norm();
          if (dist > config.longest_valid_segment_length)
          {
            long cnt = static_cast<long>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
            tesseract_common::TrajArray subtraj(cnt, swp0->position.size());
            for (long iVar = 0; iVar < swp0->position.size(); ++iVar)
              subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, swp0->position(iVar), swp1->position(iVar));

            for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
            {
              tesseract_environment::EnvState::Ptr state0 =
                  state_solver.getState(swp0->joint_names, subtraj.row(iSubStep));
              tesseract_environment::EnvState::Ptr state1 =
                  state_solver.getState(swp0->joint_names, subtraj.row(iSubStep + 1));
              if (checkTrajectorySegment(contacts, manager, state0, state1, config))
              {
                found = true;
                if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
                {
                  std::stringstream ss;
                  ss << "Continuous collision detected at step: " << iStep << " of " << (mi.size() - 1)
                     << " substep: " << iSubStep << std::endl;

                  ss << "     Names:";
                  for (const auto& name : swp0->joint_names)
                    ss << " " << name;

                  ss << std::endl
                     << "    State0: " << subtraj.row(iSubStep) << std::endl
                     << "    State1: " << subtraj.row(iSubStep + 1) << std::endl;

                  CONSOLE_BRIDGE_logError(ss.str().c_str());
                }
              }

              if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
                break;
            }

            if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
              break;
          }
        }
      }
    }
  }
  else
  {
    bool found = false;

    // Flatten results
    std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

    contacts.reserve(mi.size());
    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      const auto* swp0 = mi.at(iStep).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
      const auto* swp1 =
          mi.at(iStep + 1).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
      tesseract_environment::EnvState::Ptr state0 = state_solver.getState(swp0->joint_names, swp0->position);
      tesseract_environment::EnvState::Ptr state1 = state_solver.getState(swp1->joint_names, swp1->position);

      if (checkTrajectorySegment(contacts, manager, state0, state1, config))
      {
        found = true;
        if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : swp0->joint_names)
            ss << " " << name;

          ss << std::endl
             << "    State0: " << swp0->position << std::endl
             << "    State1: " << swp1->position << std::endl;

          CONSOLE_BRIDGE_logError(ss.str().c_str());
        }
      }

      if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }
  return found;
}

bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::DiscreteContactManager& manager,
                         const tesseract_environment::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         const tesseract_collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract_collision::CollisionEvaluatorType::DISCRETE &&
      config.type != tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
    throw std::runtime_error("contactCheckProgram was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type (Discrete)");
  bool found = false;

  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    assert(config.longest_valid_segment_length > 0);

    // Flatten results
    std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

    contacts.reserve(mi.size());
    for (std::size_t iStep = 0; iStep < mi.size(); ++iStep)
    {
      const auto* swp0 = mi.at(iStep).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
      const StateWaypoint* swp1 = nullptr;

      double dist = -1;
      if (iStep < mi.size() - 1)
      {
        swp1 = mi.at(iStep + 1).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();
        dist = (swp1->position - swp0->position).norm();
      }

      if (dist > 0 && dist > config.longest_valid_segment_length)
      {
        int cnt = static_cast<int>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, swp0->position.size());
        for (long iVar = 0; iVar < swp0->position.size(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, swp0->position(iVar), swp1->position(iVar));

        for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
        {
          tesseract_environment::EnvState::Ptr state = state_solver.getState(swp0->joint_names, subtraj.row(iSubStep));
          if (checkTrajectoryState(contacts, manager, state, config))
          {
            found = true;
            if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
            {
              std::stringstream ss;
              ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1)
                 << " substate: " << iSubStep << std::endl;

              ss << "     Names:";
              for (const auto& name : swp0->joint_names)
                ss << " " << name;

              ss << std::endl << "    State: " << subtraj.row(iSubStep) << std::endl;

              CONSOLE_BRIDGE_logError(ss.str().c_str());
            }
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            break;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
      else
      {
        tesseract_environment::EnvState::Ptr state = state_solver.getState(swp0->joint_names, swp0->position);
        if (checkTrajectoryState(contacts, manager, state, config))
        {
          found = true;
          if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
          {
            std::stringstream ss;
            ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

            ss << "     Names:";
            for (const auto& name : swp0->joint_names)
              ss << " " << name;

            ss << std::endl << "    State: " << swp0->position << std::endl;

            CONSOLE_BRIDGE_logError(ss.str().c_str());
          }
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
    }
  }
  else
  {
    // Flatten results
    std::vector<std::reference_wrapper<const Instruction>> mi = flatten(program, moveFilter);

    contacts.reserve(mi.size());
    for (std::size_t iStep = 0; iStep < mi.size() - 1; ++iStep)
    {
      const auto* swp0 = mi.at(iStep).get().cast_const<MoveInstruction>()->getWaypoint().cast_const<StateWaypoint>();

      tesseract_environment::EnvState::Ptr state = state_solver.getState(swp0->joint_names, swp0->position);
      if (checkTrajectoryState(contacts, manager, state, config))
      {
        found = true;
        if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (mi.size() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : swp0->joint_names)
            ss << " " << name;

          ss << std::endl << "    State0: " << swp0->position << std::endl;

          CONSOLE_BRIDGE_logError(ss.str().c_str());
        }
      }

      if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }
  return found;
}

bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::ContinuousContactManager& manager,
                         const tesseract_environment::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         const tesseract_collision::ContactRequest& request)
{
  tesseract_collision::CollisionCheckConfig config;
  config.contact_request = request;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  return contactCheckProgram(contacts, manager, state_solver, program, config);
}

bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::ContinuousContactManager& manager,
                         const tesseract_environment::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         double longest_valid_segment_length,
                         const tesseract_collision::ContactRequest& request)
{
  tesseract_collision::CollisionCheckConfig config;
  config.contact_request = request;
  config.type = tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS;
  config.longest_valid_segment_length = longest_valid_segment_length;
  return contactCheckProgram(contacts, manager, state_solver, program, config);
}

bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::DiscreteContactManager& manager,
                         const tesseract_environment::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         const tesseract_collision::ContactRequest& request)
{
  tesseract_collision::CollisionCheckConfig config;
  config.contact_request = request;
  config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  return contactCheckProgram(contacts, manager, state_solver, program, config);
}

bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::DiscreteContactManager& manager,
                         const tesseract_environment::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         double longest_valid_segment_length,
                         const tesseract_collision::ContactRequest& request)
{
  tesseract_collision::CollisionCheckConfig config;
  config.contact_request = request;
  config.type = tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE;
  config.longest_valid_segment_length = longest_valid_segment_length;
  return contactCheckProgram(contacts, manager, state_solver, program, config);
}

void generateNaiveSeedHelper(CompositeInstruction& composite_instructions,
                             const tesseract_environment::Environment& env,
                             const tesseract_environment::EnvState& env_state,
                             const ManipulatorInfo& manip_info)
{
  for (auto& i : composite_instructions)
  {
    if (isCompositeInstruction(i))
    {
      generateNaiveSeedHelper(*(i.cast<CompositeInstruction>()), env, env_state, manip_info);
    }
    else if (isPlanInstruction(i))
    {
      PlanInstruction* base_instruction = i.cast<PlanInstruction>();
      ManipulatorInfo mi = manip_info.getCombined(base_instruction->getManipulatorInfo());

      CompositeInstruction ci;
      ci.setProfile(base_instruction->getProfile());
      ci.setDescription(base_instruction->getDescription());
      ci.setManipulatorInfo(base_instruction->getManipulatorInfo());

      auto fwd_kin = env.getManipulatorManager()->getFwdKinematicSolver(mi.manipulator);
      Eigen::VectorXd jv = env_state.getJointValues(fwd_kin->getJointNames());

      // Get move type base on base instruction type
      MoveInstructionType move_type;
      if (base_instruction->isLinear())
        move_type = MoveInstructionType::LINEAR;
      else if (base_instruction->isFreespace())
        move_type = MoveInstructionType::FREESPACE;
      else
        throw std::runtime_error("generateNaiveSeed: Unsupported Move Instruction Type!");

      if (isStateWaypoint(base_instruction->getWaypoint()))
      {
        MoveInstruction move_instruction(base_instruction->getWaypoint(), move_type);
        move_instruction.setManipulatorInfo(base_instruction->getManipulatorInfo());
        move_instruction.setDescription(base_instruction->getDescription());
        move_instruction.setProfile(base_instruction->getProfile());
        ci.push_back(move_instruction);
      }
      else if (isJointWaypoint(base_instruction->getWaypoint()))
      {
        const auto* jwp = base_instruction->getWaypoint().cast<JointWaypoint>();
        MoveInstruction move_instruction(StateWaypoint(jwp->joint_names, jwp->waypoint), move_type);
        move_instruction.setManipulatorInfo(base_instruction->getManipulatorInfo());
        move_instruction.setDescription(base_instruction->getDescription());
        move_instruction.setProfile(base_instruction->getProfile());
        ci.push_back(move_instruction);
      }
      else
      {
        MoveInstruction move_instruction(StateWaypoint(fwd_kin->getJointNames(), jv), move_type);
        move_instruction.setManipulatorInfo(base_instruction->getManipulatorInfo());
        move_instruction.setDescription(base_instruction->getDescription());
        move_instruction.setProfile(base_instruction->getProfile());
        ci.push_back(move_instruction);
      }

      i = ci;
    }
  }
}

CompositeInstruction generateNaiveSeed(const CompositeInstruction& composite_instructions,
                                       const tesseract_environment::Environment& env)
{
  if (!composite_instructions.hasStartInstruction())
    throw std::runtime_error("Top most composite instruction is missing start instruction!");

  tesseract_environment::EnvState::ConstPtr env_state = env.getCurrentState();
  CompositeInstruction seed = composite_instructions;
  ManipulatorInfo mi = composite_instructions.getManipulatorInfo();

  Waypoint wp = NullWaypoint();
  ManipulatorInfo base_mi;
  std::string description;
  std::string profile;
  if (isPlanInstruction(seed.getStartInstruction()))
  {
    const auto* pi = seed.getStartInstruction().cast<PlanInstruction>();
    wp = pi->getWaypoint();
    base_mi = pi->getManipulatorInfo();
    description = pi->getDescription();
    profile = pi->getProfile();
  }
  else if (isMoveInstruction(seed.getStartInstruction()))
  {
    const auto* pi = seed.getStartInstruction().cast<MoveInstruction>();
    wp = pi->getWaypoint();
    base_mi = pi->getManipulatorInfo();
    description = pi->getDescription();
    profile = pi->getProfile();
  }
  else
    throw std::runtime_error("Top most composite instruction start instruction has invalid waypoint type!");

  ManipulatorInfo start_mi = mi.getCombined(base_mi);
  auto fwd_kin = env.getManipulatorManager()->getFwdKinematicSolver(start_mi.manipulator);
  Eigen::VectorXd jv = env_state->getJointValues(fwd_kin->getJointNames());

  if (isStateWaypoint(wp))
  {
    MoveInstruction move_instruction(wp, MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    move_instruction.setDescription(description);
    move_instruction.setProfile(profile);
    seed.setStartInstruction(move_instruction);
  }
  else if (isJointWaypoint(wp))
  {
    const auto* jwp = wp.cast<JointWaypoint>();
    MoveInstruction move_instruction(StateWaypoint(jwp->joint_names, jwp->waypoint), MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    move_instruction.setDescription(description);
    move_instruction.setProfile(profile);
    seed.setStartInstruction(move_instruction);
  }
  else
  {
    MoveInstruction move_instruction(StateWaypoint(fwd_kin->getJointNames(), jv), MoveInstructionType::START);
    move_instruction.setManipulatorInfo(base_mi);
    move_instruction.setDescription(description);
    move_instruction.setProfile(profile);
    seed.setStartInstruction(move_instruction);
  }

  generateNaiveSeedHelper(seed, env, *env_state, mi);
  return seed;
}
};  // namespace tesseract_planning
