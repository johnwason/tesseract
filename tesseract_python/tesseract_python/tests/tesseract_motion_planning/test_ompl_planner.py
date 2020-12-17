import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn
from tesseract.tesseract_core import Tesseract
from tesseract.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond
from tesseract.tesseract_command_language import ManipulatorInfo, JointWaypoint, CartesianWaypoint, Waypoint, \
    PlanInstructionType_FREESPACE, PlanInstructionType_START, PlanInstruction, Instruction, \
    isMoveInstruction, isStateWaypoint, CompositeInstruction, flatten, isMoveInstruction, isStateWaypoint
from tesseract.tesseract_motion_planners import PlannerRequest, PlannerResponse, generateSeed
from tesseract.tesseract_motion_planners_ompl import OMPLDefaultPlanProfile, RRTstarConfigurator, \
    OMPLProblemGeneratorFn, OMPLMotionPlanner, DefaultOMPLProblemGenerator, OMPLMotionPlannerStatusCategory

def _locate_resource(url):
    try:
        url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
        if (url_match is None):
            return ""    
        if not "TESSERACT_SUPPORT_DIR" in os.environ:
            return ""
        tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
        return os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))
    except:
        traceback.print_exc()

def get_tesseract():
    locate_resource_fn = SimpleResourceLocatorFn(_locate_resource)
    locator = SimpleResourceLocator(locate_resource_fn)
    tesseract = Tesseract()
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf"))
    srdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf"))
    assert tesseract.init(urdf_path, srdf_path, locator)
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    
    return tesseract, manip_info

def test_ompl_freespace_joint_cart():

    tesseract, manip = get_tesseract()

    fwd_kin = tesseract.getEnvironment().getManipulatorManager().getFwdKinematicSolver(manip.manipulator)
    inv_kin = tesseract.getEnvironment().getManipulatorManager().getInvKinematicSolver(manip.manipulator)
    joint_names = fwd_kin.getJointNames()
    cur_state = tesseract.getEnvironment().getCurrentState()

    wp1 = JointWaypoint(joint_names, np.array([0,0,0,-1.57,0,0,0],dtype=np.float64))
    wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(-.2,.4,0.2) * Quaterniond(0,0,1.0,0))

    start_instruction = PlanInstruction(Waypoint(wp1), PlanInstructionType_START, "TEST_PROFILE")
    plan_f1 = PlanInstruction(Waypoint(wp2), PlanInstructionType_FREESPACE, "TEST_PROFILE")

    program = CompositeInstruction("TEST_PROFILE")
    program.setStartInstruction(Instruction(start_instruction))
    program.setManipulatorInfo(manip)
    program.append(Instruction(plan_f1))

    seed = generateSeed(program, cur_state, tesseract, 3.14, 1.0, 3.14, 10)

    plan_profile = OMPLDefaultPlanProfile()
    
    test_planner = OMPLMotionPlanner()
    test_planner.plan_profiles["TEST_PROFILE"] = plan_profile
    problem_generator_fn = OMPLProblemGeneratorFn(DefaultOMPLProblemGenerator)
    test_planner.problem_generator = problem_generator_fn

    request = PlannerRequest()
    request.seed = seed
    request.instructions = program
    request.tesseract = tesseract
    request.env_state = tesseract.getEnvironment().getCurrentState()
    
    response = PlannerResponse()

    assert test_planner.solve(request, response)
    assert response.status.value() == OMPLMotionPlannerStatusCategory.SolutionFound

    results = flatten(response.results)

    assert len(results) == 11
    for instr in results:
        assert isMoveInstruction(instr)
        wp1 = instr.cast_MoveInstruction().getWaypoint()
        assert isStateWaypoint(wp1)
        wp = wp1.cast_StateWaypoint()
        assert len(wp.joint_names) == 7
        assert isinstance(wp.position,np.ndarray)
        assert len(wp.position) == 7

def test_ompl_freespace_joint_cart_rrtstar():

    tesseract, manip = get_tesseract()

    fwd_kin = tesseract.getEnvironment().getManipulatorManager().getFwdKinematicSolver(manip.manipulator)
    inv_kin = tesseract.getEnvironment().getManipulatorManager().getInvKinematicSolver(manip.manipulator)
    joint_names = fwd_kin.getJointNames()
    cur_state = tesseract.getEnvironment().getCurrentState()

    wp1 = JointWaypoint(joint_names, np.array([0,0,0,-1.57,0,0,0],dtype=np.float64))
    wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(-.2,.4,0.2) * Quaterniond(0,0,1.0,0))

    start_instruction = PlanInstruction(Waypoint(wp1), PlanInstructionType_START, "TEST_PROFILE")
    plan_f1 = PlanInstruction(Waypoint(wp2), PlanInstructionType_FREESPACE, "TEST_PROFILE")

    program = CompositeInstruction("TEST_PROFILE")
    program.setStartInstruction(Instruction(start_instruction))
    program.setManipulatorInfo(manip)
    program.append(Instruction(plan_f1))

    seed = generateSeed(program, cur_state, tesseract, 3.14, 1.0, 3.14, 10)

    plan_profile = OMPLDefaultPlanProfile()
    plan_profile.planners.clear()
    plan_profile.planners.append(RRTstarConfigurator())
    
    test_planner = OMPLMotionPlanner()
    test_planner.plan_profiles["TEST_PROFILE"] = plan_profile
    problem_generator_fn = OMPLProblemGeneratorFn(DefaultOMPLProblemGenerator)
    test_planner.problem_generator = problem_generator_fn

    request = PlannerRequest()
    request.seed = seed
    request.instructions = program
    request.tesseract = tesseract
    request.env_state = tesseract.getEnvironment().getCurrentState()
    
    response = PlannerResponse()

    assert test_planner.solve(request, response)
    assert response.status.value() == OMPLMotionPlannerStatusCategory.SolutionFound

    results = flatten(response.results)

    assert len(results) == 11
    for instr in results:
        assert isMoveInstruction(instr)
        wp1 = instr.cast_MoveInstruction().getWaypoint()
        assert isStateWaypoint(wp1)
        wp = wp1.cast_StateWaypoint()
        assert len(wp.joint_names) == 7
        assert isinstance(wp.position,np.ndarray)
        assert len(wp.position) == 7