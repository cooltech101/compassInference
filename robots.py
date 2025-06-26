# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import torch

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg, DelayedPDActuatorCfg, RemotizedPDActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg





joey = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(os.path.dirname(__file__), "../usd/joey/joey.usd"),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        copy_from_source=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.02),
        rot=(1, 0.0, 0.0, 0),
        joint_pos={
            "joey_right_wheel_joint": 0.0,
            "joey_left_wheel_joint": 0.0,
        },
        joint_vel={
            "joey_left_wheel_joint": 0.0,
            "joey_right_wheel_joint": 0.0,
        },
    ),
    actuators={
        "body":
            ImplicitActuatorCfg(
                joint_names_expr=[
                    "joey_left_wheel_joint",
                    "joey_left_wheel_joint",
                ],
                velocity_limit=1000000,
                stiffness=0.0,
                damping=17000,
            ),
    },
)






carter = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(os.path.dirname(__file__), "../usd/carter/carter.usd"),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        copy_from_source=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.02),
        rot=(1, 0.0, 0.0, 0),
        joint_pos={
            "joint_wheel_left": 0.0,
            "joint_wheel_right": 0.0,
        },
        joint_vel={
            "joint_wheel_left": 0.0,
            "joint_wheel_right": 0.0,
        },
    ),
    actuators={
        "body":
            ImplicitActuatorCfg(
                joint_names_expr=[
                    "joint_wheel_left",
                    "joint_wheel_right",
                    "joint_caster_base",
                    "joint_swing_left",
                    "joint_swing_right",
                    "joint_caster_left",
                    "joint_caster_right",
                ],
                velocity_limit=1000,
                stiffness=0.0,
                damping=0.0,
            ),
    },
)

h1 = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(os.path.dirname(__file__), "../usd/h1_minimum/h1_minimum.usd"),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.05),
        joint_pos={
            ".*_hip_yaw": 0.0,
            ".*_hip_roll": 0.0,
            ".*_hip_pitch": -0.28,    # -16 degrees
            ".*_knee": 0.79,    # 45 degrees
            ".*_ankle": -0.52,    # -30 degrees
            "torso": 0.0,
            ".*_shoulder_pitch": 0.28,
            ".*_shoulder_roll": 0.0,
            ".*_shoulder_yaw": 0.0,
            ".*_elbow": 0.52,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs":
            ImplicitActuatorCfg(
                joint_names_expr=[".*_hip_yaw", ".*_hip_roll", ".*_hip_pitch", ".*_knee", "torso"],
                effort_limit=300,
                velocity_limit=100.0,
                stiffness={
                    ".*_hip_yaw": 150.0,
                    ".*_hip_roll": 150.0,
                    ".*_hip_pitch": 200.0,
                    ".*_knee": 200.0,
                    "torso": 200.0,
                },
                damping={
                    ".*_hip_yaw": 5.0,
                    ".*_hip_roll": 5.0,
                    ".*_hip_pitch": 5.0,
                    ".*_knee": 5.0,
                    "torso": 5.0,
                },
            ),
        "feet":
            ImplicitActuatorCfg(
                joint_names_expr=[".*_ankle"],
                effort_limit=100,
                velocity_limit=100.0,
                stiffness={".*_ankle": 20.0},
                damping={".*_ankle": 4.0},
            ),
        "arms":
            ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_shoulder_pitch", ".*_shoulder_roll", ".*_shoulder_yaw", ".*_elbow"
                ],
                effort_limit=300,
                velocity_limit=100.0,
                stiffness={
                    ".*_shoulder_pitch": 40.0,
                    ".*_shoulder_roll": 40.0,
                    ".*_shoulder_yaw": 40.0,
                    ".*_elbow": 40.0,
                },
                damping={
                    ".*_shoulder_pitch": 10.0,
                    ".*_shoulder_roll": 10.0,
                    ".*_shoulder_yaw": 10.0,
                    ".*_elbow": 10.0,
                },
            ),
    },
)
"""Configuration for the Unitree H1 Humanoid robot."""

# Note: This data was collected by the Boston Dynamics AI Institute.
spot_joint_parameter_lookup = torch.tensor([
    [-2.792900, -24.776718, 37.165077],
    [-2.767442, -26.290108, 39.435162],
    [-2.741984, -27.793369, 41.690054],
    [-2.716526, -29.285997, 43.928996],
    [-2.691068, -30.767536, 46.151304],
    [-2.665610, -32.237423, 48.356134],
    [-2.640152, -33.695168, 50.542751],
    [-2.614694, -35.140221, 52.710331],
    [-2.589236, -36.572052, 54.858078],
    [-2.563778, -37.990086, 56.985128],
    [-2.538320, -39.393730, 59.090595],
    [-2.512862, -40.782406, 61.173609],
    [-2.487404, -42.155487, 63.233231],
    [-2.461946, -43.512371, 65.268557],
    [-2.436488, -44.852371, 67.278557],
    [-2.411030, -46.174873, 69.262310],
    [-2.385572, -47.479156, 71.218735],
    [-2.360114, -48.764549, 73.146824],
    [-2.334656, -50.030334, 75.045502],
    [-2.309198, -51.275761, 76.913641],
    [-2.283740, -52.500103, 78.750154],
    [-2.258282, -53.702587, 80.553881],
    [-2.232824, -54.882442, 82.323664],
    [-2.207366, -56.038860, 84.058290],
    [-2.181908, -57.171028, 85.756542],
    [-2.156450, -58.278133, 87.417200],
    [-2.130992, -59.359314, 89.038971],
    [-2.105534, -60.413738, 90.620607],
    [-2.080076, -61.440529, 92.160793],
    [-2.054618, -62.438812, 93.658218],
    [-2.029160, -63.407692, 95.111538],
    [-2.003702, -64.346268, 96.519402],
    [-1.978244, -65.253670, 97.880505],
    [-1.952786, -66.128944, 99.193417],
    [-1.927328, -66.971176, 100.456764],
    [-1.901870, -67.779457, 101.669186],
    [-1.876412, -68.552864, 102.829296],
    [-1.850954, -69.290451, 103.935677],
    [-1.825496, -69.991325, 104.986988],
    [-1.800038, -70.654541, 105.981812],
    [-1.774580, -71.279190, 106.918785],
    [-1.749122, -71.864319, 107.796478],
    [-1.723664, -72.409088, 108.613632],
    [-1.698206, -72.912567, 109.368851],
    [-1.672748, -73.373871, 110.060806],
    [-1.647290, -73.792130, 110.688194],
    [-1.621832, -74.166512, 111.249767],
    [-1.596374, -74.496147, 111.744221],
    [-1.570916, -74.780251, 112.170376],
    [-1.545458, -75.017998, 112.526997],
    [-1.520000, -75.208656, 112.812984],
    [-1.494542, -75.351448, 113.027172],
    [-1.469084, -75.445686, 113.168530],
    [-1.443626, -75.490677, 113.236015],
    [-1.418168, -75.485771, 113.228657],
    [-1.392710, -75.430344, 113.145515],
    [-1.367252, -75.323830, 112.985744],
    [-1.341794, -75.165688, 112.748531],
    [-1.316336, -74.955406, 112.433109],
    [-1.290878, -74.692551, 112.038826],
    [-1.265420, -74.376694, 111.565041],
    [-1.239962, -74.007477, 111.011215],
    [-1.214504, -73.584579, 110.376869],
    [-1.189046, -73.107742, 109.661613],
    [-1.163588, -72.576752, 108.865128],
    [-1.138130, -71.991455, 107.987183],
    [-1.112672, -71.351707, 107.027561],
    [-1.087214, -70.657486, 105.986229],
    [-1.061756, -69.908813, 104.863220],
    [-1.036298, -69.105721, 103.658581],
    [-1.010840, -68.248337, 102.372505],
    [-0.985382, -67.336861, 101.005291],
    [-0.959924, -66.371513, 99.557270],
    [-0.934466, -65.352615, 98.028923],
    [-0.909008, -64.280533, 96.420799],
    [-0.883550, -63.155693, 94.733540],
    [-0.858092, -61.978588, 92.967882],
    [-0.832634, -60.749775, 91.124662],
    [-0.807176, -59.469845, 89.204767],
    [-0.781718, -58.139503, 87.209255],
    [-0.756260, -56.759487, 85.139231],
    [-0.730802, -55.330616, 82.995924],
    [-0.705344, -53.853729, 80.780594],
    [-0.679886, -52.329796, 78.494694],
    [-0.654428, -50.759762, 76.139643],
    [-0.628970, -49.144699, 73.717049],
    [-0.603512, -47.485737, 71.228605],
    [-0.578054, -45.784004, 68.676006],
    [-0.552596, -44.040764, 66.061146],
    [-0.527138, -42.257267, 63.385900],
    [-0.501680, -40.434883, 60.652325],
    [-0.476222, -38.574947, 57.862421],
    [-0.450764, -36.678982, 55.018473],
    [-0.425306, -34.748432, 52.122648],
    [-0.399848, -32.784836, 49.177254],
    [-0.374390, -30.789810, 46.184715],
    [-0.348932, -28.764952, 43.147428],
    [-0.323474, -26.711969, 40.067954],
    [-0.298016, -24.632576, 36.948864],
    [-0.272558, -22.528547, 33.792821],
    [-0.247100, -20.401667, 30.602500],
])
"""The lookup table for the knee joint parameters of the Boston Dynamics Spot robot.

This table describes the relationship between the joint angle (rad), the transmission
ratio (in/out), and the output torque (N*m). It is used to interpolate the output
torque based on the joint angle.
"""

##
# Configuration
##

spot = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(os.path.dirname(__file__), "../usd/spot/spot.usd"),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        joint_pos={
            "[fh]l_hx": 0.1,    # all left hip_x
            "[fh]r_hx": -0.1,    # all right hip_x
            "f[rl]_hy": 0.9,    # front hip_y
            "h[rl]_hy": 1.1,    # hind hip_y
            ".*_kn": -1.5,    # all knees
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "spot_hip":
            DelayedPDActuatorCfg(
                joint_names_expr=[".*_h[xy]"],
                effort_limit=45.0,
                stiffness=60.0,
                damping=1.5,
                min_delay=0,    # physics time steps (min: 2.0*0=0.0ms)
                max_delay=4,    # physics time steps (max: 2.0*4=8.0ms)
            ),
        "spot_knee":
            RemotizedPDActuatorCfg(
                joint_names_expr=[".*_kn"],
                joint_parameter_lookup=spot_joint_parameter_lookup,
                effort_limit=None,
                stiffness=60.0,
                damping=1.5,
                min_delay=0,    # physics time steps (min: 2.0*0=0.0ms)
                max_delay=4,    # physics time steps (max: 2.0*4=8.0ms)
            ),
    },
)
"""Configuration for the Boston Dynamics Spot robot."""

g1 = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(os.path.dirname(__file__), "../usd/g1_minimum/g1_minimum.usd"),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.74),
        joint_pos={
            ".*_hip_pitch_joint": -0.20,
            ".*_knee_joint": 0.42,
            ".*_ankle_pitch_joint": -0.23,
            ".*_elbow_pitch_joint": 0.87,
            "left_shoulder_roll_joint": 0.16,
            "left_shoulder_pitch_joint": 0.35,
            "right_shoulder_roll_joint": -0.16,
            "right_shoulder_pitch_joint": 0.35,
            "left_one_joint": 1.0,
            "right_one_joint": -1.0,
            "left_two_joint": 0.52,
            "right_two_joint": -0.52,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs":
            ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_hip_yaw_joint",
                    ".*_hip_roll_joint",
                    ".*_hip_pitch_joint",
                    ".*_knee_joint",
                    "torso_joint",
                ],
                effort_limit=300,
                velocity_limit=100.0,
                stiffness={
                    ".*_hip_yaw_joint": 150.0,
                    ".*_hip_roll_joint": 150.0,
                    ".*_hip_pitch_joint": 200.0,
                    ".*_knee_joint": 200.0,
                    "torso_joint": 200.0,
                },
                damping={
                    ".*_hip_yaw_joint": 5.0,
                    ".*_hip_roll_joint": 5.0,
                    ".*_hip_pitch_joint": 5.0,
                    ".*_knee_joint": 5.0,
                    "torso_joint": 5.0,
                },
                armature={
                    ".*_hip_.*": 0.01,
                    ".*_knee_joint": 0.01,
                    "torso_joint": 0.01,
                },
            ),
        "feet":
            ImplicitActuatorCfg(
                effort_limit=20,
                joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
                stiffness=20.0,
                damping=2.0,
                armature=0.01,
            ),
        "arms":
            ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_shoulder_pitch_joint",
                    ".*_shoulder_roll_joint",
                    ".*_shoulder_yaw_joint",
                    ".*_elbow_pitch_joint",
                    ".*_elbow_roll_joint",
                    ".*_five_joint",
                    ".*_three_joint",
                    ".*_six_joint",
                    ".*_four_joint",
                    ".*_zero_joint",
                    ".*_one_joint",
                    ".*_two_joint",
                ],
                effort_limit=300,
                velocity_limit=100.0,
                stiffness=40.0,
                damping=10.0,
                armature={
                    ".*_shoulder_.*": 0.01,
                    ".*_elbow_.*": 0.01,
                    ".*_five_joint": 0.001,
                    ".*_three_joint": 0.001,
                    ".*_six_joint": 0.001,
                    ".*_four_joint": 0.001,
                    ".*_zero_joint": 0.001,
                    ".*_one_joint": 0.001,
                    ".*_two_joint": 0.001,
                },
            ),
    },
)
"""Configuration for the Unitree G1 Humanoid robot."""

DIGIT_ACTUATED_JOINT_NAMES = [
    ".*_hip_roll",
    ".*_hip_yaw",
    ".*_hip_pitch",
    ".*_knee",
    ".*_toe_a",
    ".*_toe_b",
    ".*_arm_.*",
]

digit = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(os.path.dirname(__file__), "../usd/digit_no_cam/digit_no_cam.usd"),
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 1.05),),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "all":
            ImplicitActuatorCfg(
                joint_names_expr=DIGIT_ACTUATED_JOINT_NAMES,
                stiffness=None,
                damping=None,
            ),
    },
)
"""Configuration for the Digit robot."""
