import copy
import fileinput
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
from shutil import copyfile
import dill
import rainflow
import csv
from make_urdf import create_urdf
from isaacgym import gymapi
from isaacgym import gymtorch
from isaacgym import gymutil
from isaacgym.torch_utils import *
import math
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline
import yaml
from collections import deque
import time
import matplotlib.pyplot as plt
import socket
import pickle
from make_object import create_object_urdf
from scipy.signal import medfilt


config = "./parameter.yaml"
with open(config, "r", encoding="utf-8") as fin:
    configs = yaml.load(fin, Loader=yaml.FullLoader)


class myenv:
    def __init__(
        self, view=configs["view"], process_id=0, random_material=False, test_flag=False
    ):
        ###data for RL

        self.test_flag = test_flag
        self.num_env = configs["num_env"]
        self.process_id = process_id
        self.step = 0  ##simulation step
        self.act_step = 0  # RL step
        self.sample_freq = configs["sample_freq"]
        self.max_action_step = configs["num_frames"]
        self.MAX_EPISODE_LENGTH = self.max_action_step
        self.obs_dim = configs["obs_dim"]
        self.STATE_DIM = self.obs_dim
        self.action_dim = configs["action_dim"]
        self.ACTION_DIM = self.action_dim
        self.DOMAIN_PARAMETER_DIM = configs["parameter_dim"]
        self.as_lim = np.array(configs["as_lim"])
        self.action_space_high = configs["action_space_high"]
        self.action_space_low = configs["action_space_low"]
        self.deform_his_length = configs["deform_his_length"]
        self.state_his_length = configs["state_his_length"]
        self.action_his_length = configs["action_his_length"]
        self.store_buf = np.zeros(self.num_env)
        self.cord = []
        self.stress_history = []
        self.reward_history = []
        self.distance_history = []
        self.stress_step_history = []
        self.total_life = 0.0
        self.acceleration = 0.0
        self.random_material = random_material
        print("random",self.random_material,"test flag",self.test_flag)
        self.object_mass_L, self.object_mass_H = 2, 4
        self.dfrs_L,self.dfrs_H = 0.15, 0.25
        self.static_friction_L, self.static_friction_H = 0.4, 0.7
        if self.random_material and not self.test_flag:
            print("random friction")
            self.object_mass = np.random.uniform(low=self.object_mass_L, high=self.object_mass_H)
            create_object_urdf(self.object_mass)
            self.static_friction = np.random.uniform(low=self.static_friction_L,high=self.static_friction_H)  # 靜摩擦係數
            self.dynamic_friction = np.random.uniform(low=self.dfrs_L,high=self.dfrs_H)#dynamic_friction_random_seed*self.static_friction # 動摩擦係數
            configs["object_mass"] = self.object_mass
            configs["dynamic_friction"] = self.dynamic_friction
            with open("./parameter.yaml", "w", encoding="utf-8") as file:
                    yaml.dump(configs, file)
        elif self.test_flag:
            self.static_friction = np.random.uniform(low=self.static_friction_L,high=self.static_friction_H)
            self.object_mass = configs["test_object_mass"]
            self.dynamic_friction = configs["test_dynamic_friction"]
            create_object_urdf(self.object_mass,name="test")
        else:
            self.static_friction = np.random.uniform(low=self.static_friction_L,high=self.static_friction_H)
            self.object_mass = configs["object_mass"]
            self.dynamic_friction = configs["dynamic_friction"]
        print("Mass:",self.object_mass,",Friction",self.dynamic_friction)
        self.velocity_threshold = 0.03  # 判斷為靜止的速度閾值
        self.normal_force = 9.81 * self.object_mass

        ###data for randomization
        self.young_H, self.young_L = 100, 0.1
        self.density_H, self.density_L = 7, 1
        self.b_H, self.b_L = 0.2, 0.01
        self.b_list = configs["b"]

        ###setup env
        self.frict_coeff = 0.0001
        self.view = view 
        self.gym = gymapi.acquire_gym()
        self.intial_sim()
        self.env_data_setup()
        self.node_num = self.get_node_inform()
        self.test_episode = configs["test_episode"]
        self.node_contact_history = np.zeros(self.node_num)
        
        ####flag
        self.extract_stress_flag = True

    def env_data_setup(
        self,
    ):
        self.goal_pose = np.array([-0.3, -0.27])
        self.origin_error = [0.06, -0.25]
        self.robot_error = [0.0, 0.5]
        # self.youngs = [70.000, 10.000, 1.996]
        # self.density = [2700, 1500, 1060]
        self.startend = []
        for i in range(self.num_env):
            object_pos = self.gym.get_actor_rigid_body_states(
                self.env_handles[i], self.object_handles[i], gymapi.STATE_POS
            )[2][0][0]
            object_pos = np.array([object_pos[0], object_pos[1], object_pos[2]])
            distance = np.linalg.norm(object_pos[:2] - self.goal_pose[:2]) * 100
            self.startend.append(distance)
        self.distance_history.append(self.startend)

    def intial_sim(self, random_material=False):
        self.sim = self.create_sim(
            frict_coeff=self.frict_coeff,
        )

        # add ground plane
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0, 0, 1)
        self.gym.add_ground(self.sim, plane_params)

        # create table
        self.table_asset = self.load_assets(
            root="./assets", file="ur5e/robots/table.urdf"
        )
        table_dof_props = self.gym.get_asset_dof_properties(self.table_asset)
        # create object
        if not self.test_flag:
            self.object_asset = self.load_assets(
                root="./assets", file="ur5e/robots/object.urdf"
            )
        else:
            self.object_asset = self.load_assets(
                root="./assets", file="ur5e/robots/object_test.urdf"
            )
        object_dof_props = self.gym.get_asset_dof_properties(self.object_asset)
        object_dof_props["driveMode"][:].fill(gymapi.DOF_MODE_EFFORT)
        object_dof_props["stiffness"][:].fill(0.0)
        object_dof_props["damping"][:].fill(0.0)
        object_num_dofs = self.gym.get_asset_dof_count(self.object_asset)

        # create robot
        ur5e_asset_list = []
        ur5e_dof_props_list = []
        mat = ["hard", "med", "soft"]
        for i in range(self.num_env):
            if random_material:
                self.young_mod = np.random.uniform(self.young_L, self.young_H)
                self.density = np.random.uniform(self.density_L, self.density_H)
                configs["youngs"] = self.young_mod
                configs["density"] = self.density
                print("young", self.young_mod, "dens", self.density)
                create_urdf(i, self.young_mod * 1e9, self.density * 1000)
                with open("./parameter.yaml", "w", encoding="utf-8") as file:
                    yaml.dump(configs, file)
            else:
                self.young_mod = configs["youngs"]
                self.density = configs["density"]
            
            ur5e_asset = self.load_assets(
                root="./assets", file=f"ur5e/robots/test_ur5e_hard.urdf", max_lin_vel=True
            )
            ur5e_dof_props = self.gym.get_asset_dof_properties(ur5e_asset)
            ur5e_dof_props["driveMode"][:].fill(gymapi.DOF_MODE_POS)
            ur5e_dof_props["stiffness"][:].fill(200)
            ur5e_dof_props["damping"][:].fill(15)
            ur5e_asset_list.append(ur5e_asset)
            ur5e_dof_props_list.append(ur5e_dof_props)

        # create scence
        scene_props = self.set_scene_props(num_envs=self.num_env)
        (
            self.env_handles,
            self.ur5e_handles,
            self.table_handles,
            self.object_handles,
        ) = self.create_scene(
            props=scene_props,
            assets_ur5e=ur5e_asset_list,
            ur5e_props=ur5e_dof_props_list,
            table_asset=self.table_asset,
            table_props=table_dof_props,
            object_asset=self.object_asset,
            object_props=object_dof_props,
        )

        # set cam
        if self.view:
            self.viewer = self.create_viewer(self.gym, self.sim)

        self.initial_state = np.copy(
            self.gym.get_sim_rigid_body_states(self.sim, gymapi.STATE_ALL)
        )

        self.undeform_mesh = self.get_undeform_mesh_cord()

    def create_sim(self, frict_coeff):
        sim_type = gymapi.SIM_FLEX
        sim_params = gymapi.SimParams()
        # sim_params.dt = 1 / 1200  # Control frequency
        sim_params.dt = 1 / 200
        sim_params.substeps = 3  # Physics simulation frequency (multiplier)
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.use_gpu_pipeline = False

        sim_params.stress_visualization = True  # von Mises stress
        sim_params.stress_visualization_min = 1.0e3
        sim_params.stress_visualization_max = 1.0e7

        # set Flex-specific parameters
        sim_params.flex.solver_type = 5  # PCR (GPU, global)
        sim_params.flex.num_outer_iterations = 8
        sim_params.flex.num_inner_iterations = 50
        sim_params.flex.max_rigid_contacts = 10000

        sim_params.flex.relaxation = 0.75
        sim_params.flex.warm_start = 0.8
        sim_params.flex.deterministic_mode = True
        sim_params.flex.geometric_stiffness = 1.0
        sim_params.flex.particle_friction = 0.001
        sim_params.flex.static_friction = 0.001
        sim_params.flex.shape_collision_distance = 0.0025  # Distance to be maintained between soft bodies and other bodies or ground plane
        sim_params.flex.shape_collision_margin = 0.0025  # Distance from rigid bodies at which to begin generating contact constraints
        # Distance to be maintained between soft bodies and other bodies or ground plane
        # sim_params.flex.shape_collision_distance = 0.0001
        # Distance from rigid bodies at which to begin generating contact constraints
        # sim_params.flex.shape_collision_margin = 0.000025

        # Friction about all 3 axes (including torsional)
        sim_params.flex.friction_mode = 1
        sim_params.flex.dynamic_friction = frict_coeff

        gpu_physics = 0
        gpu_render = 0
        sim = self.gym.create_sim(gpu_physics, gpu_render, sim_type, sim_params)

        return sim

    def load_assets(self, root, file, fix=True, gravity=False,max_lin_vel=False):
        """Load assets from specified URDF files."""
        asset_root = root
        asset_file = file
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = fix
        asset_options.flip_visual_attachments = False
        asset_options.armature = 0.0001
        asset_options.thickness = 0.001
        if max_lin_vel!=False:
            asset_options.max_linear_velocity = 0.3
        asset_options.linear_damping = 0.0
        asset_options.angular_damping = 0.0
        # asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
        asset_options.min_particle_mass = 1e-20
        asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)
        return asset

    def set_scene_props(self, num_envs, env_dim=1.5):
        """Set the scene and environment properties."""
        envs_per_row = int(np.ceil(np.sqrt(num_envs)))
        env_lower = gymapi.Vec3(-env_dim, -env_dim, 0)
        env_upper = gymapi.Vec3(env_dim, env_dim, env_dim)
        scene_props = {
            "num_envs": num_envs,
            "per_row": envs_per_row,
            "lower": env_lower,
            "upper": env_upper,
        }

        return scene_props

    def create_scene(
        self,
        props,
        assets_ur5e,
        ur5e_props,
        table_asset,
        table_props,
        object_asset,
        object_props,
    ):
        """Create a scene (i.e., ground plane, environments, BioTac actors, and indenter actors)."""

        env_handles = []
        actor_handles = []
        table_handles = []
        object_handles = []
        for i in range(props["num_envs"]):
            env_handle = self.gym.create_env(
                self.sim, props["lower"], props["upper"], props["per_row"]
            )
            env_handles.append(env_handle)

            collision_group = i
            collision_filter = 0
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0, 0.5, 0.525)
            actor_handle = self.gym.create_actor(
                env_handle,
                assets_ur5e[i],
                pose,
                "ur5e",
                collision_group,
                collision_filter,
            )
            actor_handles.append(actor_handle)
            # actor_soft_materials = self.gym.get_actor_soft_materials(env_handle, actor_handle)
            # asset_soft_body_count = self.gym.get_asset_soft_body_count(assets_ur5e)
            # for j in range(asset_soft_body_count):
            #     actor_soft_materials[j].youngs = np.random.uniform(self.young_L, self.young_H)
            #     actor_soft_materials[j].density = np.random.uniform(self.density_L, self.density_H)
            #     self.gym.set_actor_soft_materials(env_handle, actor_handle, actor_soft_materials)

            collision_group = i
            collision_filter = 0
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0, 0, 0.5)
            table_handle = self.gym.create_actor(
                env_handle,
                table_asset,
                pose,
                "table",
                collision_group,
                collision_filter,
            )
            table_handles.append(table_handle)

            collision_group = i
            collision_filter = 1
            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0, 0, 0.5)
            object_handle = self.gym.create_actor(
                env_handle,
                object_asset,
                pose,
                "object",
                collision_group,
                collision_filter,
            )
            object_handles.append(object_handle)
            object_pos_x = np.random.uniform(-0.03, 0.03, 1)
            object_pos_y = np.random.uniform(-0.05, 0.0, 1)
            object_pos_state = np.zeros(2, gymapi.DofState.dtype)
            object_pos_state["pos"][0] = object_pos_x
            object_pos_state["pos"][1] = object_pos_y
            self.gym.set_actor_dof_states(
                env_handle, object_handle, object_pos_state, gymapi.STATE_ALL
            )

            # robot initial pos
            default_dof_pos = [0.0, 0.0, 0.0]
            default_dof_state = np.zeros(3, gymapi.DofState.dtype)
            default_dof_state["pos"] = default_dof_pos
            self.gym.set_actor_dof_states(
                env_handle, actor_handle, default_dof_state, gymapi.STATE_ALL
            )
            self.gym.set_actor_dof_position_targets(
                env_handle, actor_handle, default_dof_pos
            )

            self.gym.set_actor_dof_properties(env_handle, actor_handle, ur5e_props[i])
            self.gym.set_actor_dof_properties(env_handle, table_handle, table_props)
            self.gym.set_actor_dof_properties(env_handle, object_handle, object_props)

        return env_handles, actor_handles, table_handles, object_handles

    def create_viewer(self, gym, sim):
        """Create viewer and axes objects."""
        camera_props = gymapi.CameraProperties()
        camera_props.horizontal_fov = 10.0
        camera_props.width = 1920
        camera_props.height = 1080
        viewer = gym.create_viewer(sim, camera_props)
        camera_pos = gymapi.Vec3(0.0, 1.0, 12.0)
        camera_target = gymapi.Vec3(0.0, -0.2, 0.0)
        gym.viewer_camera_look_at(viewer, None, camera_pos, camera_target)

        axes_geom = gymutil.AxesGeometry(0.1)

        return viewer

    def get_robot_state(self, env, actor_handle, world_cordinate=False):
        ee_pos = self.gym.get_actor_dof_states(env, actor_handle, gymapi.STATE_ALL)[
            "pos"
        ]
        ee_pos[0] = (
            ee_pos[0] + self.origin_error[0]
            if not world_cordinate
            else ee_pos[0] + self.origin_error[0] + self.robot_error[0]
        )
        ee_pos[1] = (
            ee_pos[1] + self.origin_error[1]
            if not world_cordinate
            else ee_pos[1] + self.origin_error[1] + self.robot_error[1]
        )
        return ee_pos[:2]

    def extract_elem_stresses(
        self,
    ):
        """Extract the element-wise von Mises stresses on the BioTac from each environment."""

        (_, stresses) = self.gym.get_sim_tetrahedra(self.sim)
        num_envs = self.gym.get_env_count(self.sim)
        num_tets = len(stresses)
        num_tets_per_env = int(num_tets / num_envs)
        stresses_von_mises = np.zeros((num_envs, num_tets_per_env))

        for env_index, env in enumerate(self.env_handles):
            # Get tet range (start, count) for BioTac
            tet_range = self.gym.get_actor_tetrahedra_range(env, 0, 0)

            # Compute and store von Mises stress for each tet
            # TODO: Vectorize for speed
            for global_tet_index in range(
                tet_range.start, tet_range.start + tet_range.count
            ):
                stress = stresses[global_tet_index]
                stress = np.matrix(
                    [
                        (stress.x.x, stress.y.x, stress.z.x),
                        (stress.x.y, stress.y.y, stress.z.y),
                        (stress.x.z, stress.y.z, stress.z.z),
                    ]
                )
                stress_von_mises = np.sqrt(
                    0.5
                    * (
                        (stress[0, 0] - stress[1, 1]) ** 2
                        + (stress[1, 1] - stress[2, 2]) ** 2
                        + (stress[2, 2] - stress[0, 0]) ** 2
                        + 6
                        * (stress[1, 2] ** 2 + stress[2, 0] ** 2 + stress[0, 1] ** 2)
                    )
                )
                local_tet_index = global_tet_index % num_tets_per_env
                stresses_von_mises[env_index][local_tet_index] = stress_von_mises

        return stresses_von_mises

    def move_group_to_joint_target_eepos_multi(
        self,
        env,
        actor_handle,
        target=None,
        tolerance=0.01,
        ang_tolerance=0.1,
        max_steps=800,
        quiet=False,
    ):

        num_env = self.num_env
        steps = 1
        reached_target = False
        stuck_flag = False
        success_flag = np.zeros(num_env)
        success_confirn_flag = np.ones(num_env)
        deltas = np.zeros((self.num_env, 2))
        deltas_his = []
        target_joint_value = target
        ## adjust to fake pos
        for i in range(num_env):
            target_joint_value[i][0] = target_joint_value[i][0] - self.origin_error[0]
            target_joint_value[i][1] = target_joint_value[i][1] - self.origin_error[1]
        # print("adjust target",target_joint_value)
        while not reached_target:
            for i in range(num_env):
                current_joint_values = self.gym.get_actor_dof_states(
                    env[i], actor_handle[i], gymapi.STATE_ALL
                )["pos"]
                targets = target_joint_value[i]
                targets = targets.astype("f")
                self.gym.set_actor_dof_position_targets(
                    env[i], actor_handle[i], targets
                )
                deltas[i] = abs(target_joint_value[i] - current_joint_values)
                if max(deltas[i][:2]) < tolerance:
                    success_flag[i] = 1
                # step
            deltas_his.append(deltas.copy())
            if len(deltas_his) > 100:
                for i in range(self.num_env):
                    pre_delta = deltas_his[-100][i]
                    cur_delta = deltas_his[-1][i]
                    if (
                        abs(cur_delta[0] - pre_delta[0]) < tolerance / 2
                        and cur_delta[0] >= tolerance
                    ) or (
                        abs(cur_delta[1] - pre_delta[1]) < tolerance / 2
                        and cur_delta[1] > tolerance
                    ):
                        success_flag[i] = 1
                        stuck_flag = True
            if np.array_equal(success_flag, success_confirn_flag):
                for _ in range(3):
                    self.env_step()
                print("success",self.act_step if not stuck_flag else f"step {steps}, stuck: {deltas}")
                break
            elif steps > max_steps:
                print(
                    f"Max number of steps reached after {steps} steps. Deltas: {deltas}"
                )
                break

            self.env_step()
            steps += 1

    def env_step(
        self,
    ):
        # color = gymapi.Vec3(1, 0, 0)
        # for i in range(self.num_env):
        #     self.gym.draw_env_rigid_contacts(self.viewer,self.env_handles[i],color,0.01,True)
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        if self.view:
            self.gym.step_graphics(self.sim)
            self.gym.draw_viewer(self.viewer, self.sim, False)
        self.gym.sync_frame_time(self.sim)
        if self.step % self.sample_freq == 0 and self.extract_stress_flag:
            stress = self.extract_elem_stresses()
            self.stress_history.append(stress)
            self.stress_step_history.append(stress)
            dill.dump(
                self.stress_history,
                open(f"./temp/stress_history_{self.process_id}.dill", "wb"),
            )
            if self.test_flag:
                dill.dump(
                    self.stress_history,
                    open(f"./temp/test/{self.test_episode}/stress_history_{self.process_id}.dill", "wb"),
                )
                dill.dump(
                    self.stress_step_history,
                    open(f"./temp/test/{self.test_episode}/stress_{self.act_step}.dill", "wb"),
                )
        linear_velocity = self.gym.get_actor_dof_states(self.env_handles[0], self.object_handles[0], gymapi.STATE_ALL)["vel"]
        object_pos = self.gym.get_actor_rigid_body_states(self.env_handles[0], self.object_handles[0], gymapi.STATE_POS)[2][0][0]
        self.step += 1
        self.add_friction()
        self.extract_net_forces()
        

    def action(self, action):
        self.stress_step_history = []
        action = self.action_norm(action)
        self.act_step += 1
        action = np.reshape(action, (self.num_env, self.action_dim))
        local_action = copy.deepcopy(action)
        for i in range(self.num_env):
            ee_pos = self.get_robot_state(self.env_handles[i], self.ur5e_handles[i])
            act = [local_action[i][0] + ee_pos[0], local_action[i][1] + ee_pos[1]]
            act = np.clip(act, self.action_space_low, self.action_space_high)
            if math.sqrt(act[0] ** 2 + act[1] ** 2) >= abs(self.action_space_low[1]):
                act[1] = -math.sqrt(self.action_space_low[1] ** 2 - act[0] ** 2)
            local_action[i] = act

        self.move_group_to_joint_target_eepos_multi(
            self.env_handles, self.ur5e_handles, target=local_action
        )
        while True:
            self.env_step()
            linear_velocity = self.gym.get_actor_dof_states(self.env_handles[0], self.object_handles[0], gymapi.STATE_ALL)["vel"]
            if abs(linear_velocity[0])<self.velocity_threshold and abs(linear_velocity[1])<self.velocity_threshold:
                break
        state_list = self.get_state()
        life_list, damage_list = self.get_life()
        self.total_life += life_list
        reward_list, distance, done_list, success_list, life_reward = self.get_reward(self.total_life)
        material_list = self.get_parameter()
        if self.test_flag:
            self.gym.write_viewer_image_to_file(self.viewer,f"./temp/test/{self.test_episode}/step{self.act_step}.png")


        return (
            state_list,
            reward_list,
            done_list,
            material_list,
            success_list,
            distance,
            life_reward,
        )

    def get_state(
        self,
    ):
        state_list = []
        for i in range(self.num_env):
            ee_pos = self.get_robot_state(
                self.env_handles[i], self.ur5e_handles[i], world_cordinate=True
            )
            object_pos = self.gym.get_actor_rigid_body_states(
                self.env_handles[i], self.object_handles[i], gymapi.STATE_POS
            )[2][0][0]
            rel_pos_x = object_pos[0] - ee_pos[0]
            rel_pos_y = object_pos[1] - ee_pos[1]
            goal_rel_x = self.goal_pose[0] - object_pos[0]
            goal_rel_y = self.goal_pose[1] - object_pos[1]
            state = np.array(
                [
                    rel_pos_x,
                    rel_pos_y,
                    goal_rel_x,
                    goal_rel_y,
                ]
            )
            # state_list.append(state)
        return state

    def get_reward(self, life_list):
        reward_list = []
        done_list = []
        success_list = []
        distance_list = []
        ####Task goal: distance
        for i in range(self.num_env):
            object_pos = self.gym.get_actor_rigid_body_states(
                self.env_handles[i], self.object_handles[i], gymapi.STATE_POS
            )[2][0][0]
            object_pos = np.array([object_pos[0], object_pos[1], object_pos[2]])
            goal_pos = self.goal_pose
            ee_pos = self.get_robot_state(
                self.env_handles[i], self.ur5e_handles[i], world_cordinate=True
            )
            distance = np.linalg.norm(object_pos[:2] - goal_pos[:2])
            obj_ee_dis = np.linalg.norm(object_pos[:2] - ee_pos[:2])
            x_dis = abs(object_pos[0] - goal_pos[0])
            y_dis = abs(object_pos[1] - goal_pos[1])
            reward = -(1 * (x_dis + y_dis) + 0.1 * obj_ee_dis)
            life = 0
            if self.act_step == self.max_action_step:
                # y = np.array(self.reward_history[i])
                # x = np.arange(1, len(y) + 1)
                # A = np.vstack([x, np.ones(len(x))]).T
                # m, c = np.linalg.lstsq(A, y, rcond=None)[0]
                # reward += np.clip(m, a_min=0, a_max=1)
                done_bool = 1
                if not self.test_flag:
                    life = life_list if distance<0.1 else 0
                    success = 0 if distance > 0.1 else True
                else:
                    life = life_list if distance<0.15 else 0
                    success = 0 if distance > 0.15 else True
                if self.test_flag:
                    dill.dump(
                        self.node_contact_history,
                        open(f"./temp/test/{self.test_episode}/contact_history.dill", "wb"),
                    )
            elif self.store_buf[i] == 1:
                done_bool = 1
                success = 2
                reward = 1
            else:
                done_bool = 0
                success = 0
            distance_list.append(distance)
            done_list.append(done_bool)
            success_list.append(success)
            reward_list.append(reward)
        self.distance_history.append(distance_list)
        self.reward_history.append(reward_list)

        return (
            np.array(reward_list).squeeze(),
            np.array(distance_list).squeeze(),
            np.array(done_list).squeeze(),
            np.array(success_list).squeeze(),
            life
        )

    def check_contact(self, i):
        table_idx1 = self.gym.find_asset_rigid_body_index(self.table_asset, "obs1")
        table_idx2 = self.gym.find_asset_rigid_body_index(self.table_asset, "obs2")
        object_idx = self.gym.find_asset_rigid_body_index(self.object_asset, "object")
        o_idx = self.gym.get_actor_rigid_body_index(
            self.env_handles[i], self.object_handles[i], object_idx, gymapi.DOMAIN_ENV
        )
        t_idx1 = self.gym.get_actor_rigid_body_index(
            self.env_handles[i], self.table_handles[i], table_idx1, gymapi.DOMAIN_ENV
        )
        t_idx2 = self.gym.get_actor_rigid_body_index(
            self.env_handles[i], self.table_handles[i], table_idx2, gymapi.DOMAIN_ENV
        )
        contact = self.gym.get_soft_contacts(self.sim)
        # print("contact",contact)
        check_contact = 0
        for cont in contact:
            # print("contact",o_idx)
            rigid_body_index = cont[4]
            if rigid_body_index == o_idx:
                for n in cont[2]:
                    self.node_contact_history[n] += 1                
                check_contact = 1
                break
        return check_contact

    def extract_net_forces(
        self,
    ):
        """Extract the net force vector on the BioTac for each environment."""

        contacts = self.gym.get_soft_contacts(self.sim)
        num_envs = self.gym.get_env_count(self.sim)
        net_force_vecs = np.zeros((num_envs, 3))
        num_rigid_envs = self.gym.get_env_rigid_body_count(self.env_handles[0])

        # print("contacts num",len(contacts))
        for contact in contacts:
            rigid_body_index = contact[4]
            contact_normal = np.array([*contact[6]])
            contact_force_mag = contact[7]
            env_index = rigid_body_index // num_rigid_envs
            force_vec = contact_force_mag * contact_normal
            net_force_vecs[env_index] += force_vec
        net_force_vecs = -net_force_vecs
        # print("force",net_force_vecs)
        return net_force_vecs

    def get_life(self):
        def SN_curve_equation(s, infinite_cycle=1.0e06, env_id=0):
            if s == 0:
                return infinite_cycle
            alpha_lis = [1488.2, 3797.1, 70213.8]
            beta_lis = [-0.4219, -0.561, -0.926]
            alpha = alpha_lis[env_id]  # 6574.1
            beta = beta_lis[env_id]  # -0.671
            cycle = (s / alpha) ** (1 / beta)
            return cycle
        
        def SN_curve_equation2(s, infinite_cycle=1.0e06, env_id=0):
            if s == 0:
                return infinite_cycle
            B = 1.69e22
            m = 5.63395
            cycle = B / (2 * s**m)
            return cycle
        
        def filter_plot2(data, plot=False):
            # print("data origin",data.shape)
            # 使用中值濾波器
            filtered_data = medfilt(data, kernel_size=39)  # kernel_size 可以調整窗口大小
            # print("data filt",filtered_data.shape)
            # 畫圖
            if plot:
                plt.figure(figsize=(10, 6))
                plt.plot(data, label='原始數據')
                plt.plot(filtered_data, label='濾波後數據', linewidth=2)
                plt.xlabel('時間 (s)')
                plt.ylabel('振幅')
                plt.title('使用中值濾波器去除異常值')
                plt.legend()
                plt.show()
            return filtered_data

        infinite_cycle = 1.0e06
        # stress_history_all = np.array(
        #     dill.load(open(f"./temp/stress_history_{self.process_id}.dill", "rb"))
        # )
        stress_history_all = np.array(self.stress_step_history)
        print("stress_history_all.shape",stress_history_all.shape)
        life_list = []
        damage_list = []
        for i in range(self.num_env):
            env_id = 1  # int(i % 3)
            stress_history_env = stress_history_all[:, i, :]
            self.node_maxnum = stress_history_env.shape[1]
            rainflow_list = []
            cumulative_damage_array = np.zeros(self.node_maxnum)

            for i in range(0, self.node_maxnum):
                mps_history = stress_history_env[:, i]
                filt_stress = filter_plot2(mps_history)
                cycles_data = rainflow.count_cycles(filt_stress)
                cumulative_damage = 0
                for j in cycles_data:
                    sn_cycles = SN_curve_equation(j[0] / 1e06, env_id=env_id)
                    cumulative_damage += j[1] / sn_cycles
                cumulative_damage_array[i] = cumulative_damage
                rainflow_list.append(cycles_data)

                if 1 >= cumulative_damage > 0:
                    remain_cycle = np.log10(1 / cumulative_damage)
                elif cumulative_damage > 1:
                    remain_cycle = 0
                else:
                    remain_cycle = np.log10(infinite_cycle)

                remain_cycle = (
                    remain_cycle
                    if (0 <= remain_cycle <= np.log10(infinite_cycle))
                    else 0 if remain_cycle < 0 else np.log10(infinite_cycle)
                )
                life_list.append(remain_cycle)
            damage_list.append(0)
            # life = np.sum(np.array(life_list))
            life = np.min(np.array(life_list))
            print("step life:",life)
            # self.life_buf.append(cumulative_damage_array)
            # loc = np.argmax(cumulative_damage_array)
            # history_stress = vms[:,loc]
            # print(loc)
            # print(np.max(cumulative_damage_array))
            # print('rainflow',rainflow_list[loc])
            # plt.plot(history_stress)
            # plt.title('Stress histroy')
            # plt.xlabel('Times')
            # plt.ylabel('Stress')
            # plt.savefig('Stress_plot.png')

        return np.array(life), np.array(damage_list)

    def get_log_life(self):
        stress_history_all = np.array(
            dill.load(open(f"./temp/stress_history_{self.process_id}.dill", "rb"))
        )
        life_list = []
        damage_list = []
        for env in range(self.num_env):
            b = self.b_list[env]
            stress_history_env = stress_history_all[:, env, :]
            self.node_maxnum = stress_history_env.shape[1]
            rainflow_list = []
            cumulative_damage_array = np.zeros(self.node_maxnum)
            for i in range(0, self.node_maxnum):
                mps_history = stress_history_env[:, i]
                cycles_data = rainflow.count_cycles(mps_history)
                cumulative_damage = 0
                for j in cycles_data:
                    cumulative_damage += j[1] * (j[0] / 1e06) ** b
                cumulative_damage_array[i] = np.log(cumulative_damage)
                rainflow_list.append(cycles_data)

            max_damage = np.max(cumulative_damage_array)
            log_life = max_damage
            life_list.append(log_life)
            damage_list.append(max_damage)

        return np.array(life_list), np.array(damage_list)

    def reset(
        self,
    ):
        self.extract_stress_flag = False
        self.act_step = 0
        self.step = 0
        self.stress_history = []
        self.distance_history = []
        self.reward_history = []
        self.startend = []
        for i in range(self.num_env):
            object_pos = self.gym.get_actor_rigid_body_states(
                self.env_handles[i], self.object_handles[i], gymapi.STATE_POS
            )[2][0][0]
            object_pos = np.array([object_pos[0], object_pos[1], object_pos[2]])
            distance = np.linalg.norm(object_pos[:2] - self.goal_pose[:2]) * 100
            self.startend.append(distance)
        self.distance_history.append(self.startend)
        self.store_buf = np.zeros(self.num_env)
        self.gym.set_sim_rigid_body_states(
            self.sim, self.initial_state, gymapi.STATE_ALL
        )  # reset rigid body
        for i in range(self.num_env):
            self.gym.reset_actor_particles_to_rest(
                self.env_handles[i], self.ur5e_handles[i]
            )  # reset soft object
        state_list = self.get_state()
        self.extract_stress_flag = True
        with open("./parameter.yaml", "r", encoding="utf-8") as fin:
            configs = yaml.load(fin, Loader=yaml.FullLoader)
        self.view = configs["view"]

        return state_list

    def get_nodal_cord(
        self,
    ):
        particle_state_tensor = gymtorch.wrap_tensor(
            self.gym.acquire_particle_state_tensor(self.sim)
        )
        self.gym.refresh_particle_state_tensor(self.sim)
        num_envs = self.gym.get_env_count(self.sim)
        num_particles = len(particle_state_tensor)
        num_particles_per_env = int(num_particles / num_envs)
        nodal_coords = np.zeros((num_envs, num_particles_per_env, 3))
        for global_particle_index, particle_state in enumerate(particle_state_tensor):
            pos = particle_state[:3]
            env_index = global_particle_index // num_particles_per_env
            local_particle_index = global_particle_index % num_particles_per_env
            nodal_coords[env_index][local_particle_index] = pos.cpu().numpy()
        return nodal_coords
    
    def get_node_inform(
        self,
    ):
        particle_state_tensor = gymtorch.wrap_tensor(
            self.gym.acquire_particle_state_tensor(self.sim)
        )
        num_envs = self.gym.get_env_count(self.sim)
        num_particles = len(particle_state_tensor)
        num_particles_per_env = int(num_particles / num_envs)
        return num_particles_per_env

    def get_undeform_mesh_cord(
        self,
    ):
        mesh = self.get_nodal_cord()[0]
        origin_point = mesh[0]
        mesh -= origin_point
        return mesh

    def get_deform_state(self):
        all_env_mesh = self.get_nodal_cord()
        deform_list = []
        for env_id in range(self.num_env):
            mesh = all_env_mesh[env_id]
            origin_point = mesh[0]
            deform = []
            repos_mesh = mesh - origin_point
            for i in [0, 0, 0]:
                keypoint = repos_mesh[i]
                org_keypoint = self.undeform_mesh[i]
                d = keypoint[:2] - org_keypoint[:2]
                deform += list(d)
            deform_list.append(deform)

        return np.array(deform_list)

    def action_norm(self, action: np.ndarray) -> np.ndarray:
        """Change the range (-1, 1) to (low, high)."""

        # action = action.reshape(-1, self.action_dim)
        low = np.array([-0.2, -0.2])
        high = np.array([0.2, 0.2])
        scale_factor = (high - low) / 2
        reloc_factor = high - scale_factor
        action = action * scale_factor + reloc_factor
        action = np.clip(action, low, high)

        return action

    def reset_sim(self):
        self.gym.destroy_sim(self.sim)
        self.gym.destroy_viewer(self.viewer)
        # self.intial_sim()
        # self.env_data_setup()

    def get_parameter(
        self,
    ):

        mass_norm = (self.object_mass - self.object_mass_L) / (self.object_mass_H - self.object_mass_L)
        stfc_norm = (self.static_friction - self.static_friction_L) / (self.static_friction_H - self.static_friction_L)
        dyfc_norm = (self.dynamic_friction - self.dfrs_L) / (self.dfrs_H - self.dfrs_L)
        # dyfc_norm = (self.static_friction - self.static_friction_L*self.dfrs_L) / (self.static_friction_H*self.dfrs_H - self.static_friction_L*self.dfrs_L)
        mass_norm = round(mass_norm, 4)
        stfc_norm = round(stfc_norm, 4)
        dyfc_norm = round(dyfc_norm, 4)
        # print(np.array([mass_norm, stfc_norm,dyfc_norm]))
        return np.array([mass_norm,dyfc_norm])
    
    def add_friction(self):
        applyforce = self.extract_net_forces()
        # 設定靜摩擦和動摩擦的摩擦係數
        static_friction = self.static_friction  # 靜摩擦係數d
        dynamic_friction = self.dynamic_friction # 動摩擦係數
        velocity_threshold = self.velocity_threshold  # 判斷為靜止的速度閾值
        normal_force = self.normal_force
        linear_velocity = self.gym.get_actor_dof_states(self.env_handles[0], self.object_handles[0], gymapi.STATE_ALL)["vel"]
        ur5_velocity = self.gym.get_actor_dof_states(self.env_handles[0], self.ur5e_handles[0], gymapi.STATE_ALL)["vel"]
        # 根據速度動態調整摩擦力
        if abs(linear_velocity[1]) < velocity_threshold:
            if self.check_contact(i=0):
                friction_force_x = 0#-static_friction * normal_force * (linear_velocity[0] / abs(linear_velocity[0]))
            else:
                friction_force_x = 0
        else:
            friction_force_x = -dynamic_friction * normal_force * (linear_velocity[1] / abs(linear_velocity[1]))

        if abs(linear_velocity[0]) < velocity_threshold:
            if self.check_contact(i=0):
                friction_force_y = 0#-static_friction * normal_force * (linear_velocity[0] / abs(linear_velocity[0]))
            else:
                friction_force_y = 0
        else:
            friction_force_y = -dynamic_friction * normal_force * (linear_velocity[0] / abs(linear_velocity[0]))

        # 將摩擦力施加到物體上
        friction_force = np.array([friction_force_y, friction_force_x]).astype(np.float32)
        self.gym.apply_actor_dof_efforts(self.env_handles[0], self.object_handles[0],friction_force)


# if __name__ == "__main__":
#     env = myenv(test_flag=False, view=True)
#     HOST = "0.0.0.0
#     PORT = 7000
#     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     s.bind((HOST, PORT))
#     s.listen(5)
#     # print("server start at: %s:%s" % (HOST, PORT))
#     # print("wait for connection...")
#     while True:
#         conn, addr = s.accept()
#         # print("connected by " + str(addr))

#         indata = conn.recv(1024)
#         array = pickle.loads(indata)
#         if array[0] == 1:
#             state = env.reset()
#             outdata = pickle.dumps(state)
#         elif array[0] == 2:
#             break  # end env reopen new one
#         else:
#             # action
#             (next_state, reward, done, domain_parameter, task_achievement, distance) = (
#                 env.action(array[1])
#             )
#             outdata = dill.dumps(
#                 [next_state, reward, done, domain_parameter, task_achievement, distance]
#             )
#         conn.sendall(outdata)
#         conn.close()

if __name__ == "__main__":
    env = myenv(test_flag=configs["test_flag"], view=configs["view"],random_material=configs["material_randomize"])
    HOST = "0.0.0.0"
    PORT = 5000
    while True:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        outdata = pickle.dumps(0)
        s.sendall(outdata)
        indata = s.recv(1024)
        array = pickle.loads(indata)
        if array[0] == 1:
            state = env.reset()
            outdata = pickle.dumps(state)
        elif array[0] == 2:
            break  # end env reopen new one
        else:
            # action
            (next_state, reward, done, domain_parameter, task_achievement, distance, life) = (
                env.action(array[1])
            )
            outdata = dill.dumps(
                [next_state, reward, done, domain_parameter, task_achievement, distance, life]
            )
        s.sendall(outdata)
        s.close()


#     env = myenv(test_flag=False, view=True)
#     d_list = []
#     while True:
#         s = env.reset()
#         print(s)
#         s, d, _, _, _, _ = env.action(np.tile([0.2, 0.0], (1, 1)))
#         # time.sleep(2)
#         s, d, _, _, _, _ = env.action(np.tile([0.0, -0.2], (1, 1)))
#         # time.sleep(2)
#         s, d, _, _, _, _ = env.action(np.tile([0.2, 0.0], (1, 1)))
#         s, d, _, _, _, _ = env.action(np.tile([0.0, -0.15], (1, 1)))
#         s, d, _, _, _, _ = env.action(np.tile([-0.2, -0.15], (1, 1)))
#         s, d, _, _, _, _ = env.action(np.tile([-0.2, 0.0], (1, 1)))
#         env.reset_sim()
# end = time.time()
# print("cost time", end - start)

# time_points = np.array(d_list)
# colors = ['r', 'g', 'b']

# # 遍历时间点数据并绘制关键点
# for t in range(time_points.shape[0]):
#     plt.figure(figsize=(6, 4))
#     for i in range(time_points.shape[1]):
#         x, y = time_points[t, i]
#         plt.scatter(x, y, color=colors[i], label=f'Keypoint {i+1}')
#         plt.text(x, y, f'({x}, {y})', fontsize=9)

#     # 添加图例
#     plt.legend()

#     # 添加标题和标签
#     plt.title(f'Keypoints Positions at Time Point {t+1}')
#     plt.xlabel('X Position')
#     plt.ylabel('Y Position')

#     # 显示网格
#     plt.grid(True)

#     # 显示图形
#     plt.savefig(f"./image/m/hard_{t}.png")
# while True:
#     env.env_step()
