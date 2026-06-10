import cv2
import numpy as np
import torch
import dill
import time
import os
from model import Actor
from ur_env_xy import UR_Env
import rospy
import yaml
from queue import Queue
config = "./parameter.yaml"
with open(config, "r", encoding="utf-8") as fin:
    configs = yaml.load(fin, Loader=yaml.FullLoader)
from SAC.Actor.ActorLSTM import ActorLSTM

from UserDefinedSettings import UserDefinedSettings
from datetime import datetime 

class Breakallloops(Exception):
    pass

class main:
    def __init__(
        self,
        env,
        UserDefine,
    ):
        obs_dim = configs["obs_dim"]
        self.action_dim = configs["action_dim"]
        as_lim =  np.array(configs["as_lim"])
        self.as_max = as_lim
        self.as_min = -as_lim
        self.env = env
        self.max_step = configs["num_frames"]
        self.act_step = 0
        self.goal = np.array([-0.3, -0.27])
        self.env.goal_pos = self.goal
        # device: cpu
        self.device = torch.device("cuda")
        print(self.device)
        self.coloimages = []
        self.frames = []

        # automatic entropy tuning

        # actor
        self.userDefinedSettings = UserDefine
        self.actor = ActorLSTM(obs_dim, self.action_dim, self.userDefinedSettings)
        self.total_step = 0
        self.ep_step = 0
        self.is_test = True

    def select_action(self, state: np.ndarray, pos) -> np.ndarray:
        def action_norm(action: np.ndarray) -> np.ndarray:
            """Change the range (-1, 1) to (low, high)."""

            # action = action.reshape(-1, self.action_dim)
            low = np.array([-0.2, -0.2])
            high = np.array([0.2, 0.2])
            scale_factor = (high - low) / 2
            reloc_factor = high - scale_factor
            action = action * scale_factor + reloc_factor
            action = np.clip(action, low, high)

            return action
        
        ##world cordinate offset
        # print("origin_ee",pos[:2])
        selected_action, _ = self.actor.get_action(state,step=self.act_step, deterministic=True)
        selected_action = action_norm(selected_action)
        
        print("step:", self.act_step, ",act:", selected_action)
        selected_action += pos[:2]
        # print("expected_ee(world cord)",selected_action)
        ##world cordinate to ur cordinate
        return selected_action

    def step(self, action: np.ndarray):
        """Take an action and return the response of the env."""
        # print("action", action)
        action = np.append(action, 0.115)
        # self.env.pause()
        ## F shape action limit
        # if -action[0] >= 0.21:
        #     action[0] = -0.21
        self.env.ee_move(action)
        self.act_step += 1
        next_pos, force, next_state = self.env.get_state()
        distance = np.linalg.norm(next_pos[2:] - self.goal)
        self.coloimages.append(self.env.cam.color_image)
        if distance <= 0.105 or self.act_step >= 20:
            done = 1
            self.frames.append(np.array(self.coloimages))
        # elif next_state[2] <= -0.40:
        #     done = 1
        #     self.frames.append(np.array(self.coloimages))
        #     self.env.pause()
        else:
            done = 0

        return next_state, done, distance, next_pos

    def reset(self, obj_init=False):
        if obj_init:
            self.initial_object()
        self.env.reset()
        pos, _, state = self.env.get_state()
        self.coloimages = []

        self.coloimages.append(self.env.cam.color_image)
        return state, pos

    def initial_object(self):
        # leave object
        state, _, _ = self.env.get_state()
        action = np.array(state[:2])
        action[0] += 0.08
        action[1] -= 0.05
        action = np.append(action, 0.115)
        self.env.ee_move(action)
        # raise arm
        state, _, _ = self.env.get_state()
        action = np.array(state[:2])
        action = np.append(action, 0.24)
        self.env.ee_move(action)
        # rotate arm
        action = np.array(state[2:])
        action[1] = action[1] + 0.13 - 0.5
        action = np.append(action, 0.24)
        self.env.ee_move(action, stop=3)
        # down and circle the object
        action = np.array(state[2:])
        action[1] = action[1] + 0.13 - 0.5
        action = np.append(action, 0.115)
        self.env.ee_move(action, stop=3)
        # pull back to initial x
        action = np.array(state[2:])
        action[1] = action[1] + 0.13 - 0.5
        action = np.append(action, 0.115)
        # randx = 0.08 + np.random.rand() * 0.1 ## for Tshaep_1R
        randx = 0.08 + np.random.rand() * 0.06 ## for double_T
        # randx = 0.1 + np.random.rand() * 0.08
        # print("randx",randx)
        action[0] = randx
        self.env.ee_move(action, stop=3)
        ##pull back to initial y
        state, _, _ = self.env.get_state()
        action = np.array(state[2:])
        action = np.append(action, 0.115)
        # randy = -0.08 + np.random.rand() * 0.03 + 0.11 - 0.5
        randy = -0.08 + np.random.rand() * 0.05 + 0.11 - 0.5
        action[1] = randy
        # print("randy",randy)
        self.env.ee_move(action, stop=3)
        ## go up
        state, _, _ = self.env.get_state()
        action = np.array(state[:2])
        action[1] -= 0.05
        action = np.append(action, 0.24)
        self.env.ee_move(action, stop=3)
        ## rotate back
        state, _, _ = self.env.get_state()
        action = np.array(state[:2])
        action[0] = 0.06
        action[1] = -0.25
        action = np.append(action, 0.24)
        self.env.ee_move(action, stop=0)
        ##go down
        state, _, _ = self.env.get_state()
        action = np.array(state[:2])
        action = np.append(action, 0.115)
        self.env.ee_move(action, stop=0)

    def test(self, load_pre=False):
        """Test the agent."""
        model_path = self.userDefinedSettings.TEST_DIR
        self.is_test = True
        self.load_model(model_path,load_only_policy=True)
        distances = []
        step_distances = []
        success_list= []
        end = False
        episode = 0
        if self.userDefinedSettings.init_obj:
            self.initial_object()
        try:
            while end is not True:
                episode += 1
                done = 0
                self.act_step = 0
                step_dis = []
                if episode != 1:
                    state, pos = self.reset(obj_init=True)
                else:
                    if load_pre:
                        distances = dill.load(
                            open("./replaydata/experiment_Dis_data.dill", "rb")
                        )
                        step_distances = dill.load(
                            open("./replaydata/experiment_StepDis_data.dill", "rb")
                        )
                        self.frames = dill.load(open("./replaydata/Image.dill", "rb"))
                        episode = len(distances) + 1
                    state, pos = self.reset()
                    self.env.pause()
                while not done:
                    action = self.select_action(np.float32(state),np.array(pos))
                    # self.env.pause()
                    next_state, done, distance, next_pos = self.step(action)
                    state = next_state
                    pos = next_pos
                    step_dis.append(distance)
                    if done:
                        print("done!")
                        distances.append(distance)
                        step_distances.append(step_dis)
                        if distance <= 0.12:
                            success_list.append(1)
                        else:
                            success_list.append(0)
                        success_rate = np.mean(np.array(success_list[-10:]))
                        if len(success_list)>10 and success_rate <0.2:
                            raise Breakallloops
                    # self.env.pause()
                print("episode:", episode, "final_distance", distance)
                dill.dump(self.frames, open("./replaydata/Image.dill", "wb"))
                dill.dump(distances, open("./replaydata/experiment_Dis_data.dill", "wb"))
                dill.dump(distances, open("./replaydata/experiment_StepDis_data.dill", "wb"))
        except Breakallloops:
            print("Tool Broken! Stop Experiment", datetime.now())
            
    
    def load_model(self, path=None, load_only_policy=False,train=False):
        if path is not None:
            self.model_dir = path

        self.actor.policyNetwork.load_state_dict(
            torch.load(
                os.path.join(self.model_dir, "Policy.pth"),
                map_location=torch.device(self.userDefinedSettings.DEVICE),
            )
        )
        
        self.actor.policyNetwork.eval()

if __name__ == "__main__":
    test_ur_env = UR_Env()
    userDefinedSettings = UserDefinedSettings()
    Agent = main(test_ur_env,userDefinedSettings)
    # Agent.reset()
    # act = np.array([0.02,-0.01])
    # Agent.step(act)
    # act = np.array([-0.06,0.25,0.115])
    # Agent.env.ee_move(action=act)
    try:
        Agent.test(load_pre=True)
    except KeyboardInterrupt:
        exit()
