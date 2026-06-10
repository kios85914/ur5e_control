import os
import warnings
import datetime

import numpy as np
import random
import argparse
import socket
import glob

import torch

warnings.simplefilter("ignore", FutureWarning)  # noqa

parser = argparse.ArgumentParser()
parser.add_argument("--test", help="learning flag", action="store_true")
parser.add_argument("--dir", help="directory of tested policy", type=str)
parser.add_argument(
    "--load_model", help="load trained policy flag", action="store_true"
)
parser.add_argument(
    "--life", help="life reward", action="store_true"
)
parser.add_argument(
    "--init_obj", help="life reward", action="store_true"
)
parser.add_argument("--render", help="render", action="store_true")
parser.add_argument("--save_image", help="save image", action="store_true")
parser.add_argument("--gpu", help="gpu num", type=str, default="0")
parser.add_argument("--seed", help="seed", type=int, default=1)


args = parser.parse_args()


seed_number = args.seed
os.environ["PYTHONHASHSEED"] = str(seed_number)
np.random.seed(seed_number)
random.seed(seed_number)
torch.manual_seed(seed_number)
if torch.cuda.is_available():
    torch.cuda.manual_seed_all(seed_number)
    torch.backends.cudnn.deterministic = True


class UserDefinedSettings(object):

    def __init__(self):
        self.LSTM_FLAG = True
        self.DOMAIN_RANDOMIZATION_FLAG = False

        self.DEVICE = torch.device(
            "cuda:" + str(args.gpu) if torch.cuda.is_available() else "cpu"
        )
        self.ENVIRONMENT_NAME = "Pendulum"
        current_time = datetime.datetime.now()
        file_name = "M{:0=2}D{:0=2}H{:0=2}M{:0=2}S{:0=2}{}".format(
            current_time.month,
            current_time.day,
            current_time.hour,
            current_time.minute,
            current_time.second,
            socket.gethostname(),
        )
        self.LOG_DIRECTORY = os.path.join("log", file_name)

        self.seed = args.seed
        self.save_image = args.save_image

        self.num_steps = 1e6
        self.batch_size = 16  # 16
        self.policy_update_start_episode_num = 200  # 30
        self.learning_episode_num = 6000
        self.update_episode_cycle = 1  # 1<x<150
        self.lr = 1e-4
        self.HIDDEN_NUM = 128  # 128
        self.entropy_tuning_scale = -0.5
        self.life_flag = args.life
        self.init_obj = args.init_obj

        self.learning_rate = self.lr
        self.memory_size = 1e6  # 1e6
        self.gamma = 0.99
        self.soft_update_rate = 0.005
        self.entropy_tuning = True
        self.multi_step_reward_num = 1
        self.updates_per_step = 1  # 1
        self.target_update_interval = 1  # episode num
        self.evaluate_interval = 10  # episode num
        self.initializer = "xavier"
        self.run_num_per_evaluate = 1  # 5
        self.average_num_for_model_save = 20  # 方策性能のN個の平均でモデルを保存
        self.LEARNING_REWARD_SCALE = 1.0
        self.MODEL_SAVE_INDEX = "test"  # test, train

        self.ACTION_DISCRETE_FLAG = False

        if args.load_model:
            self.TRAIN_DIR = args.dir + "model"
            self.policy_update_start_episode_num = 200
        else:
            self.TRAIN_DIR = None
        
        self.TEST_FLAG = args.test
        if self.TEST_FLAG:
            self.TEST_DIR = args.dir + "model"

        self.RENDER_FLAG = args.render
