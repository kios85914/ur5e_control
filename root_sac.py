import isaacgym
from UserDefinedSettings import UserDefinedSettings
from Environment.EnvironmentFactory import EnvironmentFactory
from SAC.SACAgent import SACAgent
from torch.utils.tensorboard import SummaryWriter  # noqa
from env2 import myenv
from LearningCommonParts.ItemDebugHandler import ItemDebugHandler


def root():

    userDefinedSettings = UserDefinedSettings()
    environmentFactory = EnvironmentFactory(userDefinedSettings)
    itemDebugHandler = ItemDebugHandler(path=userDefinedSettings.LOG_DIRECTORY)

    # env = environmentFactory.generate()
    env = myenv(view=False)
    agent = SACAgent(env, userDefinedSettings)
    agent.itemDebugHandler = itemDebugHandler

    if userDefinedSettings.TEST_FLAG:
        agent.test(
            model_path=userDefinedSettings.TEST_DIR,
            policy=None,
            test_num=1,
            render_flag=True,
            reward_show_flag=True,
        )
    else:
        agent.train(model_path=userDefinedSettings.TRAIN_DIR)


if __name__ == "__main__":
    root()
