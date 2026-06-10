On the terminal, 
- move to /home/UR2_Controller/

Usage of realsense camera
- roscore
- roslaunch ur2_env_realsense.launch 

Usage of codes
- python test_ur_env.py
  - Code for controlling bucket pose by a joy stick controller with virtual safety
- python test_ur_env_gahee.py --d 0.30 --v -0.3
  - Code for Gahee-san's experiment. d : height of bucket, v : velocity of bucket angle
  - Images are saved in /home/UR2_Controller/Saved_Image/