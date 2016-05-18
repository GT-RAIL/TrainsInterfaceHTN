

##To Do List

1. In Action.py class Pickup and Store we need to call a ROS topic to actually do the pickup
2. Class HTN is incomplete. 
3. Working on Grouping tasks together
4. Logging & undo

##How to Run
In ROS

1. rosrun pydisco heres_how_lunchpacking_htn.py 


In pure Python

1. Set up a command list from commands.json
2. run `python heres_how_lunchpacking_htn.py`

###Background Services to run with tablebot

Run the Tablebot

    roslaunch tablebot_bringup tablebot_bringup.launch
    
Run the Web Services and MJpegServer 

     roslaunch tablebot_heres_how_action_executor heres_how_web_services.launch 

Execute Actions

    rosrun tablebot_heres_how_action_executor tablebot_heres_how_action_executor
    rosrun tablebot_action_queue tablebot_action_queue

Finally, run the HTN Server

    rosrun pydisco heres_how_lunchpacking_htn.py
