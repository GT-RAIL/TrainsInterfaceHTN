##How to Run
 `rosrun pydisco heres_how_lunchpacking_htn.py false`

The second parameter `false` is for whether or not to turn suggestions on. It is by default true

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

##Problems and fixes

1. If the user is stuck in the queue, look at the user ID for that user and run

    rosservice call /rail_user_queue_manager/update_queue [user-id] false 0 30

2. If the system has an error, that is fine but to reactivate the Interface, you have to run

    rostopic pub /web_interface/execute_action_feedback std_msgs/Bool true
