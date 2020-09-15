# debugging_codes and traj_files folders
Put them inside the ~\ros_ws folder. To run the codes follow the instructions below

```
$ cd ~/ros_ws
$ ./baxter.sh
$ cd debugging_codes
$ python execute_joint_angles.py
```

# joint_trajectory_client_myplanner.py
This file is used to smoothly execute jointspace path. The joint path should be provided as a *.txt file inside the traj_files folder as described above.

In order to execute the trajectory, open two terminals and use the following two commands in them.

```
$ rosrun baxter_interface joint_trajectory_action_server.py
$ rosrun baxter_examples joint_trajectory_client_myplanner.py
```

**Note** make sure that the name of the file residing in the traj_files folder matches the filename in the $joint\_trajectory\_client\_myplanner.py$

