## Modified Attention Policy Shaping

This is the implementation of the paper [Policy Shaping with Supervisory Attention Driven Exploration](https://ieeexplore.ieee.org/abstract/document/8594312). 

This code uses the [Armpy library](https://github.com/AABL-Lab/armpy) from [AABL Lab](https://aabl.cs.tufts.edu/) which uses ROS 1 Noetic and the MOVEIT package.

Additionally, this code has been modified using [Q-Learning Demo](https://github.com/anisha1045/Q-Learning-Demo) which the author collaborated on. 

### Information about the files 

`MDP.py` : The Python file that runs the Supervisory Attention Driven Exploration algorithm for the task by creating the environment, actions, and states. 

`armpy` : The module for connecting to the Gen3 Lite arm with 6 degrees of freedom.

`action.py` : A Python file for the class for Action that creates Actions object for each task.

`state.py` : A Python file for the class State that creates State objects for each task.

`task.py` : A Python file for the class Task that creates Task objects for each task.

`robot.py` : A Python file that initializes the armpy module to create an instance of the arm.

`test.py` : A Python file that helps in testing ROS commands on the arm through the command line. 
