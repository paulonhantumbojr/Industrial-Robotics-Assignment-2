# Dishwasher Co-Unloader

This is the repository for the code to be implemented in the project for Assignment 2 of 41013 - Industrial Robotics, which comprises on the use of two robots in collaboration to manipulate everyday objects in a kitchen.
The ultimate goal is to unload a dishwasher and store these dishes on cupboards. All tasks are modelled using MATLAB software in conjunction with the [Peter Corke's Toolbox modified by UTS](https://drive.google.com/file/d/1LIiJwiqPY4zTxIY6XEu7v6YEde-dq9xP/view?usp=sharing).
Information about the robots used will be provided along with the files required to set them up.

# Industrial Robot Arms

The Industrial Robot manipulators used for this task are:
- [Universal Robot UR3e](https://www.universal-robots.com/products/); and
- [Franka Emika Panda Robot](https://www.franka.de/).

The UR3 being an ultra-lightweight, compact cobot will be used to collect the dishes from the dishwasher employing visual-servoing techniques and place them on the top of a kitchen benchtop. The Panda Robot on the other hand with its ultra-sensitivity to contact forces will be used to collect the dishes from the benchtop and place them on their respective cupboards.
Both are set up in MATLAB with the use of their official DH Parameters and corresponding CAD Models:
- [UR3 DH Parameters](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)
- [Panda Robot DH Parameters](https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters)
