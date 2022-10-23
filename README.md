# Dishwasher Co-Unloader

This is the repository for the code to be implemented in the project for Assignment 2 of 41013 - Industrial Robotics, which comprises on the use of two robots in collaboration to manipulate everyday objects in a kitchen.
The ultimate goal is to unload a dishwasher and store its dishes on cupboards. This novel solution eases the bore unloading of dishes and gives you more time to enjoy your leasure. <br />
All tasks are modelled using MATLAB software in conjunction with the [Peter Corke's Toolbox modified by UTS](https://drive.google.com/file/d/1LIiJwiqPY4zTxIY6XEu7v6YEde-dq9xP/view?usp=sharing). Information about the robots used will be provided along with the files required to set them up.

## Group Members
The members responsible for developing this project are:
- Mina Mina (@S3nt1nel22)
- Julian Taffa (@V370c1ty)
- Paulo Nhantumbo Junior (@pnhantumbojr)

Under the supervision of:
- Gavin Paul (@gapaul)
- Tony Le (@tonydle)

# Industrial Robot Arms

The Industrial Robot manipulators used for this task are:
- [Universal Robot UR3e](https://www.universal-robots.com/products/); and
- [Franka Emika Panda Robot](https://www.franka.de/).

The UR3 being an ultra-lightweight, compact cobot will be used to collect the dishes from the dishwasher employing visual-servoing techniques and place them on the top of a kitchen benchtop. The Panda Robot on the other hand with its ultra-sensitivity to contact forces will be used to collect the dishes from the benchtop and place them on their respective cupboards.
Both are set up in MATLAB with the use of their official DH Parameters and corresponding CAD Models:
- [UR3 DH Parameters](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)
- [UR3 Link Meshes](https://drive.google.com/file/d/1NZBdMVdIXxeq7fd8oqGcLOqG9xiw7q1l/view?usp=sharing)
- [Panda Robot DH Parameters](https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters)
- [Panda Link Meshes](https://drive.google.com/file/d/1mIbogNqddZfE6Wi-y3fjufj_Hij3x_Fi/view?usp=sharing)

# Setup
To run the program, first ensure that the toolbox and code folders are part of the MATLAB Directory.
1. Launch the Control Panel from the GUI folder, `ControlApp.mlapp`.
2. The Visual Servoing is launched separately with plots `Assessment2.fig`.
