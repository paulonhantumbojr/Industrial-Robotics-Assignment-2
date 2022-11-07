%% Real Robot Demo
% Initialise ROS Connection
rosshutdown;
rosinit('192.168.0.253'); % Assuming a UTS Pi, otherwise please change this

% Get the current joint states
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState'); % subscriber to get robot joint states
pause(2); % Pause to give time for a message to appear
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Send commands to joints
jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 10; % This is how many seconds the movement will take (the more time allowed the safer depending on the task)

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint'); % Message on the initial joint positions
startJointSend.Positions = currentJointState_123456; % Getting the robot's current joint states
startJointSend.TimeFromStart = rosduration(0);     
    
%% Set the joint goals to be achieved by the robot (manually)
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint'); % Message on the joint positions throughout the trajectory

% Simulate the trajectory of the joints to pick-up a plate
nextJointState_123456 = currentJointState_123456 + deg2rad([0,90,-45,0,45,0]) + deg2rad([45 -27.5 50 70 45 0]) + deg2rad([36.5 -20.4 -21.6 -10.8 0 0]) + deg2rad([0 -6 0 -8.2 0 0]) + deg2rad([0 -20.4 0 8.2 0 0]); 
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];
%% Publish the position of each of the joint states
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal); % Publish the positions to each joint state 