% Script for Assessment 2 Demo Video 
 clc;
 clf;
 clear;
 close all;
 set(0, 'DefaultFigureWindowStyle', 'docked'); % Dock figures in the workspace
%% Environment 
 kitchen = Kitchen(0); % Plot the environment
 elements = kitchen.Spawnkitchen(); % Plot the environment objects
 
 % Generate plates
 plateCount = 2; % Count of plates spawned in the environment
 standHeight = 1.23; % Height of the main kitchen stand from the ground
 cupboardHeight = 2.1; % Height of the plate cupboard
 platestackHeight = 0.05; % Height of plate
 gripperLength = 0.085; % Length of the RobotIQ gripper, gripperLength = 0.165;
 plateInitialPosition = [0.75,-1.6; ... % Plate 1 position
                         0.75,-1.5]; % Plate 2 position
 plateStandPosition = [0.175,-2.0, standHeight; ... % Plate 1 position
                       0.175,-2.1, standHeight]; % Plate 2 position
 plateCupboardPosition = [-0.5,-2.4, cupboardHeight; ... % Plate 1 position
                       -0.5,-2.4, cupboardHeight]; % Plate 2 position
                           
 for i = 1:plateCount % Generating 2 plates
    plates{i} = Plate(i,plateInitialPosition(i,1),plateInitialPosition(i,2),0); % Plate Constructor
 end
 %% Robots
 pandaBaseTransform = [-0.15,-1.35,0.725]; % Panda robot Base position
 ur3BaseTransform = [0.8,-1.875,1.03]; % UR3 Base position

 % Plot the UR3
 myUR3 = ModifiedUR3(ur3BaseTransform);
 myUR3.PlotAndColourUR3Robot()

 % Plot the Panda
 myPanda = Panda(pandaBaseTransform);
 myPanda.PlotAndColourPandaRobot()
 %% Set trajectories and animate robot
 steps = 20; % Smaller steps to animate movements faster

 % For UR3
for i = 1:plateCount
    
    q0 = deg2rad([45,-15,30,-100,-90,0]); % Joint angle for pick-up of plates
   
    platePosition{1} = transl(plates{i}.plate.base(1,4),plates{i}.plate.base(2,4),plates{i}.plate.base(3,4) + ...
    gripperLength + platestackHeight) * trotx(pi); % Transform of Initial position of plates

    q1 = myUR3.model.ikcon(platePosition{1},q0); % Inverse kinematics for joint angles 
    ur3QuinticMatrix = jtraj(myUR3.model.getpos,q1,steps); % Set quintic trajectory for linear UR3
    
    % Sequence to move robot towards the bricks' initial positions
    for j = 1:steps
     ur3Q = ur3QuinticMatrix(j,:); % UR3 motion steps
     myUR3.model.animate(ur3Q)
     drawnow();
    end

    q0 = deg2rad([135,-35,30,-100,-90,0]); % Joint angle for drop off
   
    platePosition{2} = transl(plateStandPosition(i,1),plateStandPosition(i,2),plateStandPosition(i,3) + gripperLength + platestackHeight)... 
    * trotx(pi); % Transform of Final position of plates

    q1 = myUR3.model.ikcon(platePosition{2},q0); % Inverse kinematics for joint angles 
    ur3QuinticMatrix = jtraj(myUR3.model.getpos,q1,steps); % Set quintic trajectory
    
    % Sequence to pick-up, collect, and drop the plates with the robot
    for j = 1:steps
      ur3Q = ur3QuinticMatrix(j,:); % UR3 motion steps
      
      platesNewPosition = myUR3.model.fkine(ur3Q); % Set the plate's position to the end-effector of the arm
      
   % Acquire current position of the plates
      plateX = plates{i}.plate.base(1,4);
      plateY = plates{i}.plate.base(2,4);
      plateZ = plates{i}.plate.base(3,4);

   % Calculate change in the plate's position
     dx = diff([plateX,platesNewPosition(1,4)]);
     dy = diff([plateY,platesNewPosition(2,4)]);
     dz = diff([plateZ,platesNewPosition(3,4)]);

   % Apply the change to plates' base 
     plates{i}.plate.base(:,4) = [(plateX + dx); (plateY + dy); ((plateZ + dz) - (gripperLength + platestackHeight)); 1];
          
   % Animate robot, fingers, and bricks
     myUR3.model.animate(ur3Q)
%      plates{i}.plate.base = plates{i}.plate.base * troty(pi/2);
     plates{i}.plate.animate(0) 
     drawnow();
     pause(0.01);
    end
end

    qArmUp = [-pi/2,-pi/2,0,0,-pi,0]; % Joint angles for arm up position
    
    % Move arm up after last drop
    ur3QuinticMatrix = jtraj(myUR3.model.getpos,qArmUp,steps); % Set quintic trajectory for ur3
    
    for j = 1:steps
      ur3Q = ur3QuinticMatrix(j,:);
      myUR3.model.animate(ur3Q)
      drawnow();
      pause(0.01)
    end
    disp('UR3 operation finished')
     
%     % For Panda
for i = 1:plateCount
    
    q0 = deg2rad([-55,65.4,0,-25.3,0,259,0]); % Joint angle for pick-up of plates
   
    platePosition{1} = transl(plates{i}.plate.base(1,4),plates{i}.plate.base(2,4),plates{i}.plate.base(3,4) + ...
    gripperLength + platestackHeight) * trotx(pi); % Transform of plates position at the stand

    q1 = myPanda.model.ikcon(platePosition{1},q0); % Inverse kinematics for joint angles 
    pandaQuinticMatrix = jtraj(myPanda.model.getpos,q1,steps); % Set quintic trajectory for the Panda
    
    % Sequence to move robot towards the plates initial positions
    for j = 1:steps
     pandaQ = pandaQuinticMatrix(j,:); % UR3 motion steps
     myPanda.model.animate(pandaQ)
     drawnow();
     pause(0.01)
    end

    q0 = deg2rad([-124,30.6,0,-18.7,0,252,0]); % Joint angle for drop off
   
    platePosition{2} = transl(plateCupboardPosition(i,1),plateCupboardPosition(i,2),plateCupboardPosition(i,3) + gripperLength + platestackHeight)... 
    * trotx(pi); % Transform of Final position of plates

    q1 = myPanda.model.ikcon(platePosition{2},q0); % Inverse kinematics for joint angles 
    pandaQuinticMatrix = jtraj(myPanda.model.getpos,q1,steps); % Set quintic trajectory
    
    % Sequence to pick-up, collect, and drop the plates with the robot
    for j = 1:steps
      pandaQ = pandaQuinticMatrix(j,:); % Panda motion steps
      
      platesNewPosition = myPanda.model.fkine(pandaQ); % Set the plate's position to the end-effector of the arm
      
   % Acquire current position of the plates
      plateX = plates{i}.plate.base(1,4);
      plateY = plates{i}.plate.base(2,4);
      plateZ = plates{i}.plate.base(3,4);

   % Calculate change in the plate's position
     dx = diff([plateX,platesNewPosition(1,4)]);
     dy = diff([plateY,platesNewPosition(2,4)]);
     dz = diff([plateZ,platesNewPosition(3,4)]);

   % Apply the change to plates' base 
     plates{i}.plate.base(:,4) = [(plateX + dx); (plateY + dy); ((plateZ + dz) - (gripperLength + platestackHeight)); 1];
     
   % Animate robot, fingers, and bricks
     myPanda.model.animate(pandaQ)
     plates{i}.plate.animate(0) 
     drawnow();
     pause(0.01);
    end
end

    qArmUp = zeros(1,myPanda.model.n); % Joint angles for arm up position
    
    % Move arm up last drop
    pandaQuinticMatrix = jtraj(myPanda.model.getpos,qArmUp,steps); % Set quintic trajectory for ur3
    
    for j = 1:steps
      pandaQ = pandaQuinticMatrix(j,:);
      myPanda.model.animate(pandaQ)
      drawnow();
      pause(0.01)
    end
    disp('Panda operation finished')