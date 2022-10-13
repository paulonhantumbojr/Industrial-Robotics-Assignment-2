%% Setup
% Clear and close variables whenever script is run
 clc;
 clf;
 clear;
 close all;

% Set figures to be docked
 set(0, 'DefaultFigureWindowStyle', 'docked');
 %% Base Transform of Robots
 pandaBaseTransform = [-1.0,-1.0,0.725]; % Panda robot Base transform
 ur3BaseTransform = [0,-1.0,0.725]; % UR3 Base transform
 %% Environment set as cube meshes
% Setup variables for workshop components
 floor = imread ('Floor.jpg'); % Floor placeholder
 wall = imread ('Mosaic.jpg'); % Walls placeholder
 warningSign = imread ('workinprogress.jpg'); % Waring sign placeholder
 lightSign = imread('Curtain.png'); % Curtains placeholder
 
% Workspace
 workspace = [-3 3 -3 3 0 4];  
 
% Floor
 surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[-0.01,-0.01;-0.01,-0.01],'CData',floor,'FaceColor','texturemap'); 

 hold on
 
% Walls 
 surf([-2.5,2.5;-2.5,2.5],[-2.5,-2.5;-2.5,-2.5],[0,0;4,4],'CData',wall,'FaceColor','texturemap'); % Left wall
 surf([-2.5,-2.5;-2.5,-2.5],[-2.5,2.5;-2.5,2.5],[4,4;0,0],'CData',wall,'FaceColor','texturemap'); % Right wall

% Warning signs
 surf([2.3,2.3;2.3,2.3],[1.25,2.25;1.25,2.25],[2.5,2.5;1.5,1.5],'CData',warningSign,'FaceColor','texturemap');  % on right wall
 surf([2.0,0;2.0,0],[-2.465,-2.465;-2.465,-2.465],[3.9,3.9;2.9,2.9],'CData',lightSign,'FaceColor','texturemap'); % on left wall

% Kitchen elements
 cupboard_h1 = PlaceObject('cupboard.PLY',[-0.145,1.275,-2.025]);
 verts = [get(cupboard_h1 ,'Vertices'), ones(size(get(cupboard_h1 ,'Vertices'),1),1)] * trotz(-pi/2) * troty(-pi/2);
 set(cupboard_h1 ,'Vertices',verts(:,1:3))

 fence_h1 = PlaceObject('Fence.ply',[-1.0,2.25,0.01]); % Right fence
 verts = [get(fence_h1,'Vertices'), ones(size(get(fence_h1,'Vertices'),1),1)] * trotz(pi/2);
 set(fence_h1,'Vertices',verts(:,1:3))
 
 fence_h2 = PlaceObject('Fence.ply',[0.75,2.25,0.01]); % Left fence
 verts = [get(fence_h2,'Vertices'), ones(size(get(fence_h2,'Vertices'),1),1)] * trotz(pi/2);
 set(fence_h2,'Vertices',verts(:,1:3))
 
 door_h = PlaceObject('Door.PLY',[-2.35,2.275,1.315]);
 verts = [get(door_h,'Vertices'), ones(size(get(door_h,'Vertices'),1),1)] * trotz(pi/2);
 set(door_h,'Vertices',verts(:,1:3))
 
 worker_h = PlaceObject('Worker.PLY',[2,0.05,-0.1]);
 verts = [get(worker_h,'Vertices'), ones(size(get(worker_h,'Vertices'),1),1)] * trotx(pi/2) * troty (pi) * trotz(pi/2);
 set(worker_h,'Vertices',verts(:,1:3))
 
 table_h1 = PlaceObject('WorkingTable.PLY',[1,-0.05,-0.05]);
 verts = [get(table_h1,'Vertices'), ones(size(get(table_h1,'Vertices'),1),1)] * trotz(pi/2);
 set(table_h1,'Vertices',verts(:,1:3))
 
 table_h2 = PlaceObject('WorkingTable.PLY',[1,-1,-0.05]);
 verts = [get(table_h2,'Vertices'), ones(size(get(table_h2,'Vertices'),1),1)] * trotz(pi/2);
 set(table_h2,'Vertices',verts(:,1:3))
 
 estop_h = PlaceObject('Estop.PLY',[-2.05,1,-0.6]);
 verts = [get(estop_h,'Vertices'), ones(size(get(estop_h,'Vertices'),1),1)] * trotx(-pi/2);
 set(estop_h,'Vertices',verts(:,1:3))
 
 warninglight_h = PlaceObject('warninglight.PLY',[-2,-2.49,3.65]);
 verts = [get(warninglight_h,'Vertices'), ones(size(get(warninglight_h,'Vertices'),1),1)];
 set(warninglight_h,'Vertices',verts(:,1:3))
 
 lightcurtain_h1 = PlaceObject('lightcurtain.PLY',[-1.6,0.845,-0.25]);
 verts = [get(lightcurtain_h1,'Vertices'), ones(size(get(lightcurtain_h1,'Vertices'),1),1)] * trotx(-pi/2);
 set(lightcurtain_h1,'Vertices',verts(:,1:3))
 
 lightcurtain_h2 = PlaceObject('lightcurtain.PLY',[1.8,0.845,-0.25]);
 verts = [get(lightcurtain_h2,'Vertices'), ones(size(get(lightcurtain_h2,'Vertices'),1),1)] * trotx(-pi/2);
 set(lightcurtain_h2,'Vertices',verts(:,1:3))
 
 lightcurtain_h3 = PlaceObject('lightcurtain.PLY',[1.8,0.845,2.3]);
 verts = [get(lightcurtain_h3,'Vertices'), ones(size(get(lightcurtain_h3,'Vertices'),1),1)] * trotx(-pi/2);
 set(lightcurtain_h3,'Vertices',verts(:,1:3))
 
 dishwasher_h = PlaceObject('dishwasher.PLY',[-1.2,0.125,-2.8]);
 verts = [get(dishwasher_h,'Vertices'), ones(size(get(dishwasher_h,'Vertices'),1),1)] * trotx(pi/2) * troty(pi);
 set(dishwasher_h,'Vertices',verts(:,1:3))

 plate_h = PlaceObject('plate.PLY',[0.75,-1.6,0.8]);
 verts = [get(plate_h,'Vertices'), ones(size(get(plate_h,'Vertices'),1),1)];
 set(plate_h,'Vertices',verts(:,1:3))

 cupboard_h2 = PlaceObject('platecupboard.PLY',[-1.0,-2.149,2]);
 verts = [get(cupboard_h2 ,'Vertices'), ones(size(get(cupboard_h2 ,'Vertices'),1),1)];
 set(cupboard_h2 ,'Vertices',verts(:,1:3))
 
 axis equal
%% Spawn Robots in environment
% UR3 with gripper
 myUR3 = ModifiedUR3(ur3BaseTransform);
 myUR3.PlotAndColourUR3Robot()
 
 ur3_q0 = zeros (1, myUR3.model.n); % Initial UR3 joint angles

% Generate first finger at the end-effector of the linear UR3
 ur3Finger1 = RobotIQ_Finger1(false);
 ur3Finger1.model.base = ur3Finger1.model.base * myUR3.model.fkine(ur3_q0) * troty(-pi/2) * transl(0.065,-0.0115,0); % Forward kinematics for end-effector base
 ur3Finger1.GetAndColourFinger1(); 

% Generate second finger at the end-effector of the linear UR3
 ur3Finger2 = RobotIQ_Finger2(false);
 ur3Finger2.model.base = ur3Finger2.model.base * myUR3.model.fkine(ur3_q0) * trotz(pi) * troty(-pi/2) * transl(0.065,-0.0115,0); % Forward kinematics for end-effector base
 ur3Finger2.GetAndColourFinger2(); 

 % Panda with gripper
 myPanda = Panda(pandaBaseTransform);
 myPanda.PlotAndColourPandaRobot()
 
 panda_q0 = zeros (1, myPanda.model.n); % Initial UR3 joint angles
 
% Generate first finger at the end-effector of the linear UR3
 Pandafinger1 = Panda_Finger1(false);
 Pandafinger1.model.base = Pandafinger1.model.base * myPanda.model.fkine(panda_q0) * transl(0,-0.155,0.03) * trotz(-pi/2) * trotx(pi/2); % Forward kinematics for end-effector base
 Pandafinger1.GetAndColourFinger1(); 
 
 % Generate second finger at the end-effector of the linear UR3
 Pandafinger2 = Panda_Finger2(false);
 Pandafinger2.model.base = Pandafinger2.model.base * myPanda.model.fkine(panda_q0) * transl(0,-0.155,-0.03) * trotz(-pi/2) * trotx(pi/2) * trotx(pi); % Forward kinematics for end-effector base
 Pandafinger2.GetAndColourFinger2(); 