%% Setup
% Clear and close variables whenever script is run
 clc;
 clf;
 clear;
 close all;

% Set figures to be docked
 set(0, 'DefaultFigureWindowStyle', 'docked');
 %% Constant parameters 
 plateCount = 1; % Number of plates to be spawned
 dishWasherCount = 1; % Number of dishwashers to be spawned
 cupboardCount = 1; % Number of dishwashers to be spawned
 %% Object Transforms   
 dishWasherInitialPosition = [1.5, -3.3]; % Dishwasher
 plateInitialPosition = [1.25, -1.9]; % Plate 1
 cupboardInitialPosition = [-0.5, -3.0]; % Cupboard
 %% Base Transform of Robots
 pandaBaseTransform = [-1.25,-1.5,0.725]; % Panda robot Base transform
 ur3BaseTransform = [-0.25,-1.5,0.725]; % UR3 Base transform
 %% Environment
% Setup variables for workshop components
 floor = imread ('Floor.jpg'); % Floor placeholder
 wall = imread ('Mosaic.jpg'); % Walls placeholder
 headProtection = imread ('headprotection.jpg'); % Head Protection sign placeholder
 footProtection = imread ('footprotection.jpg'); % Foot Protection sign placeholder
 warningSign = imread ('workinprogress.jpg'); % Waring sign placeholder
 
% Workspace
 workspace = [-3 3 -3 3 0 4];  
 
% Floor
 surf([-3,-3;3,3],[-3,3;-3,3],[-0.01,-0.01;-0.01,-0.01],'CData',floor,'FaceColor','texturemap'); 

 hold on
% Walls 
 surf([-3,3;-3,3],[-3,-3;-3,-3],[0,0;4,4],'CData',wall,'FaceColor','texturemap'); % Left wall
 surf([-3,-3;-3,-3],[-3,3;-3,3],[4,4;0,0],'CData',wall,'FaceColor','texturemap'); % Right wall

% Warning signs
 surf([1.95,0.95;1.95,0.95],[-2.965,-2.965;-2.965,-2.965],[3.9,3.9;2.9,2.9],'CData',footProtection,'FaceColor','texturemap'); % on left wall
 surf([3,2;3,2],[-2.965,-2.965;-2.965,-2.965],[3.9,3.9;2.9,2.9],'CData',headProtection,'FaceColor','texturemap'); % on left wall
 surf([0.85,-0.15;0.85,-0.15],[-2.965,-2.965;-2.965,-2.965],[3.9,3.9;2.9,2.9],'CData',warningSign,'FaceColor','texturemap');  % on right wall
 
 % Main kitchen cupboard
 o = PlaceObject('cupboard.PLY',[-0.175,1.65,-2.4]);
 verts = [get(o,'Vertices'), ones(size(get(o,'Vertices'),1),1)] * trotz(-pi/2) * troty(-pi/2);
 set(o,'Vertices',verts(:,1:3))
 
 %Fences and Door
 p = PlaceObject('Fence.ply',[-3,2.75,0]);
 verts = [get(p,'Vertices'), ones(size(get(p,'Vertices'),1),1)] * trotz(pi/2);
 set(p,'Vertices',verts(:,1:3))
 
 q = PlaceObject('Fence.ply',[1.35,2.75,0]);
 verts = [get(q,'Vertices'), ones(size(get(q,'Vertices'),1),1)] * trotz(pi/2);
 set(q,'Vertices',verts(:,1:3))

 t = PlaceObject('Fence.ply',[-1.5,2.75,0]);
 verts = [get(t,'Vertices'), ones(size(get(t,'Vertices'),1),1)] * trotz(pi/2);
 set(t,'Vertices',verts(:,1:3))
 
 d = PlaceObject('Door.PLY',[0.075,2.75,1.35]);
 verts = [get(d,'Vertices'), ones(size(get(d,'Vertices'),1),1)] * trotz(pi/2);
 set(d,'Vertices',verts(:,1:3))
 
 % Oversseing worker
 u = PlaceObject('Worker.PLY',[2.5,0.05,0]);
 verts = [get(u,'Vertices'), ones(size(get(u,'Vertices'),1),1)] * trotx(pi/2) * troty (pi) * trotz(pi/2);
 set(u,'Vertices',verts(:,1:3))
 
 % Fire extinguisher
 r = PlaceObject('FireExtinguisher.PLY',[2,-2.95,1.5]);
 verts = [get(r,'Vertices'), ones(size(get(r,'Vertices'),1),1)];
 set(r,'Vertices',verts(:,1:3))
 
  % Working Table 1
 s = PlaceObject('WorkingTable.PLY',[1.5,-0.2,-0.05]);
 verts = [get(s,'Vertices'), ones(size(get(s,'Vertices'),1),1)] * trotz(pi/2);
 set(s,'Vertices',verts(:,1:3))
 
  % Working Table 2
 w = PlaceObject('WorkingTable.PLY',[1.5,-1.3,-0.05]);
 verts = [get(w,'Vertices'), ones(size(get(w,'Vertices'),1),1)] * trotz(pi/2);
 set(w,'Vertices',verts(:,1:3))
 
% Emergency Stop Button
  e = PlaceObject('Estop.PLY',[0.1,0.725,1.3]);
 verts = [get(e,'Vertices'), ones(size(get(e,'Vertices'),1),1)] * trotx(-pi/2);
 set(e,'Vertices',verts(:,1:3))
 
   x = PlaceObject('warninglight.PLY',[-0.7,-3,3.75]);
 verts = [get(x,'Vertices'), ones(size(get(x,'Vertices'),1),1)];
 set(x,'Vertices',verts(:,1:3))
 
 axis equal
 %% Spawn objects in environment
 % Generate dishwasher
 for i = 1:dishWasherCount % Generating dishwasher
    % From constructor Dishwasher(dishWasherCount,x,y,setPos)
    dishWasher{i} = Dishwasher(i,dishWasherInitialPosition(i,1),dishWasherInitialPosition(i,2),0); 
 end
 
  % Generate plates
 for i = 1:plateCount % Generating plates 
    % From constructor Plate(plateCount,x,y,setPos)
    plates{i} = Plate(i,plateInitialPosition(i,1),plateInitialPosition(i,2),0); 
 end
 
   % Generate plate cupboard
 for i = 1:cupboardCount % Generating 1 plate cupboard 
    % From constructor Cupboard(cupboardCount,x,y,setPos)
    cupboard{i} = Cupboard(i,cupboardInitialPosition(i,1),cupboardInitialPosition(i,2),0); 
 end
 
 axis equal
%% Spawn Robots in environment
 myUR3 = ModifiedUR3(ur3BaseTransform);
 myUR3.PlotAndColourUR3Robot()
 
 myPanda = Panda(pandaBaseTransform);
 myPanda.PlotAndColourPandaRobot()