%% Test collisions
 clc;
 clf;
 clear;
 close all;
 set(0, 'DefaultFigureWindowStyle', 'docked'); % Dock figures in the workspace
%% Robot
T = transl(0,0,0);
robot = NewUR3(T); % Activate the PlotAndColour handle to ensure that the robots plot accordingly
robot.PlotAndColourUR3();

% UR3
centerPoints = [0.0, 0.0, 0.05; % Base 
                0.0, -0.01, 0.01; % Link 1 
                0.125, 0.0, 0.125; % Link 2 
                0.105, 0.0, 0.05; % Link 3 
                0.0, 0.0, 0.01; % Link 4 
                0.0, 0.0, 0.06; % Link 5 
                0.0, 0.0, 0.0;]; % end-effector
            
radii = [0.08, 0.09, 0.055;  
         0.075, 0.085, 0.075;
         0.175, 0.08, 0.085; 
         0.15, 0.06, 0.085; 
         0.04, 0.055, 0.065;
         0.04, 0.045, 0.125; 
         0.0, 0.0, 0.0;]; 
     
 meshPosition = [0.05,0,0.1];
 
 collision = Collisiontest(robot, centerPoints, radii);
 collision.plotEllipsoids();
 hold on;
 mesh = Mesh(0.2,10,meshPosition);
 mesh.updatePlot();
 collision.checkCollision(mesh.getPoints())