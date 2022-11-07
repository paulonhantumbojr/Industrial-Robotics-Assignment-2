%% Test collisions
 clc;
 clf;
 clear;
 close all;
 set(0, 'DefaultFigureWindowStyle', 'docked'); % Dock figures in the workspace
%% Robot
T = transl(0,0,0);
robot = NewPanda(T);
robot.PlotAndColourPanda()

% Panda
centerPoints = [-0.025, 0.0, 0.06; % Base
                0.0, 0.0975, -0.035; % Link 1 
                0.0, 0.03, 0.075; % Link 2 
                -0.075, -0.085, 0.0; % Link 3 
                0.05, 0.0, 0.035; % Link 4 
                0.0, 0.125, 0.025; % Link 5 
                -0.05,  0.025, 0.0;]; % Link 6
            
radii = [0.135, 0.125, 0.075; 
         0.1, 0.175, 0.1; 
         0.125, 0.12, 0.125; 
         0.075, 0.075, 0.095; 
         0.125, 0.125, 0.095; 
         0.085, 0.15, 0.105; 
         0.110, 0.1, 0.085;]; 
     
 meshPosition = [0.05,0,0.5];
 
 collision = Collisiontest(robot, centerPoints, radii);
 collision.plotEllipsoids();
 hold on;
 mesh = Mesh(0.2,10,meshPosition);
 mesh.updatePlot();
 collision.checkCollision(mesh.getPoints())