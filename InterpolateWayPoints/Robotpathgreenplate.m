%% Robot Motion
% Function to move Robot through the path
function [] = Robotpathgreenplate()
    close all;
    clf
    set(0, 'DefaultFigureWindowStyle', 'docked'); 
%% Load the environment in the world
 kitchen = Kitchen(0); % Plot the environment
 kitchen.Spawnkitchen(); % Plot the environment objects
 
 % Plate 1
 plate1_h = PlaceObject('plate.PLY'); % [0.265,-1.91,1.125]
 plate1_verts = get(plate1_h, 'Vertices');
 
 plate1_tverts = [plate1_verts, ones(size(plate1_verts, 1),1)] * transl(0.75,-1.525,0.775)';
 set(plate1_h ,'Vertices',plate1_tverts(:,1:3)) 
 
 % Plate 2
 plate2_h = PlaceObject('plateblack.ply'); 
 plate2_verts = get(plate2_h, 'Vertices');
 
 plate2_tverts = [plate2_verts, ones(size(plate2_verts, 1),1)] * transl(0.75,-1.45,0.775)';
 set(plate2_h ,'Vertices',plate2_tverts(:,1:3)) 
%% Robots           
 % UR3
 ur3BaseTransform = transl(0.8,-1.85,1.0) * trotz(pi); % Set UR3 Base
 myUR3 = NewUR3(ur3BaseTransform);  
 myUR3.PlotAndColourUR3();
 ur3_q0 = zeros (1, myUR3.model.n); % Initial UR3 joint angles

 % IQ Finger1
 a = IQFinger1;
 a.model.base = a.model.base * myUR3.model.fkine(ur3_q0) * troty(-pi/2) * transl(0.065,-0.0115,0); % Forward kinematics for end-effector base
 a.GetAndColourFingerIQ1(); % Plotting object from Finger 1 class
 
 % IQ Finger2
 b = IQFinger2;
 b.model.base = b.model.base * myUR3.model.fkine(ur3_q0) * trotz(pi) * troty(-pi/2) * transl(0.065,-0.0115,0); % Forward kinematics for end-effector base
 b.GetAndColourFingerIQ2(); % Plotting object from Finger 2 class

 % Panda
  pandaBaseTransform = transl(-0.15, -1.45, 0.995);
  myPanda = NewPanda(pandaBaseTransform);
  myPanda.PlotAndColourPanda();
  panda_q0 = zeros (1, myPanda.model.n);
  
  view(170,15) % Change view
%% Grippers
 % IQ Fingers
 iqFingerStart = deg2rad([0 0]);
 iqFingerOpen = deg2rad([-6.5 0]);
 iqFingerClose = deg2rad([22 10.5]);
 
 % Panda Fingers
 pandaFingerStart = deg2rad(0);
 pandaFingerOpen = deg2rad(19.5);
 pandaFingerClose = deg2rad(-13);
%% Path (UR3)
  % Set pickup path
  q1 = deg2rad([45 -27.5 50 70 90 0]);
  q2 = deg2rad([81.5 -25 42.8 59.2 90 0]);
  q3 = deg2rad([81.5 -12 50 51 90 0]);

  qWayPoints = [ur3_q0;q1;q2;q3];
  iqOpenWayPoints = [iqFingerStart;iqFingerOpen];
  iqCloseWayPoints = [iqFingerOpen;iqFingerClose];
  
  qMatrix = InterpolateWayPointRadians(qWayPoints, deg2rad(2));
  openMatrix = InterpolateWayPointRadians(iqOpenWayPoints, deg2rad(2));
  closeMatrix = InterpolateWayPointRadians(iqCloseWayPoints, deg2rad(2));

  a.model.animate(openMatrix);
  b.model.animate(openMatrix); % Open fingers
  drawnow();
  
  for i = 1:size(qMatrix,1)
      ur3Q = qMatrix(i,:);
      myUR3.model.animate(ur3Q);
            a.model.base = myUR3.model.fkine(ur3Q) * troty(-pi/2) * transl(0.065,-0.0115,0);
            a.model.animate(b.model.getpos);   
            b.model.base = myUR3.model.fkine(ur3Q)* trotz(pi) * troty(-pi/2) * transl(0.065,-0.0115,0);
            b.model.animate(b.model.getpos);
      drawnow();
  end
  a.model.animate(closeMatrix);
  b.model.animate(closeMatrix); % Close Fingers
  drawnow();
  
  % Pick up and drop plate 
  q4 = deg2rad([81.5 -33.6 64.4 59.2 90 0]);
  q5 = deg2rad([146 -81.0 57.2 93.6 90 86.6]);
  q6 = deg2rad([182 -30.6 -22 112 90 86.6]);
  q7 = deg2rad([182 -23 -22 109 90 86.6]); % ([182 -22 -22 105 90 86.6])
  
  qWayPoints = [q4;q5;q6;q7];
  
  qMatrix = InterpolateWayPointRadians(qWayPoints, deg2rad(2));
  
  for i = 1:size(qMatrix,1)
      ur3Q = qMatrix(i,:);
      myUR3.model.animate(ur3Q);
      
      a.model.base = myUR3.model.fkine(ur3Q) * troty(-pi/2) * transl(0.065,-0.0115,0);
      a.model.animate(a.model.getpos);   
      b.model.base = myUR3.model.fkine(ur3Q)* trotz(pi) * troty(-pi/2) * transl(0.065,-0.0115,0);
      b.model.animate(b.model.getpos); 
      
      tr = myUR3.model.fkine(ur3Q) * transl(0,0,0.1975); % transl(0,0,0.1925)
      plate1_tverts = [plate1_verts, ones(size(plate1_verts, 1),1)] * tr';
      set(plate1_h ,'Vertices',plate1_tverts(:,1:3)) 

      drawnow();
      pause(0.01);
  end

  q8 = deg2rad([182 -23.4 -22 112 90 86.6]);
  q9 = deg2rad([180 -90 0 0 0 0]);

  qWayPoints = [q7;q8;q9;ur3_q0];
  
  qMatrix = InterpolateWayPointRadians(qWayPoints, deg2rad(2));

  a.model.animate(openMatrix);
  b.model.animate(openMatrix); % Open fingers
  drawnow()
  for i = 1:size(qMatrix,1)
      ur3Q = qMatrix(i,:);
      myUR3.model.animate(ur3Q);
            a.model.base = myUR3.model.fkine(ur3Q) * troty(-pi/2) * transl(0.065,-0.0115,0);
            a.model.animate(b.model.getpos);   
            b.model.base = myUR3.model.fkine(ur3Q)* trotz(pi) * troty(-pi/2) * transl(0.065,-0.0115,0);
            b.model.animate(b.model.getpos);
      drawnow();
  end
  %% Path (Panda)
  % Set pickup path
  q1 = deg2rad([-45 50 0 -43.2 0 -104 43.5]);
  q2 = deg2rad([-45 55 0 -57.6 0 -107 43.5]);

  qWayPoints = [panda_q0;q1;q2];
  pandaOpenWayPoints = [pandaFingerStart;pandaFingerOpen];
  pandaCloseWayPoints = [pandaFingerOpen;pandaFingerClose];
  
  qMatrix = InterpolateWayPointRadians(qWayPoints, deg2rad(2));
  openMatrix = InterpolateWayPointRadians(pandaOpenWayPoints, deg2rad(2));
  closeMatrix = InterpolateWayPointRadians(pandaCloseWayPoints, deg2rad(2));

  for i = 1:size(qMatrix,1)
      pandaQ = qMatrix(i,:);
      myPanda.model.animate(pandaQ);
      drawnow();
  end
   
  % Pick up and drop plate 
  q3 = deg2rad([-45 50 0 -43.2 0 -104 43.5]); % [-45 50 0 -43.2 0 -104 43.5]
  q4 = deg2rad([-131 32 0 -43.2 0 -104 130]);
  q5 = deg2rad([-131 60 0 -43.2 97.2 -144 14]); % ([-131 60 0 -43.2 97.2 -144 104])
  
  qWayPoints = [q3;q4;q5];
  
  qMatrix = InterpolateWayPointRadians(qWayPoints, deg2rad(2));
  
  for i = 1:size(qMatrix,1)
      pandaQ = qMatrix(i,:);
      myPanda.model.animate(pandaQ);
      
      tr = myPanda.model.fkine(pandaQ) * transl(0,-0.2085,0) * trotz(pi/2) * trotx(-pi/2);
      plate1_tverts = [plate1_verts, ones(size(plate1_verts, 1),1)] * tr';
      set(plate1_h ,'Vertices',plate1_tverts(:,1:3)) 

      drawnow();
      pause(0.01);
  end

  plate1_tverts = [plate1_verts, ones(size(plate1_verts, 1),1)] * transl(-0.725,1.335,2.185)' * trotx(-pi/2);
  set(plate1_h ,'Vertices',plate1_tverts(:,1:3)) 

  q7 = deg2rad([-131 32 0 -43.2 0 -104 130]);
  q8 = deg2rad([0 0 0 0 0 0 0]);

  qWayPoints = [q7;q8];
  
  qMatrix = InterpolateWayPointRadians(qWayPoints, deg2rad(2));

  for i = 1:size(qMatrix,1)
      pandaQ = qMatrix(i,:);
      myPanda.model.animate(pandaQ);    
      drawnow();
  end
end