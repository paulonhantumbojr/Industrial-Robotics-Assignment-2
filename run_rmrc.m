% Run with RMRC
 clc;
 clf;
 clear;
 close all;
 set(0, 'DefaultFigureWindowStyle', 'docked'); % Dock figures in the workspace
%% Environment 
 kitchen = Kitchen(0); % Plot the environment
 elements = kitchen.Spawnkitchen(); % Plot the environment objects
 
 % Generate pates
 plateCount = 2; % Count of plates spawned in the environment
 platePosition = [0.75,-1.525; ... % Plate 1 position
                  0.75,-1.45]; % Plate 2 position
              
  for i = 1:plateCount % Generating 2 plates
    plates{i} = Plate(i,platePosition(i,1),platePosition(i,2),0); % Plate Constructor
  end
 %% Robots
 pandaBaseTransform = [-0.1,-1.35,0.725]; % Panda robot Base position
 ur3BaseTransform = [0.8,-1.875,1.015]; % UR3 Base position
 
 % Plot the UR3
 myUR3 = ModifiedUR3(ur3BaseTransform);
 myUR3.PlotAndColourUR3Robot()
 ur3_q0 = zeros(1, myUR3.model.n);
 
 % Plot the Panda
 myPanda = Panda(pandaBaseTransform);
 myPanda.PlotAndColourPandaRobot()
 panda_q0 = zeros(1, myPanda.model.n);
 
 % Plot the hand
%  myHand = Hand([0.75,-1.55,1.0]);
 %% RMRC
 % For UR3
 t = 10;             % Total time (s)
 deltaT = 0.05;      % Control frequency
 T1 = myUR3.model.fkine(ur3_q0);         % Initial transform
 T2 = transl(0.8,-1.575,1.25) * trotx(pi) * trotz(pi/2);          % Arm Up position (trot to rotate the end-effector orient
 path = RMRC(myUR3,true);
 path.ResolvedMotionRateControl(T1,T2,t,deltaT);
 
 T3 = myUR3.model.fkine(myUR3.model.getpos);
 T4 = transl(platePosition(1,1),platePosition(1,2),0.9) * trotx(pi) * trotz(pi/2);
 path.ResolvedMotionRateControl(T3,T4,t,deltaT);
 
 % For Panda
 