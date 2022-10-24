%% Test light curtain
close;
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
 
 % Plot the Hand
 h = Hand(0);
 h.hand.base = h.hand.base * transl(0,-1,-2); % [X,Z,Y]
 h.PlotAndColourHand()% Hand initial position

 view(175,15)

 [x,z] = meshgrid(-1.6:0.01:1.7, 0:0.01:1.5);  %setting location of meshgrid
 y(1:size(x,1),1:1) = 0.2;
 lightCurtainS2 = surf(x,y,z,'FaceAlpha',0.1,'EdgeColor','none');

 [f,v,data] = plyread('hand.ply','tri');
 handvertices = v;

 input('\nInitiate Light curtain Demo')

 h.hand.base = transl(0,-0.2,1) * trotx(pi/2) * trotz(pi);
 h.hand.animate(h.hand.getpos);
 drawnow();
 pause(0.2);

 handvertices(:,1) = handvertices(:,1) + h.hand.base(1,4);
 handvertices(:,2) = handvertices(:,2) + h.hand.base(2,4);
 handvertices(:,3) = handvertices(:,3) + h.hand.base(3,4);

 if max(handvertices(:,2)) >= -0.5
    fprintf("Light Curtain has been activated")
    set(gcf,'color','r')
%     eStop.isPressed = true;  eStop.press(); estop.isPressed = false; GUI.ESTOP = false;eStop.press();
 end
