%% Test light curtain
close;
%% Environment 
 kitchen = Kitchen(0); % Plot the environment
 elements = kitchen.Spawnkitchen(); % Plot the environment objects
 
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
 pandaBaseTransform = transl(-0.1,-1.35,0.725); % Panda robot Base position
 ur3BaseTransform = transl(0.8,-1.875,1.015) * trotz(pi); % UR3 Base position
 
 % Plot the UR3
 myUR3 = NewUR3(ur3BaseTransform);
 myUR3.PlotAndColourUR3()
 
 % Plot the Panda
 myPanda = NewPanda(pandaBaseTransform);
 myPanda.PlotAndColourPanda()
 
 % Plot the Hand
 h = Hand(0);
 h.hand.base = h.hand.base * transl(0,-1,-2); % [X,Z,Y]
 h.PlotAndColourHand()% Hand initial position

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
 end