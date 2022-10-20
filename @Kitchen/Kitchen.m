classdef Kitchen < handle
    % Class that plots the kitchen environment into the workspace

    properties
        % Flag to indicate if gripper is used
        useGripper = false;
    end
    
    methods
        %% Constructors
        function self = Kitchen(useGripper)
          if nargin < 1
             useGripper = false;
          end   
          self.useGripper = useGripper;
            
          self.walls();
        end
        
        function [elements] = Spawnkitchen(self)
        
        hold on;
        axis([-2.5 2.5 -2.5 2.5 -0.02 4]); % Kitchen dimensions
        
        disp('Loading kitchen');
        
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
 
        table_h = PlaceObject('WorkingTable.PLY',[-0.25,-1.25,-0.05]);
        verts = [get(table_h,'Vertices'), ones(size(get(table_h,'Vertices'),1),1)];
        set(table_h,'Vertices',verts(:,1:3))
 
        estop_h = PlaceObject('Estop.PLY',[-1.95,1,-0.75]);
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
 
        dishwasher_h = PlaceObject('dishwasher.PLY',[-1.125,0.125,-2.8]);
        verts = [get(dishwasher_h,'Vertices'), ones(size(get(dishwasher_h,'Vertices'),1),1)] * trotx(pi/2) * troty(pi);
        set(dishwasher_h,'Vertices',verts(:,1:3))
        
        rack_h = PlaceObject('dishrack.PLY',[0.2375,-2.215,1.05]);
        verts = [get(rack_h ,'Vertices'), ones(size(get(rack_h ,'Vertices'),1),1)];
        set(rack_h ,'Vertices',verts(:,1:3))
        
        worker_h = PlaceObject('worker.PLY',[1.15,0,-1.15]);
        verts = [get(worker_h ,'Vertices'), ones(size(get(worker_h ,'Vertices'),1),1)] * troty(pi/2) * trotx(-pi/2) * trotz(deg2rad(160));
        set(worker_h ,'Vertices',verts(:,1:3))

        platestand_h = PlaceObject('platestand.PLY',[-0.825,-2.15,0.95]);
        verts = [get(platestand_h ,'Vertices'), ones(size(get(platestand_h ,'Vertices'),1),1)];
        set(platestand_h ,'Vertices',verts(:,1:3))
        
        % Elements to be plotted in the kitchen
        elements = {cupboard_h1, fence_h1, fence_h2, door_h, table_h, estop_h, warninglight_h, rack_h, ...
                    lightcurtain_h1, lightcurtain_h2, lightcurtain_h3, dishwasher_h, worker_h, ...
                    platestand_h}; 
        end
    end
    
        methods (Access = private)

        function walls(self)
            
            % Setup variables for workshop components
            floor = imread ('Floor.jpg'); % Floor placeholder
            wall = imread ('Mosaic.jpg'); % Walls placeholder
            warningSign = imread ('workinprogress.jpg'); % Waring sign placeholder
            lightSign = imread('Curtain.png'); % Curtains placeholder
                        
            % Floor
            surf([-2.5,-2.5;2.5,2.5],[-2.5,2.5;-2.5,2.5],[-0.01,-0.01;-0.01,-0.01],'CData',floor,'FaceColor','texturemap'); 

            hold on
 
            % Walls 
            surf([-2.5,2.5;-2.5,2.5],[-2.5,-2.5;-2.5,-2.5],[0,0;4,4],'CData',wall,'FaceColor','texturemap'); % Left wall
            surf([-2.5,-2.5;-2.5,-2.5],[-2.5,2.5;-2.5,2.5],[4,4;0,0],'CData',wall,'FaceColor','texturemap'); % Right wall

            % Warning signs
            surf([2.3,2.3;2.3,2.3],[1.25,2.25;1.25,2.25],[2.5,2.5;1.5,1.5],'CData',warningSign,'FaceColor','texturemap');  % on right wall
            surf([2.0,0;2.0,0],[-2.465,-2.465;-2.465,-2.465],[3.9,3.9;2.9,2.9],'CData',lightSign,'FaceColor','texturemap'); % on left wall

            view(3);
        end
    end
end