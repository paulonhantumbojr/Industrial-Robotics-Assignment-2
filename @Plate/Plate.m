classdef Plate < handle
    %PLATE A way of creating a set of plates

    properties
        %> Number of plates
        plateCount = 2;
        
        %> A cell structure of plate models
        plate;
        
        %> Dishwasher dimensions in meters
        dishWasherSize = [-0.6 0.6 -0.6 0.6 -0.02 0.02]; % [-2.5 0.5,-1 1,-0.02 -0.02]
     
        %> Dimensions of the workspace in regard to the table size
        workspaceDimensions;
    end
    
    methods
        %% Constructors
        function self = Plate(plateCount,x,y,setPos)
            if 0 < nargin
                self.plateCount = plateCount;
            end
            
          %> Height of the basket in the dishwasher
           basketHeight = 0.8;
           
            self.workspaceDimensions = [-self.dishWasherSize(1)/2, self.dishWasherSize(1)/2 ...
                                       ,-self.dishWasherSize(2)/2, self.dishWasherSize(2)/2 ...
                                       ,basketHeight,basketHeight];
            
            name = ['plate',num2str(plateCount)];
            self.plate = self.GetPlateModel(name);

            % Create the required number of blocks at the positions set by matrix or randomly spawn them
            if setPos == 0 % Set spawn
                self.plate.base = self.plate.base * transl(x,y,basketHeight);
            end
            
                 % Plot 3D model
                plot3d(self.plate,0,'delay',0);
        end
    end

    methods (Static)
        %% Get Brick Model
        function model = GetPlateModel(name)
            if nargin < 1
                name = 'plate';
            end
            
         % Setting the DH parameters and vertices of the block 
            [faceData,vertexData] = plyread('plate.PLY','tri');
            L(1) = Link ([0 0 0.015 0 0]);
            model = SerialLink(L,'name',name);
            model.faces = {faceData,[]};
            vertexData(:,2) = vertexData(:,2); %+0.4
            model.points = {vertexData,[]};
            
        % Coloring the blocks
        handles = findobj('Tag', name);
        h = get(handles,'UserData');
        try 
            h.link(blockIndex+1).Children.FaceVertexCData = [plyData{blockIndex+1}.vertex.red ...
                                                          , plyData{blockIndex+1}.vertex.green ...
                                                          , plyData{blockIndex+1}.vertex.blue]/255;
            h.link(blockIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
        end
        end
    end    
end