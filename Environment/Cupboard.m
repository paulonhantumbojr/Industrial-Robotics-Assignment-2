classdef Cupboard < handle
    %DISHWASHER A way of creating a set of dishwashers

    properties
        %> Number of dishwashers
        cupboardCount = 1;
        
        %> A cell structure of dishwasher models
        cupboard;
        
        %> Kitchen space
        kitchenSize = [-0.6 0.6 -0.6 0.6 -0.02 -0.02]; % [-2.5 0.5,-1 1,1.2 1.2] x, y and z ranges of the table     
     
        %> Dimensions of the workspace in regard to the table size
        workspaceDimensions;
    end
    
    methods
        %% Constructors
        function self = Cupboard(cupboardCount,x,y,setPos)
            if 0 < nargin
                self.cupboardCount = cupboardCount;
            end
            
          %> Height of the table
           wallDistance = 2.75;
           
            self.workspaceDimensions = [-self.kitchenSize(1)/2, self.kitchenSize(1)/2 ...
                                       ,-self.kitchenSize(2)/2, self.kitchenSize(2)/2 ...
                                       ,wallDistance,wallDistance];
            
            name = ['cupboard',num2str(cupboardCount)];
            self.cupboard = self.GetCupboardModel(name);

            % Create the required number of blocks at the positions set by matrix or randomly spawn them
            if setPos == 0 % Set spawn
                self.cupboard.base = self.cupboard.base * transl(x,y,wallDistance);
            end
            
                 % Plot 3D model
                plot3d(self.cupboard,0,'delay',0);
        end
    end

    methods (Static)
        %% Get Brick Model
        function model = GetCupboardModel(name)
            if nargin < 1
                name = 'cupboard';
            end
            
         % Setting the DH parameters and vertices of the block 
            [faceData,vertexData] = plyread('platecupboard.PLY','tri');
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