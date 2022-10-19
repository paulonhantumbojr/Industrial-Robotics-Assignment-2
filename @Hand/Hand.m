classdef Hand < handle
    %HAND A way of creating a hand

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 4;
    end
    
    properties        
        %> A cell structure of plate models
        hand;
        
        %> Dishwasher dimensions in meters
        workspace = [-2.5 2.5 -2.5 2.5 -0.02 0.02]; 
     
        %> Dimensions of the workspace in regard to the table size
        workspaceDimensions;
        
        %> Transform
        transform;
    end
    
    methods         
        function self = Hand(transformation)
            self.transform = transformation;       
            
            self.workspaceDimensions = [-self.workspace(1)/2, self.workspace(1)/2 ...
                                       ,-self.workspace(2)/2, self.workspace(2)/2 ...
                                       ,-self.workspace(3)/2, self.workspace(3)/2];
            
            name = 'hand'; % ['hand',num2str(plateCount)]
            self.hand = self.GetHandModel(name);

            % Create the required number of blocks at the positions set by matrix or randomly spawn them
            self.hand.base = self.hand.base * transl(self.transform) * trotx(-pi/2) * trotz(pi);
            
            % Plot 3D model
            plot3d(self.hand,0,'delay',0); 
            drawnow();
        end  
    end

    methods (Static)
        %% Get Hand Model
        function model = GetHandModel(name)
            if nargin < 1
                name = 'hand';
            end
            
         % Setting the DH parameters and vertices of the block 
            [faceData,vertexData] = plyread('hand.PLY','tri');
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