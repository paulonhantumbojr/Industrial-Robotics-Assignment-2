classdef R19 < handle
    properties
        %> Robot model
        model;
        
        %> Workspace boundaries of the robot arm
        workspace = [-6 6 -6 6 -6 6];   
        
        %> Transform 
        transform; 
        
        %> Flag to indicate if gripper is used
        useGripper = false; 
    end
    
    methods %% Class for R19 robot simulation
function self = R19(transformation)
    self.transform = transformation;
    
%> Define the boundaries of the workspace

% robot = 
self.GetR19Robot();
% robot = 
% self.PlotAndColourRobot();%robot,workspace);

end
%% GetUR5Robot
% Given a name (optional), create and return a UR5 robot model
function GetR19Robot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['R19',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

      C(1) = Link([0    0      0       0    1]);  % Linear rail
      C(2) = Link([0   0.6     0       0    0]);  % Value for d1 is constant
      C(3) = Link([0  0.7285   0       pi/2   1]);  % d = 0.7285
      C(4) = Link([0  0.3444  -0.1     0   1]);  % d = 0.6888
 
      C(1).qlim = [0 0.8];
      C(2).qlim = [-360 360]*pi/180;
      C(3).qlim = [0 0.7885];
      C(4).qlim = [0 0.6888];

    self.model = SerialLink(C,'name',name);
    
%     Rotate robot to the correct orientation
    self.model.base = self.model.base * transl(self.transform);
end
%% Plot and Colour Robot
% Given a robot index, add the glyphs (vertices and faces) and colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 1:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['R19_link_',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['R19_link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end        
    end
end