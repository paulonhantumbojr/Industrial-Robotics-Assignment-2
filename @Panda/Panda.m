classdef Panda < handle
    properties
        %> Robot model
        model;
        
        %> Workspace boundaries of the robot arm
        workspace = [-0.6 0.6 -0.6 0.6 -0.05 1.1];  %  [-0.6 0.6 -0.6 0.6 -0.2 1.1];
        
        %> Base Transform 
        transform; 
    end
    
    methods %% Class for Panda robot simulation
function self = Panda(transformation)
    self.transform = transformation;
    
% Model Panda robot  
self.GetPandaRobot();
% Plot and colour Panda robot 
% self.PlotAndColourPandaRobot();
end
%% GetUR5Robot
% Given a name (optional), create and return a Panda robot model
function GetPandaRobot(self)
        pause(0.001);
        name = ['Panda',datestr(now,'yyyymmddTHHMMSSFFF')]; 

% Panda Robot DH Parameters (theta d a alpha joint-type)
      P(1) = Link([pi   0.333     0       -pi/2   0]);  
      P(2) = Link([0     0        0        pi/2   0]);  
      P(3) = Link([0   0.316      0.0825    pi/2   0]);  
      P(4) = Link([0     0        -0.0825    -pi/2   0]); 
      P(5) = Link([0   0.384      0    -pi/2   0]); 
      P(6) = Link([0     0        0.088        pi/2   0]); 
      P(7) = Link([0     -0.0107        0      pi/2   0]); 
 
% Joint limits
      P(1).qlim = [-2.7437 2.7437];
      P(2).qlim = [-1.7837 1.7837];
      P(3).qlim = [-2.9007 2.9007];
      P(4).qlim = [-3.0421 -0.1518];
      P(5).qlim = [-2.8065 2.8065];
      P(6).qlim = [0.5445 4.5169];
      P(7).qlim = [-3.0159 3.0159];

    self.model = SerialLink(P,'name',name);
    
%     Rotate robot to the correct orientation
    self.model.base = self.model.base * transl(self.transform);
end
%% Plot and Colour Robot
function PlotAndColourPandaRobot(self)
    for linkIndex = 0:self.model.n
        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['Pandalink_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
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