classdef FingerPanda2 < handle
    % This class is used to display each finger of the gripper
  
    properties
        % Gripper models
        model;
        
        % Workspace 
        workspace = [-0.6 0.6 -0.6 0.6 -0.05 1.1];  
         
        % Flag to indicate if gripper is used
        useGripper = false;   
    end
    
    methods
       %% Setting the constructor for the gripper class that will hold and create the objects 
        function self = FingerPanda2(useGripper)
           if nargin < 1
              useGripper = false;
           end
            self.useGripper = useGripper;
            
            %Define the gripper (DH parameters)
            self.GetFingerPanda2();
            
            % Colour the gripper
%             self.GetAndColourFingerPanda2();
        end
        %% Set up the model of the Gripper
        function GetFingerPanda2(self)
            
       name = 'Finger2';
       % Set up DH Parameters for fingers
       J(1) = Link([0 0.15 0.005 0 0]); 
 
       % Set the joint limits
       J(1).qlim = [-180 180]*pi/180;
       
       self.model = SerialLink(J,'name',name);

        end
        %% Plot and Colour the Gripper
        function GetAndColourFingerPanda2(self)%robot,workspace)
            for linkIndex = 1:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['panda2_finger',num2str(linkIndex),'.ply'],'tri');                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
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