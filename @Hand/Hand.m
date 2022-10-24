classdef Hand < handle
    % This class is used to display each finger of the gripper
  
    properties
        % Gripper models
        hand;
        
        % Workspace 
        workspace = [-0.6 0.6 -0.6 0.6 -0.05 1.1];  
         
        % Flag to indicate if gripper is used
        useGripper = false;   
    end
    
    methods
       %% Setting the constructor for the gripper class that will hold and create the objects 
        function self = Hand(useGripper)
           if nargin < 1
              useGripper = false;
           end
            self.useGripper = useGripper;
            
            %Define the gripper (DH parameters)
            self.GetHand();
            % Colour the gripper
%             self.PlotAndColourHand();
        end
        %% Set up the model of the Gripper
        function GetHand(self)
            
       name = 'Hand';
       % Set up DH Parameters for fingers
       G(1) = Link([0 0 0.015 0 0]);  
       
       self.hand = SerialLink(G,'name',name);
       self.hand.base = self.hand.base * trotx(-pi/2) * troty(pi);
        end
        %% Plot and Colour the Gripper
        function PlotAndColourHand(self)%robot,workspace)
            HandIndex = self.hand.n;
            [ faceData, vertexData, plyData{HandIndex+1} ] = plyread('hand.PLY','tri');
            self.hand.faces{HandIndex+1} = faceData;
            self.hand.points{HandIndex+1} = vertexData;

             % Display Shapes
            self.hand.plot3d(zeros(1,self.hand.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.hand.delay = 0;

            %Colour Shapes
            handles = findobj('Tag', self.hand.name);
            h = get(handles,'UserData');
            h.link(HandIndex+1).Children.FaceColor = 'red';
        end
    end      
end