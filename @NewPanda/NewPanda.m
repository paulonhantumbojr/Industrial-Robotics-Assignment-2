classdef NewPanda < handle
    properties
        %> Robot model
        model;
        
        %> Workspace
        workspace = [-0.6 0.6 -0.6 0.6 -0.05 1.1]
    end
    
    methods
        function self = NewPanda(base)
            self.GetPanda();
            self.model.base = base;
%             self.PlotAndColourPanda();
        end
        %% GetPanda
        % Create and return an UR3 robot model
        function GetPanda(self)

            name = 'Panda';
            % Panda Robot DH Parameters (theta d a alpha joint-type)
      P(1) = Link([pi   0.333     0       -pi/2   0]);  
      P(2) = Link([0     0        0        pi/2   0]);  
      P(3) = Link([0   0.316    0.0825     pi/2   0]);  
      P(4) = Link([0     0     -0.0825    -pi/2   0]); 
      P(5) = Link([0   0.384      0       -pi/2   0]); 
      P(6) = Link([0     0      0.088      pi/2   0]); 
      P(7) = Link([0  -0.0107     0        pi/2   0]); 
 
        % Joint limits
      P(1).qlim = deg2rad([-180 180]); % [-2.7437 2.7437]
      P(2).qlim = deg2rad([-180 180]); % [-1.7837 1.7837]
      P(3).qlim = deg2rad([-180 180]); % [-2.9007 2.9007]
      P(4).qlim = deg2rad([-180 180]); % [-3.0421 -0.1518]
      P(5).qlim = deg2rad([-180 180]); % [-2.8065 2.8065]
      P(6).qlim = deg2rad([-180 180]); % [0.5445 4.5169]
      P(7).qlim = deg2rad([-180 180]); % [-3.0159 3.0159]

    self.model = SerialLink(P,'name',name);
        end
        %% PlotAndColourRobot
        % Given a robot index, add the vertices and faces and colour them
        function PlotAndColourPanda(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Pandalink_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
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