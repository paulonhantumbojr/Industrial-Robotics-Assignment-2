classdef ModifiedUR3 < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-0.6 0.6 -0.6 0.6 -0.05 1.1]; %  [-0.6 0.6 -0.6 0.6 -0.2 1.1]; 
        
        %> Base Transform
        transform;
      
    end
    
    methods %% Class for UR3 robot simulation
        function self = ModifiedUR3(transformation)
        self.transform = transformation;
            
            self.GetUR3Robot();
%             self.PlotAndColourUR3Robot();

            drawnow
        end

        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetUR3Robot(self)
            pause(0.001);
            name = ['UR3_',datestr(now,'yyyymmddTHHMMSSFFF')];
            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
          
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
            self.model.base = self.model.base * transl(self.transform);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourUR3Robot(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['ur3link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
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
                    centerPoint = self.model.base(1:3,4);
                    radii = [0.20,0.1,0.1];
                    [X,Y,Z] = ellipsoid(centerPoint(1),centerPoint(2),centerPoint(3),radii(1),radii(2),radii(3));
                for i = 1:(self.model.n+1) % it's counted from 1 to 7 as the base is also considered as an ellipsoid (6 links + 1 base = 7)
                    self.model.points{i} = [X(:),Y(:),Z(:)];
                    warning off
                    self.model.faces{i} = delaunay(self.model.points{i});    
                    warning on;
                end
            end
        end        
    end
end