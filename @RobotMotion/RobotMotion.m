classdef RobotMotion < handle
    % ROBOTMOTION CLASS that controls the different actions of the robot
    
    properties
        %> Robot
        robot; 
        
        %> Object to be picked up (plate)
        plate;
        
        %> Position of plate 
        platePosition;
        
        %> Joint trajectory matrix
        qMatrix; 
        
        %> Desired configuration
        desiredQ;
        
        %> Swaps between rmrc (true) and inverse kinematics (false)
        rmrc; 
        
        %> Collidable Objects
        collideObjects;
        
        %> Collision Plot
        collisionPlot;
        
        %> Light curtain
        lightCurtain;      
    end
    
    methods (Access = public)
%% Main Constructors
        function self = RobotMotion(robot, objects, rmrc)         
            self.robot = robot;
            self.collideObjects = self.CollideObjects();
            self.plate = objects{1};
            self.lightCurtain = objects{2};
            self.rmrc = rmrc;        
        end
%% Function to pickup plates
        function PlatePickup(self)    
            self.Pickup(); % Move robot to pickup the plates        
        end
%% Moving simulated plates in the environment
        % From Subject Resources page: https://au.mathworks.com/matlabcentral/fileexchange/58774-putting-simulated-objects-into-the-environment
        
        function MoveObject(self,object,position,orientation)
            % MoveObject moves object to a specified position and orientation
            v = object.Vertices;
            
            % Get vertex count
            objectVertexCount = size(v,1);
            
            % Move center point to origin
            midPoint = sum(v)/objectVertexCount;
            objectVerts = v - repmat(midPoint,objectVertexCount,1);
            
            deltaPosition = [position(1)-midPoint(1),  position(2)-midPoint(2), position(3)-(midPoint(3))]; % x, y, -z
            
            % Create a transform to describe the location (at the origin, since it's centered
            objectPose = eye(4);
            
            % Move forwards (facing in -y direction)
            forwardTR = makehgtform('translate',deltaPosition+midPoint); % makehgtform, creates a 4x4 TF matrix
            
            % % Random rotate about Z
            rotateTR = makehgtform('zrotate',orientation);
            
            % Move the pose forward and a slight and random rotation
            objectPose = objectPose * forwardTR*rotateTR;
            updatedPoints = (objectPose * [objectVerts,ones(objectVertexCount,1)]')';
            
            % Now update the Vertices
            object.Vertices = updatedPoints(:,1:3);
            drawnow();
        end     
    end
    
    methods (Access = private)  
%% Function to pickup plates  (To work on)
        function Pickup(self)
            self.MoveMesh(); % Move the object using mesh mid point

            if self.rmrc == true
                
                % Move just above the object
                point = self.objectPosition;
                point(3) = point(3) + 0.15;
                self.MoveEEtoTarget(point,false,self.moveVelocity,launching);
                
                % Slide picker down over top of the object
                point = self.platePosition;
                point(3) = point(3) - 0.07;
                self.MoveEEtoTarget(point,false,self.moveVelocity,launching);
                
                % Move picker to pick up object
                point = self.platePosition;
                self.MoveEEtoTarget(point,false,self.moveVelocity,launching);
                
            elseif    self.rmrc == false
                
                % Move picker up to pick up object
                point = self.platePosition;
                self.MoveEEtoTarget(point,false,self.moveVelocity,launching);
            end
  
        end
%% Animate to Joint Configuration
            function AnimateToJointConfigTrajectory(self,jointNumber,Q,CCW)
            
            startQ = self.robot.model.getpos;
            q = startQ;
            qConfig = self.robot.model.getpos;

            % Animation step increment size to have the number of steps equal self.trajectoryLength
            increment = (Q-startQ(jointNumber))/self.trajectoryLength;
            
            qArray = zeros(self.trajectoryLength,size(q,2));
            row = 0;
            
            % Animate specified joint number to specified angle
            for i=startQ(jointNumber):increment:Q
                row = row+1;
                q (jointNumber) = i;
                qArray(row,:)=q;
                %
            end
            
            self.qMatrix = qArray;
            
        end
%% Moving robot end-effector to desired target        
        function MoveEEtoTarget(self,point,moveObject)

            if self.rmrc == false % If rmrc fails, use normal inverse kinematics
                % Calculate the end pose joint config using inverse kinematics (ikcon)
                self.CalculateJoint(point);
                
                % Move to calculate Q joint configuration and move object if "moveObject == true""
                self.MovetoConfig(self.desiredQ,moveObject);
                
            elseif self.rmrc == true % If Resolve Motion Rate Control (rmrc) is used
                
                %  Move to specified point using rmrc
                self.rmrc(point,moveObject);      
            end
        end
%% Calculate Joint Configuration
        function CalculateJoint(self,point)
            % Calculating rotation of the end transform depending on object position
            if sign(point(2)) ~= 0
                rotationTransformX = sign(point(1))*-trotx(pi/2);
            else
                rotationTransformX = 1;
            end
            
            if sign(point(1)) ~= 0
                rotationTransformY = sign(point(1))*troty(pi/2);
            else
                rotationTransformY = 1;
            end
            
            % End tranformation for the end effector
            endTransform = transl(point) * rotationTransformY * rotationTransformX;
            
            % Use endTransform to find the joint orientations for the end position
            self.desiredQ = self.robot.model.ikcon(endTransform,self.robot.model.getpos);
            
            endTransformCheck = self.robot.model.fkine(self.desiredQ);
            
            % Difference (absolute positive magnitude) between the specified end Transform and actual
            % end Tansform calculated by ikcon (inverse kinematics)
            transformError = abs(endTransformCheck) - abs(endTransform);
            
            % Position error
            translationError = transformError(1:3,4)';
            
            % Rotation error
            rotationError = rad2deg(tr2rpy(transformError(1:3,1:3)));
            
            % Display errors
            disp('Transformation Error: ')
            disp(transformError);
            
            disp('Translation Error: ')
            disp(translationError);
            
            disp('Rotation Error (degrees): ')
            disp(rotationError);
        end
        
        function MovetoConfig(self,desiredQ,moveObject)
            
            % Finding the robot joint positions trajectory required to move the end effector to the end point
            self.qMatrix = jtraj(self.robot.model.getpos(),desiredQ,self.trajectoryLength);
            
%             if self.avoidCollisions == true
%                 
%                 self.resolveMotionRateControl = false;
%                 % Avoid collisions to reach the end point
%                 self.CollisionAvoidance(moveObject,launching);
%                 self.resolveMotionRateControl = true;
%                 
%             end
            
            % Animate a joint config trajectory
            self.AnimatePath(self.qMatrix,moveObject);
        end
        
        function AnimatePath (self,trajectory,moveObject)
            
            % Iterate the robot arms through their movement
            for i = 1:size(trajectory,1)
                
                q = trajectory(i,:);
                
                % calculate end effector position using fkine
                fkine = self.robot.model.fkine(self.robot.model.getpos());
                endEffectorPosition = fkine(1:3,4);
                
                if moveObject == true
                    % Move object to the end effector position for each
                    % trajectory step (simulates ball movement)
                    self.MoveObject(self.object,endEffectorPosition,0);
                end
                
                % Animate robot through a fraction of the total movement
                self.robot.model.animate(q);
                drawnow();
            end
        end
%% Move Mesh        
        function MoveMesh(self)
            
            v = self.object.Vertices; % Vertices

            objectVertexCount = size(v,1); % Get vertex count

            midPoint = sum(v)/objectVertexCount; % Move center point to origin           
            
            self.platePosition = midPoint+0.0065;
            
            sizeplatePosition = size(self.platePosition);
            sizeplatePosition = sizeplatePosition(2);
            
            for i=1:1:sizeplatePosition  
                if abs(self.platePosition(i)) <= 0.001
                    self.platePosition(i) = 0;
                end
            end
        end
%% rmrc (Lab 9)    
        function ResolvedMotionRateControl(self,startTr,endTr,time,deltaT)

            steps = time/deltaT;   % No. of steps for simulation (the more steps the better as there are more sample points on the sinusoid)
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares (value less than or equal to 0.1 is preferable)
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix at 1 1 1 for angular velocities to avoid errors. Usually set to 0.1 0.1 0.1 for angular velocities
 
            % Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            self.qMatrix = zeros(steps,self.robot.model.n);       % Array for joint anglesR
            qdot = zeros(steps,self.robot.model.n);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error
            
            %Set up trajectory
            s = lspb(0,1,steps);                % lspb - Linear segment with parabolic bends
            q0 = zeros(1, self.robot.model.n);       % Initial joint angles

            for i = 1:steps 
                TMatrix = ctraj(startTr,endTr,s);               % Cartesian Motion trajectory represented by a linear segment with parabolic bends                                        
                theta = tr2rpy(TMatrix);                        % Extract the roll, pitch, and yaw angles from the homogeneous transformation matrix
            end

            T0 = TMatrix(:,:,1);                                % Create a transformation of first point and angle (This sets a starting position to the robot that is at the start) of the trajectory
            self.qMatrix(1,:) = self.robot.model.ikcon(T0,q0);  % Solve joint angles to achieve first waypoint (this might or might not have some error)

            % 1.4) Track the trajectory with rmrc (error = desired - actual)
            for i = 1:steps-1
                Tcurrent = self.robot.model.fkine(self.qMatrix(i,:));                             % Get forward transformation at current joint state (as the joint angles obtained from ikcon are not always accurate)
                deltaX = TMatrix(1:3,4,i+1) - Tcurrent(1:3,4);                          % x are the position sets and T represents the actual position that the robot is in                  
                Rd = rpy2r(theta(i+1,1),theta(i+1,2),theta(i+1,3));                     % Get next RPY angles, convert to rotation matrix
                Ra = Tcurrent(1:3,1:3);                                                 % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(abs(Rd*Ra'));                                       % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    
                J = self.robot.model.jacob0(self.qMatrix(i,:));                                   % Get Jacobian at current joint state
                m(i) = sqrt(abs(det(J*J')));                                         % Manipulability formula
                if m(i) < epsilon                                                       % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
            invJ = inv(J'*J + lambda * eye(self.robot.model.n))* J';                                   % DLS Inverse
            qdot(i,:) = (invJ*xdot)';                                               % Solve the rmrc equation (you may need to transpose the         vector)
            for j = 1:self.robot.model.n                                                             % Loop through joints 1 to 6
                if self.qMatrix(i,j) + deltaT*qdot(i,j) < self.robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                    qdot(i,j) = 0;                                                             % Stop the motor
                elseif self.qMatrix(i,j) + deltaT*qdot(i,j) > self.robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                    qdot(i,j) = 0;                                                             % Stop the motor
                end
            end
            self.qMatrix(i+1,:) = self.qMatrix(i,:) + deltaT * qdot(i,:);                         	% Update next joint state based on joint velocities
            positionError(:,i) = TMatrix(1:3,4,i+1) - Tcurrent(1:3,4);                               % For plotting
            angleError(:,i) = deltaTheta;                                           % For plotting
            end
            
%             updatedQ = self.qMatrix(i+1,:);
%             self.robot.model.animate(updatedQ)
%             drawnow();
        end
%% RandomCollision Avoidance
        function CollisionAvoidance(self,moveObject)
            % Number of collision objects created
            noOfCollisionObjects = size(self.collisionObjects,2);
            
            % Initialising results container
            results = zeros(1,noOfCollisionObjects);
            
            % End joint config
            self.desiredQ = self.qMatrix(end,:);
            q = self.desiredQ;
            
            % Loop trajectory until possible solutions are found without
            % solutions
            while true  
                if q ~= self.desiredQ | self.rmrc == false
                    % Finding the robot joint positions required to move the end effector to the end point
                    trajectory = jtraj(self.robot.model.getpos(),q,self.trajectoryLength);
                elseif q == self.desiredQ & self.rmrc == true
                    endEffector = self.robot.model.fkine(q);
                    endPoint = endEffector(1:3,4)';
%                     rmrc(self,startTr,endTr,time,deltaT)
                    self.ResolveMotionRateControlCalculateTrajectory(endPoint);
                    trajectory = self.qMatrix();
                end
                
                delete(self.collisionPlot);
                self.collisionPlot = [];
                
                % Check for trajectory collision before animating for each obstacle
                for i=1:1:noOfCollisionObjects
                    obstacle = self.collisionObjects{i};
                    results(i) = self.IsCollision(trajectory,obstacle{1},obstacle{2},obstacle{3});
                end
                
                % If collision is along trajectory calculate a random new trajectory
                if ismember(1,results) 
                    disp('Collision along trajectory: recalculating alternate trajectory');
                    
                    % Calculate a random joint config and its end effector endPoint
                    [q,endPoint] = self.PrimativeRRT();                
                else
                    
                    % Move to random pose that is not in collision. Set q to equal the goal end Q and check if it is now in collision
                    if q ~= self.desiredQ
                        disp('Moving to a random pose that is not in collision');
                        self.AnimatePath(trajectory,moveObject);
                        q = self.desiredQ; 
                    else % if there is no collisions to move toward the final q pose
                        disp('No collisions along trajectory: moving to target pose');
                        break;
                    end
                end
                % Clear result collision checker
                results = [];
            end
        end
%% Random Selection of waypoints in the trajectory (Lab 5, 3.3)
        % Autonomously detect the waypoints (through random points)
        function [q,point] = PrimativeRRT(self)
            qRand = (2 * rand(1,self.robot.model.n) - 1) * pi;
            
            q = qRand;
            
            transform = self.robot.model.fkine(q);
            point = transform(1:3,4)';
            
            while hypot(point(1),point(2)) < 0.2 % hypot: Square root of sum of squares (hypotenuse) 
                qRand = (2 * rand(1,self.robot.model.n) - 1) * pi;   
                q = qRand;
                transform = self.robot.model.fkine(q);
                point = transform(1:3,4)';
            end  
        end
%% Define collision objects in the environment
        function [collisionObjects] = Collidebjects(self)
            % Hand above plate
            centerpnt = [0.5,-1.8,1.15];
            side = 0.15;
            plotOptions.plotFaces = true;
            translucency = false;
            
            [vertex,faces,faceNormals,patch] = self.RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions,translucency);
            
            handBox_plate = {vertex,faces,faceNormals,patch};
            
            % Hand in between rack
            centerpnt = [0.75,-1.4,1.0];
            side = 0.2;
            plotOptions.plotFaces = true;
            translucency = true;
            
            [vertex,faces,faceNormals,patch] = self.RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions,translucency);
            
            handBox_rack = {vertex,faces,faceNormals,patch};
            
            centerpnt = [0.825,-2,0.65];
            side = 0.7;
            plotOptions.plotFaces = true;
            translucency = true;
            
            [vertex,faces,faceNormals,patch] = self.RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions,translucency);
            
            dishwasherBox = {vertex,faces,faceNormals,patch};
            
            % Define Collision Objects in the environment
            collisionObjects = {handBox_plate,handBox_rack,dishwasherBox};
        end
%% IsIntersectionPointInsideTriangle (Lab 5)
        function result = IsIntersectionPointInsideTriangle(self,intersectP,triangleVerts)
            % Given a point which is known to be on the same plane as the triangle
            % determine if the point is
            % inside (result == 1) or
            % outside a triangle (result ==0 )
            
            u = triangleVerts(2,:) - triangleVerts(1,:);
            v = triangleVerts(3,:) - triangleVerts(1,:);
            
            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);
            
            w = intersectP - triangleVerts(1,:);
            wu = dot(w,u);
            wv = dot(w,v);
            
            D = uv * uv - uu * vv;
            
            % Get and test parametric coords (s and t)
            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
                result = 0;
                return;
            end
            
            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
                return;
            end
            
            result = 1;                      % intersectP is in Triangle
        end
%% IsCollision (Lab 5)
        function result = IsCollision(self,qMatrix,faces,vertex,faceNormals,returnOnceFound)
            if nargin < 6
                returnOnceFound = true;
            end
            result = false;
            
            row = 1;
            collisionPoints = zeros(row,3);
            
            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr = self.GetLinkPoses(qMatrix(qIndex,:));
                
                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = self.LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                        if check == 1 && self.IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                            collisionPoints(row,:) = intersectP;
                            row = row + 1;
                            result = true;
                        end
                    end
                end
            end
            
            numberOfIntersections = size(collisionPoints);
            numberOfIntersections = numberOfIntersections(end,1);
            
            collisionPlotSize = size(self.collisionPlot);
            collisionPlotSize = collisionPlotSize(end,1);
            
            for i=1:1:numberOfIntersections
                self.collisionPlot(i+collisionPlotSize,:) = plot3(collisionPoints(i,1),collisionPoints(i,2),collisionPoints(i,3),'r*');
            end
            
        end
%% GetLinkPoses (Lab 5)
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
        function [transforms] = GetLinkPoses(self, q)
            
            links = self.robot.model.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = self.robot.model.base;
            
            for i = 1:length(links)
                L = links(1,i);
                
                current_transform = transforms(:,:, i);
    
                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;
            end
        end
%% Line Plane Intersection (Lab 5)
        function [intersectionPoint,check] = LinePlaneIntersection(self,planeNormal,pointOnPlane,point1OnLine,point2OnLine)
            % Given a plane (normal and point) and two points that make up another line, get the intersection
            % Check == 0 if there is no intersection
            % Check == 1 if there is a line plane intersection between the two points
            % Check == 2 if the segment lies in the plane (always intersecting)
            % Check == 3 if there is intersection point which lies outside line segment
            
            intersectionPoint = [0 0 0];
            u = point2OnLine - point1OnLine;
            w = point1OnLine - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);
            check = 0; 
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end
            
            % Compute the intersection parameter
            sI = N / D;
            intersectionPoint = point1OnLine + sI.*u;
            
            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end
%% Rectangular Prism placeholder     
        function [vertex,face,faceNormals,collisionObject] = RectangularPrism(self,lower,upper,plotOptions,translucency,axis_h)
            if nargin<4
                axis_h=gca;
                if nargin<3
                    plotOptions.plotVerts=false;
                    plotOptions.plotEdges=true;
                    plotOptions.plotFaces=true;
                end
            end
            hold on
            
            vertex(1,:)=lower;
            vertex(2,:)=[upper(1),lower(2:3)];
            vertex(3,:)=[upper(1:2),lower(3)];
            vertex(4,:)=[upper(1),lower(2),upper(3)];
            vertex(5,:)=[lower(1),upper(2:3)];
            vertex(6,:)=[lower(1:2),upper(3)];
            vertex(7,:)=[lower(1),upper(2),lower(3)];
            vertex(8,:)=upper;
            
            face=[1,2,3;1,3,7;
                1,6,5;1,7,5;
                1,6,4;1,4,2;
                6,4,8;6,5,8;
                2,4,8;2,3,8;
                3,7,5;3,8,5;
                6,5,8;6,4,8];
            
            if 2 < nargout
                faceNormals = zeros(size(face,1),3);
                for faceIndex = 1:size(face,1)
                    v1 = vertex(face(faceIndex,1)',:);
                    v2 = vertex(face(faceIndex,2)',:);
                    v3 = vertex(face(faceIndex,3)',:);
                    faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
                end
            end
            
            % If you want to plot the vertices
            if isfield(plotOptions,'plotVerts') && plotOptions.plotVerts
                for i=1:size(vertex,1)
                    plot3(vertex(i,1),vertex(i,2),vertex(i,3),'r*');
                    text(vertex(i,1),vertex(i,2),vertex(i,3),num2str(i));
                end
            end
            
            % If you want to plot the edges
            if isfield(plotOptions,'plotEdges') && plotOptions.plotEdges
                links=[1,2;
                    2,3;
                    3,7;
                    7,1;
                    1,6;
                    5,6;
                    5,7;
                    4,8;
                    5,8;
                    6,4;
                    4,2;
                    8,3];
                
                for i=1:size(links,1)
                    plot3(axis_h,[vertex(links(i,1),1),vertex(links(i,2),1)],...
                        [vertex(links(i,1),2),vertex(links(i,2),2)],...
                        [vertex(links(i,1),3),vertex(links(i,2),3)],'k')
                end
            end
            
            % To plot the edges
            if isfield(plotOptions,'plotFaces') && plotOptions.plotFaces
                tcolor = [.2 .2 .8];
                if translucency == true 
                    translucency = 1; % 1 = 100% opaque
                else
                    translucency = 0; % 0 = 100% transparent 
                end
                collisionObject = patch('Faces',face,'Vertices',vertex,'FaceVertexCData',tcolor,'FaceColor','flat','FaceAlpha',translucency,'lineStyle','none');
            end
        end
    end  
end