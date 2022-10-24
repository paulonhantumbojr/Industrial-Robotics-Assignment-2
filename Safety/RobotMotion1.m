classdef RobotMotion1 < handle
    % ROBOTMOTION handles the motion of the robots and ply files.
    % Additionally it logs movement details
    
    properties
        %> Robot
        robot;
        
        %> Emergency stop state: false = Normal Operation, true = Emergency stop activated
        estop = false; 
        
        %> Signal to resume process after estop. true = signal to keep moving, false = robot disabled
        signal = true;
        
        %> Move Hand
        togglehand = false; 
        
        %> Hand movement
        movehand;
        
        %> Hand Transform
        handtransform = "-y";
        
        %> Hand Translation
        handtranslation = [0.5,0.5,0.5];
        
        %> Hand Mesh
        handmesh
    end
    
    methods
        function self = RobotMotion(robot,initialQ,objects,collisionAvoidance,rmrc)
            self.robot = robot; % robot model
            self.object = objects; % Plate
            self.rmrc = rmrc; % rmrc
            self.collisionAvoidance = collisionAvoidance;
        end        
%% Move the Robot to the object in the environmrnt       
        function [qOut, qMatrix] = RobotMotionToObject(self, loc_T, qGuess, ...
                steps)
            %This function takes a SerialLink robot, a desired 4x4 Transformation ...
            %Matrix to reach, and animates a calculated minimum jerk trajectory
            %to the configuration.
            
            % For collision avoidance
            [dishwasher_translation, dishrack_translation,cupboard_translation, dishwasher_centerpoint, dishwasher_width, ...
            dishwasher_height, dishwasher_depth, dishrack_centerpoint, dishrack_width, ...
            dishrack_height, dishrack_depth, cupboard_centerpoint, cupboard_width, ...
            cupboard_height, cupboard_depth] = self.spawnCollisionBlocks(0);

            [hand_centerpoint, hand_width, hand_height, hand_depth] = PLY_Obstacle_Dimensions(1,0);

            % Inverse Kinematics and Trajectory setting to move towards
            % location
            newQ = self.robot.model.ikcon(loc_T, qGuess); % loc
            qMatrix = jtraj(self.robot.model.getpos(), newQ, steps);

            % Moving to the spot above the desired location
            for i = 1:steps
                   if((i + 5) > (steps-1)) % if((i + 5) > (steps-1))
                        viewCollisionDishwasher = Collisiondetectionplot(self.robot, qMatrix(i,:), qMatrix(i,:),dishwasher_centerpoint, dishwasher_translation, dishwasher_width, dishwasher_height, dishwasher_depth);
                        viewCollisionDishrack = Collisiondetectionplot(self.robot, qMatrix(i,:), qMatrix(i,:),dishrack_centerpoint, dishrack_translation, dishrack_width, dishrack_height, dishrack_depth);          
                        viewCollisionCupboard = Collisiondetectionplot(self.robot, qMatrix(i,:), qMatrix(i,:),cupboard_centerpoint, cupboard_translation, cupboard_width, cupboard_height, dishrack_depth);   
                        viewCollisionHand = Collisiondetectionplot(self.robot, qMatrix(i,:), qMatrix(i,:),hand_centerpoint, self.handtranslation, hand_width, hand_height, hand_depth); % toggle hand 
                   else
                        viewCollisionDishwasher = Collisiondetectionplot(self.robot, qMatrix(i+4,:), qMatrix(i+4,:),dishwasher_centerpoint, dishwasher_translation, dishwasher_width, dishwasher_height, dishwasher_depth);
                        viewCollisionDishrack = Collisiondetectionplot(self.robot, qMatrix(i+4,:), qMatrix(i+4,:),dishrack_centerpoint, dishrack_translation, dishrack_width,  dishrack_height, dishrack_depth);
                        viewCollisionCupboard = Collisiondetectionplot(self.robot, qMatrix(i+4,:), qMatrix(i+4,:),cupboard_centerpoint, cupboard_translation, cupboard_width,  cupboard_height, cupboard_depth);   
                        viewCollisionHand = Collisiondetectionplot(self.robot, qMatrix(i+4,:), qMatrix(i+4,:),hand_centerpoint, self.handtranslation, hand_width, hand_height, hand_depth); % toggle hand
                   end
                   
                   %If either of them are true, the estop is triggered
                   if(viewCollisionDishwasher||viewCollisionDishrack||viewCollisionCupboard||viewCollisionHand)
                        self.estop = true;
                        self.signal = false;
                   end
                   
                   % Display a message if robot collides with any specific 
                   if(viewCollisionDishwasher == true)
                        disp("There is a collision with the dishwasher");
                   end
                   if(viewCollisionDishrack == true)
                        disp("There is a collision with the dishrack");
                   end
                   if(viewColissionHand == true)
                        disp("There is a collision with the hand");                  
                   end
                   if(viewCollisionCupboard == true)
                        disp("There is a collision with the dishrack");
                   end
                   
                % Plot Robot Moving (if protective stop not called)
               if self.signal == 1
                    self.robot.model.animate(qMatrix(i, :));
               end
               drawnow();
               
               viewHandinWorkspace = LightCurtainTrigger(self.togglehand,1); % Check when hand is in the light curtain area
               if(viewHandinWorkspace == true) % If the hand is in the workspace trigger the estop
                    self.estop = true;
                    self.signal = false;
                    disp('Detection at light curtain. Aborting operation.')
               end
               
               % When the estop is active it is locked in a while loop that
               % moves its actions back in the frame
               if self.estop == true
                   i = i-2; 
               end
               while (self.estop == true || self.signal == false)
                  % If the condition for the hand motion is true, translate
                  % the hand
                   if self.togglehand == true
                       self.handmotion;
                   end
                   pause(0.2);
               end
               
                  % If the condition for the hand motion is true, translate
                  % the hand
               if self.togglehand == true
                   self.handmotion;
               end   
            end
            
            qOut = qMatrix(end,:);
        end
%% Move Robot With the Plate 
        function [qOut, qMatrix] = RobotMotionWithObject(self, loc_T, ...
                 objMesh_h, objVertices, qGuess, steps)
            %This function takes a SerialLink robot, a desired 4x4 Transformation ...
            %Matrix to reach, and animates a calculated minimum jerk trajectory
            %to the configuration.
            
            % For collision avoidance
            [dishwasher_translation, dishrack_translation,cupboard_translation, dishwasher_centerpoint, dishwasher_width, ...
            dishwasher_height, dishwasher_depth, dishrack_centerpoint, dishrack_width, ...
            dishrack_height, dishrack_depth, cupboard_centerpoint, cupboard_width, ...
            cupboard_height, cupboard_depth] = self.spawnCollisionBlocks(0);

            [hand_centerpoint, hand_width, hand_height, hand_depth] = PLY_Obstacle_Dimensions(1,0);

            % Inverse Kinematics and Trajectory setting to move towards
            % location
            newQ = self.robot.model.ikcon(loc_T, qGuess);
            qMatrix = jtraj(self.robot.model.getpos(), newQ, steps);

            % Moving to the spot above the desired location
            for i = 1:steps
                   if((i + 5) > (steps-1)) % if((i + 5) > (steps-1))
                        viewCollisionDishwasher = Collisiondetectionplot(self.robot, qMatrix(i,:), qMatrix(i,:),dishwasher_centerpoint, dishwasher_translation, dishwasher_width, dishwasher_height, dishwasher_depth);
                        viewCollisionDishrack = Collisiondetectionplot(self.robot, qMatrix(i,:), qMatrix(i,:),dishrack_centerpoint, dishrack_translation, dishrack_width, dishrack_height, dishrack_depth);          
                        viewCollisionCupboard = Collisiondetectionplot(self.robot, qMatrix(i,:), qMatrix(i,:),cupboard_centerpoint, cupboard_translation, cupboard_width, cupboard_height, dishrack_depth);   
                        viewCollisionHand = Collisiondetectionplot(self.robot, qMatrix(i,:), qMatrix(i,:),hand_centerpoint, self.handtranslation, hand_width, hand_height, hand_depth); % may be toggle hand   
                   else
                        viewCollisionDishwasher = Collisiondetectionplot(self.robot, qMatrix(i+4,:), qMatrix(i+4,:),dishwasher_centerpoint, dishwasher_translation, dishwasher_width, dishwasher_height, dishwasher_depth);
                        viewCollisionDishrack = Collisiondetectionplot(self.robot, qMatrix(i+4,:), qMatrix(i+4,:),dishrack_centerpoint, dishrack_translation, dishrack_width,  dishrack_height, dishrack_depth);
                        viewCollisionCupboard = Collisiondetectionplot(self.robot, qMatrix(i+4,:), qMatrix(i+4,:),cupboard_centerpoint, cupboard_translation, cupboard_width,  cupboard_height, cupboard_depth);   
                        viewCollisionHand = Collisiondetectionplot(self.robot, qMatrix(i+4,:), qMatrix(i+4,:),hand_centerpoint, self.handtranslation, hand_width, hand_height, hand_depth); % may betoggle hand
                   end
                   
                   %If either of them are true, the estop is triggered
                   if(viewCollisionDishwasher||viewCollisionDishrack||viewCollisionCupboard||viewCollisionHand)
                        self.estop = true;
                        self.signal = false;
                   end
                   
                   % Display a message if robot collides with any specific 
                   if(viewCollisionDishwasher == true)
                        disp("There is a collision with the dishwasher");
                   end
                   if(viewCollisionDishrack == true)
                        disp("There is a collision with the dishrack");
                   end
                   if(viewColissionHand == true)
                        disp("There is a collision with the hand");                  
                   end
                   if(viewCollisionCupboard == true)
                        disp("There is a collision with the dishrack");
                   end
               
                   endEffector = self.robot.model.fkine(qMatrix(i, :));               
                   object_transform = endEffector * trotx(pi/2); % Transform for object pick-up
               
               % Plot the robot moving
               if self.signal == true
                   self.robot.model.animate(qMatrix(i, :));
                   
                   % Transform Object to this Pose
                   objTransformVertices = [objVertices,ones(size(objVertices,1),1)] ...
                       * object_transform';
                   set(objMesh_h, 'Vertices', objTransformVertices(:,1:3));
               end
               
               viewHandinWorkspace = LightCurtainTrigger(self.togglehand,1); % Check when hand is in the light curtain area
               if(viewHandinWorkspace == true) % If the hand is in the workspace trigger the estop
                    self.estop = true;
                    self.signal = false;
                    disp('Detection at light curtain. Aborting operation.')
               end
               
               % If 'translateBoy' boolean is active and boy is translating 
               % inside the region, we no longer want the boy to be translating.
               if self.translateBoy == 1 && self.boyTranslationDir == "+x"
                    self.translateBoy = 0;
               end
               
               % When the estop is active it is locked in a while loop that
               % moves its actions back in the frame
               if self.estop == true
                   i = i-2; 
               end
               while (self.estop == true || self.signal == false)
                  % If the condition for the hand motion is true, translate
                  % the hand
                   if self.togglehand == true
                       self.handmotion;
                   end
                   pause(0.2);
               end
               
                  % If the condition for the hand motion is true, translate
                  % the hand
               if self.togglehand == true
                   self.handmotion;
               end   
               
               drawnow();
            end
            
            qOut = qMatrix(end,:);
        end
 %% RMRC
         function [qOut,qMatrix] = ResolvedMotionRateControl(self,startTr,endTr,time,deltaT,plotPath,plotData)

            % If there is no input for 'plotTrail' value input, set 'plotTrail' to 0 (do not ...
            % show path trail).
            if nargin < 5
                plotPath = 0;
            end

            % If there is no input for 'plotData' value input, set 'plotData' to 0 (do not ...
            % show plot data).
            if nargin < 6
                plotData = 0;
            end
            
             % For collision avoidance
            [dishwasher_translation, dishrack_translation,cupboard_translation, dishwasher_centerpoint, dishwasher_width, ...
            dishwasher_height, dishwasher_depth, dishrack_centerpoint, dishrack_width, ...
            dishrack_height, dishrack_depth, cupboard_centerpoint, cupboard_width, ...
            cupboard_height, cupboard_depth] = self.spawnCollisionBlocks(0);

            [hand_centerpoint, hand_width, hand_height, hand_depth] = PLY_Obstacle_Dimensions(1,0);
            
            % Set parameters for simulation
            steps = time/deltaT;   % No. of steps for simulation (the more steps the better as there are more sample points on the sinusoid)
            delta = 2*pi/steps; % Small angle change
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares (value less than or equal to 0.1 is preferable)
            lambda_factor = 0.075;
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix at 1 1 1 for angular velocities to avoid errors. Usually set to 0.1 0.1 0.1 for angular velocities
 
            % Allocate array data
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,self.robot.model.n);       % Array for joint anglesR
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

            % 1.4) Track the trajectory with RMRC (error = desired - actual)
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
                    lambda = (1 - m(i)/epsilon)* lambda_factor;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda * eye(self.robot.model.n))* J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';
               
                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:self.robot.model.n                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < self.robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0;                                                             % Stop the motor
                        disp('The motors have stopped.');
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > self.robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0;                                                             % Stop the motor
                        disp('The motors have stopped.');
                    end
                end
                
            qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot(i,:);                         	% Update next joint state based on joint velocities
            positionError(:,i) = TMatrix(1:3,4,i+1) - Tcurrent(1:3,4);                               % For plotting
            angleError(:,i) = deltaTheta;                                           % For plotting
           
            % Animate robot through joint states
            updatedQ = qMatrix(i+1,:);
            self.robot.model.animate(updatedQ)
            drawnow();
            end
        end
 %% Function to Move Hand   
            function handmotion(self)
            if self.handtransform == "+y"
                self.handtransl = self.handtransl * transl(0, -0.005, 0);
                %self.handtransl = self.handtransl;
                boyTransformVertices = [self.boyVertices,ones(size(self.boyVertices,1),1)] ...
                    * self.handtransl';
                set(self.boyMesh_h, 'Vertices', boyTransformVertices(:,1:3));
                drawnow();
                
                self.boyTranslation = self.handtransl(1:3, 4)';  % Translation of Boy as a Row Vector
            end
            
            % IF -X, MOVE BOY IN GLOBAL NEGATIVE X (POSITIVE Y IN BOY
            % REFERENCE FRAME)
            if self.handtransform == "-y"
                if self.boyTranslation(1) > -0.55      
                    self.handtransl = self.handtransl*transl(0, 0.005, 0);
                    %self.handtransl = self.handtransl;
                    boyTransformVertices = [self.boyVertices,ones(size(self.boyVertices,1),1)] ...
                        * self.handtransl';
                    set(self.boyMesh_h, 'Vertices', boyTransformVertices(:,1:3));
                    drawnow();

                    self.boyTranslation = self.handtransl(1:3, 4)';  % Translation of Boy as a Row Vector
                end
            end            
        end
%% Create collision boxes in the environment 
        % Create collision boxes in the environment for the dishwasher, cupboard, and dishrack
        function [dishwasher_translation, cupboard_translation, dishrack_translation, dishwasher_centerpoint, dishwasher_width, dishwasher_depth, dishwasher_height, cupboard_centerpoint, cupboard_width, cupboard_depth, cupboard_height, ...
                  dishrack_centerpoint, dishrack_width, dishrack_depth, dishrack_height] = spawnCollisionBlocks(self, spawn) 
            
            plotOptions.plotFaces = true;
            switch spawn
                case 0 
                    %Sets up rectangular prisms for collision
                    dishwasher_translation = [0,0,0];    
                    dishrack_translation = [0,0,1];
                    cupboard_translation = [0,0,0];
                               
                    [dishwasher_centerpoint, dishwasher_width, dishwasher_depth, dishwasher_height] = CollidablePLYs(1,0);        
                    [dishwasher_vertex,dishwasher_faces,dishwasher_faceNormals] = RealRectangularPrism( dishwasher_translation,dishwasher_centerpoint,dishwasher_width, dishwasher_height ,dishwasher_depth, plotOptions);
                    axis equal

                    [dishrack_centerpoint, dishrack_width, dishrack_depth, dishrack_height] = CollidablePLYs(2,0);
                    [dishrack_vertex,dishrack_faces,dishrack_faceNormals] = RealRectangularPrism(dishrack_translation, dishrack_centerpoint, dishrack_width, dishrack_height ,dishrack_depth, plotOptions);
                    axis equal
                    
                    [cupboard_centerpoint, cupboard_width, cupboard_depth, cupboard_height] = CollidablePLYs(3,0);
                    [cupboard_vertex,cupboard_faces,cupboard_faceNormals] = RealRectangularPrism(cupboard_translation, cupboard_centerpoint, cupboard_width, cupboard_height, cupboard_depth,plotOptions);
                    axis equal
            end
        end
    end
end