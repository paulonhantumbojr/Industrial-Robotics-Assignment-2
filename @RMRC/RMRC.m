classdef RMRC < handle
    % control contains functions and parameters to control robot movements
    
    properties
        %> Robot
        robot; 
        
        %> Object to be picked up
        object;
        
        %> Joint trajectory
        qMatrix; 
        
        %> Swaps between RMRC (true) and inverse kinematics (false)
        resolvedMotionRateControl; 
    end
    
    methods 
        
        function self = RMRC(robot,resolvedMotionRateControl)         
            self.robot = robot;
            self.resolvedMotionRateControl = resolvedMotionRateControl;        
        end
        
        function MoveRobot (self)
            % Moves robot with RMRC
            self.Follow();
        end
        
        function Follow(self)
            
            if self.resolvedMotionRateControl == true           
                % Apply RMRC
                self.ResolvedMotionRateControl(startTr,endTr,time,deltaT);
                
            elseif self.resolvedMotionRateControl == false
                disp('No motion');
            end
            
        end
        
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

            % 1.4) Track the trajectory with RMRC (error = desired - actual)
            for i = 1:steps-1
                % Solve for the desired linear and angular velocities
                Tcurrent = self.robot.model.fkine(self.qMatrix(i,:));                    % Get forward transformation at current joint state (as the joint angles obtained from ikcon are not always accurate)
                deltaX = TMatrix(1:3,4,i+1) - Tcurrent(1:3,4);                          % x are the position sets and T represents the actual position that the robot is in                  
                Rd = rpy2r(theta(i+1,1),theta(i+1,2),theta(i+1,3));                     % Get next RPY angles, convert to rotation matrix
                Ra = Tcurrent(1:3,1:3);                                                 % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric of roll, pitch, and yaw velocities. (Used as the derivative of the rotation matrix is equal to the negative derivative of the transpose of the rotation matrix)
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(abs(Rd*Ra'));                                       % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    
                % Solve for the Jacobian
                J = self.robot.model.jacob0(self.qMatrix(i,:));                         % Get Jacobian at current joint state
                m(i) = sqrt(abs(det(J*J')));                                            % Manipulability formula
                if m(i) < epsilon                                                       % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                
            invJ = inv(J'*J + lambda * eye(self.robot.model.n))* J';                                   % Pseudo Inverse of the Jacobian DLS
            qdot(i,:) = (invJ*xdot)';    % Inverse kinematics of velocities (RMRC equation)                                           % Solve the RMRC equation (you may need to transpose the vector)
            
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
            updatedQ = self.qMatrix(i+1,:);
            self.robot.model.animate(updatedQ)
            drawnow();          
            end
        end
    end
end