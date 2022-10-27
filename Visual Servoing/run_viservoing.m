%% Test Visual Servoing
close all;
clf
set(0, 'DefaultFigureWindowStyle', 'docked'); 
%% Load the environment in the world
 kitchen = Kitchen(0); % Plot the environment
 elements = kitchen.Spawnkitchen(); % Plot the environment objects
 %% Load Robots
 % Make a UR3
 ur3BaseTransform = [0.85,-2.125,1.0]; % Set UR3 Base
 myUR3 = ModifiedUR3(ur3BaseTransform);  
 myUR3.PlotAndColourUR3Robot()
%  ur3_q0 = zeros(1, myUR3.model.n);
 
%  % Generate first finger at the end-effector of the linear UR3
%  a = IQFinger1;
%  iq1_transform = troty(-pi/2) * transl(0.065,-0.0115,0);
%  a.model.base = a.model.base * myUR3.model.fkine(ur3_q0) * iq1_transform; % Forward kinematics for end-effector base
%  a.GetAndColourFingerIQ1(); % Plotting object from Finger 1 class
%  
%  % Generate second finger at the end-effector of the linear UR3
%  b = IQFinger2;
%  iq2_transform = trotz(pi) * troty(-pi/2) * transl(0.065,-0.0115,0);
%  b.model.base = b.model.base * myUR3.model.fkine(ur3_q0) * iq2_transform;  % Forward kinematics for end-effector base
%  b.GetAndColourFingerIQ2(); % Plotting object from Finger 2 class
 
  % Plot the Panda
 pandaBaseTransform = [-0.5,-1.3,0.97875]; % Panda robot Base position
 myPanda = Panda(pandaBaseTransform);
 myPanda.PlotAndColourPandaRobot()
%  panda_q0 = zeros(1, myPanda.model.n);
  
%  % Generate first finger at the end-effector of the panda
%  d = FingerPanda1;
%  finger1_transform = transl(0,-0.1,0.015) * trotz(-pi/2) * trotx(pi/2);
%  d.model.base = d.model.base * myPanda.model.fkine(panda_q0) * finger1_transform; % Forward kinematics for end-effector base
%  d.GetAndColourFingerPanda1(); % Plotting object from Panda Finger 1 class % -0.155
%  
%  % Generate second finger at the end-effector of the panda
%  e = FingerPanda2;
%  finger2_transform = transl(-0.15,-0.1,-0.015) * trotz(-pi/2) * trotx(pi/2) * trotx(pi);
%  e.model.base = e.model.base * myPanda.model.fkine(panda_q0) * finger2_transform; % Forward kinematics for end-effector base
%  e.GetAndColourFingerPanda2(); % Plotting object from Panda Finger 2 class
 
 view(170,15);
 %% Initiate Visual Servoing
 
            % Create 3D points
            guide_q0 = deg2rad([-18;74;0;0;0;-72.4;0]);
            myPanda.model.animate(guide_q0');
            eePoints = myPanda.model.fkine(guide_q0);
            eePoints = eePoints(1:3,4);
            P = [eePoints(1),eePoints(1),eePoints(1),eePoints(1);
                eePoints(2)-0.1,eePoints(2)+0.1,eePoints(2)+0.1,eePoints(2)-0.1;
                eePoints(3)+0.1,eePoints(3)+0.1,eePoints(3)-0.1,eePoints(3)-0.1];

            % Frame rate
            fps = 25;
            
            % Controller gain
            lambda = 0.8;
            
            %depth of the IBVS
            depth = mean (P(1,:));
            
            % Define Camera settings     
            name = 'UR3Cam';
            cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
            'resolution', [1024 1024], 'centre', [512 512],'name', name); 

            % Assign the location of the end-effector to the camera 
            follow_q0 = deg2rad([80;-50;10;-45;0;0]);
            Tc0 = myUR3.model.fkine(follow_q0);
            myUR3.model.animate(follow_q0');
%             a.model.animate(myUR3.model.fkine(myUR3.model.getpos));
%             b.model.animate(myUR3.model.fkine(myUR3.model.getpos));
            drawnow();
            cam.T = Tc0; 
            
            % Plot the camera at the end-effector
            cam.plot_camera('Tcam', Tc0, 'label','scale',0.075);
%             lighting gouraud
%             light
            
            plot_sphere(P, 0.05, 'r');

            pStar = bsxfun(@plus, 200*[-1 -1 1 1; -1 1 1 -1], cam.pp');
            %% Initialising simulation
            p = cam.plot(P, 'Tcam', Tc0);
            % show ref location, wanted view when Tc = Tct_star
            cam.clf()
            cam.plot(pStar, '*'); % create the camera view
            cam.hold(true);
            cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
            pause(2)
            cam.hold(true);

            cam.plot(P); % show initial view
            %% Move Panda with RMRC
            rmrc = true;
            time = 5;
            deltaT = 0.02;
            pandaT1 = transl(eePoints(1), eePoints(2), eePoints(3)) * trotx(pi/2);
            pandaT2 = transl(eePoints(1), eePoints(2), eePoints(3)+0.2) * trotx(pi/2);
            pandacontrol = RMRC(myPanda, rmrc);
            pandacontrol.ResolvedMotionRateControl(pandaT1,pandaT2,time,deltaT);  % towards plate
            %%
            ksteps = 0;
            while true
                ksteps = ksteps + 1;
                Zest = [];

                % comute the view
                uv = cam.plot(P);

                % compute image plane error as a column
                e = uv - pStar; % feature error
                e = e(:);

                % compute Jacobian
                if isempty(depth)
                    % exact depth from simulation
                    pt = homtrans(inv(Tcam), P);
                    J = cam.visjac_p(uv, pt(3,:));
                elseif ~isempty(Zest)
                    J = cam.visjac_p(uv, Zest);
                else
                    J = cam.visjac_p(uv, depth);
                end

                % compute velocity of camera in camera frame
                try
                    v = -lambda * pinv(J) * e;
                catch
                    status = -1;
                    return
                end
                fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

                % ROBOT MOVEMENT
                J2 = myUR3.model.jacobn(follow_q0); % jacobian for robot in pose q0
                Jinv = pinv(J2);
                qp = Jinv*v; % joint velocities
                % V = dx, dy, dz, dRx, dRy, dRz

                ind=find(qp>deg2rad(320));
                if ~isempty(ind)
                    qp(ind)=deg2rad(320);
                end
                ind=find(qp<-deg2rad(320));
                if ~isempty(ind)
                    qp(ind)=-deg2rad(320);
                end
                q = follow_q0 + (1/fps)*qp;

                myUR3.model.animate(q');

                % compute new camera pose
                Tcam = myUR3.model.fkine(q);
                % update camera pose
                cam.T = Tcam;

                drawnow
                pause (1/fps)
                if ~isempty(200) && (ksteps > 50)
                    break;
                end
                follow_q0 = q; % update current joint position
            end   