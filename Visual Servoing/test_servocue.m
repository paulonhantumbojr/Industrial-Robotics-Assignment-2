%% Test Visual Servoing with Visual cue on Panda Robot
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
 
  % Plot the Panda
 pandaBaseTransform = [-0.5,-1.3,0.97875]; % Panda robot Base position
 myPanda = Panda(pandaBaseTransform);
 myPanda.PlotAndColourPandaRobot()

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