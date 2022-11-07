%% Test collisions
 clc;
 clf;
 clear;
 close all;
 set(0, 'DefaultFigureWindowStyle', 'docked'); 
%% Environment
 kitchen = Kitchen(0); 
 kitchen.Spawnkitchen(); 
 
 % Green Plate
 plate1_h = PlaceObject('plate.PLY'); 
 plate1_verts = get(plate1_h, 'Vertices');
 
 plate1_tverts = [plate1_verts, ones(size(plate1_verts, 1),1)] * transl(0.75,-1.525,0.775)';
 set(plate1_h ,'Vertices',plate1_tverts(:,1:3)) 
 
 % Black Plate
 plate2_h = PlaceObject('plateblack.ply'); 
 plate2_verts = get(plate2_h, 'Vertices');
 
 plate2_tverts = [plate2_verts, ones(size(plate2_verts, 1),1)] * transl(0.75,-1.45,0.775)';
 set(plate2_h ,'Vertices',plate2_tverts(:,1:3)) 
 
 % Hand
  hand_h = PlaceObject('hand.PLY'); 
  hand_verts = get(hand_h, 'Vertices');
 
  hand_tverts = [hand_verts, ones(size(hand_verts, 1),1)] * transl(-0.7,-1.5,-1.0)' * trotz(pi) * trotx(pi); %(0.85,-1.5,1.15)
  set(hand_h ,'Vertices',hand_tverts(:,1:3)) 
%% Robot
  % UR3
  ur3BaseTransform = transl(0.8,-1.85,1.0) * trotz(pi);
  a = NewUR3(ur3BaseTransform); 
  a.PlotAndColourUR3()

  % Panda
  pandaBaseTransform = transl(-0.15, -1.45, 0.995);
  myPanda = NewPanda(pandaBaseTransform);
  myPanda.PlotAndColourPanda()

  centerpnt = [0.75,-1.5,1.15];
  side = 0.25;
  plotOptions.plotFaces = false;
  [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
  axis equal
  camlight
  
 view(170,15) % Change view
 
% Creating transform for every joint  
q = a.model.getpos();
tr = zeros(4,4,a.model.n+1);
tr(:,:,1) = a.model.base;
L = a.model.links;
for i = 1 : a.model.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            disp('Collision');
        end
    end    
end

% Go through until there are no step sizes larger than 1 degree
q1 = a.model.getpos();
q2 = deg2rad([81.5 -12 50 51 90 0]);

steps = 2;

while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

% Check each of the joint states in the trajectory to work out which ones are in collision. 
% Return a logical vector of size steps which contains 0 = no collision (safe) 
% and 1 = yes collision (Unsafe).

result = true(steps,1);
for i = 1: steps
    fprintf("Step: %d\n", i)
    result(i) = IsCollision(a,qMatrix(i,:),faces,vertex,faceNormals,false);
    a.model.animate(qMatrix(i,:));
    drawnow();
    pause(0.02);
    if result(i) == true
         disp('UNSAFE: Object detected. Robot stopped')
         break
     end
end
%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

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
%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(a,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), a);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                disp('Collision');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, a)

links = a.model.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = a.model.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end