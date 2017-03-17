%% OMNIDIRECTIONAL STATIC WALKING OF QUADRUPED
% TRAJECTORY PLANNING OF OMNIDIRECTIONAL WALKING OF QUADRUPED BY MINIMIZING
% THE TRANSITION TIME BETWEEN DIFFERENT GAITS

% Summer Internship Project, IIT Jodhpur (2016)
% Author: Nirav Savaliya
% E-mail: niravjwst@gmail.com | GitHub: @Astronirav


%% Defining Basic Variables and Transformations
clc;
clear;
global BodyDim;  %%Dimensions of Body
global LinkLengths;  %% Link lengths of Leg
global DHParam;  %% DH Parameters of Legs
global G2BTmat;  %% Ground to Body Conter Transformation Matrix
global BC2LTmat; %% Body Center to Leg Transformation Matrix

BodyDim = [0.2895 0.2895]; %% Dimensions: X Y (Width Length)
LinkLengths = [0.047; 0.090; 0.10];
DHParam = [[0;0;0], LinkLengths, [90;0;0]]; % DH Parameters d a alpha

%**************************************************************************
% Body Center to Leg Transformation Matrix
% Body Frame is located at the center of the Body, equidistant...
% from X and Y Dimensions. 
% Transformation is used to move frame to the corners of quadruped...
% where all legs are attached. 
% Frame 1 and 3 have -1 because of the change in direction (-X) relative to
% body frame. Similarly BodyDim(X&Y) are used with appropriate signs. 

BC2LTmat = zeros(4,4,4);
BC2LTmat(:,:,1) = [-1 0 0 -BodyDim(1)/2; 0 1 0 BodyDim(2)/2; 0 0 1 0; 0 0 0 1];
BC2LTmat(:,:,2) = [1 0 0 BodyDim(1)/2; 0 1 0 BodyDim(2)/2; 0 0 1 0; 0 0 0 1];
BC2LTmat(:,:,3) = [-1 0 0 -BodyDim(1)/2; 0 1 0 -BodyDim(2)/2; 0 0 1 0; 0 0 0 1];
BC2LTmat(:,:,4) = [1 0 0 BodyDim(1)/2; 0 1 0 -BodyDim(2)/2; 0 0 1 0; 0 0 0 1];



%% Parameters of Quadruped and Type of Gaits 
%**************************************************************************
% LOOK UP TABLE for Different Gaits
% [phi4 phi3 phi1 phi2 beta]
% phi = Phase of Legs. Which leg will move at what (normalized) time

LookUp = [    0 0.5 0.75 0.25 0.75;... % Crawl Gait Beta 0.75
              0.9 0.4 0.7 0.2 0.8;...  % Crawl Gait Beta 0.8  
              0.5 0 0.5 0 0.5;...      % Pace Gait
              0 0 0.5 0.5 0.5];        % Gallop Gait
%Gaits:
% 1 - Crawl (beta 0.0.75)
% 2 - Crawl (beta 0.8)
% 3 - Pace
% 4 - Gallop
gait = 1;
%**************************************************************************

%**************************************************************************
CT = 4; %Cycle Time
Fc = 0.07; %foot clearance or foot hieght to be lifted
Hb = 0.09; %hieght of body;
Ls = 0.14; %step length
sc = 0.15; % Side Clearence 
%**************************************************************************
G2BTmat = eye(4);
G2BTmat(3,4) = Hb;
%**************************************************************************
% Velocity of Quadruped
% V = Step Length/Cycle Time
velocity = Ls/(LookUp(gait,5)*CT);
display(velocity);
%**************************************************************************
%% Generating Gait Specific Trajectory 

no_steps = 100; % Number of Steps (Computation)

% CRAWL:
% 1 for Y Crawl
% 2 for X Crawl
crawl = 1;

% There will be 2 types of trajectories; Linear and Rotation. 
% LTrajectory will generate set of points (=no_steps) in space for ...
% linear motion.
% Output matrix is 4x3x100 where each matrix contains x, y and z position
% for all the 4 legs for all 100 interations.

configs_exp = zeros(4,3);
tyx_configs = zeros(4,3,4);
txy_configs = zeros(4,3,4);
x_configs = zeros(4,3,4);
y_configs = zeros(4,3,4);
y_stm = zeros(1,100);
x_stm = zeros(1,100);
t_stm = zeros(1,100);

y_cycles = 1;
x_cycles = 2;

y_cont = zeros(4,3,4);  %Continuously move in Y direction for no_cycles times
y_cont(:,2,:) = 1;

x_cont = zeros(4,3,4);  %Continuously move in Y direction for no_cycles times
x_cont(:,1,:) = 1;

for j = 1:y_cycles
    
LinearY_Trajectory = LYTrajectory(LookUp(gait,:), Fc, Ls, Hb, no_steps,crawl);



for i=1:no_steps
    
    ly_footpos = [  0.15 LinearY_Trajectory(1,2,i) LinearY_Trajectory(1,3,i);...
                    0.15 LinearY_Trajectory(2,2,i) LinearY_Trajectory(2,3,i);...
                    0.15 LinearY_Trajectory(3,2,i) LinearY_Trajectory(3,3,i);...
                    0.15 LinearY_Trajectory(4,2,i) LinearY_Trajectory(4,3,i)];
                
   G2BTmat(2,4) =  ((i*Ls)/(LookUp(gait,5)*no_steps));
   
   
   
   leg_Yangles = inverseKinematics(ly_footpos);
   y_positions = fwdkinematics(leg_Yangles(:,:,2));
   configs_exp(:,:,i) = ly_footpos;      % Experimental Matrix to check z coordinates
   
   world_Ypos = local2world(y_positions);
   y_configs(:,:,:,i) = world_Ypos + ((j-1)*velocity*CT*y_cont);
   
end


for i = 1:no_steps
    y_stm(i) = draw(y_configs(:,:,:,i));
    pause(0.0005);
   
end

end


%TRANSITION Y-X :::: Not Completed Yet

% Transition_Trajectory = T_Trajectory(Hb, Fc, no_steps, order, LinearY_Trajectory);


Transition_Trajectory = T_Trajectory2(Hb, Fc, no_steps); 
% Also add Final position Matrix and make new positions matrix from Final
% Positions


%DEFINE X POS AND Y POS OF FEET IN POSITION MATRIX

for i=1:175
    
    t_footpos = [   Transition_Trajectory(1,1,i) Transition_Trajectory(1,2,i) Transition_Trajectory(1,3,i);...
                    Transition_Trajectory(2,1,i) Transition_Trajectory(2,2,i) Transition_Trajectory(2,3,i);...
                    Transition_Trajectory(3,1,i) Transition_Trajectory(3,2,i) Transition_Trajectory(3,3,i);...
                    Transition_Trajectory(4,1,i) Transition_Trajectory(4,2,i) Transition_Trajectory(4,3,i)];
   if i<= 75
        G2BTmat(2,4) =  ((i*Ls)/(LookUp(gait,5)*no_steps)); % CHANGE: G2BTmat(2,4) to G2BTmat(1,4)
   end
   %y_cont = zeros(4,3,4);  %Continuously move in Y direction for no_cycles times
   %y_cont(:,2,:) = 1;
   
   t_leg_angles = inverseKinematics(t_footpos);
   t_positions = fwdkinematics(t_leg_angles(:,:,2));
   t_world_pos = local2world(t_positions);
   tyx_configs(:,:,:,i) = t_world_pos + ((y_cycles)*velocity*CT*y_cont);
   
end


for i = 1:175
    t_stm(i) = draw(tyx_configs(:,:,:,i));
    pause(0.0005);
   
end

%% Crawling in X Direction
% X Crawl

X_Trajectory = zeros(4,3,no_steps);

for j = 1:x_cycles
    
LinearX_Trajectory = LXTrajectory(LookUp(gait,:), Fc, Ls, Hb, no_steps, sc);



for i=1:no_steps
    
    lx_footpos = [  -LinearX_Trajectory(1,1,i)  0.15 LinearX_Trajectory(1,3,i);...
                     LinearX_Trajectory(2,1,i)  0.15 LinearX_Trajectory(2,3,i);...
                    -LinearX_Trajectory(3,1,i) -0.15 LinearX_Trajectory(3,3,i);...
                     LinearX_Trajectory(4,1,i) -0.15 LinearX_Trajectory(4,3,i)];
   
   X_Trajectory(:,:,i) = lx_footpos;              
   
   G2BTmat(1,4) =  ((i*Ls)/(LookUp(gait,5)*no_steps));
   
   y_cont = zeros(4,3,4);  %Continuously move in Y direction for no_cycles times
   y_cont(:,2,:) = 1;
   
   leg_Xangles = inverseKinematics(lx_footpos);
   x_positions = fwdkinematics(leg_Xangles(:,:,2));
   configs_exp(:,:,i) = lx_footpos;      % Experimental Matrix to check z coordinates
   
   world_Xpos = local2world(x_positions);
   x_configs(:,:,:,i) =  world_Xpos + ((j-1)*velocity*CT*x_cont) + ((y_cycles)*velocity*CT*y_cont);
   
end


for i = 1:no_steps
    x_stm(i) = draw(x_configs(:,:,:,i));
    pause(0.0005);
   
end

end

%order = 2;
%Transition_Trajectory = T_Trajectory(Hb, Fc, no_steps, order); 
% Also add Final position Matrix and make new positions matrix from Final
% Positions


%DEFINE X POS AND Y POS OF FEET IN POSITION MATRIX

Transition_Trajectory2 = TXY_Trajectory2(Hb, Fc, no_steps); 
% Also add Final position Matrix and make new positions matrix from Final
% Positions


%DEFINE X POS AND Y POS OF FEET IN POSITION MATRIX

for i=1:150
    
    t_footpos = [   Transition_Trajectory2(1,1,i) Transition_Trajectory2(1,2,i) Transition_Trajectory2(1,3,i);...
                    Transition_Trajectory2(2,1,i) Transition_Trajectory2(2,2,i) Transition_Trajectory2(2,3,i);...
                    Transition_Trajectory2(3,1,i) Transition_Trajectory2(3,2,i) Transition_Trajectory2(3,3,i);...
                    Transition_Trajectory2(4,1,i) Transition_Trajectory2(4,2,i) Transition_Trajectory2(4,3,i)];
   if i<= 50
        G2BTmat(1,4) =  ((i*Ls)/(LookUp(gait,5)*no_steps)); % CHANGE: G2BTmat(2,4) to G2BTmat(1,4)
   end
   x_cont = zeros(4,3,4);  %Continuously move in Y direction for no_cycles times
   x_cont(:,1,:) = 1;
   
   t_leg_angles = inverseKinematics(t_footpos);
   t_positions = fwdkinematics(t_leg_angles(:,:,2));
   t_world_pos = local2world(t_positions);
   txy_configs(:,:,:,i) = t_world_pos + ((y_cycles)*velocity*CT*y_cont) + ((x_cycles)*velocity*CT*x_cont);
   
end


for i = 1:150
    t_stm(i) = draw(txy_configs(:,:,:,i));
    pause(0.0005);
   
end


% Rotation Transformations from center to all the 4 legs

% Distance between body center and initial rotation foot position
leg_1pos = (BC2LTmat(1:3,1:3,1)*[0.15; -0.014; 0])+[-BodyDim(1)/2; BodyDim(2)/2; 0];
leg_2pos = (BC2LTmat(1:3,1:3,2)*[0.15; -0.014; 0])+[BodyDim(1)/2; BodyDim(2)/2; 0];
leg_3pos = (BC2LTmat(1:3,1:3,3)*[0.15; 0.014; 0])+[-BodyDim(1)/2; -BodyDim(2)/2; 0];
leg_4pos = (BC2LTmat(1:3,1:3,4)*[0.15; 0.014; 0])+[BodyDim(1)/2; -BodyDim(2)/2; 0];

radius = sqrt(leg_1pos(1)^2+leg_1pos(2)^2);
% Radius of the circle on which rotation will take place in Body Center
% Frame

% rot_steps = Number of steps of Rotation.

rot_steps = 100;

%Angles of Legs on intersection of rotation trajectory and linear
%trajectory

leg_ang(1) = atan2d(leg_1pos(2),leg_1pos(1));
leg_ang(2) = atan2d(leg_2pos(2),leg_2pos(1));
leg_ang(3) = atan2d(leg_3pos(2),leg_3pos(1));
leg_ang(4) = atan2d(leg_4pos(2),leg_4pos(1));


%Sample angles in the span of 30 degrees including common point.
% for ground motion, therefore 75 steps
leg_angles_gnd = zeros(75,4);

leg_angles_gnd(:,1) = linspace(leg_ang(1)-20,leg_ang(1)+10,75)';
leg_angles_gnd(:,2) = linspace(leg_ang(2)-20,leg_ang(2)+10,75)';
leg_angles_gnd(:,3) = linspace(leg_ang(3)-10,leg_ang(3)+20,75)';
leg_angles_gnd(:,4) = linspace(leg_ang(4)-10,leg_ang(4)+20,75)';

% leg_angles matrix composition:
% 4 columns = 4 legs
% 50 rows for each steps
% [136,166] leg 1
% [3.9,33.9] leg 2
% [-166,-136] leg 3
% [-33.9,-3.9] leg 4


% Calculation of x,y position for 75 angles and radius R

rot_world_pos = zeros(75,3,4);

rot_world_pos(:,3,:) = -Hb;

for i=1:75
   
    rot_world_pos(i,1,1) = radius*cosd(leg_angles_gnd(i,1));
    rot_world_pos(i,2,1) = radius*sind(leg_angles_gnd(i,1));
    
    rot_world_pos(i,1,2) = radius*cosd(leg_angles_gnd(i,2));
    rot_world_pos(i,2,2) = radius*sind(leg_angles_gnd(i,2));
    
    rot_world_pos(i,1,3) = radius*cosd(leg_angles_gnd(i,3));
    rot_world_pos(i,2,3) = radius*sind(leg_angles_gnd(i,3));
    
    rot_world_pos(i,1,4) = radius*cosd(leg_angles_gnd(i,4));
    rot_world_pos(i,2,4) = radius*sind(leg_angles_gnd(i,4));
    
end

%Coordinate Transformation from Body Center frame to Leg frams of all 4
%legs

rot_pos = zeros(75,3,4);

for i = 1:75
    
    rot_pos(i,:,1) = (((BC2LTmat(1:3,1:3,1)')*(rot_world_pos(i,:,1))')+[-BodyDim(1)/2; -BodyDim(2)/2; 0])';
    rot_pos(i,:,2) = (((BC2LTmat(1:3,1:3,2)')*(rot_world_pos(i,:,2))')+[-BodyDim(1)/2; -BodyDim(2)/2; 0])';
    rot_pos(i,:,3) = (((BC2LTmat(1:3,1:3,3)')*(rot_world_pos(i,:,3))')+[-BodyDim(1)/2; BodyDim(2)/2; 0])';
    rot_pos(i,:,4) = (((BC2LTmat(1:3,1:3,4)')*(rot_world_pos(i,:,4))')+[-BodyDim(1)/2; BodyDim(2)/2; 0])';
    
end

r_data = zeros(4,3,no_steps);


for i = 1:75
    r_data(1,:,i) = rot_pos(i,:,1);
    r_data(2,:,i) = rot_pos(i,:,2);
    r_data(3,:,i) = rot_pos(i,:,3);
    r_data(4,:,i) = rot_pos(i,:,4);
end


%........................................................................
%% Leg Angles during swing

leg_angles_swing = zeros(25,4);

leg_angles_swing(:,1) = linspace(leg_ang(1)+10,leg_ang(1)-20,25)';
leg_angles_swing(:,2) = linspace(leg_ang(2)+10,leg_ang(2)-20,25)';
leg_angles_swing(:,3) = linspace(leg_ang(3)+20,leg_ang(3)-10,25)';
leg_angles_swing(:,4) = linspace(leg_ang(4)+20,leg_ang(4)-10,25)';

rot_world_pos_swing = zeros(25,3,4);

rot_world_pos_swing(:,3,:) = -Hb;

for i=1:25
   
    rot_world_pos_swing(i,1,1) = radius*cosd(leg_angles_swing(i,1));
    rot_world_pos_swing(i,2,1) = radius*sind(leg_angles_swing(i,1));
    
    rot_world_pos_swing(i,1,2) = radius*cosd(leg_angles_swing(i,2));
    rot_world_pos_swing(i,2,2) = radius*sind(leg_angles_swing(i,2));
    
    rot_world_pos_swing(i,1,3) = radius*cosd(leg_angles_swing(i,3));
    rot_world_pos_swing(i,2,3) = radius*sind(leg_angles_swing(i,3));
    
    rot_world_pos_swing(i,1,4) = radius*cosd(leg_angles_swing(i,4));
    rot_world_pos_swing(i,2,4) = radius*sind(leg_angles_swing(i,4));
    
end

%% Visulization of the whole algorithm on Stick-Model of Quadruped

%Coordinate Transformation from Body Center frame to Leg frams of all 4
%legs

rot_pos_swing = zeros(25,3,4);

for i = 1:25
    
    rot_pos_swing(i,:,1) = (((BC2LTmat(1:3,1:3,1)')*(rot_world_pos_swing(i,:,1))')+[-BodyDim(1)/2; -BodyDim(2)/2; 0])';
    rot_pos_swing(i,:,2) = (((BC2LTmat(1:3,1:3,2)')*(rot_world_pos_swing(i,:,2))')+[-BodyDim(1)/2; -BodyDim(2)/2; 0])';
    rot_pos_swing(i,:,3) = (((BC2LTmat(1:3,1:3,3)')*(rot_world_pos_swing(i,:,3))')+[-BodyDim(1)/2; BodyDim(2)/2; 0])';
    rot_pos_swing(i,:,4) = (((BC2LTmat(1:3,1:3,4)')*(rot_world_pos_swing(i,:,4))')+[-BodyDim(1)/2; BodyDim(2)/2; 0])';
    
end

for i = 76:100
    r_data(1,:,i) = rot_pos_swing(i-75,:,1);
    r_data(2,:,i) = rot_pos_swing(i-75,:,2);
    r_data(3,:,i) = rot_pos_swing(i-75,:,3);
    r_data(4,:,i) = rot_pos_swing(i-75,:,4);
end


for i = 76:100
    r_data(1,3,i) = (Fc*(1-cos( 2*pi*((i-no_steps*LookUp(gait,5))-no_steps*LookUp(gait,1))/(no_steps*(1-LookUp(gait,5))) ))/2-Hb) ;
    r_data(2,3,i) = (Fc*(1-cos( 2*pi*((i-no_steps*LookUp(gait,5))-no_steps*LookUp(gait,1))/(no_steps*(1-LookUp(gait,5))) ))/2-Hb) ;
    r_data(3,3,i) = (Fc*(1-cos( 2*pi*((i-no_steps*LookUp(gait,5))-no_steps*LookUp(gait,1))/(no_steps*(1-LookUp(gait,5))) ))/2-Hb) ;
    r_data(4,3,i) = (Fc*(1-cos( 2*pi*((i-no_steps*LookUp(gait,5))-no_steps*LookUp(gait,1))/(no_steps*(1-LookUp(gait,5))) ))/2-Hb) ;
end


Rotation_Trajectory(1,1,:) = circshift(r_data(1,1,:),[50,0]);
Rotation_Trajectory(2,1,:) = circshift(r_data(2,1,:),[25,0]);
Rotation_Trajectory(3,1,:) = circshift(r_data(3,1,:),[75,0]);
Rotation_Trajectory(4,1,:) = circshift(r_data(4,1,:),[0,0]);

Rotation_Trajectory(1,2,:) = circshift(r_data(1,2,:),[50,0]);
Rotation_Trajectory(2,2,:) = circshift(r_data(2,2,:),[25,0]);
Rotation_Trajectory(3,2,:) = circshift(r_data(3,2,:),[75,0]);
Rotation_Trajectory(4,2,:) = circshift(r_data(4,2,:),[0,0]);

Rotation_Trajectory(1,3,:) = circshift(r_data(1,3,:),[50,0]);
Rotation_Trajectory(2,3,:) = circshift(r_data(2,3,:),[25,0]);
Rotation_Trajectory(3,3,:) = circshift(r_data(3,3,:),[75,0]);
Rotation_Trajectory(4,3,:) = circshift(r_data(4,3,:),[0,0]);