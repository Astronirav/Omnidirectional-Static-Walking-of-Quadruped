function [final_trajectory] = T_Trajectory2(Hb, Fc, no_steps)

% This function provides the way-points for transition trajectory

Ls = 0.14;
Hb = 0.09;
Fc = 0.07;
no_steps = 100;
gait = [0 0.5 0.75 0.25 0.75];
crawl = 1;
          
y_traj = LYTrajectory(gait, Fc, Ls, Hb, no_steps, crawl);

y_traj(:,1,:) = 0.15;


beta = 0.75;
gait_1 = 0;


%**************************************************************************
%Z Position Calculation

 z_pos = zeros(25,1);

for i = 1:25
   z_pos(i,1) =  Fc*(1-cos( 2*pi*((i-no_steps*beta)-no_steps*gait_1)/(no_steps*(1-beta)) ))/2-Hb ;
    
end


sc = 0.15; % side clearence

xi_1 = 0.15;
xf_1 = 0.0246;

yi_1 = -0.07;
yf_1 = 0.15;

xi_2 = 0.15;
xf_2 = 0.07;

yi_2 = 0.0246;
yf_2 = 0.15;

xi_3 = 0.15;
xf_3 = 0.07;

yi_3 = 0.07;
yf_3 = -0.15;

xi_4 = 0.15;
xf_4 = -0.0227;

yi_4 = -0.0227;
yf_4 = -0.15;



    
final_trajectory = zeros(4,3,175);

final_trajectory (:,3,:) = -Hb;

final_trajectory(:,:,1:75) = y_traj(:,:,1:75);

%**************************************************************************
%Leg 1    
%final_trajectory (1,1,76:100) = linspace(xi_1,xf_1,25); % Xfinal, Xnew
%final_trajectory (1,1,101:175) = xf_1;

%final_trajectory (1,2,76:100) = linspace(yi_1,yf_1,25);  %Yfinal, Ynew
%final_trajectory (1,2,101:175) = yf_1;

%final_trajectory (1,3,76:100) = z_pos;
%**************************************************************************
%Leg 2
%final_trajectory (2,1,76:100) = xi_2;
%final_trajectory (2,1,101:125) = linspace(xi_2,xf_2,25); % Xfinal, Xnew
%final_trajectory (2,1,126:175) = xf_2;

%final_trajectory (2,2,76:100) = yi_2;
%final_trajectory (2,2,101:125) = linspace(yi_2,yf_2,25);  %Yfinal, Ynew
%final_trajectory (2,2,126:175) = yf_2;

%final_trajectory (2,3,101:125) = z_pos;
%**************************************************************************
%Leg 3
%final_trajectory (3,1,76:125) = xi_3;
%final_trajectory (3,1,126:150) = linspace(xi_3,xf_3,25); % Xfinal, Xnew
%final_trajectory (3,1,151:175) = xf_3;

%final_trajectory (3,2,76:125) = yi_3;
%final_trajectory (3,2,126:150) = linspace(yi_3,yf_3,25);  %Yfinal, Ynew
%final_trajectory (3,2,151:175) = yf_3;

%final_trajectory (3,3,126:150) = z_pos;
%**************************************************************************
%Leg 4
%final_trajectory (4,1,76:150) = xi_4;
%final_trajectory (4,1,151:175) = linspace(xi_4,xf_4,25); % Xfinal, Xnew

%final_trajectory (4,2,76:150) = yi_4;
%final_trajectory (4,2,151:175) = linspace(yi_4,yf_4,25);  %Yfinal, Ynew

%final_trajectory (4,3,151:175) = z_pos;
%**************************************************************************
%Leg 1
final_trajectory (1,1,76:100) = linspace(xi_1,xf_1,25); % Xfinal, Xnew
final_trajectory (1,1,101:175) = xf_1;

final_trajectory (1,2,76:100) = linspace(yi_1,yf_1,25);  %Yfinal, Ynew
final_trajectory (1,2,101:175) = yf_1;

final_trajectory (1,3,76:100) = z_pos;
%**************************************************************************
%Leg 2
final_trajectory (4,1,76:100) = xi_4;
final_trajectory (4,1,101:125) = linspace(xi_4,xf_4,25); % Xfinal, Xnew
final_trajectory (4,1,126:175) = xf_4;

final_trajectory (4,2,76:100) = yi_4;
final_trajectory (4,2,101:125) = linspace(yi_4,yf_4,25);  %Yfinal, Ynew
final_trajectory (4,2,126:175) = yf_4;

final_trajectory (4,3,101:125) = z_pos;
%**************************************************************************
%Leg 4
final_trajectory (3,1,76:125) = xi_3;
final_trajectory (3,1,126:150) = linspace(xi_3,xf_3,25); % Xfinal, Xnew
final_trajectory (3,1,151:175) = xf_3;

final_trajectory (3,2,76:125) = yi_3;
final_trajectory (3,2,126:150) = linspace(yi_3,yf_3,25);  %Yfinal, Ynew
final_trajectory (3,2,151:175) = yf_3;

final_trajectory (3,3,126:150) = z_pos;
%**************************************************************************
%Leg 3
final_trajectory (2,1,76:150) = xi_2;
final_trajectory (2,1,151:175) = linspace(xi_2,xf_2,25); % Xfinal, Xnew

final_trajectory (2,2,76:150) = yi_2;
final_trajectory (2,2,151:175) = linspace(yi_2,yf_2,25);  %Yfinal, Ynew

final_trajectory (2,3,151:175) = z_pos;
%**************************************************************************
end