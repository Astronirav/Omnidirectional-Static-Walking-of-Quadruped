function [final_trajectory] = TXY_Trajectory2(Hb, Fc, no_steps, order)

% This function provides transition trajectory for x to y transition

sc = 0.15;
Ls = 0.14;
Hb = 0.09;
Fc = 0.07;
no_steps = 100;
gait = [0 0.5 0.75 0.25 0.75];
crawl = 1;


pos_i =        [0.15   0.07    -0.090;
               0.15  -0.0227  -0.090;
               0.15   0.0246  -0.090;
               0.15  -0.07    -0.090];
           
pos_f =        [0.0246  0.15  -0.090;
                0.07    0.15  -0.090;
               -0.07    0.15  -0.090;
               -0.0227  0.15  -0.090];

x_traj = zeros(4,3,100);
           
x_traj = LXTrajectory(gait, Fc, Ls, Hb, no_steps, sc);

for i = 1:100

    lx_footpos = [  -x_traj(1,1,i)  0.15 x_traj(1,3,i);...
                     x_traj(2,1,i)  0.15 x_traj(2,3,i);...
                    -x_traj(3,1,i) -0.15 x_traj(3,3,i);...
                     x_traj(4,1,i) -0.15 x_traj(4,3,i)];
   
    x_traj(:,:,i) = lx_footpos;

end
           
xi_1 = x_traj(1,1,50);
xf_1 = sc;

yi_1 = x_traj(1,2,50);
yf_1 = 0.07;

xi_2 = x_traj(2,1,50);
xf_2 = sc;

yi_2 = x_traj(2,2,50);
yf_2 = -0.0227;

xi_3 = x_traj(3,1,50);
xf_3 = sc;

yi_3 = x_traj(3,2,50);
yf_3 = 0.0246;

xi_4 = x_traj(4,1,50);
xf_4 = sc;

yi_4 = x_traj(4,2,50);
yf_4 = -0.07;

tdata = zeros(4,3,100);
tdata(:,3,:) = -Hb;


beta = 0.75;
gait_1 = 0;


%**************************************************************************
%Z Position Calculation

 z_pos = zeros(25,1);

for i = 1:25
   z_pos(i,1) =  Fc*(1-cos( 2*pi*((i-no_steps*beta)-no_steps*gait_1)/(no_steps*(1-beta)) ))/2-Hb ;
    
end

    
final_trajectory = zeros(4,3,150);

final_trajectory (:,3,:) = -Hb;

final_trajectory(:,:,1:50) = x_traj(:,:,1:50);


%**************************************************************************
%Leg 1
%final_trajectory (1,1,51:75) = linspace(xi_1,xf_1,25); % Xfinal, Xnew
%final_trajectory (1,1,76:150) = xf_1;

%final_trajectory (1,2,51:75) = linspace(yi_1,yf_1,25);  %Yfinal, Ynew
%final_trajectory (1,2,76:150) = yf_1;

%final_trajectory (1,3,51:75) = z_pos;
%**************************************************************************
%Leg 2
%final_trajectory (2,1,51:75) = xi_2;
%final_trajectory (2,1,76:100) = linspace(xi_2,xf_2,25); % Xfinal, Xnew
%final_trajectory (2,1,100:150) = xf_2;

%final_trajectory (2,2,51:75) = yi_2;
%final_trajectory (2,2,76:100) = linspace(yi_2,yf_2,25);  %Yfinal, Ynew
%final_trajectory (2,2,100:150) = yf_2;

%final_trajectory (2,3,76:100) = z_pos;
%**************************************************************************
%Leg 3
%final_trajectory (3,1,51:100) = xi_3;
%final_trajectory (3,1,101:125) = linspace(xi_3,xf_3,25); % Xfinal, Xnew
%final_trajectory (3,1,126:150) = xf_3;

%final_trajectory (3,2,51:100) = yi_3;
%final_trajectory (3,2,101:125) = linspace(yi_3,yf_3,25);  %Yfinal, Ynew
%final_trajectory (3,2,126:150) = yf_3;

%final_trajectory (3,3,101:125) = z_pos;
%**************************************************************************
%Leg 4
%final_trajectory (4,1,51:125) = xi_4;
%final_trajectory (4,1,126:150) = linspace(xi_4,xf_4,25); % Xfinal, Xnew

%final_trajectory (4,2,51:125) = yi_4;
%inal_trajectory (4,2,126:150) = linspace(yi_4,yf_4,25);  %Yfinal, Ynew

%final_trajectory (4,3,126:150) = z_pos;
%**************************************************************************
%**************************************************************************
%**************************************************************************
%Leg 1
final_trajectory (1,1,51:75) = linspace(xi_1,xf_1,25); % Xfinal, Xnew
final_trajectory (1,1,76:150) = xf_1;

final_trajectory (1,2,51:75) = linspace(yi_1,yf_1,25);  %Yfinal, Ynew
final_trajectory (1,2,76:150) = yf_1;

final_trajectory (1,3,51:75) = z_pos;
%**************************************************************************
%Leg 2
final_trajectory (3,1,51:75) = xi_3;
final_trajectory (3,1,76:100) = linspace(xi_3,xf_3,25); % Xfinal, Xnew
final_trajectory (3,1,100:150) = xf_3;

final_trajectory (3,2,51:75) = yi_3;
final_trajectory (3,2,76:100) = linspace(yi_3,yf_3,25);  %Yfinal, Ynew
final_trajectory (3,2,100:150) = yf_3;

final_trajectory (3,3,76:100) = z_pos;
%**************************************************************************
%Leg 3
final_trajectory (2,1,51:100) = xi_2;
final_trajectory (2,1,101:125) = linspace(xi_2,xf_2,25); % Xfinal, Xnew
final_trajectory (2,1,126:150) = xf_2;

final_trajectory (2,2,51:100) = yi_2;
final_trajectory (2,2,101:125) = linspace(yi_2,yf_2,25);  %Yfinal, Ynew
final_trajectory (2,2,126:150) = yf_2;

final_trajectory (2,3,101:125) = z_pos;
%**************************************************************************
%Leg 4
final_trajectory (4,1,51:125) = xi_4;
final_trajectory (4,1,126:150) = linspace(xi_4,xf_4,25); % Xfinal, Xnew

final_trajectory (4,2,51:125) = yi_4;
final_trajectory (4,2,126:150) = linspace(yi_4,yf_4,25);  %Yfinal, Ynew

final_trajectory (4,3,126:150) = z_pos;

end