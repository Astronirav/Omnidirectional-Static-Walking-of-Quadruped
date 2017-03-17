
function [tdata] = T_Trajectory(Hb, Fc, no_steps, order)

% This function provides the way-points for transition trajectory

Hb = 0.09;
Fc = 0.07;
no_steps = 100;


pos_i =        [0.15   0.07    -0.090;
               0.15  -0.0227  -0.090;
               0.15   0.0246  -0.090;
               0.15  -0.07    -0.090];
           
pos_f =        [0.0246  0.15  -0.090;
                0.07    0.15  -0.090;
               -0.07    0.15  -0.090;
               -0.0227  0.15  -0.090];
           
Xfinal_1 = 0.15;
Xnew_1 = -(0.0246);
Yfinal_1 = 0.07;
Ynew_1 = 0.15;

Xfinal_2 = 0.15;
Xnew_2 = (0.07);
Yfinal_2 = -0.0227 ;
Ynew_2 = 0.15;

Xfinal_3 = 0.15;
Xnew_3 = 0.07;
Yfinal_3 = 0.0246;
Ynew_3 = -(0.15);

Xfinal_4 = 0.15;
Xnew_4 = -(0.0227);
Yfinal_4 = -0.07;
Ynew_4 = -(0.15);
           
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


if order == 1
    %**************************************************************************
    %Leg 1
    tdata (1,1,1:25) = linspace(Xfinal_1,Xnew_1,25); % Xfinal, Xnew
    tdata (1,1,26:100) = Xnew_1;

    tdata (1,2,1:25) = linspace(Yfinal_1,Ynew_1,25);  %Yfinal, Ynew
    tdata (1,2,26:100) = Ynew_1;

    tdata (1,3,1:25) = z_pos;
    %**************************************************************************
    %Leg 2
    tdata (2,1,1:25) = Xfinal_2;
    tdata (2,1,26:50) = linspace(Xfinal_2,Xnew_2,25); % Xfinal, Xnew
    tdata (2,1,51:100) = Xnew_2;

    tdata (2,2,1:25) = Yfinal_2;
    tdata (2,2,26:50) = linspace(Yfinal_2,Ynew_2,25);  %Yfinal, Ynew
    tdata (2,2,51:100) = Ynew_2;

    tdata (2,3,26:50) = z_pos;
    %**************************************************************************
    %Leg 3
    tdata (3,1,1:50) = Xfinal_3;
    tdata (3,1,51:75) = linspace(Xfinal_3,Xnew_3,25); % Xfinal, Xnew
    tdata (3,1,76:100) = Xnew_3;

    tdata (3,2,1:50) = Yfinal_3;
    tdata (3,2,51:75) = linspace(Yfinal_3,Ynew_3,25);  %Yfinal, Ynew
    tdata (3,2,76:100) = Ynew_3;

    tdata (3,3,51:75) = z_pos;
    %**************************************************************************
    %Leg 4
    tdata (4,1,1:75) = Xfinal_4;
    tdata (4,1,76:100) = linspace(Xfinal_4,Xnew_4,25); % Xfinal, Xnew

    tdata (4,2,1:75) = Yfinal_4;
    tdata (4,2,76:100) = linspace(Yfinal_4,Ynew_4,25);  %Yfinal, Ynew

    tdata (4,3,76:100) = z_pos;
%**************************************************************************
else
        %Leg 1
    tdata (1,1,1:25) = linspace(Xnew_1,Xfinal_1,25); % Xfinal, Xnew
    tdata (1,1,26:100) = Xfinal_1;

    tdata (1,2,1:25) = linspace(Ynew_1,Yfinal_1,25);  %Yfinal, Ynew
    tdata (1,2,26:100) = Ynew_1;

    tdata (1,3,1:25) = z_pos;
    %**************************************************************************
    %Leg 2
    tdata (2,1,1:25) = Xnew_2;
    tdata (2,1,26:50) = linspace(Xnew_2,Xfinal_2,25); % Xfinal, Xnew
    tdata (2,1,51:100) = Xfinal_2;

    tdata (2,2,1:25) = Ynew_2;
    tdata (2,2,26:50) = linspace(Ynew_2,Yfinal_2,25);  %Yfinal, Ynew
    tdata (2,2,51:100) = Yfinal_2;

    tdata (2,3,26:50) = z_pos;
    %**************************************************************************
    %Leg 3
    tdata (3,1,1:50) = Xnew_3;
    tdata (3,1,51:75) = linspace(Xnew_3,Xfinal_3,25); % Xfinal, Xnew
    tdata (3,1,76:100) = Xfinal_3;

    tdata (3,2,1:50) = Ynew_3;
    tdata (3,2,51:75) = linspace(Ynew_3,Yfinal_3,25);  %Yfinal, Ynew
    tdata (3,2,76:100) = Yfinal_3;

    tdata (3,3,51:75) = z_pos;
    %**************************************************************************
    %Leg 4
    tdata (4,1,1:75) = Xnew_4;
    tdata (4,1,76:100) = linspace(Xnew_4,Xfinal_4,25); % Xfinal, Xnew

    tdata (4,2,1:75) = Ynew_4;
    tdata (4,2,76:100) = linspace(Ynew_4,Yfinal_4,25);  %Yfinal, Ynew

    tdata (4,3,76:100) = z_pos;
    


end
           
 