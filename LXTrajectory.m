function [LinearX_Trajectory] = LXTrajectory(gait, Fc, Ls, Hb, no_steps, sc)
% The function provides the way-points for the trajectory in x direction

LinearX_Trajectory = zeros(4,3,no_steps);

tdata = zeros(no_steps, 3);

A = -(120*Ls)/(gait(5)*(gait(5)-1).^5);
B = (Ls*(gait(5).^5-3*gait(5)^4+10*gait(5).^2+5*gait(5)-1))/(2*((gait(5)-1).^5));

for i = 1:no_steps*gait(5)
    tdata(i,3) = -Hb ;      %Z Component of Linear Trajectory will be -Hb till beta
    tdata(i,1) = Ls/2-((i-1)*Ls)/(no_steps*gait(5)-1);  %X position of the leg changes from Ls/2 to -Ls/2 in beta seconds
end

for i = no_steps*gait(5)+1:no_steps
    
    tdata(i,3) = Fc*(1-cos( 2*pi*((i-no_steps*gait(5))-no_steps*gait(1))/(no_steps*(1-gait(5))) ))/2-Hb ; 
    % Trajectory Data in Z direction. 1-cos function. Starts from i = 
    
    t = i/no_steps;
    tdata(i,1) = A*((t^5)/20-((gait(5)+1)*t^4)/8 +((gait(5)^2+4*gait(5)+1)*t^3)/12 -(gait(5)*(gait(5)+1)*t^2)/4 + (gait(5)*gait(5)*t)/4) - (Ls*t)/gait(5) + B;

end

LinearX_Trajectory(1,3,:) = circshift(tdata(:,3),gait(3)*no_steps);%1st
LinearX_Trajectory(2,3,:) = circshift(tdata(:,3),gait(1)*no_steps);%3rd
LinearX_Trajectory(3,3,:) = circshift(tdata(:,3),gait(4)*no_steps);%4th
LinearX_Trajectory(4,3,:) = circshift(tdata(:,3),gait(2)*no_steps);%2nd

LinearX_Trajectory(1,1,:) = circshift(tdata(:,1),gait(3)*no_steps);
LinearX_Trajectory(2,1,:) = circshift(tdata(:,1),gait(1)*no_steps);
LinearX_Trajectory(3,1,:) = circshift(tdata(:,1),gait(4)*no_steps);
LinearX_Trajectory(4,1,:) = circshift(tdata(:,1),gait(2)*no_steps);


end