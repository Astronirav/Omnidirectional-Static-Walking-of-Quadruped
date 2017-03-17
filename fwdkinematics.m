% Forward Kinematics for All the points on all the legs
function [positions] = fwdkinematics(angles)
      
global DHParam;
        
A01 = zeros(4,4,4);
A12 = zeros(4,4,4);
A23 = zeros(4,4,4);

Tmat1 = zeros(4,4,4);
Tmat2 = zeros(4,4,4);
Tmat3 = zeros(4,4,4);

for i = 1:4
    
 A01(:,:,i) = [cosd(angles(i,1))   0  sind(angles(i,1))  DHParam(1,2)*cosd(angles(i,1));...
               sind(angles(i,1))   0  -cosd(angles(i,1)) DHParam(1,2)*sind(angles(i,1));...
               0              1  0             0;...
               0             0   0             1];

end

for i = 1:4
   A12(:,:,i) = [cosd(angles(i,2)) -sind(angles(i,2)) 0 DHParam(2,2)*cosd(angles(i,2));...
                 sind(angles(i,2))  cosd(angles(i,2)) 0 DHParam(2,2)*sind(angles(i,2));...
                 0             0            1 0;...
                 0             0            0 1]; 
    
end

for i = 1:4
   
  A23(:,:,i) = [cosd(angles(i,3)) -sind(angles(i,3)) 0 DHParam(3,2)*cosd(angles(i,3));...
                sind(angles(i,3))  cosd(angles(i,3)) 0 DHParam(3,2)*sind(angles(i,3));...
                0             0            1 0;...
                0             0            0 1]; 
    
end

for i = 1:4
    Tmat1(:,:,i) = A01(:,:,i);
end

for i = 1:4
    Tmat2(:,:,i) = A01(:,:,i)* A12(:,:,i);
end

for i = 1:4
    Tmat3(:,:,i) = A01(:,:,i)* A12(:,:,i)* A23(:,:,i);
end

positions = zeros(4,3,4);

for i = 1:4
positions(2,:,i) = Tmat1(1:3,4,i);
positions(3,:,i) = Tmat2(1:3,4,i);
positions(4,:,i) = Tmat3(1:3,4,i);
end

end