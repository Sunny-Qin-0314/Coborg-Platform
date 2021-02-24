function m = roty(theta)
%Homogeneous transform matrix for a rotation about y
%theta is in radians
    m = [cos(theta),  0, sin(theta), 0;
         0,           1, 0,          0;
         -sin(theta), 0, cos(theta), 0;
         0,           0, 0,          1];
end
