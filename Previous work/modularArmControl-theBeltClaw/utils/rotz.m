function m = rotz(theta)
%Homogeneous transform matrix for a rotation about z
%theta is in radians
    m = [cos(theta), -sin(theta), 0, 0;
         sin(theta),  cos(theta), 0, 0;
         0          , 0, 1,          0;
         0,           0, 0,          1];
end
