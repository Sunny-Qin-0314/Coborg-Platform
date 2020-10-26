function m = rotx(theta)
%Homogeneous transform matrix for a rotation about x
%theta is in radians
    m = [1,  0,          0,          0;
         0,  cos(theta),-sin(theta), 0;
         0,  sin(theta), cos(theta), 0;
         0,           0, 0,          1];
end
