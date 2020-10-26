
if count(py.sys.path,'') == 0
    insert(py.sys.path,int32(0),'');
end
controller = py.leapPythonTest.getController;

R = [ [0; 1; 0], [1; 0; 0], [0;0;-1] ];% [xnew ynew znew] 
p = [0;0;0];

while true

frame = py.leapPythonTest.getFrame(controller);

if ~(frame.hands.is_empty)
%         frame.hands.rightmost.palm_position
point = [frame.hands.rightmost.stabilized_palm_position.x;...
    frame.hands.rightmost.stabilized_palm_position.y;...
    frame.hands.rightmost.stabilized_palm_position.z]/1000; % measures in mm
pointTransformed = [R p; 0 0 0 1]*[point; 1];
disp(pointTransformed(1:3).')

end

end