clc; close all;
% https://www.mathworks.com/help/supportpkg/kinectforwindowsruntime/ug/acquire-image-and-body-data-using-kinect-v2.html
imaqreset;
depthVid = videoinput('kinect',2);
% triggerconfig(depthVid, 'manual');
depthVid.FramesPerTrigger = 1;
depthVid.TriggerRepeat = inf;
% set(getselectedsource(depthVid), 'TrackingMode', 'Skeleton');
set(getselectedsource(depthVid), 'EnableBodyTracking', 'on');
% viewer = vision.DeployableVideoPlayer();
start(depthVid);
himg = figure;
while ishandle(himg)
%     tic
% trigger(depthVid);
% fprintf('trigger: ');
% toc
tic
[depthMap, ~, depthMetaData] = getdata(depthVid);
fprintf('get data: ');
toc
% idx = find(depthMetaData.IsSkeletonTracked);
idx = find(depthMetaData.IsBodyTracked);
tic
imshow(depthMap, [0 4096]);
fprintf('imshow: ');
toc
% disp(depthMetaData.IsBodyTracked);

if ~isempty(idx)
    idx = idx(1);
tic
rightHand = depthMetaData.DepthJointIndices(12,:,idx);
rightHandWorldCoordinate = depthMetaData.JointPositions(12,:,idx);
zCoord = 1e3*min(depthMetaData.JointPositions(12,:,idx));
radius = round(90 - zCoord / 50);
rightHandBox = [rightHand-0.5*radius 1.2*radius 1.2*radius];
fprintf('compute: ');
toc
tic
rectangle('position', rightHandBox, 'EdgeColor', [1 1 0])
fprintf('box: ');
toc
end

end
stop(depthVid);