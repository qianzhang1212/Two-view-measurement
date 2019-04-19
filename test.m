clear all
close all
videoFileReader = vision.VideoFileReader('43.mp4');
framefirst = videoFileReader();
framefirst = videoFileReader();

rotate = 0;

if rotate==1    
    video{1} = imrotate(framefirst,90);
    t = 2;
    while ~isDone(videoFileReader)
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        video{t} = imrotate(frame,90);
        t = t+1;
    end
else
    video{1} = framefirst;
    t = 2;
    while ~isDone(videoFileReader)
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        video{t} =frame;
        t = t+1;
    end
end

img1_origin = video{1};
img2_origin = video{end};
I1 = img1_origin;
I2 = img2_origin;
disp('Processing images......');
%%%% intrinsic matrix
[M,N,~] = size(img1_origin);
ccd_width = 6.25;
focal = 4.67;%mm
% ccd_width = 5.22;
% focal = 4.26;%mm
if(M>N)
    focal = (focal/ccd_width)*M;
else
    focal = (focal/ccd_width)*N;
end
IntrinsicMatrix = [focal 0 0; 0 focal 0; N/2 M/2 1];
% IntrinsicMatrix = [focal 0 0; 0 focal 0; M/2 N/2 1];
K1 = [focal 0 N/2; 0 focal M/2; 0 0 1];
K2 = K1;
cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix);

border = 50;
roi = [border, border, size(I1, 2)- 2*border, size(I1, 1)- 2*border];

% imagePoints1 = detectMinEigenFeatures(rgb2gray(I1));
% imagePoints1 = detectBRISKFeatures(rgb2gray(I1),'NumOctaves', 8, 'ROI', roi,'MinContrast',0.005,'MinQuality',0.001);
imagePoints1 = detectBRISKFeatures(rgb2gray(I1));
% imagePoints1 = detectSURFFeatures(rgb2gray(I1));
% imagePoints1 = detectFASTFeatures(rgb2gray(I1));
% imagePoints1 = detectHarrisFeatures(rgb2gray(I1));
% imagePoints1 = detectMSERFeatures(rgb2gray(I1));

figure
imshow(I1, 'InitialMagnification', 50);
title('150 Strongest Corners from the First Image');
hold on
plot(selectStrongest(imagePoints1, 150));

% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);
% tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5,'BlockSize',[101 101],'MaxIterations',100);
% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);
% Track the points
% [imagePoints2, validIdx] = step(tracker, I2);
for t=2:length(video)
    frame = video{t};
%     frame = imresize(frame,[M/4,N/4]);
    [imagePoints2,validIdx] = tracker(frame);
end
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);
% Visualize correspondences
figure
showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2,'montage');
title('Tracked Features');
% Estimate the fundamental matrix
[E, epipolarInliers] = estimateEssentialMatrix(...
    matchedPoints1, matchedPoints2, cameraParams, 'Confidence', 99.99,'MaxNumTrials',50000,'MaxDistance',0.1);
[F,~] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,'NumTrials',50000);
% Find epipolar inliers
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);
% Display inlier matches
figure
showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2,'montage');
title('Epipolar Inliers');

[orient, loc] = relativeCameraPose(E, cameraParams, inlierPoints1, inlierPoints2);

% [orient, loc, inlierIdx] = helperEstimateRelativePose(...
%         matchedPoints1, matchedPoints2, cameraParams);
% inlierPoints1 = matchedPoints1(inlierIdx, :);
% inlierPoints2 = matchedPoints2(inlierIdx, :);
% % Display inlier matches
% figure
% showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2,'montage');
% title('Epipolar Inliers');

roi = [30, 30, size(I1, 2) - 30, size(I1, 1) - 30];
imagePoints1 = detectMinEigenFeatures(rgb2gray(I1), 'ROI', roi, 'MinQuality', 0.001);

% border = 50;
% roi = [border, border, size(I1, 2)- 2*border, size(I1, 1)- 2*border];
% 
% imagePoints1 = detectBRISKFeatures(rgb2gray(I1),'NumOctaves', 8, 'ROI', roi,'MinContrast',0.05,'MinQuality',0.01);

% Create the point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

% Initialize the point tracker
imagePoints1 = imagePoints1.Location;
initialize(tracker, imagePoints1, I1);

% Track the points
% [imagePoints2, validIdx] = step(tracker, I2);
for t=2:length(video)
    frame = video{t};
%     frame = imresize(frame,[M/4,N/4]);
    [imagePoints2,validIdx] = tracker(frame);
end
matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

% Compute the camera matrices for each position of the camera
% The first camera is at the origin looking along the Z-axis. Thus, its
% rotation matrix is identity, and its translation vector is 0.
camMatrix1 = cameraMatrix(cameraParams, eye(3), [0 0 0]);

% Compute extrinsics of the second camera
[R, t] = cameraPoseToExtrinsics(orient, loc);
camMatrix2 = cameraMatrix(cameraParams, R, t);

% Compute the 3-D points
points3D = triangulate(matchedPoints1, matchedPoints2, camMatrix1, camMatrix2);

% Get the color of each reconstructed point
numPixels = size(I1, 1) * size(I1, 2);
allColors = reshape(I1, [numPixels, 3]);
colorIdx = sub2ind([size(I1, 1), size(I1, 2)], round(matchedPoints1(:,2)), ...
    round(matchedPoints1(:, 1)));
color = allColors(colorIdx, :);

% Create the point cloud
ptCloud = pointCloud(points3D, 'Color', color);

% Visualize the camera locations and orientations
cameraSize = 0.3;
figure
plotCamera('Size', cameraSize, 'Color', 'r', 'Label', '1', 'Opacity', 0);
hold on
grid on
plotCamera('Location', loc, 'Orientation', orient, 'Size', cameraSize, ...
    'Color', 'b', 'Label', '2', 'Opacity', 0);

% Visualize the point cloud
pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);

% Rotate and zoom the plot
camorbit(0, -30);
camzoom(1.5);

% Label the axes
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis')

title('Up to Scale Reconstruction of the Scene');
