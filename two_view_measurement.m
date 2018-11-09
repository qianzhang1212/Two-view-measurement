function varargout = two_view_measurement(varargin)
% TWO_VIEW_MEASUREMENT MATLAB code for two_view_measurement.fig
%      TWO_VIEW_MEASUREMENT, by itself, creates a new TWO_VIEW_MEASUREMENT or raises the existing
%      singleton*.
%
%      H = TWO_VIEW_MEASUREMENT returns the handle to a new TWO_VIEW_MEASUREMENT or the handle to
%      the existing singleton*.
%
%      TWO_VIEW_MEASUREMENT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TWO_VIEW_MEASUREMENT.M with the given input arguments.
%
%      TWO_VIEW_MEASUREMENT('Property','Value',...) creates a new TWO_VIEW_MEASUREMENT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before two_view_measurement_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to two_view_measurement_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help two_view_measurement

% Last Modified by GUIDE v2.5 09-Jul-2018 13:01:32

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
global range;
range = 500;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @two_view_measurement_OpeningFcn, ...
                   'gui_OutputFcn',  @two_view_measurement_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before two_view_measurement is made visible.
function two_view_measurement_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to two_view_measurement (see VARARGIN)

% Choose default command line output for two_view_measurement
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes two_view_measurement wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = two_view_measurement_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in selectImage.
function selectImage_Callback(hObject, eventdata, handles)
% hObject    handle to selectImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global plots;
global range;
[filename, pathname] = uigetfile( ...
    {'*.avi;*.mp4;*.mpeg','Video Files (*.avi,*.mp4,*.mpeg)';
     '*.*',  'All Files (*.*)'}, ...
     'Select a video file');
%Read video
videoFileReader = vision.VideoFileReader(fullfile(pathname,filename));
%First frame of the video
framefirst = videoFileReader();
% framefirst = framefirst(101:1380,1:end,1:3);
%Define point tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);
if get(handles.rotate,'Value')==1
    %Rotate the video
    video{1} = imrotate(framefirst,90);    
    img1_origin = video{1};
    disp('Processing video......');
    %%%% intrinsic matrix
    [M,N,~] = size(img1_origin);
    ccd_width = str2double(get(handles.ccd_width, 'String'));%mm
    focal = str2double(get(handles.focal, 'String'));%mm
    focal = (focal/ccd_width)*N;
    IntrinsicMatrix = [focal 0 0; 0 focal 0; N/2 M/2 1];
    cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix);
    imagePoints1 = detectBRISKFeatures(rgb2gray(img1_origin));
    imagePoints1 = imagePoints1.Location;
    initialize(tracker, imagePoints1, img1_origin);
    t = 2;
    while ~isDone(videoFileReader)
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        video{t} = imrotate(frame,90);
        [imagePoints2,validIdx] = tracker(video{t});
        t = t+1;
    end
else
    video{1} = framefirst;    
    img1_origin = video{1};
    disp('Processing video......');
    %%%% intrinsic matrix
    [M,N,~] = size(img1_origin);
    ccd_width = str2double(get(handles.ccd_width, 'String'));%mm
    focal = str2double(get(handles.focal, 'String'));%mm   
    focal = (focal/ccd_width)*M;    
    IntrinsicMatrix = [focal 0 0; 0 focal 0; N/2 M/2 1];
    cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix);
    imagePoints1 = detectBRISKFeatures(rgb2gray(img1_origin));
    imagePoints1 = imagePoints1.Location;
    initialize(tracker, imagePoints1, img1_origin);    
    t = 2;
    while ~isDone(videoFileReader)
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        frame = videoFileReader();
        video{t} = frame;
        [imagePoints2,validIdx] = tracker(video{t});
        t = t+1;
    end
end

matchedPoints1 = imagePoints1(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);
% Estimate the fundamental matrix
[E, epipolarInliers] = estimateEssentialMatrix(...
    matchedPoints1, matchedPoints2, cameraParams, 'Confidence', 99.99,'MaxNumTrials',50000);
[F,~] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,'NumTrials',50000);
% Find epipolar inliers
inlierPoints1 = matchedPoints1(epipolarInliers, :);
inlierPoints2 = matchedPoints2(epipolarInliers, :);
% Display inlier matches
% figure
% showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
% title('Epipolar Inliers');
[orient, loc] = relativeCameraPose(E, cameraParams, inlierPoints1, inlierPoints2);
camMatrix1 = cameraMatrix(cameraParams, eye(3), [0 0 0]);

% F=0;
% [orient, loc, ~] = helperEstimateRelativePose(...
%         matchedPoints1, matchedPoints2, cameraParams);
    
% Compute extrinsics of the second camera
[R, t] = cameraPoseToExtrinsics(orient, loc);
camMatrix2 = cameraMatrix(cameraParams, R, t);
disp('Process Completed !');
%%show
axes(handles.axes1);
hold all;
delete(get(gca,'Children'));
imshow(img1_origin);
plots = [];
handles.camMatrix1 = camMatrix1;
handles.camMatrix2 = camMatrix2;
handles.F = F;
handles.video = video;
guidata(hObject, handles);

function referenceDistance_Callback(hObject, eventdata, handles)
% hObject    handle to referenceDistance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of referenceDistance as text
%        str2double(get(hObject,'String')) returns contents of referenceDistance as a double
handles.referenceValue = str2double(get(hObject,'string'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function referenceDistance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to referenceDistance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in selectTarget.
function selectTarget_Callback(hObject, eventdata, handles)
% hObject    handle to selectTarget (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global plots;
global range;
hold off
video = handles.video;
F = handles.F;
camMatrix1 = handles.camMatrix1;
camMatrix2 = handles.camMatrix2;

frame1 = video{1};
[M,N,~] = size(frame1);
axes(handles.axes1);
delete(get(gca,'Children'));
imshow(frame1);
plots = [];
%%%% select reference points
disp('Select two points as target points');
[x1,y1] = ginput(1);
[x2,y2] = ginput(1);
% region1 = [x1-10,y1-10,20,20];
% region2 = [x2-10,y2-10,20,20];
region1 = [x1-7,y1-7,14,14];
region2 = [x2-7,y2-7,14,14];
points1 = detectMinEigenFeatures(rgb2gray(frame1),'ROI',region1);
points2 = detectMinEigenFeatures(rgb2gray(frame1),'ROI',region2);

% if (points1.Count==0||points2.Count==0)
    
tracker1 = vision.PointTracker('MaxBidirectionalError',3,'NumPyramidLevels', 5);
tracker2 = vision.PointTracker('MaxBidirectionalError',3,'NumPyramidLevels', 5);

initialize(tracker1,points1.Location,frame1);
initialize(tracker2,points2.Location,frame1);

for t=2:length(video)
    frame = video{t};
    [targetPoints1_img2,validity1] = tracker1(frame);
    [targetPoints2_img2,validity2] = tracker2(frame);
end
targetPoints1_img2 = mean(targetPoints1_img2(validity1,:),1);
targetPoints2_img2 = mean(targetPoints2_img2(validity2,:),1);
targetPoints_img2 = [targetPoints1_img2;targetPoints2_img2];
img2_origin = video{end};
imshow(img2_origin);
hold on
plot(targetPoints1_img2(1),targetPoints1_img2(2),'o','markersize',8);
plot(targetPoints2_img2(1),targetPoints2_img2(2),'o','markersize',8);
targetPoints_framefirst = [x1 y1;x2 y2];
targetPoints_framelast = double(targetPoints_img2);
pause(1)
if(isempty(find(validity1==1))||isempty(find(validity2==1)))
    img1_origin = video{1};
    targetPoints_img1 = [x1 y1;x2 y2];
    targetPoints_img2 = [];
    gray_1 = rgb2gray(img1_origin);
    gray_2 = rgb2gray(img2_origin);
    hold off
    imshow(img2_origin);
    [M,N,~] = size(img2_origin);
    hold on
    if(isempty(find(validity1==1)))
        p_img1 = [targetPoints_img1(1,:) 1];
        l = F*p_img1';
        linex1 = 1;
        liney1 = (-l(3)-l(1)*linex1)/l(2);
        linex2 = N;
        liney2 = (-l(3)-l(1)*linex2)/l(2);
        plots(size(plots,2)+1) = plot([linex1 linex2], [liney1 liney2], 'g');
        [feature1, ~] = extractFeatures(gray_1,[p_img1(1) p_img1(2)],'BlockSize',15);
        %%%% corresponding points in img2
        max = 10^7;
        p_img2 = [];
        for x=p_img1(1)-range:0.05:p_img1(1)+range
            y = (-l(3)-l(1)*x)/l(2);
            if(y<=M-20&&y>=20)
                p_img2 = [p_img2;[x y]];
            end
        end
        [feature2, validpoint] = extractFeatures(gray_2,p_img2,'BlockSize',15);
        [n,~] = size(feature2);
        for i=1:n
            error = norm(double(feature1)-double(feature2(i,:)));
            if(error<max)
                best_point = validpoint(i,:);
                max = error;
            end
        end
        plot(best_point(1),best_point(2),'o','markersize',8);
        targetPoints_framelast(1,:) = best_point;
        plot(targetPoints_framelast(2,1),targetPoints_framelast(2,2),'o','markersize',8);
    end
    if(isempty(find(validity2==1)))
        p_img1 = [targetPoints_img1(2,:) 1];
        l = F*p_img1';
        linex1 = 1;
        liney1 = (-l(3)-l(1)*linex1)/l(2);
        linex2 = N;
        liney2 = (-l(3)-l(1)*linex2)/l(2);
        plots(size(plots,2)+1) = plot([linex1 linex2], [liney1 liney2], 'g');
        [feature1, ~] = extractFeatures(gray_1,[p_img1(1) p_img1(2)],'BlockSize',15);
        %%%% corresponding points in img2
        max = 10^7;
        p_img2 = [];
        for x=p_img1(1)-range:0.05:p_img1(1)+range
            y = (-l(3)-l(1)*x)/l(2);
            if(y<=M-20&&y>=20)
                p_img2 = [p_img2;[x y]];
            end
        end
        [feature2, validpoint] = extractFeatures(gray_2,p_img2,'BlockSize',15);
        [n,~] = size(feature2);
        for i=1:n
            error = norm(double(feature1)-double(feature2(i,:)));
            if(error<max)
                best_point = validpoint(i,:);
                max = error;
            end
        end
        plot(best_point(1),best_point(2),'o','markersize',8);
        targetPoints_framelast(2,:) = best_point;
        plot(targetPoints_framelast(1,1),targetPoints_framelast(1,2),'o','markersize',8);
    end
end

points3D = triangulate(targetPoints_framefirst, targetPoints_framelast, camMatrix1, camMatrix2);
targetDistance = norm(points3D(1,:)-points3D(2,:))*handles.referenceValue;
set(handles.targetDistance,'String',num2str(targetDistance));


function targetDistance_Callback(hObject, eventdata, handles)
% hObject    handle to targetDistance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of targetDistance as text
%        str2double(get(hObject,'String')) returns contents of targetDistance as a double


% --- Executes during object creation, after setting all properties.
function targetDistance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to targetDistance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ccd_width_Callback(hObject, eventdata, handles)
% hObject    handle to ccd_width (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ccd_width as text
%        str2double(get(hObject,'String')) returns contents of ccd_width as a double


% --- Executes during object creation, after setting all properties.
function ccd_width_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ccd_width (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function focal_Callback(hObject, eventdata, handles)
% hObject    handle to focal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of focal as text
%        str2double(get(hObject,'String')) returns contents of focal as a double


% --- Executes during object creation, after setting all properties.
function focal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to focal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in rotate.
function rotate_Callback(hObject, eventdata, handles)
% hObject    handle to rotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rotate
