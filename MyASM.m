function varargout = MyASM(varargin)
% MYASM M-file for MyASM.fig
%      MYASM, by itself, creates a new MYASM or raises the existing
%      singleton*.
%
%      H = MYASM returns the handle to a new MYASM or the handle to
%      the existing singleton*.
%
%      MYASM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MYASM.M with the given input arguments.
%
%      MYASM('Property','Value',...) creates a new MYASM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MyASM_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MyASM_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MyASM

% Last Modified by GUIDE v2.5 29-Jun-2012 07:56:40

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @MyASM_OpeningFcn, ...
    'gui_OutputFcn',  @MyASM_OutputFcn, ...
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

% --- Executes just before MyASM is made visible.
function MyASM_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MyASM (see VARARGIN)
% Choose default command line output for MyASM
global S 
global LandmarkGroups;
handles.output = hObject;
load('AlignedImages\All_Mesh.txt');
handles.ShapeData = All_Mesh;
% Update handles structure
handles.SelectedRBtnHandle = handles.radiobutton3;
load('AlignedImages/EigenFaceVec.txt');
load('AlignedImages/EigenFaceValue.txt');
load('AlignedImages/MeanFace.txt');
handles.EigVec = EigenFaceVec;
handles.EigVal = EigenFaceValue;
handles.Mean = MeanFace;
S = 2;
LandmarkGroups={[35:39, 52] ... %1: Right eyebrow
                              [64 30:34], ... %2: Left eyebrow
                              22:29, ...%3: Right eye
                              14:21, ...%4: Left eye
                              [26:29 22],... %5: Right upper eyelid
                              22:26,... %6: Right lower eyelid
                              [18:21 14],... %7: Left upper eyelid
                              14:18,... %8: Right lower eyelid
                              52:64,... %9: Nose
                              52:56,... %10: Nose right bondry
                              60:64,... %11: Nose left bndry
                              40:51,... %12: Mouth
                              [40:42 44:46],... %13: Mouth upper bndry
                              [46:51 40],... %14: Mouth lower bondry
                              [1:13] %15: Jaw
                              };
guidata(hObject, handles);

% UIWAIT makes MyASM wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MyASM_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in listbox1.
% Show the selected input image and align it.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1
contents = cellstr(get(hObject,'String'));
SelectedIdx = get(hObject,'Value');
SelectedFile = contents{SelectedIdx};
handles.CurSel = SelectedIdx;
handles.InImg = imread(['TrainingImages\' SelectedFile]);
[Height Width] = size(handles.InImg);
% set(handles.ImageAxe, 'Visible', 'on');
axes(handles.ImageAxe);
axis off;
axis ij;
axis image;
imshow(handles.InImg);
hold on;
len = length(handles.ShapeData(SelectedIdx,:));
Sx = handles.ShapeData(SelectedIdx,1:2:len);
Sy = handles.ShapeData(SelectedIdx,2:2:len);
plot(Sx, Sy, 'r.');
ref = handles.ShapeData(1,:);
[handles.aligned v A] = AlignShape(handles.ShapeData(SelectedIdx,:), ref);
handles.warpImg = WarpImage(handles.InImg, v);
% set(handles.WarpAxe, 'Visible', 'on');
axes(handles.WarpAxe);
axis ij;
axis image;
axis off;
imshow(handles.warpImg);
hold on;
len1 = length(handles.aligned);
Sx = handles.aligned(1:2:len);
Sy = handles.aligned(2:2:len);
plot(Sx, Sy, 'r.');
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
FileList = dir('TrainingImages/*.jpg');
FileList = struct2cell(FileList);
handles.DataListFName = FileList(1,:);
set(hObject, 'String', handles.DataListFName);
guidata(hObject, handles);

% --- Executes on button press in pushbutton1.
% Align shapes
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
NoOfData =length(handles.DataListFName);
ref = handles.ShapeData(1,:);
aligned =  handles.ShapeData;
h = waitbar(0,'Please wait...');
AlignedFileNameList =  handles.DataListFName;
% AlignedFileNameList{1} =  ['align_' handles.DataListFName{1}];
for i=1:NoOfData
    Img = imread( ['TrainingImages\'  handles.DataListFName{i}]);
    [aligned(i,:) v A] = AlignShape(handles.ShapeData(i,:), ref);
    WImg = WarpImage(Img, v);
    AlignedFileNameList{i} =  ['AlignedImages\align_' AlignedFileNameList{i}];
    imwrite(WImg, AlignedFileNameList{i} , 'jpg');
    waitbar(i / NoOfData, h, [num2str(i) '/' num2str(NoOfData)]);
end
save('AlignedImages\AlignedFileNameList.mat', 'AlignedFileNameList');
save('AlignedImages\All_Aligned.txt', '-ascii', 'aligned');
close(h);
set(handles.text1, 'String', 'Shape alignment finished.');

% --- Executes on button press in pushbutton2.
% Do the PCA on shapes
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load('AlignedImages\All_Aligned.txt'); % load the All_Aligned variable
[V,D]=eig(cov(All_Aligned));
MeanMesh = mean(All_Aligned);
MeanMesh = MeanMesh';
newV=[];
[lambda, I] = sort(diag(D),'descend');% 將特徵值 從大排到小
N = length(lambda);
for j=1:N
    newV=horzcat(newV,V(:,I(j:j):I(j:j)));  %根據特徵值大小  將特徵向量從大排到小
end
save('AlignedImages\MeanFace.txt', '-ascii', 'MeanMesh');
save('AlignedImages\EigenFaceValue.txt', '-ascii', 'lambda');
save('AlignedImages\EigenFaceVec.txt', '-ascii', 'newV');
handles.EigVec = newV;
handles.EigVal = lambda;
handles.Mean = MeanMesh;
set(handles.text1, 'String', 'PCA finished.');
guidata(hObject, handles);


% --- Executes on button press in pushbutton3.
% Extract the landmark features
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global S
load('AlignedImages\AlignedFileNameList.mat'); % load the AlignFileNameList variable into memory
NoOfData = length(AlignedFileNameList);
load('AlignedImages\All_Aligned.txt');
NoOfLandmarks = length(All_Aligned(1,:))/2;
WindowSize = 11;
FeatData = zeros(NoOfData, WindowSize*WindowSize, NoOfLandmarks);
d = floor(WindowSize/2);
h = waitbar(0,'Please wait...');
for i=1:NoOfData
    xy =floor( [All_Aligned(i, 1:2:NoOfLandmarks*2)' All_Aligned(i, 2:2:NoOfLandmarks*2)']);
    Img = imread(AlignedFileNameList{i});
    Img = rgb2gray(Img);
    for j=1:NoOfLandmarks
        FeatData(i, :, j) = FeatExtract(xy(j,1),xy(j,2),Img,S,d,WindowSize);%reshape(Img(xy(j,2)-S*d:S:xy(j,2)+S*d,xy(j,1)-S*d:S:xy(j,1)+S*d), [1 WindowSize*WindowSize]);
    end
    waitbar(i / NoOfData, h, [num2str(i) '/' num2str(NoOfData)]);
end
close(h);
save('AlignedImages\FeatData.mat', 'FeatData');
set(handles.text1, 'String', 'Landmark feature collection finished.');

% --- Executes on button press in pushbutton4.
% Train the landmark features
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load('AlignedImages\FeatData.mat');
[NoOfData Dim NoOfLandmarks] = size(FeatData);
Data = zeros(NoOfData, Dim);
EigMeanData = zeros(NoOfLandmarks, Dim);
EigStdData = zeros(NoOfLandmarks, Dim);
NewV = zeros(Dim, Dim, NoOfLandmarks);
lambda = zeros(NoOfLandmarks, Dim);
for i=1:NoOfLandmarks
    for j=1:NoOfData
        Data(j,:) = FeatData(j, :, i)/norm(FeatData(j,:,i));
    end
    EigMeanData(i,:) = mean(Data);
    EigMeanData(i,:) = EigMeanData(i,:)';
    EigStdData(i,:) = std(Data);
    EigStdData(i,:) = EigStdData(i,:)';
%     [V D] = eig(cov(Data));
%     [lambda(i,:), I] = sort(diag(D),'descend');% 將特徵值 從大排到小
%     N = length(lambda(i,:));
%     newV = zeros(Dim, Dim, NoOfLandmarks);
%     VV=[];
%     for j=1:N
%         VV=horzcat(VV,V(:,I(j:j):I(j:j)));  %根據特徵值大小  將特徵向量從大排到小
%     end
%     newV(:,:,i) = VV;
end
save('AlignedImages\EigFeatMean.mat',  'EigMeanData');
save('AlignedImages\EigFeatStd.mat',  'EigStdData');
% save('AlignedImages\EigFeatVal.mat', 'lambda');
% save('AlignedImages\EigFeatVec.mat', 'newV');
set(handles.text1, 'String', 'Landmark feature training finished.');


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
HighLightMark(handles);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
HighLightMark(handles);

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
HighLightMark(handles);

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
HighLightMark(handles);

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
HighLightMark(handles);

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
HighLightMark(handles);

% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
HighLightMark(handles);

% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider8_Callback(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
HighLightMark(handles);

% --- Executes during object creation, after setting all properties.
function slider8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes when selected object is changed in uipanel2.
function uipanel2_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uipanel2
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
handles.SelectedRBtnHandle = eventdata.NewValue;
HighLightMark(handles);
guidata(hObject, handles);


% --- Executes on button press in pushbutton6.
% Search the feature points
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global LandmarkID
if isfield(handles, 'warpImg') && isfield(handles, 'X') && isfield(handles, 'Y')
    GrayIn = rgb2gray(handles.warpImg);
    InitPt = [handles.X handles.Y];
    Items = get(handles.popupmenu1, 'String');
    HalfWndSize = str2num(Items{get(handles.popupmenu1, 'Value')})/2;
    load('AlignedImages\EigFeatMean.mat');
    load('AlignedImages\EigFeatStd.mat');
    Template = EigMeanData(LandmarkID, :);
    TemplateStd = EigStdData(LandmarkID, :);
%    Iter = log2(HalfWndSize)+1;
%     pt = LogSearch(GrayIn, InitPt, HalfWndSize, sqrt(length(Template)), Template, Iter);
    [pt Map] = FeatureSearch(GrayIn, InitPt, HalfWndSize, sqrt(length(Template)), Template, TemplateStd, 1, 1);
    figure(1); imshow(Map);
    HighLightMark(handles);
    hold on;
    plot(pt(1), pt(2), 'yd', handles.X, handles.Y, 'yo');
end

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
% Initialize a position for search all landmarks.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.warpImg = InitLandmarks(handles);
% RefLandmarkID = [22 14 40 46 58 7];
% [X Y] = ginput(length(RefLandmarkID))
% XY = [X';Y'];
% Input = reshape(XY, [1,size(XY,1)*size(XY,2)]);
% Idx = zeros(1, 2*length(RefLandmarkID));
% for i = 1:length(RefLandmarkID)
%     Idx(2*i-1) = 2*RefLandmarkID(i)-1;
%     Idx(2*i) = 2*RefLandmarkID(i);
% end
% Ref = handles.ShapeData(1,Idx);
% [Aligned V] = AlignShape(Input, Ref);
% handles.warpImg = WarpImage(handles.InImg, V);
handles.aligned = handles.ShapeData(1,:);
axes(handles.WarpAxe);
axis ij;
axis image;
axis off;
imshow(handles.warpImg);
hold on;
len = length(handles.aligned);
Sx = handles.aligned(1:2:len);
Sy = handles.aligned(2:2:len);
plot(Sx, Sy, 'r.');
guidata(hObject, handles);

% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
pt = get(handles.WarpAxe, 'CurrentPoint');
HighLightMark(handles);
hold on;
handles.X = pt(1,1);
handles.Y = pt(1,2);
plot(pt(1,1), pt(1,2), 'yo');
guidata(hObject, handles);


% --- Executes on button press in pushbutton8.
% Search all landmarks
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if isfield(handles, 'warpImg') && isfield(handles, 'aligned')
    GrayIn = rgb2gray(handles.warpImg);
    InitXYs = handles.aligned;
    Items = get(handles.popupmenu1, 'String');
    SearchWSize = str2num(Items{get(handles.popupmenu1, 'Value')});
    load('AlignedImages\EigFeatMean.mat');
    load('AlignedImages\EigFeatStd.mat');
    Template = EigMeanData;
    TemplateStd = EigStdData;
%    Iter = log2(HalfWndSize)+1;
%     pt = LogSearch(GrayIn, InitPt, HalfWndSize, sqrt(length(Template)), Template, Iter);
%     [XYs Errs Xs Ys] = SearchAllLandmarks(GrayIn, InitXYs, SearchWSize, sqrt(length(Template(1,:))), Template, TemplateStd, 2);
    [XYs Errs Xs Ys] = SearchAllLandmarksByDP(GrayIn, InitXYs, SearchWSize, sqrt(length(Template(1,:))), Template, TemplateStd);
    axes(handles.WarpAxe);
    hold off;
    HighLightMark(handles);
    hold on;
    LX = Xs; %XYs(1:2:length(XYs));
    LY = Ys; %XYs(2:2:length(XYs));
    plot(LX, LY, 'y*');
    LXYs = reshape([LX;LY], [length(LX)*2 1]);
    DrawShape(LXYs, 'y', '', '-', 2);
    handles.BestXYs = LXYs';
    [E I] = sort(Errs, 'ascend');
    I = I(1:64);
    I1 = 2*I-1;
    I2 = 2*I;
    GX = XYs(I1);
    GY = XYs(I2);
    hold on;
%     plot(GX, GY, 'gv', 'MarkerSize', 15);
    plot(GX, GY, 'gd');
    handles.Errs = Errs;
    hold off;
    guidata(hObject, handles);
end


% --- Executes on button press in pushbutton9.
% Reconstruct the shape from the searched landmarks
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if isfield(handles, 'BestXYs')
    d = (reshape(handles.BestXYs, [1 length(handles.BestXYs)]) - handles.Mean');
    bound = max(handles.EigVal, zeros(size(handles.EigVal,1), size(handles.EigVal,2)));
    b = min(max(d*handles.EigVec,-1*sqrt(bound')),1*sqrt(bound'));
    k = round(128*1);
    ReconstructedShape = b(1:k) * handles.EigVec(:,1:k)' + handles.Mean';
    hold on;
    DrawShape(ReconstructedShape, 'b', '', '-', 3);
    hold off;
    handles.Recon = ReconstructedShape;
    guidata(hObject, handles);
end

% --- Executes on button press in pushbutton10.
% Reconstruct shape by constraints
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Diff = abs(reshape(handles.BestXYs,[1 length(handles.BestXYs)]) - reshape(handles.aligned,[1, length(handles.aligned)]));
CurShape = reshape(handles.BestXYs, [length(handles.BestXYs) 1]);
Ref = reshape(handles.Recon, [length(handles.Recon) 1]);
%Ref = reshape(handles.aligned, [length(handles.aligned) 1]);
Diff = (CurShape - Ref).^2;
diffx = Diff(1:2:length(Diff));
diffy = Diff(2:2:length(Diff));
diff = sqrt(diffx + diffy);
diff = reshape([diff';diff'], [1 2*length(diff)]);
ind = diff<=3;
[SortedErrs ErrInd] = sort(handles.Errs);
K = 30;
ind2 = false(1, length(ind));
for kkk = 1:K
    ind2(ErrInd(kkk)*2-1) = true;
    ind2(ErrInd(kkk)*2) = true;
end
ind = ind2&ind;
%W = diag(1-(1+exp(-log(diff))).^(-1));
alpha = 1000;
W = diag(exp(-alpha*(diff)));
%  W = eye(length(diff));
%bbb = ConstrainedReconstruct(W, handles.EigVec, (handles.Recon'-handles.Mean), (CurShape-handles.Mean), -2*sqrt(abs(handles.EigVal)), 2*sqrt(abs(handles.EigVal)), ind);
bound = max(handles.EigVal, zeros(size(handles.EigVal,1), size(handles.EigVal,2)));
bbb = ConstrainedReconstruct(W, handles.EigVec, (CurShape-handles.Mean), (CurShape-handles.Mean), -5*sqrt(abs(bound)), 5*sqrt(abs(bound)), ind, handles.Recon, 10);
k = round(128*1);
CReconstructedShape = handles.EigVec(:,1:k)*bbb(1:k) + handles.Mean;
hold on;
DrawShape(CReconstructedShape, 'k', '', '-', 3);
mark = CurShape(ind);
spx4 = mark(1:2:length(mark));
spy4 = mark(2:2:length(mark));
hold on;
% plot(spx4, spy4, 'go','LineWidth',2, 'markersize', 10);
hold off;
handles.CurCRecon = CReconstructedShape;
guidata(hObject, handles);


% --- Executes on button press in pushbutton11.
% Copy the current reconstruction as the new initialized landmarks
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.aligned = handles.Recon;
guidata(hObject, handles);


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.aligned = handles.CurCRecon;
guidata(hObject, handles);


% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.aligned = handles.BestXYs;
guidata(hObject, handles);


% --- Executes on button press in pushbutton14.
% Start to track the face after initialize landmarks.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if isfield(handles, 'warpImg') && isfield(handles, 'aligned')
    GrayIn = rgb2gray(handles.warpImg);
    InitXYs = handles.aligned;
    SearchWSize = 100;
    load('AlignedImages\EigFeatMean.mat');
    load('AlignedImages\EigFeatStd.mat');
    Template = EigMeanData;
    TemplateStd = EigStdData;
    axes(handles.WarpAxe);
    kkk = 1;
    while SearchWSize>=12
        [XYs Errs Xs Ys] = SearchAllLandmarksByDP(GrayIn, InitXYs, SearchWSize, sqrt(length(Template(1,:))), Template, TemplateStd);
        hold off;
        HighLightMark(handles);
        hold on;
        facialParts = [1 2 5 6 7 8 13 14 15]; %Jaw part
        orders = [3 3 3 3 3 3 3 3 3];
        [newXs newYs] = RefineLandmarks(facialParts, orders, Xs, Ys);
        LX = newXs; %XYs(1:2:length(XYs));
        LY = newYs; %XYs(2:2:length(XYs));
        plot(LX, LY, 'y*');
        LXYs = reshape([LX;LY], [length(LX)*2 1]);
        DrawShape(LXYs, 'y', '', '-', 2);
        handles.BestXYs = LXYs';
        d = (reshape(handles.BestXYs, [1 length(handles.BestXYs)]) - handles.Mean');
        bound = max(handles.EigVal, zeros(size(handles.EigVal,1), size(handles.EigVal,2)));
        b = min(max(d*handles.EigVec,-1*sqrt(bound')),1*sqrt(bound'));
        k = round(128*1);
        ReconstructedShape = b(1:k) * handles.EigVec(:,1:k)' + handles.Mean';
        DrawShape(ReconstructedShape, 'b', '', '-', 2);
        handles.Recon = ReconstructedShape;
        handles.aligned = handles.Recon;
        InitXYs = handles.aligned;
        if (rem(kkk,3)==0)
            SearchWSize = SearchWSize - 5;
        end
        kkk = kkk + 1;
        handles.Errs = Errs;
        drawnow;
    end
    hold off;
end

% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    GrayIn = rgb2gray(handles.warpImg);
    InitXYs = handles.aligned;
    SearchWSize = 30;
    load('AlignedImages\EigFeatMean.mat');
    load('AlignedImages\EigFeatStd.mat');
    Template = EigMeanData;
    TemplateStd = EigStdData;
    axes(handles.WarpAxe);
    kkkk = 1;
    while SearchWSize>=8
        [XYs Errs Xs Ys] = SearchAllLandmarksByDP(GrayIn, InitXYs, SearchWSize, sqrt(length(Template(1,:))), Template, TemplateStd);
        handles.Errs = Errs;
        hold off;
        HighLightMark(handles);
        hold on;
        LXYs = reshape([Xs;Ys], [length(Xs)*2 1]);
        DrawShape(LXYs, 'y', '*', '-', 1);
        handles.BestXYs = LXYs';
        d = (reshape(handles.BestXYs, [1 length(handles.BestXYs)]) - handles.Mean');
        bound = max(handles.EigVal, zeros(size(handles.EigVal,1), size(handles.EigVal,2)));
        b = min(max(d*handles.EigVec,-1*sqrt(bound')),1*sqrt(bound'));
        k = round(128*1);
        ReconstructedShape = b(1:k) * handles.EigVec(:,1:k)' + handles.Mean';
%         DrawShape(ReconstructedShape, 'b', '', '-', 2);
        handles.Recon = ReconstructedShape;
        CurShape = reshape(handles.BestXYs, [length(handles.BestXYs) 1]);
        Ref = reshape(handles.Recon, [length(handles.Recon) 1]);
%Ref = reshape(handles.aligned, [length(handles.aligned) 1]);
        Diff = (CurShape - Ref).^2;
        diffx = Diff(1:2:length(Diff));
        diffy = Diff(2:2:length(Diff));
        diff = sqrt(diffx + diffy);
        diff = reshape([diff';diff'], [1 2*length(diff)]);
        ind = diff<=3;
        [SortedErrs ErrInd] = sort(handles.Errs);
        K = 20;
        ind2 = false(1, length(ind));
        for kkk = 1:K
            ind2(ErrInd(kkk)*2-1) = true;
            ind2(ErrInd(kkk)*2) = true;
        end
        ind = ind2&ind;
        alpha = 1000;
        W = diag(exp(-alpha*(diff)));
%  W = eye(length(diff));
%bbb = ConstrainedReconstruct(W, handles.EigVec, (handles.Recon'-handles.Mean), (CurShape-handles.Mean), -2*sqrt(abs(handles.EigVal)), 2*sqrt(abs(handles.EigVal)), ind);
        bound = max(handles.EigVal, zeros(size(handles.EigVal,1), size(handles.EigVal,2)));
        bbb = ConstrainedReconstruct(W, handles.EigVec, (CurShape-handles.Mean), (CurShape-handles.Mean), -5*sqrt(abs(bound)), 5*sqrt(abs(bound)), ind,  handles.Recon, 10);
        k = round(128*1);
        CReconstructedShape = handles.EigVec(:,1:k)*bbb(1:k) + handles.Mean;
        handles.aligned =CReconstructedShape;
        handles.CurCRecon = CReconstructedShape;
        hold on;
        DrawShape(CReconstructedShape, 'g', '', '-', 1);
        mark = CurShape(ind);
        spx4 = mark(1:2:length(mark));
        spy4 = mark(2:2:length(mark));
        hold on;
% plot(spx4, spy4, 'go','LineWidth',2, 'markersize', 10);
%         hold off;
        InitXYs = handles.aligned;
        if rem(kkkk,2)==0
            SearchWSize = SearchWSize - 1;
        end
        kkkk = kkkk + 1;
        handles.Errs = Errs;
        drawnow;
    end
    hold off;
