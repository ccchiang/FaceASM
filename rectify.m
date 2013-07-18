function varargout = rectify(varargin)
% RECTIFY M-file for rectify.fig
%      RECTIFY, by itself, creates a new RECTIFY or raises the existing
%      singleton*.
%
%      H = RECTIFY returns the handle to a new RECTIFY or the handle to
%      the existing singleton*.
%
%      RECTIFY('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RECTIFY.M with the given input arguments.
%
%      RECTIFY('Property','Value',...) creates a new RECTIFY or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before rectify_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to rectify_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help rectify

% Last Modified by GUIDE v2.5 02-May-2012 19:08:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @rectify_OpeningFcn, ...
                   'gui_OutputFcn',  @rectify_OutputFcn, ...
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


% --- Executes just before rectify is made visible.
function rectify_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to rectify (see VARARGIN)

% Choose default command line output for rectify
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes rectify wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = rectify_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in pidList.
function markList_Callback(hObject, eventdata, handles)
% hObject    handle to pidList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns pidList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from pidList
markX = handles.markPos(:,1);
markY = handles.markPos(:,2);
markID = get(hObject, 'Value');
plot(handles.faceAxes, markX, 60-markY, 'o', markX(markID), 60-markY(markID), 'rx', markX(markID), 60-markY(markID), 'ro'); 
hold on;
plot(handles.faceAxes, handles.meanMarkX, 60-handles.meanMarkY, 'r+', handles.meanMarkX(markID), 60-handles.meanMarkY(markID), 'ro');
hold off;

% --- Executes during object creation, after setting all properties.
function pidList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pidList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in upBtn.
function upBtn_Callback(hObject, eventdata, handles)
% hObject    handle to upBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in leftBtn.
function leftBtn_Callback(hObject, eventdata, handles)
% hObject    handle to leftBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in rightBtn.
function rightBtn_Callback(hObject, eventdata, handles)
% hObject    handle to rightBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in downBtn.
function downBtn_Callback(hObject, eventdata, handles)
% hObject    handle to downBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in mouseBtn.
function mouseBtn_Callback(hObject, eventdata, handles)
% hObject    handle to mouseBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x y] = ginput(1);
hold on;
plot(handles.faceAxes, x, y, 'go');
markID = get(handles.markList, 'Value');
%plot(handles.faceAxes, handles.markX, 60-handles.markY, 'o', markX(markID), 60-markY(markID), 'rx', markX(markID), 60-markY(markID), 'ro');
newY = 60 + y;
[x y] = rectifyPoint(x, newY, markID, handles.markPos(:,1), handles.markPos(:,2), handles.assocTable, handles.assocVec);
plot(handles.faceAxes, x, 60-y, 'g+', x, 60-y, 'go');
plot(handles.faceAxes, handles.meanMarkX, 60-handles.meanMarkY, 'r+');
plot(handles.faceAxes, handles.meanMarkX(markID), 60-handles.meanMarkY(markID), 'ro');
hold off;



% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pidList.
function pidList_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to pidList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over mouseBtn.
function mouseBtn_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to mouseBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in rectifyAll.
function rectifyAll_Callback(hObject, eventdata, handles)
% hObject    handle to rectifyAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hold on;
for markID=1:length(handles.markPos(:,1))
    xx = handles.markPos(markID, 1);
    yy = handles.markPos(markID, 2);
    [x y] = rectifyPoint(xx, yy, markID, handles.markPos(:,1), handles.markPos(:,2), handles.assocTable, handles.assocVec);
    plot(handles.faceAxes, x, 60-y, 'k*', x, 60-y, 'ko');
end
hold off;


% --- Executes on selection change in markList.
function pidList_Callback(hObject, eventdata, handles)
% hObject    handle to markList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns markList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from markList
pid = get(hObject, 'Value');
data = guidata(handles.fig);
data.truth = handles.allMarkPos(:,2*(pid-1)+1:2*(pid-1)+2);
data.markPos = perturb(data.truth,10);
plot(handles.faceAxes, data.markPos(:,1), 60-data.markPos(:,2), 'o');
hold on;
plot(handles.faceAxes, data.truth(:,1), 60-data.truth(:,2), 'r+');
hold off;
guidata(handles.fig, data);

% --- Executes during object creation, after setting all properties.
function markList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to markList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
