function HighLightMark(handles)
global LandmarkID
if ~isfield(handles, 'warpImg')||~isfield(handles,'SelectedRBtnHandle')
    return;
end
handles.SelectedCompId = get(handles.SelectedRBtnHandle, 'UserData');
cmd = ['LandmarkID = round(get(handles.slider' num2str(handles.SelectedCompId) ', ''Value''));'];
eval(cmd);
X = handles.aligned(1:2:length(handles.aligned));
Y = handles.aligned(2:2:length(handles.aligned));
axes(handles.WarpAxe);
% hold off;
% imshow(handles.warpImg);
hold on;
plot(X, Y, 'r.', X(LandmarkID), Y(LandmarkID), 'go');
hold off;