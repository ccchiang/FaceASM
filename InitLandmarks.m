function WarpImg = InitLandmarks(handles)
axes(handles.ImageAxe);
RefLandmarkID = [26 18 56 60 7];
[X Y] = ginput(length(RefLandmarkID))
XY = [X';Y'];
Input = reshape(XY, [1,size(XY,1)*size(XY,2)]);
Idx = zeros(1, 2*length(RefLandmarkID));
for i = 1:length(RefLandmarkID)
    Idx(2*i-1) = 2*RefLandmarkID(i)-1;
    Idx(2*i) = 2*RefLandmarkID(i);
end
stop = false;
Ref = handles.ShapeData(1,Idx);
handles.aligned = handles.ShapeData(1,:);
iter = 0;
axes(handles.WarpAxe);
axis ij;
axis image;
axis off;
load('AlignedImages\EigFeatMean.mat');
load('AlignedImages\EigFeatStd.mat');
Template = EigMeanData;
TemplateStd = EigStdData;
lastnpts = -1;
MAX_ITER = 20;
% Frames = moviein(MAX_ITER); 
nochange = 0;
while iter < MAX_ITER
    [Aligned V A] = AlignShape(Input, Ref);
    NewWarpImg = WarpImage(handles.InImg, V);
    hold off;
    imshow(NewWarpImg);
    len = length(handles.aligned);
    Sx = handles.aligned(1:2:len);
    Sy = handles.aligned(2:2:len);
    hold on;
    plot(Sx, Sy, 'b.');
    GrayIn = rgb2gray(NewWarpImg);
    InitXYs = handles.aligned;
    Items = get(handles.popupmenu1, 'String');
    SearchWSize = str2num(Items{get(handles.popupmenu1, 'Value')});
    XYs = SearchAllLandmarks(GrayIn, InitXYs, SearchWSize, sqrt(length(Template(1,:))), Template, TemplateStd, 2);
%     [XYs Errs Xs Ys] = SearchAllLandmarksByDP(GrayIn, InitXYs, SearchWSize, sqrt(length(Template(1,:))), Template, TemplateStd);
%     d = (reshape(XYs, [1 length(XYs)]) - handles.Mean');
%     b = min(max(d*handles.EigVec,-1.0*(handles.EigVal')),1.0*(handles.EigVal'));
%     k = round(128*1);
%     ReconstructedShape = b(1:k) * handles.EigVec(:,1:k)' + handles.Mean';
%     CurShape = reshape([Xs';Ys'], [1 length(Xs)*2]);
    CurShape = reshape(XYs, [1 length(XYs)]);
    Ref = reshape(handles.ShapeData(1,:), [1 length( handles.ShapeData(1,:))]);
%     hold on;
%     DrawShape(CurShape, 'r', 'x', '', 2);
%     DrawShape(Ref, 'g', 'x', '', 2);
%     hold off;
    Diff = (CurShape - Ref).^2;
    diffx = Diff(1:2:length(Diff));
    diffy = Diff(2:2:length(Diff));
    diff = sqrt(diffx + diffy);
    diff = reshape([diff;diff], [1 2*length(diff)]);
    ind = diff<=6;
    npts = sum(ind)
    if npts <lastnpts || nochange==5
        break;
    elseif npts==lastnpts
        nochange = nochange + 1;
    else
        nochange = 0;
        lastnpts = npts;
        WarpImg = NewWarpImg;
    end
    mark = CurShape(ind);
    spx4 = mark(1:2:length(mark));
    spy4 = mark(2:2:length(mark));
    hold on;
    plot(spx4, spy4, 'yd','LineWidth',2, 'markersize', 10);   
    drawnow;
    ux = V(1);
    uy = V(2);
    vx = V(3);
    vy = V(4);
    tx = V(5);
    ty = V(6);
    BestPts = CurShape(ind);
    a = BestPts(1:2:length(BestPts));
    b = BestPts(2:2:length(BestPts));
    shifted_ab = [a-tx;b-ty];
    xy =  floor(inv([ux -vx;uy vy])*(shifted_ab));
    Ref = handles.ShapeData(1,ind);
    Input = reshape(xy, [1 size(xy,1)*size(xy,2)]);
    iter = iter + 1;
end