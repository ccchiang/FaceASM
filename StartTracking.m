function StartTracking(handles, PCAGroups, RefinedVGs, PolyOrders)
global LandmarkGroups;
global FacialPartName;
if isfield(handles, 'warpImg') && isfield(handles, 'aligned')
    GrayI = rgb2gray(handles.warpImg);
    GrayIn = cv.bilateralFilter(GrayI, 'SigmaColor', 60, 'Diameter', 11);
    load('AlignedImages\EigFeatMean.mat');
    load('AlignedImages\EigFeatStd.mat');
%     PCAGroups = {1,12}; %first: all, second: mouth 
%     RefinedVGs = {[1 2 5 6 7 8 13 14 15],[13 14]};
%     PolyOrders = {[3 3 3 3 3 3 3 3 3 4],[3 3]};
    NoGroups = length(PCAGroups);
    EigVal = cell(1, NoGroups);
    EigVec = cell(1, NoGroups);
    Mean = cell(1, NoGroups);
    for g = 1:NoGroups
        SearchWSize = 50-g*10;
        InitXYs = handles.aligned;
        if PCAGroups(g)==1
            EigVal{g} = load('AlignedImages\EigenFaceValue.txt');
            EigVec{g} = load('AlignedImages\EigenFaceVec.txt');
            Mean{g} = load('AlignedImages\MeanFace.txt');
        else
            contents = cellstr(get(handles.listbox2,'String')) ;
            GName = contents{PCAGroups(g)};
            EigVal{g} = load(['AlignedImages\' GName '_EigenFaceValue.txt']);
            EigVec{g} = load(['AlignedImages\' GName '_EigenFaceVec.txt']);
            Mean{g} = load(['AlignedImages\' GName '_MeanFace.txt']);
            set(handles.text1, 'String', [GName ' Tracking started!']);
            set(handles.text1, 'ForegroundColor', [1 0 0]);
            drawnow;
        end
        landmarkIDs = GetLandmarksForComponentGroup(PCAGroups(g));
        landmarks = reshape([2*landmarkIDs-1;2*landmarkIDs], [1 2*length(landmarkIDs)]);
        sort(landmarks);
        Template = EigMeanData;
        TemplateStd = EigStdData;
        axes(handles.WarpAxe);
        kkk = 1;
        while SearchWSize>=4
            [XYs Errs Xs Ys] = SearchAllLandmarksByDP(GrayIn, InitXYs, SearchWSize, sqrt(length(Template(1,:))), Template, TemplateStd);
            hold off;
            imshow(handles.warpImg);
            if get(handles.checkbox2, 'value')==1
                HighLightMark(handles);
            end
            hold on;
            facialParts = RefinedVGs{g}; %Jaw part
            orders = PolyOrders{g};
            LX = InitXYs(1:2:end, 1)';
            LY = InitXYs(2:2:end, 1)';
            [newXs newYs] = RefineLandmarks(facialParts, orders, Xs, Ys);
            LX(landmarkIDs) = newXs(landmarkIDs) ; %XYs(1:2:length(XYs));
            LY(landmarkIDs)  = newYs(landmarkIDs) ; %XYs(2:2:length(XYs));
            if get(handles.checkbox2, 'value')==1
                plot(LX, LY, 'y*');
            end
            LXYs = reshape([LX;LY], [length(LX)*2 1]);
            %         if get(handles.checkbox2, 'value')==1
            %             DrawShape(LXYs, 'y', '', '-', 2);
            %         end
            handles.BestXYs = LXYs';
            ReconstructedShape = InitXYs;
            alpha = 0.8;
            d = (reshape(handles.BestXYs(landmarks), [1 length(handles.BestXYs(landmarks))]) - Mean{g}');
            bound = max(EigVal{g}, zeros(size(EigVal{g},1), size(EigVal{g},2)));
            b = min(max(d*EigVec{g},-alpha*sqrt(bound')),alpha*sqrt(bound'));
            k = length(b);%round(128*1);
            ReconstructedShape(landmarks) = b(1:k) * EigVec{g}(:,1:k)' + Mean{g}';
            if get(handles.checkbox2, 'value')==1
                lineStyle = 'o-';
            else
                lineStyle = '-';
            end
            DrawShape(LXYs, 'y', '', lineStyle, 2);
            %DrawShape(ReconstructedShape, 'g', '', lineStyle, 2);
            handles.Recon = ReconstructedShape;
            handles.aligned = handles.Recon;
            InitXYs = handles.aligned;
            if (rem(kkk,3)==0)
                SearchWSize = SearchWSize - 2;
            end
            kkk = kkk + 1;
            handles.Errs = Errs;
            drawnow;
        end
    end
    hold off;
    set(handles.text1, 'String', 'Tracking finished!');
    set(handles.text1, 'ForegroundColor', [0 0 1]);
end