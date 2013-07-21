function [XYs Errs NewXs NewYs] = SearchAllLandmarksByDP(GrayImg, InitXYs, SearchWSize, FeatWSize, Templates, TemplateStds)
global S
[h w] = size(GrayImg);
p =ceil(SearchWSize/2);
XYs = InitXYs;
sz = floor(FeatWSize/2);
NoOfLandmarks = length(InitXYs)/2;
load('AlignedImages\Neighbor.txt');
NodeErrs = zeros(NoOfLandmarks, 2*p+1);
XX = NodeErrs;
YY = NodeErrs;
Errs = zeros(NoOfLandmarks, 1);
NewXs = zeros(1, NoOfLandmarks);
NewYs = zeros(1, NoOfLandmarks);
for n=1:NoOfLandmarks
    X = round(InitXYs(n*2-1));
    Y = round(InitXYs(n*2));
    BestMatchScore = 99999999;
    N1 = [InitXYs(Neighbor(n,1)*2-1) InitXYs(Neighbor(n,1)*2)];
    N2 = [InitXYs(Neighbor(n,2)*2-1) InitXYs(Neighbor(n,2)*2)];
    NormalDir = N1 - N2;
    NV = [-NormalDir(2) NormalDir(1)];
    NormalDir = 0.5*NV/norm(NV);
    for np = -p:p
        CurXY = [InitXYs(n*2-1) InitXYs(n*2)] + np * NormalDir;
        xxx = round(CurXY(1));
        yyy = round(CurXY(2));
        XX(n, np+p+1) = xxx;
        YY(n, np+p+1) = yyy;
        if ((yyy-S*sz)<1||(xxx-S*sz)<1||(yyy+S*sz)>h||(xxx+S*sz)>w)
            NodeErrs(n, np+p+1) = 999999.0;
            continue;
        end
        Feat = FeatExtract(xxx,yyy,GrayImg,S,sz,FeatWSize);%reshape(double(GrayImg(yyy-S*sz:S:yyy+S*sz, xxx-S*sz:S:xxx+S*sz)),[1 FeatWSize*FeatWSize]);
        %Feat = Feat./norm(Feat);
        MatchScore = sum(abs(Feat-Templates(n,:))./TemplateStds(n,:));
        NodeErrs(n, np+p+1) = MatchScore;
        if (MatchScore < BestMatchScore)
            BestMatchScore = MatchScore;
            XYs(n*2-1) = xxx;
            XYs(n*2) = yyy;
        end
    end
    Errs(n) = BestMatchScore;
end
ComponentMarkIDs = {1:13 14:21 22:29 30:34 35:39 40:51 52:64};
AccErrs = NodeErrs;
Idx = NodeErrs;
for c = 1:length(ComponentMarkIDs)
    MarkList = ComponentMarkIDs{c};
    R = p;
    for k=MarkList(2):MarkList(length(MarkList));
        for pp = 1:2*p+1;
            if pp<=R
                [AccErrs(k, pp) I] = min(AccErrs(k-1,1:pp+R));
                Idx(k,pp) = I;
            elseif pp>=2*p+2-R
                [AccErrs(k, pp) I] = min(AccErrs(k-1, pp-R:2*p+1));
                Idx(k,pp) = I + pp - R - 1;
            else
                [AccErrs(k, pp) I] = min(AccErrs(k-1, pp-R:pp+R));
                Idx(k,pp) = I + pp - R - 1;
            end
            AccErrs(k, pp) = AccErrs(k, pp) + NodeErrs(k, pp);
        end
    end
    [MinErr I] = min(AccErrs(MarkList(length(MarkList)), :));
    % NewXYs = zeros(length(MarkList)*2, 1);
    NewXs(MarkList(length(MarkList))) = XX(MarkList(length(MarkList)), I);
    NewYs(MarkList(length(MarkList))) = YY(MarkList(length(MarkList)), I);
    Errs(MarkList(length(MarkList))) = NodeErrs(MarkList(length(MarkList)), I);
    for k = MarkList(length(MarkList)):-1:MarkList(2)
        I = Idx(k, I);
        Errs(k-1) = NodeErrs(k-1, I);
        NewXs(k-1) = XX(k-1, I);
        NewYs(k-1) = YY(k-1, I);
    end
end