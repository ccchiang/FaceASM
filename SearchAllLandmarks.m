function [XYs Errs NewXs NewYs] = SearchAllLandmarks(GrayImg, InitXYs, SearchWSize, FeatWSize, Templates, TemplateStds, SearchType)
global S
[h w] = size(GrayImg);
p =SearchWSize/2;
XYs = InitXYs;
sz = floor(FeatWSize/2);
NoOfLandmarks = length(InitXYs)/2;
if (SearchType==2)
    load('AlignedImages\Neighbor.txt');
end
NodeErrs = zeros(NoOfLandmarks, 2*p+1);
XX = NodeErrs;
YY = NodeErrs;
Errs = zeros(NoOfLandmarks, 1);
MarkList =52:64;
for n=1:NoOfLandmarks
    X = round(InitXYs(n*2-1));
    Y = round(InitXYs(n*2));
    BestMatchScore = 99999999;
    if (SearchType==1) % search within a rectangle area
        for yyy = Y-p:Y+p
            for xxx = X-p:X+p
                if ((yyy-S*sz)<1||(xxx-S*sz)<1||(yyy+S*sz)>h||(xxx+S*sz)>w)
                    continue;
                end
                Feat = FeatExtract(xxx,yyy,GrayImg,S,sz,FeatWSize);%reshape(double(GrayImg(yyy-S*sz:S:yyy+S*sz, xxx-S*sz:S:xxx+S*sz)),[1 FeatWSize*FeatWSize]);
                %Feat = Feat./norm(Feat);
                MatchScore = sum(abs(Feat-Templates(n,:))./TemplateStds(n,:));
                if (MatchScore < BestMatchScore)
                    BestMatchScore = MatchScore;
                    XYs(n*2-1) = xxx;
                    XYs(n*2) = yyy;
                end
            end
        end
    else % search along the normal direction
        N1 = [InitXYs(Neighbor(n,1)*2-1) InitXYs(Neighbor(n,1)*2)];
        N2 =  [InitXYs(Neighbor(n,2)*2-1) InitXYs(Neighbor(n,2)*2)];
        NormalDir = N1 - N2;
        NV = [-NormalDir(2) NormalDir(1)];
        NormalDir = NV/norm(NV);
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
    end
    Errs(n) = BestMatchScore;
end
AccErrs = NodeErrs;
Idx = NodeErrs;
R = 4;
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
NewXs = XX(MarkList(length(MarkList)), I);
NewYs = YY(MarkList(length(MarkList)), I);
for k = MarkList(length(MarkList)):-1:MarkList(2)
    I = Idx(k, I);
    NewXs = [NewXs XX(k-1, I)];
    NewYs = [NewYs YY(k-1, I)];
end
a = 1