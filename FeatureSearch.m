function [BestPt ScoreMap] = FeatureSearch(Img, InitPt, HWndSize, FeatWndSize, Template, TemplateStd, Skip, SearchType)
global S
[w h c] = size(Img);
p =HWndSize;
BestPt = InitPt;
sz = floor(FeatWndSize/2);
BestMatchScore = 99999999;
ScoreMap = zeros(2*p+1,2*p+1);
if (SearchType==2)
    load('AlignedImages\Neighbor.txt');
    NormalDir = InitXYs(Neighbor(n,1),:)-InitXYs(Neighbor(n,2),:);
    NormalDir = NormalDir/norm(NormalDir);
    for np = -p:p
        CurXY = InitXYs(n) + np * NormalDir;
        xxx = CurXY(1);
        yyy = CurXY(2);
        if ((yyy-S*sz)<1||(xxx-S*sz)<1||(yyy+S*sz)>h||(xxx+S*sz)>w)
            continue;
        end
        Feat = FeatExtract(xxx,yyy,Img,S,sz,FeatWndSize);%reshape(double(Img(yyy-S*sz:S:yyy+S*sz, xxx-S*sz:S:xxx+S*sz)),[1 FeatWndSize*FeatWndSize]);
        %Feat = Feat./norm(Feat);
        MatchScore = sum(abs(Feat-Template)./TemplateStd);
        ScoreMap(yyy-InitPt(2)+p+1, xxx-InitPt(1)+p+1) = exp(-0.04*MatchScore);
        if (MatchScore < BestMatchScore)
            BestMatchScore = MatchScore;
            BestPt = [xxx yyy];
        end
    end
else
    for yyy = InitPt(2)-p:Skip:InitPt(2)+p
        for xxx = InitPt(1)-p:Skip:InitPt(1)+p
            if ((yyy-S*sz)<1||(xxx-S*sz)<1||(yyy+S*sz)>h||(xxx+S*sz)>w)
                continue;
            end
            Feat = FeatExtract(xxx,yyy,Img,S,sz,FeatWndSize);%reshape(double(Img(yyy-S*sz:S:yyy+S*sz, xxx-S*sz:S:xxx+S*sz)),[1 FeatWndSize*FeatWndSize]);
            %Feat = Feat./norm(Feat);
            MatchScore = sum(abs(Feat-Template)./TemplateStd);
            ScoreMap(yyy-InitPt(2)+p+1, xxx-InitPt(1)+p+1) = exp(-0.04*MatchScore);
            if (MatchScore < BestMatchScore)
                BestMatchScore = MatchScore;
                BestPt = [xxx yyy];
            end
        end
    end
end