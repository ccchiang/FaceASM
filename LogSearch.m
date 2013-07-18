function pt = LogSearch(Img, InitPt, HWndSize, FeatWndSize, Template, Iter)
[w h c] = size(Img);
p =HWndSize;
pt = InitPt;
sz = floor(FeatWndSize/2);
BestMatchScore = 99999999;
for t = 1:Iter
    Offset = [-p -p; 0 -p; p -p; -p 0; 0 0; p 0; -p p; 0 p; p p];
    for k = 1:9
        yyy = pt(2)+Offset(k,2);
        xxx = pt(1)+Offset(k,1);
        if ((t==1&&k==5)||(yyy-sz)<1||(xxx-sz)<1||(yyy+sz)>h||(xxx+sz)>w)
            continue;
        end
        Feat = reshape(double(Img(yyy-sz:yyy+sz, xxx-sz:xxx+sz)),[1 FeatWndSize*FeatWndSize]);
        Feat = Feat./norm(Feat);
        MatchScore = sum(abs(Feat-Template));
        if (MatchScore < BestMatchScore)
            BestMatchScore = MatchScore;
            BestPt = [xxx yyy];
        end
    end
    pt = BestPt;
    if (p==1)
        break;
    end
    p = p/2;
end