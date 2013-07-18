function [assocVec assocTable markPos meanMarkX meanMarkY] = trainAssoc( assocFile, markPosFile )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    markPos = load(markPosFile, '-ascii');
    sz2 = size(markPos);
    meanMarkX = zeros(sz2(1),1);
    meanMarkY = zeros(sz2(1),1);
    for i=1:sz2(1)
        meanMarkX(i) = mean(markPos(i, 1:2:sz2(2))');
        meanMarkY(i) = mean(markPos(i, 2:2:sz2(2))');
    end
    assocTable = load(assocFile, '-ascii');
    sz1 = size(assocTable);
    assocTable = assocTable(:, 2:sz1(2));
    assocVec = zeros(sz2(1), sz1(2)-1, 2);
    for i=1:sz1(1)
        for j=1:sz1(2)-1
            assocVec(i,j,1) = meanMarkX(i) - meanMarkX(assocTable(i,j)+1);
            assocVec(i,j,2) = meanMarkY(i) - meanMarkY(assocTable(i,j)+1);
        end
    end
end

