function [ outX outY ] = rectifyPoint(x, y, markID, markXPos, markYPos, assocTable, assocVec)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
table = assocTable(markID, :);
len = length(table);
pts = zeros(len, 2);
for i=1:length(table)
    pts(i,1) = (markXPos(markID) + markXPos(table(i)+1) +  assocVec(markID, i, 1))/2;
    pts(i,2) = (markYPos(markID) + markYPos(table(i)+1) +  assocVec(markID, i, 2))/2;
end
m = mean(pts);
outX = m(1);
outY = m(2);
end

