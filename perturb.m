function newMarkPos = perturb( markPos, scale )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
sz = size(markPos);
perturbation = scale*rand(sz(1), sz(2));
newMarkPos = markPos + perturbation;
end

