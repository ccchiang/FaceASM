%FacialParts enumerates the parts whose landmarks need to be refined
%XYs is a Mx2 matrix containing the X-Y coordinates of landmarks to be refined
function [refinedXs refinedYs] = RefineLandmarks(FacialParts, orders, Xs, Ys)
global LandmarkGroups;
no_parts = length(FacialParts);
refinedXs = Xs;
refinedYs = Ys;
for i = 1:no_parts
    landmarks = LandmarkGroups{FacialParts(i)};
    partXs = Xs(landmarks);
    partYs = Ys(landmarks);
    threshold = 3;
    %order = 3; %3: quadratic polynomial, 4:cubic polynomial
    partXs = reshape(partXs, length(partXs), 1);
    partYs = reshape(partYs, length(partYs), 1);
    [coeff outXs outYs] = PolyFitting(partXs, partYs, orders(i), threshold);
    refinedXs(landmarks) = outXs;
    refinedYs(landmarks) = outYs;
end