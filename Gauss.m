function prob = Gauss( x, m, S )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    sz = size(x);
    prob = zeros(sz(1),1);
    for i=1:sz(1);
        prob(i) = 1/det(S)*exp(-0.5*(x(i,:)-m)*inv(S)*(x(i,:)-m)');
    end
end

