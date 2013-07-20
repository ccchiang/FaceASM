function [coeff outXs outYs] = PolyFitting(xx, yy, order, th)
iter = 1
len = length(xx);
A = zeros(len,order);
Y = zeros(len,1);
for i=1:len
    for j=order:-1:1
        A(i,order-j+1) = xx(i)^(j-1);
    end
    Y(i) = yy(i);
end
coeff = pinv(A)*Y;
val = A*coeff;
% [xx yy val]
MaxIter = 0;
while iter <= MaxIter
    err = abs(val-yy);
    flag = err<=(MaxIter-iter)/2+1;
    xxx = xx(flag);
    yyy = yy(flag);
    len = length(xxx);
    AA = zeros(len,order);
    YY = zeros(len,1);
    for i=1:len
        for j=order:-1:1
            AA(i,order-j+1) = xxx(i)^(j-1);
        end
        YY(i) = yyy(i);
    end
    coeff = pinv(AA)*YY;
    val = A*coeff;
    iter = iter + 1
%     [xx yy val];
end
val = polyval(coeff, xx);
err = abs(yy-val);
ind = (find(err<=th));
xxx = (xx(err<=th));
n = length(ind);
outXs = xx;
outYs = yy;
for i=1:n
    if ind(i)==1
        dir = [xx(1)-xx(2) yy(1)-yy(2)];
    elseif ind(i) ==len
        dir = [xx(len-1)-xx(len) yy(len-1)-yy(len)];
    else
        dir = [xx(ind(i)-1)-xx(ind(i)) yy(ind(i)-1)-yy(ind(i))];
    end
    normalDir = [-dir(2) dir(1)];
    normalDir = normalDir/norm(normalDir);
    delta = 10;
    range = [-delta:delta];
    xs = xx(ind(i))+range*normalDir(1);
    ys = yy(ind(i))+range*normalDir(2);
    err = abs(polyval(coeff, xs)-ys);
    outXs(ind(i)) = xs(err==min(err));
    %outYs(ind(i)) = ys(err==min(err));
end
outYs = polyval(coeff, outXs);
%[outXs xx outYs yy]
% for i = 1:n-1
%     dx = (xxx(i+1)-xxx(i))/(ind(i+1)-ind(i));
%     for j=ind(i)+1:ind(i+1)-1
%         outXs(j) = outXs(j-1) + dx; 
%     end
% end
% if ind(1)~=1
%     for j=ind(1)-1:-1:1
%         outXs(j) = outXs(j+1) - dx;
%     end
% end
% if ind(n)~=length(xx)
%     for j=ind(n)+1:length(xx)
%         outXs(j) = outXs(j-1) + dx;
%     end
% end
% 
% outYs = polyval(coeff, outXs);
% plot(xx,yy,'gx-',xx,val,'r-', outXs, outYs, 'bo-');
