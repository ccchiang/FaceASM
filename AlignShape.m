function [out v A] = AlignShape(input, ref)
len = length(input)/2;
A = [];
for i=1:len
    A = [A ; input((i-1)*2+1) 0 -input((i-1)*2+2) 0 1 0;0 input((i-1)*2+1) 0 input((i-1)*2+2) 0 1];
end
v = pinv(A)*ref';
out = A*v;