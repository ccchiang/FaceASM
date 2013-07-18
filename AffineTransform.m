function TransformedXYs = AffineTransform(input, v)
len = length(input)/2;
A = [];
for i=1:len
    A = [A ; input((i-1)*2+1) 0 -input((i-1)*2+2) 0 1 0;0 input((i-1)*2+1) 0 input((i-1)*2+2) 0 1];
end
TransformedXYs = A*v;