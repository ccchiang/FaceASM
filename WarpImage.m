function wimg = WarpImage(img, v)
ux = v(1);
uy = v(2);
vx = v(3);
vy = v(4);
tx = v(5);
ty = v(6);
[h w c] = size(img);
wimg = img;
[B A] = meshgrid(1:h,1:w);
b = reshape(B, [1 h*w]);
a = reshape(A, [1 h*w]);
ab = ([a;b]);
shifted_ab = [a-tx;b-ty];
xy =  floor(inv([ux -vx;uy vy])*(shifted_ab));
for i = 1:length(xy(1,:));
        if (xy(2,i)>=1&&xy(2,i)<=h&&xy(1,i)>=1&&xy(1,i)<=w)
            wimg(ab(2,i), ab(1,i), :) = img(xy(2,i), xy(1,i),:);
        else
            wimg(ab(2,i), ab(1,i), :) = 255;            
        end    
end
% for b = 1:h
%     for a = 1:w
%         pos = round(inv([ux -vx;uy vy])*[a-tx;b-ty]);
%         if (pos(2)>=1&&pos(2)<=h&&pos(1)>=1&&pos(1)<=w)
%             wimg(b,a,:) = img(pos(2), pos(1),:);
%         else
%             wimg(b,a,:) = 255;            
%         end
%     end
% end