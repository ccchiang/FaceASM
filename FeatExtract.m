function feat = FeatExtract(x, y, img, s, d, wnd_sz)
feat = reshape(double(img(y-s*d:s:y+s*d, x-s*d:s:x+s*d)),[1 wnd_sz*wnd_sz]);
feat = feat./norm(feat);
