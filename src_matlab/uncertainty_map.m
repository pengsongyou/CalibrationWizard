function uncertainty_map(Sigma, intrinsicPara, basicInfo, gap_size)

num_intr = length(fieldnames(intrinsicPara));
f = intrinsicPara.f;
u = intrinsicPara.u;
v = intrinsicPara.v;

map = zeros(ceil(basicInfo.image_Height/gap_size),ceil(basicInfo.image_Width/gap_size));
for i = 1 : gap_size: basicInfo.image_Height
    
    switch num_intr
        case 3 % No distortion
            for j = 1 : gap_size : basicInfo.image_Width
    
                Jt = [(j-u)/f,1,0;
                      (i-v)/f,0,1]; % S13 = (j-u)/f, S23 = (i-v)/f
                uncertain = Jt * Sigma * Jt';
                map(ceil(i/gap_size),ceil(j/gap_size)) = trace(uncertain);
            end
        case 4 % k1
            k1 = intrinsicPara.k1;
            for j = 1 : gap_size : basicInfo.image_Width
                fun = @(x) (j- u - (1 + k1*x(1)^2+k1*x(2)^2)*f*x(1))^2 + (i - v - (1 + k1*x(1)^2+k1*x(2)^2)*f*x(2))^2;
                x0 = [j-u,i-v]./f;
                x = fminsearch(fun,x0);
                S13 = x(1);
                S23 = x(2);
                
                Jt = [(j-u)/f,1,0,(S13^2+S23^2)*f*S13; 
                      (i-v)/f,0,1,(S13^2+S23^2)*f*S23];
                uncertain = Jt * Sigma * Jt';
                map(ceil(i/gap_size),ceil(j/gap_size)) = trace(uncertain);
            end
        case 5
            k1 = intrinsicPara.k1;
            k2 = intrinsicPara.k2;
            for j = 1 : gap_size : basicInfo.image_Width
                fun = @(x) (j- u - (1 + k1*(x(1)^2+x(2)^2) + k2*(x(1)^2+x(2)^2)^4)*f*x(1))^2 + (i - v - (1 + k1*(x(1)^2+x(2)^2) + k2*(x(1)^2+x(2)^2)^4)*f*x(2))^2;
                x0 = [j-u,i-v]./f;
                x = fminsearch(fun,x0);
                
                S13 = x(1);
                S23 = x(2);
                
                Jt = [(j-u)/f,1,0,(S13^2+S23^2)*f*S13, (S13^2+S23^2)^2*f*S13; 
                      (i-v)/f,0,1,(S13^2+S23^2)*f*S23, (S13^2+S23^2)^2*f*S23];
                uncertain = Jt * Sigma * Jt';
                map(ceil(i/gap_size),ceil(j/gap_size)) = trace(uncertain);
            end    
    end        
end

figure;
map = imresize(map,[basicInfo.image_Height,basicInfo.image_Width]);
imagesc(map);
c = colorbar;
c.FontSize = 25;
drawnow
axis off
end