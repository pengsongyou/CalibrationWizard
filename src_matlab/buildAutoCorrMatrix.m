function ACMat = buildAutoCorrMatrix(S, intrinsicPara, basicInfo)


num_intr = length(fieldnames(intrinsicPara));
num_frame = basicInfo.num_frame;
f = intrinsicPara.f;
u = intrinsicPara.u;
v = intrinsicPara.v;
% 
board_Width = basicInfo.board_Width;
board_Height = basicInfo.board_Height;

%% Calculate the image points
P = zeros(2,board_Height*board_Width, num_frame);

for m = 1 : num_frame
    for i = 1 : board_Height
        for j = 1 : board_Width
            pos = j + (i - 1) * board_Width;
            S1 = S(1,pos,m);
            S2 = S(2,pos,m);
            S3 = S(3,pos,m);
        
            switch num_intr
                case 3 % no distortion
                    x_ = S1/S3;
                    y_ = S2/S3;
                    P(1,pos,m) = f * x_ + u; 
                    P(2,pos,m) = f * y_ + v;
                case 4 % k1 radial distortion
                    k1 = intrinsicPara.k1;
                    r = (1/S3)*sqrt(S1^2 + S2^2);
                    x_ = S1/S3;
                    y_ = S2/S3;
                    P(1,pos,m) = (1+k1*r^2)*f * x_ + u; 
                    P(2,pos,m) = (1+k1*r^2)*f * y_ + v;
                case 5 % k1 k2 radial distortion
                    k1 = intrinsicPara.k1;
                    k2 = intrinsicPara.k2;
                    r = (1/S3)*sqrt(S1^2 + S2^2);
                    x_ = S1/S3;
                    y_ = S2/S3;
                    P(1,pos,m) = (1 + k1*r^2 + k2*r^4)*f * x_ + u; 
                    P(2,pos,m) = (1 + k1*r^2 + k2*r^4)*f * y_ + v;
                case 7 % fisheye
                    k1 = intrinsicPara.k1;
                    k2 = intrinsicPara.k2;
                    k3 = intrinsicPara.k3;
                    k4 = intrinsicPara.k4;
                    r = (1/S3)*sqrt(S1^2 + S2^2);
                    theta = atan(r);
                    theta_d = theta + k1 * theta^3 + k2 * theta^5 + k3 * theta^7 + k4 * theta^9;
                    P(1,pos,m) = u + theta_d * f * S1 / (r*S3);
                    P(2,pos,m) = v + theta_d * f * S2 / (r*S3);
            end
        end
    end
end
%%

ACMat = zeros(2*board_Height*board_Width*num_frame, 2*board_Height*board_Width*num_frame);
for m = 1 : num_frame
    [ACMat_cur, eigenVec2] = buildSingleAutoCorrMatrix(P(:,:,m), basicInfo);    
    ACMat(1 + (m-1)*2*board_Height*board_Width : m*2*board_Height*board_Width,...
          1 + (m-1)*2*board_Height*board_Width : m*2*board_Height*board_Width) = ACMat_cur;
end

end
