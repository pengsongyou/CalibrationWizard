function P = compute_nextpose_points(x, corners,intrinsicPara, basicInfo)
f = intrinsicPara.f;
u = intrinsicPara.u;
v = intrinsicPara.v;

num_intr = length(fieldnames(intrinsicPara));

board_Width = basicInfo.board_Width;
board_Height = basicInfo.board_Height;

Rx = [1 0 0;
      0 cos(x(1)) -sin(x(1));    
      0 sin(x(1)) cos(x(1))];
Ry = [cos(x(2)) 0 sin(x(2));
      0 1 0;
      -sin(x(2)) 0 cos(x(2))];
Rz = [cos(x(3)) -sin(x(3)) 0;
      sin(x(3)) cos(x(3)) 0;
      0 0 1];
R = Rz * Ry * Rx; % Rotation matrix in the next frame
t = x(4:6)';

S_new = zeros(3, board_Width * board_Height);

P = zeros(2,board_Height*board_Width);

for i = 1 : board_Height
    for j = 1 : board_Width
        pos = j + (i - 1) * board_Width;
        % Calculate 3D points under the camera coordinate with a new pose 
        S_new(:,pos) = R * corners(:,pos) + t;
        S1 = S_new(1,pos);
        S2 = S_new(2,pos);
        S3 = S_new(3,pos);
        
        switch num_intr
            case 3 % no distortion
                x_ = S1/S3;
                y_ = S2/S3;
                P(1,pos) = f * x_ + u; 
                P(2,pos) = f * y_ + v;
            case 4 % k1 radial distortion
                k1 = intrinsicPara.k1;
                r = (1/S3)*sqrt(S1^2 + S2^2);
                x_ = S1/S3;
                y_ = S2/S3;
                P(1,pos) = (1+k1*r^2)*f * x_ + u; 
                P(2,pos) = (1+k1*r^2)*f * y_ + v;
            case 5 % k1 k2 radial distortion
                k1 = intrinsicPara.k1;
                k2 = intrinsicPara.k2;
                r = (1/S3)*sqrt(S1^2 + S2^2);
                x_ = S1/S3;
                y_ = S2/S3;
                P(1,pos) = (1 + k1*r^2 + k2*r^4)*f * x_ + u; 
                P(2,pos) = (1 + k1*r^2 + k2*r^4)*f * y_ + v;
            case 7 % fisheye
                k1 = intrinsicPara.k1;
                k2 = intrinsicPara.k2;
                k3 = intrinsicPara.k3;
                k4 = intrinsicPara.k4;
                r = (1/S3)*sqrt(S1^2 + S2^2);
                theta = atan(r);
                theta_d = theta + k1 * theta^3 + k2 * theta^5 + k3 * theta^7 + k4 * theta^9;
                P(1,pos) = u + theta_d * f * S1 / (r*S3);
                P(2,pos) = v + theta_d * f * S2 / (r*S3);
        end
                
    end
end

end