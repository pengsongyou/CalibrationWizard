function out = cost_function(par,A,B, corners,intrinsicPara, basicInfo, ACMat)
    
num_intr = length(fieldnames(intrinsicPara));
f = intrinsicPara.f;
u = intrinsicPara.u;
v = intrinsicPara.v;
% 
board_Width = basicInfo.board_Width;
board_Height = basicInfo.board_Height;
image_Width = basicInfo.image_Width;
image_Height = basicInfo.image_Height;
dist_border = basicInfo.dist_border;
dist_neighbor = basicInfo.dist_neighbor;
 
%% Verify if all the points fall inside the image plane after projection

% Define the rotation matrix and translation vector of next pose
x = par;
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


P = zeros(2,board_Height*board_Width);
S_new = zeros(3, board_Width * board_Height);

OUT_OF_RANGE = 0;
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
        end
        
        % The next pose should not be too close to the border
        if P(1,pos) < dist_border || P(1,pos) > image_Width - dist_border
            OUT_OF_RANGE = 1;
            break;
        end
        if P(2,pos) < dist_border || P(2,pos) > image_Height - dist_border
            OUT_OF_RANGE = 1;
            break;
        end
        % two neighbor points in a row should not be too near
%         if j>1 && sqrt((P(1,pos) - P(1,pos - 1)).^2 + (P(2,pos) - P(2,pos - 1)).^2)<dist_neighbor 
%             OUT_OF_RANGE = 1;
%             break;
%         end
    end
    
    % Constraints on the length of a row
    %if sqrt((P(1,pos) - P(1,pos - board_Width + 1)).^2 + (P(2,pos) - P(2,pos - board_Width + 1)).^2)<120
    %    OUT_OF_RANGE = 1;
    %    break;
    %end

    if OUT_OF_RANGE == 1
        break;
    end
end


if OUT_OF_RANGE == 1
    out = realmax;
    return;
end

%% Build Jacobian for the next pose
[A_new, B_new] = build_Jacobian_nextpose(intrinsicPara, basicInfo, corners, x);
A = [A;A_new];
B = [B, zeros(size(B,1),6);zeros(2*board_Width*board_Height,size(B,2)), B_new];
J = [A,B];
J = sparse(J);

if nargin == 7
    [ACMat_new,~] = buildSingleAutoCorrMatrix(P, basicInfo);
    ACMat (end-size(ACMat_new,1)+1 : end, end-size(ACMat_new,1)+1 : end) = ACMat_new;
    M = J' * sparse(ACMat)* J;
else
    M = J' * J;
end

U = M(1:num_intr,1:num_intr);
W = M(1:num_intr,num_intr+1:end);
V = M(num_intr+1:end,num_intr+1:end);
F = inv(U- W*inv(V)*W');

out = trace(F);

end