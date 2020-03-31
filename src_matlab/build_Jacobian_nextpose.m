function [A_new, B_new] = build_Jacobian_nextpose(intrinsicPara, basicInfo, corners, x)

num_intrinsic = length(fieldnames(intrinsicPara));
f = intrinsicPara.f;
board_Width = basicInfo.board_Width;
board_Height = basicInfo.board_Height;

% Define the rotation matrix and translation vector of next pose

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
%% Build Jacobian for the next pose
A_new = zeros(2 * board_Width * board_Height,num_intrinsic); % 2n * k
B_new = zeros(2 * board_Width * board_Height, length(x)); % 2n * 6
for i = 1 : board_Height
    for j = 1 : board_Width
        pos = j + (i - 1) * board_Width;
        % Calculate 3D points under the camera coordinate with a new pose 
        S_new(:,pos) = R * corners(:,pos) + t;
        S1 = S_new(1,pos);
        S2 = S_new(2,pos);
        S3 = S_new(3,pos);
        
        Q1 = corners(1, pos);
        Q2 = corners(2, pos);
        Q3 = corners(3, pos);
        Cross = [Q1 Q3 -Q2;
                 -Q3 Q2 Q1;
                 Q2 -Q1 Q3];
        dSdR = R * Cross;
        
        switch num_intrinsic
            case 3 % no distortion
                % Calculate the intrinsic part in J
                A_new(2*pos - 1, :) =  [S1/S3,1,0];
                A_new(2*pos, :) =  [S2/S3,0,1];

                %Calculate the extrinsic part in J
                BRx = [f * (1/S3), 0, -f * (S1/S3^2)] * dSdR;     
                Btx = [f * (1/S3), 0, -f * (S1/S3^2)];     
                BRy = [0, f * (1/S3), -f * (S2/S3^2)] * dSdR;
                Bty = [0, f * (1/S3), -f * (S2/S3^2)];
                B_new(2*pos-1, :) = [BRx,Btx];
                B_new(2*pos,:) = [BRy,Bty];
                
            case 4 % k1
                k1 = intrinsicPara.k1;
                r = (1/S3)*sqrt(S1^2 + S2^2);
                
                % Build intrinsic parts in the Jacobian matrix
                A_new(2*pos - 1, :) = [(1 + k1*r^2)*(S1/S3),1,0,r^2*f*(S1/S3)];
                A_new(2*pos, :) = [(1 + k1*r^2)*(S2/S3),0,1,r^2*f*(S2/S3)];

                % Build extrinsic parts in the Jacobian matrix
                BRx = [f/S3 + f*k1*(3*S1^2 + S2^2)/S3^3, 2*f*k1*S1*S2/S3^3, -f * (S1/S3^2) - 3*f*k1*S1*(S1^2 + S2^2)/S3^4] * dSdR;     
                Btx = [f/S3 + f*k1*(3*S1^2 + S2^2)/S3^3, 2*f*k1*S1*S2/S3^3, -f * (S1/S3^2) - 3*f*k1*S1*(S1^2 + S2^2)/S3^4];     
                BRy = [2*f*k1*S1*S2/S3^3, f/S3 + f*k1*(S1^2 + 3*S2^2)/S3^3, -f * (S2/S3^2) - 3*f*k1*S2*(S1^2 + S2^2)/S3^4] * dSdR;
                Bty = [2*f*k1*S1*S2/S3^3, f/S3 + f*k1*(S1^2 + 3*S2^2)/S3^3, -f * (S2/S3^2) - 3*f*k1*S2*(S1^2 + S2^2)/S3^4];
                B_new(2*pos-1, :) = [BRx,Btx];
                B_new(2*pos,:) = [BRy,Bty];
                
            case 5 % k1 & k2     
                k1 = intrinsicPara.k1;
                k2 = intrinsicPara.k2;
                r = (1/S3)*sqrt(S1^2 + S2^2);

                % Build intrinsic parts in the Jacobian matrix
                A_new(2*pos - 1, :) = [(1 + k1*r^2 + k2*r^4)*(S1/S3),1,0,r^2*f*(S1/S3), r^4*f*(S1/S3)];
                A_new(2*pos, :) = [(1 + k1*r^2 + k2*r^4)*(S2/S3),0,1,r^2*f*(S2/S3), r^4*f*(S2/S3)];

                % Build extrinsic parts in the Jacobian matrix
                BRx = [f*(1+k1*r^2 + k2*r^4)/S3 + f*S1^2*(2*k1+4*k2*r^2)/S3^3, 2*f*S1*S2*(k1+2*k2*r^2)/S3^3, -2*S1*f*r^2*(k1+2*k2*r^2)/S3^2 - S1*f*(1+k1*r^2+k2*r^4)/S3^2] * dSdR;     
                Btx = [f*(1+k1*r^2 + k2*r^4)/S3 + f*S1^2*(2*k1+4*k2*r^2)/S3^3, 2*f*S1*S2*(k1+2*k2*r^2)/S3^3, -2*S1*f*r^2*(k1+2*k2*r^2)/S3^2 - S1*f*(1+k1*r^2+k2*r^4)/S3^2];     
                BRy = [2*f*S1*S2*(k1+2*k2*r^2)/S3^3, f*(1+k1*r^2 + k2*r^4)/S3 + f*S2^2*(2*k1+4*k2*r^2)/S3^3, -2*S2*f*r^2*(k1+2*k2*r^2)/S3^2 - S2*f*(1+k1*r^2+k2*r^4)/S3^2] * dSdR;
                Bty = [2*f*S1*S2*(k1+2*k2*r^2)/S3^3, f*(1+k1*r^2 + k2*r^4)/S3 + f*S2^2*(2*k1+4*k2*r^2)/S3^3, -2*S2*f*r^2*(k1+2*k2*r^2)/S3^2 - S2*f*(1+k1*r^2+k2*r^4)/S3^2];
                B_new(2*pos-1, :) = [BRx,Btx];
                B_new(2*pos,:) = [BRy,Bty];
        end
        
    end
end
