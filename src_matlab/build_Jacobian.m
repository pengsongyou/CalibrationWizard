function [A,B] = build_Jacobian(intrinsicPara,extrinsicPara, basicInfo, S,corners)

board_Width = basicInfo.board_Width;
board_Height = basicInfo.board_Height;
num_frame = basicInfo.num_frame;

num_intrinsic = length(fieldnames(intrinsicPara));
A = zeros(2 * board_Width * board_Height * num_frame,num_intrinsic); % (2n * m) * k
B = zeros(2 * board_Width * board_Height * num_frame,6 * num_frame); % (2n * m) * 6m
f = intrinsicPara.f;

for m = 1 : num_frame
    for i = 1 : board_Height
        for j = 1 : board_Width
            pos = (m - 1) * 2 * board_Width * board_Height + 2 * (j + (i - 1) * board_Width - 1) + 1;
            S1 = S(1,j + (i - 1) * board_Width,m);
            S2 = S(2,j + (i - 1) * board_Width,m);
            S3 = S(3,j + (i - 1) * board_Width,m);
            Q1 = corners(1, j + (i - 1) * board_Width);
            Q2 = corners(2, j + (i - 1) * board_Width);
            Q3 = corners(3, j + (i - 1) * board_Width);
            
            Cross = [Q1 Q3 -Q2;
                     -Q3 Q2 Q1;
                     Q2 -Q1 Q3];
            dSdR = extrinsicPara.rot_Mat(:,:,m) * Cross;
            
            switch num_intrinsic
                case 3 % no distortion
                    % Build intrinsic parts in the Jacobian matrix
                    A(pos, :) = [S1/S3,1,0];
                    A(pos + 1, :) = [S2/S3,0,1];
                    
                    % Build extrinsic parts in the Jacobian matrix
                    BRx = [f * (1/S3), 0, -f * (S1/S3^2)] * dSdR;     
                    Btx = [f * (1/S3), 0, -f * (S1/S3^2)];     
                    BRy = [0, f * (1/S3), -f * (S2/S3^2)] * dSdR;
                    Bty = [0, f * (1/S3), -f * (S2/S3^2)];
                    B(pos, 6*(m-1) + 1: 6*m) = [BRx,Btx];
                    B(pos+1, 6*(m-1) + 1: 6*m) = [BRy,Bty];
                case 4 % k1
                    
                    k1 = intrinsicPara.k1;
                    r = (1/S3)*sqrt(S1^2 + S2^2);
                    % Build intrinsic parts in the Jacobian matrix
                    A(pos, :) = [(1 + k1*r^2)*(S1/S3),1,0,r^2*f*(S1/S3)];
                    A(pos + 1, :) = [(1 + k1*r^2)*(S2/S3),0,1,r^2*f*(S2/S3)];

                    % Build extrinsic parts in the Jacobian matrix
                    BRx = [f/S3 + f*k1*(3*S1^2 + S2^2)/S3^3, 2*f*k1*S1*S2/S3^3, -f * (S1/S3^2) - 3*f*k1*S1*(S1^2 + S2^2)/S3^4] * dSdR;     
                    Btx = [f/S3 + f*k1*(3*S1^2 + S2^2)/S3^3, 2*f*k1*S1*S2/S3^3, -f * (S1/S3^2) - 3*f*k1*S1*(S1^2 + S2^2)/S3^4];     
                    BRy = [2*f*k1*S1*S2/S3^3, f/S3 + f*k1*(S1^2 + 3*S2^2)/S3^3, -f * (S2/S3^2) - 3*f*k1*S2*(S1^2 + S2^2)/S3^4] * dSdR;
                    Bty = [2*f*k1*S1*S2/S3^3, f/S3 + f*k1*(S1^2 + 3*S2^2)/S3^3, -f * (S2/S3^2) - 3*f*k1*S2*(S1^2 + S2^2)/S3^4];
                    B(pos, 6*(m-1) + 1: 6*m) = [BRx,Btx];
                    B(pos+1, 6*(m-1) + 1: 6*m) = [BRy,Bty];
                    
                case 5  % k1 & k2 
                    k1 = intrinsicPara.k1;
                    k2 = intrinsicPara.k2;
                    r = (1/S3)*sqrt(S1^2 + S2^2);
                    
                    % Build intrinsic parts in the Jacobian matrix
                    A(pos, :) = [(1 + k1*r^2 + k2*r^4)*(S1/S3),1,0,r^2*f*(S1/S3), r^4*f*(S1/S3)];
                    A(pos + 1, :) = [(1 + k1*r^2 + k2*r^4)*(S2/S3),0,1,r^2*f*(S2/S3), r^4*f*(S2/S3)];
                    
                    % Build extrinsic parts in the Jacobian matrix
                    BRx = [f*(1+k1*r^2 + k2*r^4)/S3 + f*S1^2*(2*k1+4*k2*r^2)/S3^3, 2*f*S1*S2*(k1+2*k2*r^2)/S3^3, -2*S1*f*r^2*(k1+2*k2*r^2)/S3^2 - S1*f*(1+k1*r^2+k2*r^4)/S3^2] * dSdR;     
                    Btx = [f*(1+k1*r^2 + k2*r^4)/S3 + f*S1^2*(2*k1+4*k2*r^2)/S3^3, 2*f*S1*S2*(k1+2*k2*r^2)/S3^3, -2*S1*f*r^2*(k1+2*k2*r^2)/S3^2 - S1*f*(1+k1*r^2+k2*r^4)/S3^2];     
                    BRy = [2*f*S1*S2*(k1+2*k2*r^2)/S3^3, f*(1+k1*r^2 + k2*r^4)/S3 + f*S2^2*(2*k1+4*k2*r^2)/S3^3, -2*S2*f*r^2*(k1+2*k2*r^2)/S3^2 - S2*f*(1+k1*r^2+k2*r^4)/S3^2] * dSdR;
                    Bty = [2*f*S1*S2*(k1+2*k2*r^2)/S3^3, f*(1+k1*r^2 + k2*r^4)/S3 + f*S2^2*(2*k1+4*k2*r^2)/S3^3, -2*S2*f*r^2*(k1+2*k2*r^2)/S3^2 - S2*f*(1+k1*r^2+k2*r^4)/S3^2];
                    B(pos, 6*(m-1) + 1: 6*m) = [BRx,Btx];
                    B(pos+1, 6*(m-1) + 1: 6*m) = [BRy,Bty];    
            end
            
        end
    end
end

end