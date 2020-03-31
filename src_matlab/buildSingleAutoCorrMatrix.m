function [ACMat, eigenVec] = buildSingleAutoCorrMatrix(P, basicInfo)

eigenVec = [];
board_Width = basicInfo.board_Width;
board_Height = basicInfo.board_Height;

ACMat = zeros(board_Height*board_Width*2, board_Height*board_Width*2);

for i = 1 : board_Height
    for j = 1 : board_Width
        pos = j + (i - 1) * board_Width;
        
        if j ~= board_Width
            heightVec = P(:, pos+1) - P(:, pos);
        else
            heightVec = P(:, pos) - P(:, pos-1);
        end
        if i ~= board_Height
            widthVec = P(:, pos+board_Width) - P(:, pos);
        else
            widthVec = P(:, pos) - P(:, pos-board_Width);
        end
        % angle of the current feature point
        ang = acosd(dot(heightVec./norm(heightVec), widthVec./norm(widthVec)));
        
        if ang>90
            v = heightVec./norm(heightVec) + widthVec./norm(widthVec);
            v = v./norm(v);
            R_cur = [v(1),-v(2); v(2), v(1)];
        else
            v = heightVec./norm(heightVec) - widthVec./norm(widthVec);
            v = v./norm(v);
            R_cur = [v(1),-v(2); v(2), v(1)];
        end
        
        % autocorrelation matrix
        s = 5e+5;% empirical value 
        AC_cur = [max(ang2cornerness(180 - ang),ang2cornerness(ang))/s 0;
                  0 min(ang2cornerness(180 - ang),ang2cornerness(ang))/s];
        ACMat(2*pos-1 : 2*pos, 2*pos-1 : 2*pos) = R_cur * AC_cur * R_cur';
    end
end
end

% The polynomial function that we acquired from synthetic corners. Details
% can be found in the paper
function y = ang2cornerness(x)
    y = 1404.50760.*x.^2 - 49.2631560 .* x.^3 + 0.94482.*x.^4 - 0.0093798 .* x.^5 + 0.0000455668.*x.^6 - 8.6160*1e-8.* x.^7;
end
