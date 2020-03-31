%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The Matlab implementation of the paper:                        %
% "Calibration Wizard: A Guidance System for Camera Calibration  %
% Based on Modelling Geometric and Corner Uncertainty" ICCV 2019 %
% Songyou Peng, Peter Sturm                                      %
%                                                                %
% The code can only be used for research purposes.               %
%                                                                %
% Copyright (C) 2019 Songyou Peng                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
close all;
clear;

%% Set parameters
filepath = '../out/';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
autoCorr_flag = 1;          % 1: consider autocorretion matrix  0: otherise
dist_border = 30;           % smallest distance between the next pose and border (in pixel)
dist_neighbor = 15;         % smallest distance of neighboring corner points (in pixel)
tranlation_bound = 1000;    % bound of translation in optimization, may need to change for different cameras
% display                   %
optim_display = 'final';    % level of display. 'iter'|'final'|'off'
plot_uncertainty = 1;       % 1: display uncertainty map  0: not display
pixel_gap = 10;             % calculate uncertainty every pixel_gap pixels, e.g. every 10 pixels
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Extract the calibration infomation of initial calibration process
[basicInfo, intrinsicPara, extrinsicPara] = extract_info(filepath);

% Shorthands
basicInfo.dist_border = dist_border;
basicInfo.dist_neighbor = dist_neighbor;
board_Width = basicInfo.board_Width;
board_Height = basicInfo.board_Height;
image_Width = basicInfo.image_Width;
image_Height = basicInfo.image_Height;
square_Size = basicInfo.square_Size;
num_frame = basicInfo.num_frame;
rot_Mat = extrinsicPara.rot_Mat;
t_Vec = extrinsicPara.t_Vec;
%% Define 3D points in the world coordinate
corners = zeros(board_Width * board_Height, 3);
for i = 1 : board_Height
    for j = 1 : board_Width
        corners(j + (i - 1) * board_Width, 1) = (j - 1) * square_Size;
        corners(j + (i - 1) * board_Width, 2) = (i - 1) * square_Size;
    end
end
corners = corners';

%% Transform all the 3D points from the world coordinate to the camera coordinate
% S = R * Q + t
S = zeros(3, board_Width * board_Height, num_frame);

for m = 1 : num_frame
    for i = 1 : board_Height
        for j = 1 : board_Width
            S(:,j + (i - 1) * board_Width,m) = rot_Mat(:,:,m) * corners(:,j + (i - 1) * board_Width) + t_Vec(:,m);
        end
    end
end

%% Build Jacobian J = [A,B]
[A,B] = build_Jacobian(intrinsicPara, extrinsicPara, basicInfo, S, corners);

%% Build big autocorrelation matrix
ACMat = eye(size(A,1));
if autoCorr_flag == 1
    ACMat = buildAutoCorrMatrix(S, intrinsicPara, basicInfo);
end
%% Show the uncertainty map
if plot_uncertainty
    num_intr = length(fieldnames(intrinsicPara));
    J = [A,B];
    M = J'*ACMat*J;

    U = M(1:num_intr,1:num_intr);
    W = M(1:num_intr,num_intr+1:end);
    V = M(num_intr+1:end,num_intr+1:end);
    Sigma = inv(U- W*inv(V)*W');

    uncertainty_map(Sigma, intrinsicPara, basicInfo, pixel_gap);
end
%% Numerically get the next pose using optimizer (Local or Global(SA))

% Set initial extrinsic parameters
x = [0, 0, 0, mean(t_Vec(1,:)),mean(t_Vec(2,:)),mean(t_Vec(3,:))];

% Global optimization method
tic
lb = [0, 0, 0, -tranlation_bound, -tranlation_bound, 0]; % lower bound
ub = [2*pi, pi, 2*pi, tranlation_bound, tranlation_bound, mean(t_Vec(3,:))]; % upper bound

options = saoptimset('Display', optim_display); % check other options for SA in https://www.mathworks.com/help/gads/saoptimset.html
if autoCorr_flag == 1
    ACMat_extend = zeros(size(ACMat) + [2*board_Height*board_Width, 2*board_Height*board_Width]);
    ACMat_extend(1:size(ACMat,1), 1:size(ACMat,2)) = ACMat;
    [x,fval,exitFlag,output] = simulannealbnd(@(x)cost_function(x,A,B, corners, intrinsicPara, basicInfo, ACMat_extend), x, lb, ub, options)
else
    [x,fval,exitFlag,output] = simulannealbnd(@(x)cost_function(x,A,B, corners, intrinsicPara, basicInfo), x, lb, ub, options)
end
toc

P_next = compute_nextpose_points(x, corners, intrinsicPara, basicInfo);
% Plot the expected images with new pose
plot_nextpose(P_next, basicInfo);
% Save nextpose points
dlmwrite(strcat(filepath,'nextpose_points.txt'), P_next);
%% Expected ncertainty map after estimating the new pose
if plot_uncertainty
    [A_new, B_new] = build_Jacobian_nextpose(intrinsicPara, basicInfo, corners, x);

    A = [A;A_new];
    B = [B, zeros(size(B,1),6);zeros(2*board_Width*board_Height,size(B,2)), B_new];
    J = [A,B];

    if autoCorr_flag == 1
        [ACMat_new,~] = buildSingleAutoCorrMatrix(P_next, basicInfo);
        ACMat_extend = zeros(size(ACMat) + size(ACMat_new));
        ACMat_extend(1:size(ACMat,1), 1:size(ACMat,2)) = ACMat;
        ACMat_extend (end-size(ACMat_new,1)+1 : end, end-size(ACMat_new,1)+1 : end) = ACMat_new;
    else
        ACMat_extend = eye(size(J,1));
    end

    M = J' * ACMat_extend * J;
    U = M(1:num_intr,1:num_intr);
    W = M(1:num_intr,num_intr+1:end);
    V = M(num_intr+1:end,num_intr+1:end);
    F = inv(U- W*inv(V)*W');

    uncertainty_map(F, intrinsicPara, basicInfo, pixel_gap);
end