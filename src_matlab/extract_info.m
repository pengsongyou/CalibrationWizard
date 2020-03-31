function [basicInfo, intrinsicPara, extrinsicPara] = extract_info(filepath)

Info = xmlread(strcat(filepath,'out_camera_data.xml'));
board_Width_parent = Info.getElementsByTagName('board_Width');
board_Width_node = board_Width_parent.item(0);
board_Width = board_Width_node.getTextContent;
board_Width = str2double(board_Width);

board_Height_parent = Info.getElementsByTagName('board_Height');
board_Height_node = board_Height_parent.item(0);
board_Height = board_Height_node.getTextContent;
board_Height = str2double(board_Height);

square_Size_parent = Info.getElementsByTagName('square_Size');
square_Size_node = square_Size_parent.item(0);
square_Size = square_Size_node.getTextContent;
square_Size = str2double(square_Size);


image_Width_parent = Info.getElementsByTagName('image_Width');
image_Width_node = image_Width_parent.item(0);
image_Width = image_Width_node.getTextContent;
image_Width = str2double(image_Width);

image_Height_parent = Info.getElementsByTagName('image_Height');
image_Height_node = image_Height_parent.item(0);
image_Height = image_Height_node.getTextContent;
image_Height = str2double(image_Height);

basicInfo.board_Width = board_Width;
basicInfo.board_Height = board_Height;
basicInfo.image_Width = image_Width;
basicInfo.image_Height = image_Height;
basicInfo.square_Size = square_Size;

%% Acquire Intrinsic parameters (including distortion)

k1_opt_parent = Info.getElementsByTagName('k1_Dist');
k1_opt_node = k1_opt_parent.item(0);
k1_opt = k1_opt_node.getTextContent;
k1_opt = str2double(k1_opt);

k2_opt_parent = Info.getElementsByTagName('k2_Dist');
k2_opt_node = k2_opt_parent.item(0);
k2_opt = k2_opt_node.getTextContent;
k2_opt = str2double(k2_opt);

if k1_opt ~= 0
    if k2_opt ~= 0
        dist_type = 'no_dist'
    end
else
    if k2_opt ~= 0
        dist_type = 'radial1'
    else
        dist_type = 'radial2'
    end
end

raw_cameraMatrix = importdata(strcat(filepath,'out_camera_matrix.txt'),',');
raw_distortCoeff = importdata(strcat(filepath,'out_distort_coeff.txt'),',');

cameraMatrix_row1 = sscanf(raw_cameraMatrix{1},'%*c%f%*c%f%*c%*c%f%*c',[1 Inf]);
cameraMatrix_row2 = sscanf(raw_cameraMatrix{2},'%f%*c%f%*c%f%*c',[1 Inf]);
cameraMatrix_row3 = sscanf(raw_cameraMatrix{3},'%f%*c%f%*c%f%*c',[1 Inf]);
cameraMatrix  = [cameraMatrix_row1;
                 cameraMatrix_row2;
                 cameraMatrix_row3];                 

intrinsicPara.f = cameraMatrix(1,1);
intrinsicPara.u = cameraMatrix(1,3);
intrinsicPara.v = cameraMatrix(2,3);

switch dist_type
    case 'no_dist'
        % Does not have distortion parameters
    case 'radial1'
        dist_coeff = sscanf(raw_distortCoeff{1},'%*c%f%*c');
        intrinsicPara.k1 = dist_coeff(1); 
    case 'radial2'
        dist_coeff = sscanf(raw_distortCoeff{1},'%*c%f%*c');
        intrinsicPara.k1 = dist_coeff(1);
        intrinsicPara.k2 = dist_coeff(2);    
    otherwise
        disp('No suitable type');
        return;
end

%% Acquire Extrinsic parameters in different poses

raw_rotation = importdata(strcat(filepath,'out_rotation_matrix.txt'),',');
raw_translate = importdata(strcat(filepath,'out_translation_vector.txt'),',');
num_frame = size(raw_rotation,1);
rot_Mat = zeros(3,3,num_frame);
t_Vec = zeros(3,num_frame);
for m = 1 : num_frame
    R = sscanf(raw_rotation{m},'%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c',[1 Inf]);
    rot_Mat(:,:,m) = reshape(R,[3,3])';
    t_Vec(:,m) = sscanf(raw_translate{m},'%*c%f%*c%f%*c%f%*c',[1 Inf])';
    
end

extrinsicPara.rot_Mat = rot_Mat;
extrinsicPara.t_Vec = t_Vec;
basicInfo.num_frame = num_frame;
end