clear
clc
close all

file_path =  'D:/__Programing/matlab/stereo/';       % ͼ���ļ���·��  
img_path_list = dir(strcat(file_path, '*.jpg'));    %��ȡ���ļ���������bmp��ʽ��ͼ��  
img_num = length(img_path_list);%��ȡͼ�������� 
I = cell(1,img_num);

img_l_dir = [file_path, 'left/'];
img_r_dir = [file_path, 'right/'];

if img_num > 0 %������������ͼ��          
    
    for j = 1:img_num %��һ��ȡͼ��              
        image_name = img_path_list(j).name;% ͼ����              
        image =  imread(strcat(file_path,image_name));              
        I{j}=image;           
        fprintf('%d %d %s\n',i,j,strcat(file_path,image_name));% ��ʾ���ڴ�����ͼ����                
    end
    
    for j = 1 : img_num
        img = I{j};

        img_width  = size(img, 2);
        img_height = size(img, 1);

        img_R = img(:, 1 : img_width / 2, :);
        img_L = img(:, img_width / 2 + 1 : img_width, :);
        
        % figure
        % imshow(img_R)
        % figure
        % imshow(img_L)
        
        imwrite(img_L, [img_l_dir, num2str(j), '.jpg'])
        imwrite(img_R, [img_r_dir, num2str(j), '.jpg'])
        
    end

end

% https://blog.csdn.net/qq_38236355/article/details/89280633
% https://blog.csdn.net/u013451663/article/details/89954646
% stereoCameraCalibrator

