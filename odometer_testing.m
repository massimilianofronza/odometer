% %%%% Fundamentals of Image and Video Processing %%%%
%      
%       Project script
%       Odometer, identification of bounding boxes
%       Massimiliano Fronza - 220234
%       May 2022

close all;
clear all;
clc;

img = imread('odometro1.jpg'); 



ROI = [553 604 308 116];    % Hard-coded coordinates of rectangle

figure; imshow(img); title('Odometro1.jpg');
ROI = getrect;              % Just comment this if you want the fixed coordinates
rect = imcrop(img, ROI);

close all;
figure; imshow(rect); title('Odometro1 ROI');
%figure; imhist(img); title('Odometro1 - Histogram');
