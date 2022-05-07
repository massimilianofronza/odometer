% %%%% Fundamentals of Image and Video Processing %%%%
%      
%       Odometer project, identification of bounding boxes
%       Massimiliano Fronza - 220234
%       May 2022

close all;
clear all;
clc;

img = imread('odometro1.jpg');

rect = [545 594 335 145];    % Hard-coded coordinates of rectangle

figure; imshow(img); title('Odometro1.jpg');

% Select the Region of Interest(ROI) by hand
%rect = getrect;              % Just comment this if you want the fixed coordinates
ROI = imcrop(img, rect);
close all;

figure; imshow(ROI); title('ROI');
%figure; imhist(ROI); title('ROI - Histogram');

% Tests with gray scale 1
%grayImage = ind2gray(ROI, gray(200));
%figure; imshow(grayImage); title('ROI gray');
%figure; imhist(grayImage); title('ROI gray - Histogram');

% Test with gray scale 2
grayROI = rgb2gray(ROI);
figure; imshow(grayROI); title('ROI gray');

[H, theta, rho] = hough(grayROI);

% Display part
subplot(2,1,1); imshow(ROI);
subplot(2,1,2); imshow(imadjust(rescale(H)),'XData',theta,'YData',rho,...
      'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;
colormap(gca,hot);



