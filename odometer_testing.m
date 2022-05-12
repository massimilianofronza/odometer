% %%%% Fundamentals of Image and Video Processing %%%%
%      
%       Odometer project, identification of bounding boxes
%       Massimiliano Fronza - 220234
%       May 2022

close all;
%clear all;
clc;

img = imread('odometro1.jpg');

rect = [545 594 335 145];    % Hard-coded coordinates of rectangle

%figure; imshow(img); title('Odometro1.jpg');

% Select the Region of Interest(ROI) by hand
%rect = getrect;              % Just comment this if you want the fixed coordinates
ROI = imcrop(img, rect);
close all;

%figure; imshow(ROI); title('ROI');
%figure; imhist(ROI); title('ROI - Histogram');

% Tests with gray scale 1
%grayImage = ind2gray(ROI, gray(200));
%figure; imshow(grayImage); title('ROI gray');
%figure; imhist(grayImage); title('ROI gray - Histogram');

% Test with gray scale 2
grayROI = rgb2gray(ROI);
figure; imshow(grayROI); title('ROI gray');

[H, theta, rho] = hough(grayROI);%, 'Theta', 0:1:10);

% Get coordinates of lines that occurred above some threshold in the Hough process
threshold = 355;
logic_nonzero = H>=threshold;
[rows, cols] = find(logic_nonzero);

% If you want to pick the absolute maximum element
% [rows, cols] = find(ismember(H, max(H(:))));
% [rows, cols] = find(ismember(H, max(max(H))));   % alternative method

% Plot every line above the {threshold}
x = 0:size(grayROI, 2);
y = 0:size(grayROI, 2);
hold on;
for i = 1:size(rows, 1)
    hold on;
    for j = 1:size(x, 2)
        y(j) = (rho(rows(i))-x(j)*cos(theta(cols(i))))/sin(theta(cols(i)));
    end
    y = -y; % We need this to be able to plot on the actual figure or maybe
            % not because the points were already computed on the gray 
            % image. Who knows
    plot(x, y, '-oy');
    log = sprintf('%d/%d', i, size(rows, 1));
    disp(log);
end
hold off;
disp('all done');


%BW = edge(grayROI);
%figure; imshow(BW);

% Display part
% figure; imshow(imadjust(rescale(H)),'XData',theta,'YData',rho,...
%      'InitialMagnification','fit');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal;
% colormap(gca,hot);



