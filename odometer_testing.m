% %%%% Fundamentals of Image and Video Processing %%%%
%      
%       Odometer project, identification of bounding boxes
%       Massimiliano Fronza - 220234
%       May 2022

close all;
%clear all;
clc;

img = imread('odometro1.jpg');
%img = imread('test.jpg');

rect = [545 594 335 145];    % Hard-coded coordinates of rectangle

% Select the Region of Interest(ROI) by hand
figure; imshow(img); title('Odometro1.jpg');    % Comment this...
rect = getrect;         % ...and this if you want the fixed coordinates
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
figure; imshow(grayROI); title('My attempt');%title('ROI gray');

% Extract the edges of the gray image
edges = edge(grayROI, "canny");

[H, theta, rho] = hough(edges, 'RhoResolution', 0.5, 'Theta', -90:1:89); %'Theta', 0:1:10);

% Get rows and thetas of lines occurring above the {threshold}
threshold = 150;
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
    for j = 1:size(x, 2)
        y(j) = (rho(rows(i))-x(j)*cos(theta(cols(i))))/sin(theta(cols(i)));
    end
%      y = -y; % We need this to be able to plot on the actual figure or maybe
            % not because the points were already computed on the gray 
            % image. Who knows
    plot(x, y, '-oy');
    log = sprintf('%d/%d', i, size(rows, 1));
    disp(log);
end
hold off;

% Debugging alternative:
% hold on;
% for i = 1:size(rows, 1)
%     this_rho = rho(rows(i));
%     this_theta = theta(cols(i));
%     a = cos(this_theta);
%     b = sin(this_theta);
%     x0 = a*this_rho;
%     y0 = b*this_rho;
%     log = sprintf('%d, %d', x0, y0);
%     disp(log);
% end
% hold off;

disp('all done');

for i = 1:size(rows, 1)
    log = sprintf('%d, %d', rho(rows(i)), theta(cols(i)));
    disp(log);
end

% Display part
% figure; imshow(imadjust(rescale(H)),'XData',theta,'YData',rho,...
%      'InitialMagnification','fit');
% xlabel('\theta'), ylabel('\rho');
% axis on, axis normal;
% colormap(gca,hot);






% MATLAB DOCUMENTATION EXAMPLES:
BW = edge(grayROI,'canny');

[H,theta,rho] = hough(BW);

P = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));

lines = houghlines(BW,theta,rho,P,'FillGap',5,'MinLength',7);

figure, imshow(grayROI), title('Matlab documentation example'), hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
% highlight the longest line segment
plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','red');

