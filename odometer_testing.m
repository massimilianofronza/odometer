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

[H, theta, rho] = hough(grayROI, 'Theta', -45:0.5:45);

% Retrieving lines that occured at last once in the Hough process
%for i = 1:size(H)
%x = -10:10;
%y = (rho-x*cos(theta))/sin(theta);
%plot(x, y)     % error

% Pick the most rated line from the accumulator and print it
[rho_id, theta_id] = find(ismember(H, max(H(:))));
%[row, col] = find(ismember(H, max(max(H))));   % alternative method

x = 0:size(grayROI, 2);

hold on;
a = 1;

for i = 1:size(rho_id)
    rho_max = rho(rho_id(i));
    theta_max = theta(theta_id(i));
    y = (rho_max-x*cos(theta_max))/sin(theta_max);
    %y = -y;      % We need this to be able to plot on the actual figure
                   % or maybe not because the points were already 
                   % computed on the gray image. Who knows
    
    log = sprintf('New plot: %d\n', y(1));
    disp(log);
    
    if a==1
        plot(x, y, '-og');     % error
        a = 2;
    elseif a==2
        plot(x, y, '-sr');     % error
        a = 3;
    elseif a==3
        plot(x, y, '-y');
    end
    %plot(10, 10, '-og');
end

hold off;

%plot(x, y, '-');

%BW = edge(grayROI);
%figure; imshow(BW);

% Display part
%figure; imshow(imadjust(rescale(H)),'XData',theta,'YData',rho,...
%      'InitialMagnification','fit');
%xlabel('\theta'), ylabel('\rho');
%axis on, axis normal;
%colormap(gca,hot);



