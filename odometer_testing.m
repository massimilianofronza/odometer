% %%%% Fundamentals of Image and Video Processing %%%%
%      
%       Odometer project, identification of bounding boxes
%       Massimiliano Fronza - 220234
%       May 2022

close all;
clear all; % Slows down computation since it de-allocates the variables
clc;

% Read the image
% img = imread('odometro1.jpg');
img = imread('test.jpg');
% img = imread('radiant.jpg');  % For detecting a range of angles

% (1a) Hard-coded coordinates of rectangle
% rect = [545 594 335 145];
rect = [192 184 180 188];

% (1b) Select the Region of Interest(ROI) by hand
% figure; imshow(img); title('Odometro1.jpg');    % Comment this...
% rect = getrect;         % ...and this if you want the fixed coordinates

% (2) Crop the ROI
ROI = imcrop(img, rect);

% (3a) Convert into gray scale the 3-dimensional image
grayROI = rgb2gray(ROI);
% figure; imshow(grayROI); title('My attempt');%title('ROI gray');

% (3b) If you use the radiant image
% grayROI = ROI;

% (4) Extract the edges of the gray image
edges_canny = edge(grayROI, "canny");

% (5) Horizontal angles to be identified in the plate identification scenario
angles = [-90:1:-45, 45:1:89];

% (6) Run te Hough Lines algorithm for the detection of horizontal lines
[H, theta, rho] = hough(edges_canny, 'RhoResolution', 1, 'Theta', angles); %'Theta', -90:0.5:89);

% (7a) Identify a set of peaks in the Hough accumulation matrix
doc_peaks = houghpeaks(H, 5);

% (7b) Get rows and thetas of lines occurring above the {threshold}
threshold = 135;
logic_nonzero = H>=threshold;
[rows, cols] = find(logic_nonzero);
my_peaks = [rows, cols];

% TODO FINO A QUI TUTTO BENE IN TEORIA, VERIFICA I VALORI DI peaks ANCHE
% CON QUELLI SUL FOGLIO.

% (8a) Find the top P lines previously identified
% TODO Fillgap and MinLength are the ones I have to use to obtain continuous lines
doc_lines = houghlines(edges_canny, theta, rho, doc_peaks, 'FillGap', 20, 'MinLength', 7);
my_lines = houghlines(edges_canny, theta, rho, my_peaks, 'FillGap', 20, 'MinLength', 7);
lines = doc_lines;       % Change this to see which version is in use

% TODO: note on the .odt document about houghpeaks and my method.
close all;

% (9a) Plot every peak line - Documentation example
figure, imshow(grayROI), title('Detected lines on ROI'), hold on;
max_len = 0;
for i = 1:length(lines)
    xy = [lines(i).point1; lines(i).point2];
    plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'green');

    % Determine the length of the line.
    % norm(v) returns the Euclidean norm given the vector v
    len = norm(lines(i).point1 - lines(i).point2);
    if (len > max_len)
        max_len = len;
        xy_long = xy;
    end

    % Progression output
    log = sprintf('%d/%d', i, size(lines, 2));
    disp(log);
end
% highlight the longest line segment
plot(xy_long(:,1), xy_long(:,2), 'LineWidth', 2, 'Color', 'red');
hold off;

disp('all done.');
disp('Printing rows and cols actual rhos and thetas:');

% for i = 1:length(rows)
%     log = sprintf('%d, %d', rho(rows(i)), theta(cols(i)));
%     disp(log);
% end

disp(' DEBUGGING NOW')

% Debug code:
peaks = doc_peaks
for i = 1:size(peaks, 1)
%     disp(peaks(i, :))
%     disp(H(peaks(i, 1), peaks(i, 2)))
end

% for i = 1:size(peaks, 1)
%     log = sprintf('%d - %d', rows(i), cols(i));
%     disp(log)
%     disp(H(rows(i), cols(i)))
% end
