% %%%% Fundamentals of Image and Video Processing %%%%
%      
%       Odometer project, identification of bounding boxes
%       Massimiliano Fronza - 220234
%       May 2022

close all;
clear all; % Slows down computation since it de-allocates the variables
clc;

% Read the image
img = imread('odometro1.jpg');
% img = imread('test.jpg');
% img = imread('radiant.jpg');  % For detecting a range of angles

% (1a) Hard-coded coordinates of rectangle
rect = [545 594 335 145];
% rect = [192 184 180 188];

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
doc_peaks = houghpeaks(H, 10);

% (7b) Get rows and thetas of lines occurring above the {threshold}
threshold = 85;
logic_nonzero = H>=threshold;
[rows, cols] = find(logic_nonzero);
my_peaks = [rows, cols];

% TODO FINO A QUI TUTTO BENE IN TEORIA, VERIFICA I VALORI DI peaks ANCHE
% CON QUELLI SUL FOGLIO.

% (8) Find the top P lines previously identified
% Fillgap: Distance between two line segments associated with the same
% Hough transform bin - default: 20
% MinLength: Minimum line length - default: 40
doc_lines = houghlines(edges_canny, theta, rho, doc_peaks, 'FillGap', 45, 'MinLength', 40);
my_lines = houghlines(edges_canny, theta, rho, my_peaks, 'FillGap', 45, 'MinLength', 300);

% TODO: note on the .odt document about houghpeaks and my method.
close all;

% (9a) Plot every peak line - Documentation lines
lines = doc_lines;
figure, imshow(grayROI), title('Detected lines on ROI - Documentation version'), hold on;
max_len = 0;        % Length of the longest line
xy_long = 0;        % xy coordinates of the longest line
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
if xy_long ~= 0
    plot(xy_long(:,1), xy_long(:,2), 'LineWidth', 2, 'Color', 'green'); % TODO change this into red to have th longest line highlighted
else
    error("MY DEBUG INFO: No lines were found!");
end
hold off;

% (9b) Plot every peak line - My lines
lines = my_lines;
figure, imshow(grayROI), title('Detected lines on ROI - My version'), hold on;
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
plot(xy_long(:,1), xy_long(:,2), 'LineWidth', 2, 'Color', 'green');
hold off;

disp('all done.');

% disp('Printing rows and cols actual rhos and thetas:');
% for i = 1:length(rows)
%     log = sprintf('%d, %d', rho(rows(i)), theta(cols(i)));
%     disp(log);
% end

disp(' DEBUGGING NOW');
peaks = doc_peaks;
disp("Printing doc peaks");
for j = 1:2
    disp(peaks);
    for i = 1:size(peaks, 1)
        disp(H(peaks(i, 1), peaks(i, 2)));
    end
    if j == 1
        disp("Printing my peaks");
        peaks = my_peaks;
    end
end
