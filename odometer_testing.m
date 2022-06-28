% %%%% Fundamentals of Image and Video Processing %%%%
%      
%       Odometer project, identification of bounding boxes
%       Massimiliano Fronza - 220234
%       May 2022

close all;
clear all; % Slows down computation since it re-allocates all the variables,
            % but makes sure that you don't use variables from previous executions
clc;

%%% Global settings
IMAGES = "./odometers/";    % Images folder 
DEBUG = false;              % If true, shows debug info in the console
FILE = 1;                   % File number to pick from the images folder
FIXED_ROI = false;          % If true, picks the hard-coded ROI. If false, take it manually
N_PEAKS = 10;               % Amount of desired peaks in the first identification method
HOUGH_THRESHOLD = 105;      % The more confused the image, the higher this should be
MIN_LEN_FRACTION = 0.85;    % Minimum (fraction of) length for a line to be considered
FILL_GAP_FRACTION = 0.15;   % Minimum (fraction of) space between each number on the odometer

% (0) Read the image
files = dir(IMAGES + '*.jpg');
nFiles = length(files);
currentFileName = files(FILE).name;
img = imread(IMAGES + currentFileName);

% (1) Take the hard-coded ROI or a manual one
if FIXED_ROI
    rect = [545 594 335 145];       % These are for odometro1.jpg
else
    figure; imshow(img); title(currentFileName);
    rect = getrect;
end

% (2) Crop the ROI
ROI = imcrop(img, rect);

% (3) Convert into gray scale the 3-dimensional image
grayROI = rgb2gray(ROI);

% (4) Extract the edges of the gray image
edges_canny = edge(grayROI, "canny");

% (5) Horizontal angles to be identified in the plate identification scenario
angles = [-90:0.5:-60, 30:0.5:89]; % [-90:1:-45, 45:1:89]

% (6) Run te Hough Lines algorithm for the detection of horizontal lines
[H, theta, rho] = hough(edges_canny, 'RhoResolution', 1, 'Theta', angles); %'Theta', -90:0.5:89);

% (7a) Identify a set of peaks in the Hough accumulation matrix
doc_peaks = houghpeaks(H, N_PEAKS);

% (7b) Get rows and thetas of lines occurring above the {HOUGH_THRESHOLD}
logic_nonzero = H>=HOUGH_THRESHOLD;
[rows, cols] = find(logic_nonzero);
my_peaks = [rows, cols];

% (8) Find the top P lines previously identified
% Fillgap: Distance between two line segments associated with the same
% Hough transform bin - default: 20
% MinLength: Minimum line length - default: 40, suggested value: ~85% of
% size(grayROI, 2)
minLength = size(grayROI, 2)*MIN_LEN_FRACTION;
fillGap = size(grayROI, 2)*FILL_GAP_FRACTION;
doc_lines = houghlines(edges_canny, theta, rho, doc_peaks, 'FillGap', fillGap, 'MinLength', minLength);
my_lines = houghlines(edges_canny, theta, rho, my_peaks, 'FillGap', fillGap, 'MinLength', minLength);

close all;

% (9a) Plot every peak line - Documentation lines
lines = doc_lines;
subplot(2,1,1), imshow(grayROI), title('(9a) Detected lines - Doc version'), hold on;
for i = 1:length(lines)
    xy = [lines(i).point1; lines(i).point2];
    plot(xy(:, 1), xy(:, 2), 'LineWidth', 1, 'Color', 'green');

    % Progression output
    log = sprintf('%d/%d doc lines', i, size(lines, 2));
    disp(log);
end
hold off;

% (9b) Plot every peak line - My lines
lines = my_lines;
subplot(2,1,2), imshow(grayROI), title('(9b) Detected lines - My version'), hold on;
for i = 1:length(lines)
    xy = [lines(i).point1; lines(i).point2];
    plot(xy(:, 1), xy(:, 2), 'LineWidth', 1, 'Color', 'green');

    % Progression output
    log = sprintf('%d/%d my lines', i, size(lines, 2));
    disp(log);
end
hold off;

% (10) Plot only the lines with the most frequent theta on the ROI

% Calculate the most common theta
lines = my_lines;
rotations = zeros(length(lines), 1);
for i = 1:length(lines)
    rotations(i) = lines(i).theta;
end
rotation_factor = mode(rotations);

% Plot lines on the ROI
figure('Name','(10) Only the lines with the most frequent theta'), imshow(grayROI);
hold on;
j = 0;
for i = 1:length(lines)
    if lines(i).theta == rotation_factor
        xy = [lines(i).point1; lines(i).point2];
        plot(xy(:, 1), xy(:, 2), 'LineWidth', 1, 'Color', 'green');
    
        % Progression output
        j = j + 1;
        log = sprintf('%d final lines', j);
        disp(log);
    end
end
hold off;

% (11) Rotate the image
if rotation_factor > 0
    rotation = -(90 - rotation_factor);
else
    rotation =  (90 + rotation_factor);
end

if isnan(rotation)
    error(['ERROR: No horizontal lines where identified! Try decreasing the ' ...
        '{HOUGH_THRESHOLD} or changing the region of interest']);
else
    rotatedROI = imrotate(grayROI, rotation, 'bilinear');
    figure, imshow(rotatedROI), title('(11) Rotated ROI according to lines');
end

disp('all done.');

if DEBUG
    disp(' DEBUGGING NOW');

    % disp('Printing rows and cols actual rhos and thetas:');
    % for i = 1:length(rows)
    %     log = sprintf('%d, %d', rho(rows(i)), theta(cols(i)));
    %     disp(log);
    % end
    
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
end