% MOMENTS
%
% Function calculates the moments of a binary image and returns the centroid,
% the angle of axis of minimum inertia, a measure of 'roundness', and a vector
% of invariant moments. The function assumes that there is only one object in
% the binary image. Function also displays the image and overlays the position
% of the centroid and the axis of minimum inertia.
%
% Usage: [centroid, theta, roundness, inmo] = moments(im, plotchoice)
%
% Args: im         = binary image containing values of 0 or 1
%       plotchoice = display image and axis; 0 for display, 1 for no display
%
% Returns: centroid  = 2-element vector
%          theta     = angle of axis of minimum inertia (radians)
%          roundness = ratio of (minimum inertia)/(maxiumum inertia)
%          inmo      = 4-element vector containing the first four
%                      invariant moments
%
% Note: Positive x is to the right and positive y is downwards, thus
%       angles are positive clockwise.

function [centroid, theta, roundness, inmo] = moments(im, plotchoice)

    % calculate centroid
    area = sum(sum(im));
    [rows,cols] = size(im);
    x = ones(rows,1)*[1:cols];
    y = [1:rows]'*ones(1,cols);
    meanx = sum(sum(double(im).*x))/area;
    meany = sum(sum(double(im).*y))/area;
    centroid = [meanx,meany];

    % calculate theta and roundness
    xdash = x - meanx;
    ydash = y - meany;
    a = sum(sum(double(im).*xdash.*xdash));
    b = 2*(sum(sum(double(im).*xdash.*ydash)));
    c = sum(sum(double(im).*ydash.*ydash));
    aminusc = (a-c);
    denom = sqrt((b*b)+(aminusc*aminusc));
    if denom == 0
	theta = 0;
	roundness = 1;
    else
	sin2theta = b/denom;
	cos2theta = aminusc/denom;
	twotheta = atan2(sin2theta,cos2theta);
	theta = twotheta/2;

	costheta2 = cos(theta)*cos(theta);
	minaxis = a*(1-costheta2)+c*costheta2-b*sin2theta/2;
	maxtheta = atan2(-1*sin2theta, -1*cos2theta);
	costheta2 = cos(maxtheta)*cos(maxtheta);
	maxaxis = a*(1-costheta2)+c*costheta2+b*sin2theta/2;
	roundness = minaxis/maxaxis;
    end

    % display axis of minimum inertia
    if plotchoice == 0
	startx = 0;
	startrho = -meanx / cos(theta);
	starty = meany + startrho * sin(theta);

	endx = cols;
	endrho = (endx - meanx) / cos(theta);
	endy = meany + endrho * sin(theta);

	imagesc(im);
	hold on;
	plot([startx; meanx; endx], [starty; meany; endy],'gx--');
    end

    % calculate invariant moments
    % phi1
    mu20 = sum(sum(double(im).*xdash.*xdash));
    gamma = (2 + 2)/2;
    n20 = mu20/(area^gamma);
    mu02 = sum(sum(double(im).*ydash.*ydash));
    n02 = mu02/(area^gamma);
    phi1 = n20 + n02;
    % phi2
    mu11 = sum(sum(double(im).*xdash.*ydash));
    n11 = mu11/(area^gamma); % gamma is the same
    phi2 = (n20 - n02)^2 + 4*n11^2;
    % phi3 and phi4
    mu30 = sum(sum(double(im).*xdash.*xdash.*xdash));
    gamma = (3 + 2)/2;
    n30 = mu30/(area^gamma);
    mu12 = sum(sum(double(im).*xdash.*ydash.*ydash));
    n12 = mu12/(area^gamma);
    mu21 = sum(sum(double(im).*xdash.*xdash.*ydash));
    n21 = mu21/(area^gamma);
    mu03 = sum(sum(double(im).*ydash.*ydash.*ydash));
    n03 = mu03/(area^gamma);
    phi3 = (n30 - 3*n12)^2 + (3*n21 - n03)^2;
    phi4 = (n30 - n12)^2 + (n21 - n03)^2;
    % inmo
    inmo = [phi1 phi2 phi3 phi4];