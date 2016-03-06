
function [I] = inpaintl(I, Win_size, Element, Sample_area)
% Usage:   [Filled Image] = inpaintl (Image, Win_size, Element, Sample_area)
% Inputs: 
%   Image         An image where pixels with value 0 need to be inpainted
%   Win_size      The defined window size in leung's algorithm
%   Element       The dilation element used to find unfilled neighbors,
%                 usually a 3x3 matrix of 0 and 1
%   Sample_area   The area to get samples used in bestmatch
% Output:
%   Filled Image  The inpainted image 
   
Sigma = (Win_size)/6.4;    % sigma in gaussian
ErrThreshold = 0.1;  
% get samples in original image
Sample = im2col(Sample_area,[Win_size Win_size]);  
% remove useless samples
[r,c] = find(Sample == 0); 
Sample(:,c) = []; 
% find margin of unfilled neighbors
    BW = logical(I);
    margin = imdilate(BW,Element) - BW;    
    [r,c] = find(margin);
    Pixel_list = [r,c];
% inpainting
while ~isempty(Pixel_list) % while image is not filled   
    for i = 1 : size(r)           
        % get neighborhood window for a pixel
        Template = zeros(Win_size, Win_size); 
        [a,b] = size (I);
        % get Template
        for m = 1 : Win_size
            for n = 1: Win_size
                if ((Pixel_list(i,1)-(Win_size+1)/2 + m)>0 && (Pixel_list(i,1)-(Win_size+1)/2 + m) <= a && (Pixel_list(i,2)-(Win_size+1)/2 + n) > 0 && (Pixel_list(i,2)-(Win_size+1)/2 + n) <= b)          
                Template(m,n) = I(Pixel_list(i,1)-(Win_size+1)/2 + m,Pixel_list(i,2)-(Win_size+1)/2 + n);          
                end
            end
        end
        % find best match
        Valid_mask = logical(Template);    % 1s filled, 0s otherwise
        Gauss_mask = fspecial('gaussian', Win_size, Sigma); 
        Weight = Gauss_mask .* Valid_mask;
        TotWeight = sum(sum(Gauss_mask .* Valid_mask)); 
        Weight_new = Weight(:) * ones(1,size(Sample,2));  % same size Weight as Sample
        Template_new = Template(:) * ones(1,size(Sample,2));  % same size Template as Sample
        Sample = double(Sample);
        dist = Sample - Template_new;
        dist = dist.^2;  % get distance
        SSD = sum(dist .* Weight_new)./TotWeight; % get SSD of the samples
        Match_list = find(SSD <= min(SSD) .* (1 + ErrThreshold));  % list of all best matches
        Bestmatches = Sample(((Win_size^2+1)/2),Match_list); %  all best matches
        Bestmatch = Bestmatches((floor(rand(1))+1) .* length(Bestmatches)); % random pick
        I(Pixel_list(i,1), Pixel_list(i,2)) = Bestmatch;  % set the pixel to best
        %debug in real-time
        imagesc(I); 
        axis image; colormap gray; 
        drawnow; 
    end 
    % find margin of new unfilled neighbors
    BW = logical(I); 
    margin = imdilate(BW,Element) - BW;    
    [r,c] = find(margin);
    Pixel_list = [r,c];
end