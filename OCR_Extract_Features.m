% 
% OCR_Extract_Features
% Extract features of characters in an image.
%
% Inputs: im: An image in which characters need to be feature extracted.
%         plot:'0' to display all the plots, '1' to not
% Output: Features: [ #characters * 6 ] matrix, every column represents a character and every row represents the 6 features of a character
%                   The 6 features are 1 theta, 1 roundness and 4 from Hu moments(inmo)
%
function Features = OCR_Extract_Features (im, plot)
th = 200;
im2 = im;
im2(im >= th) = 0;
im2(im < th) = 1;
A = [0 1 0
    1 1 1
    0 1 0];
im2 = bwmorph(im2,'close');  %close operation
im2 = imdilate(im2,A);       %dilation
L = bwlabel(im2);
Nc=max(max(L));
Features=[];
if plot == 0
    figure
    imagesc(L)
    title('Connected_Components')
end
for i = 1 : Nc
    [r,c] = find( L == i);
    maxr = max(r);
    minr = min(r);
    maxc = max(c);
    minc = min(c);
    if ((maxr - minr)<80 && (maxr - minr)>7 && (maxc - minc)<80  && (maxc - minc)>7 && (maxr-minr)*(maxr-minr)>70)
        if plot == 0
        rectangle('Position',[minc,minr,maxc-minc+1,maxr-minr+1], 'EdgeColor','w');
        end
        [centroid, theta, roundness, inmo] = moments(im2(minr:maxr,minc:maxc), 1);
        Features = [Features; theta, roundness, inmo];
    end    
end

   