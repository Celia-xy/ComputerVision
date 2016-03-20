I_0 = (imread('test_im31.jpg'));
col_1 = [80   80   95   90   95   95   88   88];
row_1 = [128   177   176   160   150   137   133   126];
cow_2 = [0   203   245   146   0];
row_2 = [226   158   157   240   240];
cow_3 = [285   284   276   284   301   294   290   290];
row_3 = [240   205   200   184   188   207   206   240];
%--------- criminisi -- person -------------
% mark the unfilled region
Mask = uint8(roipoly(I_0,col_1,row_1));    % 0 for filled region; 1 for unfilled region    
Red = I_0(:,:,1);
Red = uint8(Red) .* (1-Mask);
Green = (I_0(:,:,2));
Green = 255 * Mask + Green .* (1-Mask);
Blue = (I_0(:,:,3));
Blue = Blue .* (1-Mask);
I = uint8(cat(3,double(Red),double(Green),double(Blue)));
% fill the region
[I_1,i2,i3,col,d,mov] = inpaintc(I_0,I,[0 255 0]);
I_1 = uint8(I_1);

%--------- criminisi -- light area -------------
% image sample used to inpaint
I = I_1;
col = [0   206   265   278   0];
row = [220   157   161   240   240];
Mask = uint8(roipoly(I,col,row));
Red = I(:,:,1);
Red_1 = Red .* Mask;
Green = I(:,:,2);
Green_1 = Green .* Mask;
Blue = I(:,:,3);
Blue_1 = Blue .* Mask;
I_sample = uint8(cat(3,double(Red_1),double(Green_1),double(Blue_1)));
% mark the unfilled region
Mask_2 = uint8(roipoly(I,cow_2,row_2));  % 0 for filled region; 1 for unfilled region   
Red_1 = Red_1 .* (1-Mask_2);
Green_1 = (Mask_2 + (1 - Mask) ) *255 + Green_1 .* (1-Mask_2);
Blue_1 = Blue_1 .* (1-Mask_2);
I = uint8(cat(3,double(Red_1),double(Green_1),double(Blue_1)));
% fill the region
[I_2,i2,i3,col,d,mov] = inpaintc(I_sample,I,[0 255 0]);
% unify the origin image and filled image sample
Red_2 = Red .* (1-Mask) + uint8(I_2(:,:,1)) .* Mask;
Green_2 = Green .* (1-Mask) + uint8(I_2(:,:,2)) .* Mask;
Blue_2 = Blue .* (1-Mask) + uint8(I_2(:,:,3)) .* Mask;
I_2 = uint8(cat(3,double(Red_2),double(Green_2),double(Blue_2)));

%--------- criminisi -- sign -------------
% mark the unfilled region
I = I_2;
Red = I(:,:,1);
Green = I(:,:,2);
Blue = I(:,:,3);
Mask = uint8(roipoly(I_0,cow_3,row_3));   % 0 for filled region; 1 for unfilled region   
Red = Red .* (1-Mask);
Green = 255 * Mask + Green .* (1-Mask);
Blue = Blue .* (1-Mask);
I = uint8(cat(3,double(Red),double(Green),double(Blue)));
% fill the region
[I_3,i2,i3,col,d,mov] = inpaintc(I_2,I,[0 255 0]);
I_3 = uint8(I_3);

%--------- output -------------
figure;imshow(uint8(I_1)); title('T8 Man');
figure;imshow(uint8(I_2)); title('T8 Light');
figure;imshow(uint8(I_3)); title('T8 Sign');