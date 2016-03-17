Win_size = 5;  % window size, odd integer such as 5,9,11...
%% image test1
I0 = (imread('test_im1.bmp'));   
Element = [1 1 1; 1 1 1; 1 1 1];   % the element for dilation
Test1 = inpaintl(I0, Win_size, Element, I0);
%% image test2
I0 = (imread('test_im2.bmp'));
Element = [0 1 0; 0 1 0; 0 1 0];   % the element for dilation
Tes2 = inpaintl(I0, Win_size, Element, I0);
%% figure
figure;
subplot(121);imshow(uint8(Test1)); title('Test1');
subplot(122);imshow(uint8(Test2)); title('Test2');
% figure; imshow(uint8(Test1)); title('Test1 win5');
% figure; imshow(uint8(Test2)); title('Test2 win5');