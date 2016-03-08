%% image T1
T1 = imread('T1.gif');
T1 = quilting(T1, [200,200], 32, 6);

%% image T2
T2 = imread('T2.gif');
T2 = quilting(T2, [200,200], 32, 6);

%% image T3
T3 = imread('T3.gif');
T3 = quilting(T3, [200,200], 18, 4);

%% image T4
T4 = imread('T4.gif');
T4 = quilting(T4, [200,200], 18, 4);

%% image T5
T5 = imread('T5.gif');
T5 = quilting(T5, [200,200], 18, 4);
 
%% figure
figure;
subplot(231);imshow(uint8(T1)); title('T1');
subplot(232);imshow(uint8(T2)); title('T2');
subplot(233);imshow(uint8(T3)); title('T3');
subplot(234);imshow(uint8(T4)); title('T4');
subplot(235);imshow(uint8(T5)); title('T5');
% figure; imshow(uint8(T1)); title('T1 quilting');
% figure; imshow(uint8(T2)); title('T2 quilting');
% figure; imshow(uint8(T3)); title('T3 quilting');
% figure; imshow(uint8(T4)); title('T4 quilting');
% figure; imshow(uint8(T5)); title('T5 quilting');
