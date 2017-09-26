I1 = imread('startII.png');
I2 = imread('goalII.png');

% % Environment I - 381x610
% I1 = imcrop(I1, [200 240 809-200 620-240]);
% I2 = imcrop(I2, [200 240 809-200 620-240]);
% Environment II
I1 = imcrop(I1, [200 250 809-200 630-250]);
I2 = imcrop(I2, [200 250 809-200 630-250]);


for i = 1:size(I1,2)
    I1(1,i,:) = [0 0 0];
    I1(end,i,:) = [0 0 0];
    I2(1,i,:) = [0 0 0];
    I2(end,i,:) = [0 0 0];
end
for i = 1:size(I1,1)
    I1(i,1,:) = [0 0 0];
    I1(i,end,:) = [0 0 0];
    I2(i,1,:) = [0 0 0];
    I2(i,end,:) = [0 0 0];
end

I = [I1; I2];
imshow(I);

imwrite(I, 'envII.png');

