clear all
clc

%%

I1 = imread('startII.png');
I2 = imread('goalII.png');

% % Environment I - 381x610
I1 = imcrop(I1, [200 240 809-200 620-240]);
I2 = imcrop(I2, [200 240 809-200 620-240]);
% Environment II
% I1 = imcrop(I1, [200 250 809-200 630-250]);
% I2 = imcrop(I2, [200 250 809-200 630-250]);
% % Environment III
% I1 = imcrop(I1, [180 330 809-200 620-240]);
% I2 = imcrop(I2, [180 330 809-200 620-240]);
%%
for j = 0:3
    for i = 1:size(I1,2)
        I1(1+j,i,:) = [0 0 0];
        I1(end-j,i,:) = [0 0 0];
        I2(1+j,i,:) = [0 0 0];
        I2(end-j,i,:) = [0 0 0];
    end
    
    for i = 1:size(I1,1)
        I1(i,1+j,:) = [0 0 0];
        I1(i,end-j,:) = [0 0 0];
        I2(i,1+j,:) = [0 0 0];
        I2(i,end-j,:) = [0 0 0];
    end
end

I = [I1 I2];

I = insertText(I,[0 -10],'(c)','FontSize',60,'BoxColor',...
    'w','BoxOpacity',0,'TextColor','black');

imshow(I);

imwrite(I, 'envIII.png');

