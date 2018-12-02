clc
clear all
close all

%% Original Image

I_o = imread ('TestImgResized.jpg');


%% Original Image to Denoised Image

I_den = medfilt3 (I_o);

figure, subplot (1,2,1); imshow (I_o), title ('Original Image');
subplot (1,2,2); imshow (I_den), title ('Denoised Image');


%% Original Image to Gray Image

I_gray = rgb2gray (I_o);

figure, subplot (1,2,1); imshow (I_o), title ('Original Image');
subplot (1,2,2); imshow(I_gray), title ('Gray Image');


%% Denoised Image to Black & White (Binary) Image

I_bw2 = im2bw (I_den);
I_bw = imcomplement (I_bw2);

figure, subplot (1,3,1); imshow (I_o), title ('Original Image');
subplot (1,3,2); imshow (I_den), title ('Denoised Image');
subplot (1,3,3); imshow (I_bw), title ('Black & White Image');


%% Original Image into RGB Images

rimg = I_den (:,:,1);
gimg = I_den (:,:,2);
bimg = I_den (:,:,3);

I_red = im2bw (rimg, 0.5);
I_green = im2bw (gimg, 0.5);
I_blue = im2bw (bimg, 0.4);

I_total = (I_red & I_green & I_red);
%figure, imshow (I_total), title ('Combined RGB Photo')

% Complimenting the Black & White Image

I_bw3 = imcomplement (I_total);
str = strel ('disk', 10);
figure, subplot (1,2,1); imshow (I_total), title ('Original Black & White Image');
subplot (1,2,2); imshow (I_bw3), title ('Complimentary Black & White Image');


%% All Images in One Figure

%figure ('color', 'white', 'name', 'Color Partition');

se = strel ('disk', 10);
I_bwclr = imopen (I_bw, se);

%figure ('color', 'white', 'name', 'Black & White Image'); imshow (I_bwclr)


%% Morphological Operations for Marking of Pins

markpin = regionprops (I_bwclr, 'Centroid', 'MajorAxisLength', 'MinorAxisLength');
cen = cat (1, markpin.Centroid);
majo = cat (1, markpin.MajorAxisLength);
mino = cat (1, markpin.MinorAxisLength); 

[L, num] = bwlabel (I_bwclr);

for i=1: length (markpin)
dia (i,1) = mean ([majo(i,1) mino(i,1)],2);
radii (i,1) = dia (i,1)*2/1.5
end

figure ('color', 'white', 'name', 'Total Number of Colored Pins'); imshow (I_o)
title (['Total ', num2str(num), ' Colored Pins are Present'])

hold on;
plot(cen(:,1), cen(:,2), 'k*', 'MarkerSize',20)
hold off;



%% Finding Individual Color Objects - Red Pins

red = I_den(:,:,1); green = I_den(:,:,2); blue = I_den(:,:,3); 
r_p = red > 90 & green < 80 & blue < 100 ;  

ser = strel ('disk', 4);
r_pins = imopen (r_p, ser);

%figure ('color', 'white', 'name', 'Red Pins'); imshow (r_pins)

markred = regionprops(r_pins, 'Centroid', 'MajorAxisLength','MinorAxisLength');
cenr = cat(1, markred.Centroid);
major = cat(1, markred.MajorAxisLength);
minor = cat(1, markred.MinorAxisLength);
[L1,num1] = bwlabel(r_pins);

for i=1: length(markred)
    diar(i,1) = mean([major(i,1) minor(i,1)],2);
    radiir(i,1) = diar(i,1)/2*1.5;
end


%% Finding Individual Color Objects - Green Pins

red = I_den(:,:,1); green = I_den(:,:,2); blue = I_den(:,:,3); 
g_p = red < 50 & red > 0 & green > 40 & green < 200 & blue < 105 ;  

seg = strel ('disk', 4);
g_pins = imopen (g_p, seg);

%figure ('color', 'white', 'name', 'Green Pins'); imshow (g_pins)

markgreen = regionprops(g_pins, 'Centroid', 'MajorAxisLength','MinorAxisLength');
ceng = cat(1, markgreen.Centroid);
majog = cat(1, markgreen.MajorAxisLength);
minog = cat(1, markgreen.MinorAxisLength);
[L2,num2] = bwlabel(g_pins);

for i=1: length(markgreen)
    diag(i,1) = mean([majog(i,1) minog(i,1)],2);
    radiig(i,1) = diag(i,1)/2*2;
end


%% Finding Individual Color Objects - Blue Pins

red = I_den(:,:,1); green = I_den(:,:,2); blue = I_den(:,:,3); 
b_p = red < 55 & green > 40 & green < 125 & blue < 200 & blue > 90 ;  

seb = strel ('disk', 4);
b_pins = imopen (b_p, seb);

%figure ('color', 'white', 'name', 'Blue Pins'); imshow (b_pins)

markblue = regionprops(b_pins, 'Centroid', 'MajorAxisLength','MinorAxisLength');
cenb = cat(1, markblue.Centroid);
majob = cat(1, markblue.MajorAxisLength);
minob = cat(1, markblue.MinorAxisLength);
[L3,num3] = bwlabel(b_pins);

for i=1: length(markblue)
    diab(i,1) = mean([majob(i,1) minob(i,1)],2);
    radiib(i,1) = diab(i,1)/2*2;
end


%% Finding Individual Color Objects - Yellow Pins

red = I_den(:,:,1); green = I_den(:,:,2); blue = I_den(:,:,3); 
y_p = red < 230 & red > 100 & green > 100 & green < 200 & blue < 50 & blue > 0 ;  

sey = strel ('disk', 4);
y_pins = imopen (y_p, sey);

%figure ('color', 'white', 'name', 'Yellow Pins'); imshow (ypins)

markyellow = regionprops(y_pins, 'Centroid', 'MajorAxisLength','MinorAxisLength');
ceny = cat(1, markyellow.Centroid);
majoy = cat(1, markyellow.MajorAxisLength);
minoy = cat(1, markyellow.MinorAxisLength);
[L4,num4] = bwlabel(y_pins);

for i=1: length(markyellow)
    diay(i,1) = mean([majoy(i,1) minoy(i,1)],2);
    radiiy(i,1) = diay(i,1)/2*2;
end


%% Plotting results for individual colored object detection results - for Red, Green, Blue and Yellow

figure ('color', 'white', 'name', 'Individual Colored Pins Detected'); subplot (2,2,1); imshow (I_den), title ([num2str(num1), ' RED Pins'])
hold on; 
viscircles(cenr, radiir, 'edgecolor', [0,0,0]);
hold off;

subplot (2,2,2); imshow (I_den), title ([num2str(num2), ' GREEN Pins'])
hold on;
viscircles(ceng, radiig, 'edgecolor', [0,0,0]);
hold off;

subplot (2,2,3); imshow (I_den), title ([num2str(num3), ' BLUE Pins'])
hold on;
viscircles(cenb, radiib, 'edgecolor', [0,0,0]);
hold off;

subplot (2,2,4); imshow (I_den), title ([num2str(num4), ' YELLOW Pins'])
hold on;
viscircles(ceny, radiiy, 'edgecolor', [0,0,0]);
hold off;


