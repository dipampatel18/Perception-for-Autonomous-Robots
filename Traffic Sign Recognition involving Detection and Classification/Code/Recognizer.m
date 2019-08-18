%%
%% Traffic Sign Recognition

load ('Classifier.mat')
tic
warning off

%% Location of Input Frames

cd '/Input Frames/'

ListFiles = ls('*jpg');
TempFiles = ListFiles;

%% Creating the Video

v = VideoWriter('Traffic Sign Detection','MPEG-4');
v.FrameRate = 30;
open(v);

for frame = 530:2700
    
    FrameNumber = frame
    
    File = TempFiles(frame,:);
    ImageFile = imread(File);
    
%% Applying the Gaussian Filter to Denoise the Frame
    
    mu = 5;
    sigma = 2;
    
    index = -floor(mu/2) : floor(mu/2);
    [X Y] = meshgrid(index, index);
    
    H = exp(-(X.^2 + Y.^2) / (2*sigma*sigma));
    H = H / sum(H(:));
    IMF = imfilter(ImageFile, H);
    
%% Contrast Normalization
     
    I = imadjust(IMF, stretchlim(IMF), []);
    
%% RGB Normalization & Grayscaling of Frame
    
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    
    Red = uint8(max(0, min(R-B, R-G)));
    RR = im2bw(Red, .15);
    
    Blue = uint8(max(0, min(B-R, B-G)));
    BB = im2bw(Blue, .15);
    
% TO detect both the Red and Blue signs, their addition is used

    RB = Red + Blue;
    
%% Applying Mask to Bottom-Half of the Frame 
    
    x = [1 1628 1628 1];
    y = [1 1 618 618];
    
    Mask = poly2mask(x, y, 1236, 1628);
    Crop = uint8(immultiply(RB, Mask));
    
%% Using the MSER Algorithm

% The vl_functions have been used from the VLFeat Tutorial

    [r, f] = vl_mser(Crop, 'MinDiversity', 0.7, 'MaxVariation', 0.2, 'Delta',8);
    
    M = zeros(size(Crop));
    
    for x = r'
        s = vl_erfill(Crop,x);
        M(s) = M(s) + 1;
    end
    
    thresh = graythresh(M);
    M = im2bw(M, thresh);
    
    se = strel('octagon',6);
    M = imdilate(M, se);
    
%% Area Filtering of the Frame

    M = bwareafilt(M, [950 10000]);
    
    regions = regionprops(M, 'BoundingBox');
    
%% Get Bounding Boxes for the Signs given by MSER
    
    figure(1) ;
    clf;
    imagesc(ImageFile);
    hold on;
    axis equal off;
    colormap gray ;
    
    for k = 1 : length(regions)
        box = regions(k).BoundingBox;
        ratio = box(3)/box(4);
        
%% Bounding Box based on the Ratio for Detection
        
        if ratio < 1.1 && ratio > 0.6 %Aspect Ration of detections
            sign = imcrop(ImageFile, box);
            sign = im2single(imresize(sign,[64 64]));
            
%% Get HOG Features of Detection
            
            hog = [];
            hog_det = vl_hog(sign, 4);
            [hog_1, hog_2] = size(hog_det);
            dim = hog_1 * hog_2;
            
%% Rearranging the Dimensions of the Array
            
            hog_det_trans = permute(hog_det, [2 1 3]);
            hog_det=reshape(hog_det_trans,[1 dim]);
            hog = hog_det;

%% Fetching the Samples 
            
            [predictedLabels, score] = predict(classifier, hog);
            
            label = str2num(predictedLabels);
            
            for j = 1:length(score)

%% Actual Plotting of Bounding Box around the Sign in Blue Color

                if (score(j) > -0.04)
                    rectangle('Position', box, 'EdgeColor', 'b', 'LineWidth', 2 )
                    
                    cd ..
                    cd 'Sample'         % Fetching sample images of the signs to be detected
                    
                    TempName = strcat(predictedLabels,'.jpg');
                    TempImage = imread(TempName);
                    
                    TempImage = im2single(imresize(TempImage,[box(4) box(3)]));
                    image([int64(box(1) - box(3)) int64(box(1) - box(3))], [int64(box(2)) int64(box(2))], TempImage);
                    
                    cd ..
                    cd 'Input Frames'
                    
                end
                
            end
            
        end
        
    end
    
    
    frame = getframe(gca);
    writeVideo(v,frame);
    
end

cd '..'   

close(v)

toc