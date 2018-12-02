clear all
close all
clc

warning off                                     % To Turn Off Warnings about Image Dimension

%% Extracting Camera Parameters
    
[fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel('E:\UMD\1UMD- Robotics\Courses\ENPM 673- Perception for Autonomous Robots\Project\Project-2\Oxford_dataset\stereo\centre',...
    'E:\UMD\1UMD- Robotics\Courses\ENPM 673- Perception for Autonomous Robots\Project\Project-2\Oxford_dataset\model');

%% Recovering Color Images using Demosaic Function

% Uncomment the following section and execute. Also, enter the directory
% path where the Bayer Images lie in your computer (alongwith the file extension .png)

% BayerImages = dir('E:\UMD\1UMD- Robotics\Courses\ENPM 673- Perception for Autonomous Robots\Project\Project-2\Bayer Images\*.png');
% 
% for i = 1 : length(BayerImages)
% 
%     filename = strcat('E:\UMD\1UMD- Robotics\Courses\ENPM 673- Perception for Autonomous Robots\Project\Project-2\Bayer Images\',BayerImages(i).name);
%     
%     BIR = imread(filename);
%     DI = demosaic (BIR, 'gbrg');
%     
%     imwrite(DI, sprintf('%d.png', i));
%     
% end

% The RGB Images will be saved in the folder where the code is present.

%% Defining Camera Matrix and Camera Parameters

C = [fx 0  cx;                      % Defining C as per the definition of Camera Matrix
     0  fy cy; 
     0  0  1];
     
cameraParams = cameraParameters('IntrinsicMatrix',C');
 
J = [0 0 0];
K = [1 0 0;
     0 1 0;
     0 0 1];    

L = J;
M = K;

v = VideoWriter('Visual Odometry','MPEG-4');        % To Save the Output in Video Format
v.FrameRate = 25;                                   % Specifying the Frame Rate of the Video
open(v);

for i = 30:3400
    
    FrameNumber = i                     % To Display the Current Frame Number while the Loop is being executed
    
    % First Frame
    
    AA = imread (sprintf('%d.png', i));         % Fetching the images from the folder sequentially
    IMG1 = imgaussfilt3(AA, 0.6);               % Gaussian Smoothing Kernel with Standard Deviation of 0.6
    [UD1] = UndistortImage(IMG1, LUT);          % Using the UndistortImage.m function, the filtered images are undistorted
    
    % Next Frame
    
    BB = imread (sprintf('%d.png', i+1));
    IMG2 = imgaussfilt3(BB, 0.6);                
    [UD2] = UndistortImage(IMG2, LUT);
    
%% Finding Point Correspondence between a Pair of Images using Local Neighbhorhoods
   % and the Harris-Stephens Algorithm
    
    % Converting RGB Images to Grayscale Images
    
    GR1 = rgb2gray (UD1);
    GR2 = rgb2gray (UD2);
    
    % Finding the Corners in the Images using the Harris-Stephens Algorithm
    
    CRN1 = detectHarrisFeatures(GR1);
    CRN2 = detectHarrisFeatures(GR2);
    
    % Extracting the Neighborhood Features based on the Corners found in the Images
    
    [Feat1, VP1] = extractFeatures(GR1, CRN1);
    [Feat2, VP2] = extractFeatures(GR2, CRN2);
    
    % Matching the Features and Creating their Pair
    
    IndexPair = matchFeatures(Feat1, Feat2);
    
    % Retrieve the Locations of the Corresponding Points for Each Image 
    
    MatchPoint1 = VP1 (IndexPair(:,1),:);
    MatchPoint2 = VP2 (IndexPair(:,2),:);
    
    % Visualize the Corresponding Points and showing their Correspondence in the Successive Frames
    
%     figure;
%     showMatchedFeatures (GR1, GR2, MP1, MP2);         % Uncomment this section to print the Correspondence Points between two Images
%     hold on;
    
    % Finding the Location of the Correspondence Points
    
    PLoc1 = MatchPoint1.Location;
    PLoc2 = MatchPoint2.Location;

%% Normalization of Correspondence Points 

    X1 = PLoc1(:, 1);
    Y1 = PLoc1(:, 2);
    
    X2 = PLoc2(:, 1);
    Y2 = PLoc2(:, 2);

    SZ = size(X1);
    
    XX = [X1'; Y1']';
    YY = [X2'; Y2']';     

    [Tl,X1n] = Normalization(XX);
    [Tr,X2n] = Normalization(YY);


%% Applying the RANSAC Function and Selecting 8 Points

    Threshold = .002;
    SampNum = 500;   

for i = 1: SampNum
    
% Selecting 8 Points Randomly in Each Iteration
    
    Indice = randperm(size(XX,1), 8);       
      
    Samp1 = XX(Indice, :);
    Samp2 = YY(Indice, :);
     
    [T1, Samp1N] = Normalization(Samp1);
    [T2, Samp2N] = Normalization(Samp2);
    
    FN = FundamentalMatrix(Samp1N, Samp2N);         % Normalized Fundamental Matrix
    
%% Finding Inliers

    Initial = 0;
    Count = [];
    
    Diff = zeros(8, 1) + 1; 
    
    for j = 1: 8
       
        EV = [Samp2N(j,:) 1] * FN * [Samp1N(j,:) 1]';
    
       if (abs(EV) < Threshold)
          
           Initial = Initial + 1;
           Count = [Count; Indice(1,j)];
           Diff(j) = abs(EV); 
       
       end
       
    end
   
   GoodInliers = 0;
   
   if (Initial > GoodInliers)
       GoodInliers = Initial;
       FTemp = T2'*FN*T1;
       F = FTemp/norm(FTemp);       % Fundamental Matrix
       
       if F(end) < 0
          F = -F; 
       end
       
       BD = Diff;
       BI = Count;
       
       if GoodInliers ~= 8          % To quickly select the 8 random points
           continue
           
       else
           break
       end
       
   end
   
end

    Inliers1 = XX(BI,:);
    Inliers2 = YY(BI,:);
    
    p1 = [Inliers1(1,:) 1]';
    p2 = [Inliers2(1,:) 1]';
    
    E = C' * F * C;                 % Essential Matrix
    
    [U, D, V] = svd(E);
    D(1,1) = 1;
    D(2,2) = 1;
    D(3,3) = 0;
    
    E = U * D * V';
    [U, ~, V] = svd(E);         % Ignoring the 2nd Matrix as it is not needed

    W = [0 -1 0;
         1  0 0; 
         0  0 1];
     
    Rot1 = U * W * V';    
    
    if det(Rot1) < 0
        Rot1 = -Rot1;
    end
    
    Rot2 = U * W' * V';
    
    if det(Rot2) < 0
        Rot2 = -Rot2;
    end
   
    Trans1 = U(:,3)';
    Trans2 = -Trans1;
    
    RR = cat(3, Rot1, Rot1, Rot2, Rot2);
    TT = cat(1, Trans1, Trans2, Trans1, Trans2);
    
%% Choose the Right Solution for Rotation and Translation Matrices
    
    NA = zeros(1, 4);
    P1 = cameraMatrix(cameraParams, eye(3), [0,0,0]);

for k = 1:size(TT, 1)

    P2 = cameraMatrix(cameraParams,RR(:,:,k)', TT(k, :));
    
%% Triangulation

       PointT1 = zeros(size(Inliers1, 1), 3, 'like', Inliers1);
       
       P1Temp = P1';
       P2Temp = P2';
       
       M1 = P1Temp(1:3, 1:3);
       M2 = P2Temp(1:3, 1:3);
       
       O1 = -M1 \ P1Temp(:,4);
       O2 = -M2 \ P2Temp(:,4);

       for kk = 1:size(Inliers1,1)
          
           u1 = [Inliers1(kk,:), 1]';
           u2 = [Inliers2(kk,:), 1]';
           
           a1 = M1 \ u1;
           a2 = M2 \ u2;
           
           A = [a1, -a2];
           y = O2 - O1;
          
           Prod = (A' * A) \ A' * y;
           p = (O1 + Prod(1) * a1 + O2 + Prod(2) * a2) / 2;
           PointT1(kk, :) = p';
       
       end
       
       PointT2 = bsxfun(@plus, PointT1 * RR(:,:,k)', TT(k, :));
       NA(k) = sum((PointT1(:,3) < 0) | (PointT2(:,3) < 0));
    
    end
    
    [val, idx] = min(NA);
    Rot = RR(:,:,idx)';
    Trans = TT(idx, :);
    TNorm = norm(Trans);
    
    if TNorm ~= 0
       Trans = Trans ./ TNorm;
       
    end
    
    
%% Extracting Rotation and Translation Matrices
    
    Rot = Rot';                     % Rotation Matrix
    Trans = -Trans * Rot;           % Translation Matrix
     
    K = Rot * K;                    
    J = J + Trans * K;              % Final Position of Camera
    
%% Finding out the Fundamental Matrix using the Inbuilt Function 

% The Inliers are used as input to this function and the 8 points are found
% out in this

    FTheoretical = estimateFundamentalMatrix(Inliers1,Inliers2,'Method','Norm8Point');

% The Relative Camera Pose function

    [relativeOrientation,relativeLocation] = relativeCameraPose(FTheoretical, cameraParams, Inliers1, Inliers2);
     
    M = relativeOrientation * M;    
    L = L + relativeLocation * M;   % Final Position of Camera
    
    % Plotting the Trajectory Based on the Fundamental Matrix found out Conventionally   
    
    figure(1);    
    subplot(1,3,1);
    title('Implementation Trajectory'); plot(J(1),J(3),'*', 'MarkerEdgeColor', 'c');
    hold on;
    
    % Plotting the Images
    
    subplot(1,3,2); title('Camera Input');
    imshow(UD2);
    hold on;
    
    % Plotting the Trajectory Based on the MATLAB's unbuilt function
    subplot(1,3,3);
    title('Theoretical Trajectory'); plot(L(1),L(3),'*', 'MarkerEdgeColor', 'b');
    hold on;
    
    frame = getframe(gcf);                  % Fetching the Frames
    writeVideo(v,frame);                    % Saving the Video File from Frames
    
    pause(0.005)                            % Giving a Pause to display the Process of Plotting

end 

close(v)