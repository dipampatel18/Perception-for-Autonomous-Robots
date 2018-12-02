clearvars
close all
clc

%% Fetching the Video into Matlab

V = VideoReader ('Project Video.mp4');                                     % Fetching a Video named 'ProjectVideo.mp4' and assigning it to variable 'V'
NF = V.NumberOfFrames();                                                   % This Video is divided into Number of Frames            

OutVid = VideoWriter ('Lane Detection');                                   % The Output Video is to be saved under the name 'LDTest.mp4'
OutVid.FrameRate = 25;                                                     % The Frame Rate is used while dividing the Video 
open(OutVid);

%% Extracting Frames from the Raw Video

for Img = 750 : 1250;
        
%% Reading Frames from Number of Frames
    
    F = read (V, Img);
    
%% Denoising the Frames
    
    M = medfilt3 (F);
    
%% Binarization of Frames
    
    BW = im2bw (M, .70);   
    
%% Edge Detection using Canny Function (To recognize the Vertical Edges in the Frames)

    ED = edge (BW, 'canny');    
    
%% Extracting Region of Interest (Masking the Upper Half of the Lane)
    
    x = [215 555 715 1275];                                                % Providing X & Y Co-ordinates for Extracting ROI
    y = [715 445 455 715];
    
    PolyMax = y(2) + 20;
    Mask = poly2mask (x, y, 720, 1280);                                    % Masking ROI with the co-ordinates
    Crop = immultiply (ED, Mask);                                          % Multiplying the images stored in ED and Mask
    
%% Finding Hough Peaks
    
    [H, T, R] = hough (Crop);
    HP = houghpeaks (H, 10, 'threshold', ceil (0.1 * max (H(:))));         % Defining different parameters for 'houghpeaks'
    
%% Finding Hough Lines and Plotting Them
    
    Lines = houghlines (Crop, T, R, HP, 'FillGap', 3, 'MinLength', 15);    % Defining different parameters for 'houghlines'
    imshow(M), hold on
     
    for L = 1 : length (Lines)                                             % Plotting of houghlines using 'for' loop
        xy = [Lines(L).point1; Lines(L).point2];
    
    end 

%% Detecting Slope of Hough Lines
    
    MaxSlope = 0.37;                                                       % Defining the Best Value of Slope based on Trial & Error Method
    ImgSize = size(F);
    center = ImgSize(2)/2;                                                 % Finding the Center of the Image
    i = 1; j = 1; l =1;
        
    for k = 1 : length (Lines)
        Initial = Lines(k).point1;
        Final = Lines(k).point2;
         
        if Final(1) - Initial(1) == 0                                      % Providing Constraints when Slope might become Infinity
           slope = 100; 
        
        else
           slope = (Final(2) - Initial(2))/(Final(1) - Initial(1));        % Formula for Calculating Slope of a Line
        
        end
        
        if abs(slope) > MaxSlope
           FinSlope(l) = slope;
           Lines(l) = Lines(k);
           l = l + 1;
        
        end
        
%% Dividing Lines into Left & Right Stripes

        if FinSlope(k) > 0 && Final(1) > center && Initial(1) > center     
           RightLane(i) = Lines(k);
           RightTemp = 1;
           i = i + 1;
        
        elseif FinSlope(k) < 0 && Final(1) < center && Initial(1) < center
               LeftLane(j) = Lines(k);
               LeftTemp = 1;
               j = j + 1;
        
        else
            RightTemp = 0;
            LeftTemp = 0;
        
        end
        
    end
    
%% Linear Regression to fit a Polynomial for Right & Left Lines
    
    if (RightTemp == 0 || LeftTemp == 0)
        continue
    end   
    
%% Right Side Stripes
    
    i = 1;
    if RightTemp == 1
    
        for k = 1 : length(RightLane)
            Initial = RightLane(k).point1;
            Final = RightLane(k).point2;
    
            RightX(i) = Initial(1);
            RightY(i) = Initial(2);
            i = i + 1;
    
            RightX(i) = Final(1);
            RightY(i) = Final(2);
            i = i + 1;
        
        end
        
        if length(RightX) > 0
            % y = m*x + b
            Pol = polyfit (RightX, RightY, 1);
            RightA = Pol(1);
            RightB = Pol(2);
        
        else
            RightA = 1;
            RightB = 1;
        
        end
        
    end
    
%% Left Side Stripes

    i = 1;
    if LeftTemp == 1
    
        for k = 1:length(LeftLane)
            Initial = LeftLane(k).point1;
            Final = LeftLane(k).point2;
    
            LeftX(i) = Initial(1);
            LeftY(i) = Initial(2);
            i = i + 1;
    
            LeftX(i) = Final(1);
            LeftY(i) = Final(2);
            i = i + 1;
        
        end
        
        if length(LeftX) > 0
            Pol = polyfit(LeftX, LeftY, 1);
            LeftA = Pol(1);
            LeftB = Pol(2);
        
        else
            LeftA = 1;
            LeftB = 1;
        
        end
        
    end
    
%% Detecting Endpoints with the Equation of a Line
    
    YInitial = ImgSize(1);
    YFinal = PolyMax;
    
    XInitialRight = (YInitial - RightB) / RightA;
    XFinalRight = (YFinal - RightB) / RightA;
    
    XInitialLeft = (YInitial - LeftB) / LeftA;
    XFinalLeft = (YFinal - LeftB) / LeftA;
    
    
%% Plotting of Points

    Point1 = [XInitialLeft, YInitial];                                     % Defining different points using the location
    Point2 = [XFinalLeft, YFinal];
    Point3 = [XFinalRight, YFinal];
    Point4 = [XInitialRight, YInitial];
    
    PointX = [Point1(1) Point2(1) Point3(1) Point4(1)];                    % First Point of all the 4 Points
    PointY = [Point1(2) Point2(2) Point3(2) Point4(2)];                    % Second Point of all the 4 Points
    
    BW = poly2mask(PointX, PointY, 720, 1280);                             % Masking 
    clr = [255 0 0];            
    a = 0.3;                   
    z = false(size(BW));
    
    Mask = cat(3, BW, z, z); F(Mask) = a*clr(1) + (1-a) * F(Mask);
    Mask = cat(3, z, BW, z); F(Mask) = a*clr(2) + (1-a) * F(Mask);
    Mask = cat(3, z, z, BW); F(Mask) = a*clr(3) + (1-a) * F(Mask);
    
    imshow(F)
    hold on
    
%% Plotting of Lines
    
    plot([XInitialLeft, XFinalLeft],[YInitial, YFinal],'LineWidth',5,'Color','green');
    TextString ='Solid Yellow Stripe'
    T = text(150, 600, TextString, 'Color', 'c', 'FontSize', 12);
    
    plot([XInitialRight, XFinalRight],[YInitial, YFinal],'LineWidth',5,'Color','green');
    TextString = 'Dashed White Stripes'
    T = text(1050, 600, TextString, 'Color', 'c', 'FontSize', 12);

%% Predicting Turn
    
if  LeftA > -.70                                                           % Using the Slope of Right Line to Predict the Right Turn Ahead 
    text_str = 'Right Turn Ahead';                                         
    t = text(550, 600, text_str, 'Color', 'k', 'FontSize',20);
    text_str = '   ========>   ';
    t = text(550, 635, text_str, 'Color', 'c', 'FontSize',20);

elseif  LeftA < -.85                                                       % Using the Slope of Left Line to Predict the Left Turn Ahead 
        text_str = 'Left Turn Ahead';
        t = text(550, 600, text_str, 'Color', 'k', 'FontSize',20);
        text_str = '   <========   ';
        t = text(550, 635, text_str, 'Color', 'c', 'FontSize',20);
        
        
else
        text_str = 'Straight Ahead';
        t = text(550, 600, text_str, 'Color', 'k', 'FontSize',20);

end

    Frames = getframe (gca);                                               % Captures Image Frames as a Video
    writeVideo (OutVid, Frames);                                           % Saves the Video in the Origin Folder
    
    pause (0.00001)                                                        % Time Period for Pausing the Frames during their Transition
        
end

    close(OutVid)