function [T, NPT] = Normalization(points)

% The Normalization of Points has been carried out by finding the Centroid
% of all the points and shifting it to origin. Next, their mean is to be
% found out which should be sqrt(2). Hence, their values are scaled accordingly.

MP = mean(points);
YMP = MP(1,2);
XMP = MP(1,1);

    RP = repmat(MP, size(points, 1), 1);
    NP = points - RP;
    
    temp = sqrt((mean(sum((NP.^2),2))));
    scale = sqrt(2)/temp;
    
    C1 = [scale   0  0
           0   scale 0
           0     0   1];
       
    C2 = [1 0 -XMP
          0 1 -YMP
          0 0   1];
    
    T = C1 * C2;
    
    SZ = size(points);
    NPT = zeros(SZ);
    
    for i = 1: size(points, 1)
        temp2 = points(i, :);
        XY = T * [temp2(1, 1); temp2(1, 2); 1 ];

        NPT(i, :) = [XY(1) XY(2)];
   
    end
     
    NPT = NPT;
    
end