function [Fnorm] = FundamentalMatrix(XI, XF)

% The Fundamental Matrix is found out using the input values from XI and
% XF. Matrix A is plotted thereafter and svd is carried out to split it
% into 3 parts. At last, the matrix is denormalized to get the actual
% fundamental matrix.

for i = 1:size(XI,1)
        
    X1 = XI(i, 1);
    Y1 = XI(i, 2);
    
    X2 = XF(i, 1);
    Y2 = XF(i, 2);
    
    A(i, :) = [X2*X1 X2*Y1 X2 Y2*X1 Y2*Y1 Y2 X1 Y1 1]; 

end

[U, S, V] = svd(A);
TP = reshape(V(:,end), [3 3])';

[U, S, V] = svd(TP);

S(3,3) = 0;
S2 = S;

Fnorm = U * S2 * V';

end