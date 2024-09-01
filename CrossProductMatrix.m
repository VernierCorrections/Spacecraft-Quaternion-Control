% calculates the cross-product equivalent matrix of a three-vector
% where the cross-product equivalent matrix is the matrix which performs the equivalent transformation as taking a cross product
function CrossProductEquivalent = CrossProductMatrix(vector)
    CrossProductEquivalent = [0 -vector(3) vector(2); vector(3) 0 -vector(1); -vector(2) vector(1) 0;];
end
