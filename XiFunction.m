% calculates the leading three columns of the Xi matrix
% where the Xi matrix performs the equivalent transformation as taking the quaternion outer product
function XiMatrix = XiFunction(quaternion)
    XiMatrix = vertcat((quaternion(4) * eye(3) + CrossProductMatrix(quaternion(1:3))), -(transpose(quaternion(1:3))));
end