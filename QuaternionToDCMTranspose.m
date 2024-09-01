% calculates the direction cosine matrix, or attitude matrix, from a quaternion
function DCM = QuaternionToDCMTranspose(q)
    DCM = ((q(4))^2 - (norm(q(1:3)))^2) * eye(3) - 2 * q(4) * CrossProductMatrix(q(1:3)) + 2 * q(1:3) * transpose(q(1:3));
end

