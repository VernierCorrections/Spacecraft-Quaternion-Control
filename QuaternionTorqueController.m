function [torque_c, sliding, saturated] = QuaternionTorqueController(quaternion_c, omega_c, omegadot_c, inertia, quaternion, omega, k, G, epsilon_sliding)
    % find current error in the imaginary part of the quaternion
    delta_imaginary = transpose(XiFunction(quaternion_c)) * quaternion;
    % find current error in the real part of the quaternion
    delta_real = transpose(quaternion) * quaternion_c;
    % find sliding-mode surface vector
    sliding = (omega - omega_c) + k * sign(delta_real) * delta_imaginary;
    saturated = saturate(sliding, epsilon_sliding);
    torque_c = inertia * ((k / 2) * (abs(delta_real) * (omega_c - omega) - sign(delta_real) * cross(delta_imaginary, (omega + omega_c))) + omegadot_c - G * saturated) + CrossProductMatrix(omega) * inertia * omega;
end

function saturated_output = saturate(saturated_input, epsilon_sliding)
    saturated_output = saturated_input;
    for i = 1:3
        if saturated_input(i) > epsilon_sliding(i)
            saturated_output(i) = 1;
        elseif saturated_input(i) < -epsilon_sliding(i)
            saturated_output(i) = -1;
        else
            saturated_output(i) = saturated_input(i) / epsilon_sliding (i);
        end
    end
end


% convert commanded quaternion to DCM form to calculate the current DCM error
% DCM = QuaternionToDCMTranspose(quaternion);
% DCM_c = QuaternionToDCMTranspose(quaternion_c);
% delta_DCM = DCM * transpose(DCM_c);
% find current angular rate error
% delta_omega = omega - delta_DCM * omega_c;
% find current quaternion error (we only need part of the result)
% delta_imaginary = transpose(XiFunction(quaternion_c)) * quaternion;
% calculate torque command
% torque_c = CrossProductMatrix(delta_DCM * omega_c) * inertia * delta_DCM * omega_c + inertia * delta_DCM * omegadot_c - k_p * delta_imaginary - k_d * delta_omega; 