function [time_matrix, forward_matrix, left_matrix, up_matrix, omega_matrix, torque_matrixc, sliding_matrix, saturated_matrix] = QuaternionODESolver(t0, tf, S0, quaternion_c, omega_c, omegadot_c, inertia, k, G, epsilon_sliding)
    beta = 0.8;
    Fmin = 0.125;
    Fmax = 4.0;
    attemptspertimestep = 12;
    epsilon_relative = 10^(-6);
    epsilon_absolute = 10^(-6);
    min_error = 10^(-18);
    t_guess = tf;
    h_min = 10^(-12);
    stepsizecontrol = [beta; Fmin; Fmax; attemptspertimestep; epsilon_relative; epsilon_absolute; min_error; t_guess; h_min];
    h_initial = 10^(-9);
    t = t0;
    S = S0;
    i = 1;
    h = h_initial;
    preallocate = 10^2;
    time_matrix = [t0 zeros(1, preallocate)];
    DCM = transpose(QuaternionToDCMTranspose(S(1:4)));
    forward_matrix = horzcat(DCM(:, 1), zeros(3, preallocate));
    left_matrix = horzcat(DCM(:, 2), zeros(3, preallocate));
    up_matrix = horzcat(DCM(:, 3), zeros(3, preallocate));
    omega_matrix = horzcat(S(5:7), zeros(3, preallocate));
    torque_matrixc = horzcat([0; 0; 0], zeros(3, preallocate));
    sliding_matrix = horzcat([0; 0; 0], zeros(3, preallocate));
    saturated_matrix = horzcat([0; 0; 0], zeros(3, preallocate));
    while true
        [torque_c, sliding, saturated] = QuaternionTorqueController(quaternion_c, omega_c, omegadot_c, inertia, S(1:4), S(5:7), k, G, epsilon_sliding);
        [t, S, i, h] = QuaternionRKF78(t, S, i, h, stepsizecontrol, torque_c, inertia);
        DCM = transpose(QuaternionToDCMTranspose(S(1:4)));
        time_matrix(1, i) = t;
        forward_matrix(:, i) = DCM(:, 1);
        left_matrix(:, i) = DCM(:, 2);
        up_matrix(:, i) = DCM(:, 3);
        omega_matrix(:, i) = S(5:7);
        torque_matrixc(:, i) = torque_c;
        sliding_matrix(:, i) = sliding ./ epsilon_sliding;
        saturated_matrix(:, i) = saturated;
        if t >= tf
            break
        end
    end
end