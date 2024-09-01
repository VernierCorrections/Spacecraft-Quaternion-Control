t0 = 0;
tf = 30;
yaw_i = 180 * (pi / 180);
pitch_i = 0 * (pi / 180);
roll_i = 45 * (pi / 180);
quaternion_i = EulerToQuaternion(yaw_i, -pitch_i, roll_i);
omega_i = [0; 0; 0];
S0 = [quaternion_i; omega_i];
inertia = [5 0 0; 0 5 0; 0 0 5];
k = 0.5;
G = [20 0 0; 0 20 0; 0 0 20];
epsilon_sliding = [2; 2; 2];
omega_a = 15;
zeta_a = 0.9;
actuator_dynamics = [0 1; -(omega_a)^2 -2*zeta_a*omega_a]; 
actuator_response = [0; (omega_a)^2];
yaw_c = 0 * (pi / 180);
pitch_c = 0 * (pi / 180);
roll_c = 45 * (pi / 180);
DCM_c = transpose(EulerToDCM(yaw_c, -pitch_c, roll_c));
quaternion_c = EulerToQuaternion(yaw_c, -pitch_c, roll_c);
omega_c = [0; 0; 0];
omegadot_c = [0; 0; 0];

[time_matrix, forward_matrix, left_matrix, up_matrix, omega_matrix, torque_matrixc, sliding_matrix, saturated_matrix] = QuaternionODESolver(t0, tf, S0, quaternion_c, omega_c, omegadot_c, inertia, k, G, epsilon_sliding);


subplot(3, 2, 1)
plot(time_matrix, forward_matrix(1, :), 'r', time_matrix, left_matrix(1, :), 'g', time_matrix, up_matrix(1, :), 'b')
title('Attitude Vectors')
subplot(3, 2, 3)
plot(time_matrix, forward_matrix(2, :), 'r', time_matrix, left_matrix(2, :), 'g', time_matrix, up_matrix(2, :), 'b')
subplot(3, 2, 5)
plot(time_matrix, forward_matrix(3, :), 'r', time_matrix, left_matrix(3, :), 'g', time_matrix, up_matrix(3, :), 'b')
subplot(3, 2, 2)
plot(time_matrix, omega_matrix(1, :))
title('Angular Velocities')
subplot(3, 2, 4)
plot(time_matrix, omega_matrix(2, :))
subplot(3, 2, 6)
plot(time_matrix, omega_matrix(3, :))
figure;

subplot(3, 2, 1)
plot(time_matrix, torque_matrixc(1, :), 'r')
title('Torques')
subplot(3, 2, 3)
plot(time_matrix, torque_matrixc(2, :), 'r')
subplot(3, 2, 5)
plot(time_matrix, torque_matrixc(3, :), 'r')
subplot(3, 2, 2)
plot(time_matrix, saturated_matrix(1, :), 'r', time_matrix, sliding_matrix(1, :), 'b')
title('Sliding Mode')
subplot(3, 2, 4)
plot(time_matrix, saturated_matrix(2, :), 'r', time_matrix, sliding_matrix(2, :), 'b')
subplot(3, 2, 6)
plot(time_matrix, saturated_matrix(3, :), 'r', time_matrix, sliding_matrix(3, :), 'b')







