function dSdt = QuaternionLaggyODE(S, torque_c, inertia, actuator_dynamics, actuator_response)
    quaternion = S(1:4);
    omega = S(5:7);
    torque_l = S(8:10);
    torquedot_l = S(11:13);
    S_actuator = vertcat(transpose(torque_l), transpose(torquedot_l));
    quaternion_dot = (1 / 2) * XiFunction(quaternion) * omega;
    angular_acceleration = inertia\torque_l;
    Sdot_actuator_temp = actuator_dynamics * S_actuator + actuator_response * transpose(torque_c);
    Sdot_actuator = vertcat(transpose(Sdot_actuator_temp(1, :)), transpose(Sdot_actuator_temp(2, :)));
    dSdt = [quaternion_dot; angular_acceleration; Sdot_actuator];
end