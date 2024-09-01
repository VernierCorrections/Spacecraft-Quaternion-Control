function dSdt = QuaternionODE(S, torque_c, inertia)
    quaternion = S(1:4);
    omega = S(5:7);
    quaternion_dot = (1 / 2) * XiFunction(quaternion) * omega;
    angular_acceleration = inertia\torque_c;
    dSdt = [quaternion_dot; angular_acceleration];
end