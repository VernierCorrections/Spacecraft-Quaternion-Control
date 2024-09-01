function quaternion = EulerToQuaternion(yaw, pitch, roll)
    q1 = cos(yaw / 2) * cos(pitch / 2) * sin(roll / 2) - sin(yaw / 2) * sin(pitch / 2) * cos(roll / 2);
    q2 = cos(yaw / 2) * sin(pitch / 2) * cos(roll / 2) + sin(yaw / 2) * cos(pitch / 2) * sin(roll / 2);
    q3 = sin(yaw / 2) * cos(pitch / 2) * cos(roll / 2) - cos(yaw / 2) * sin(pitch / 2) * sin(roll / 2);
    q4 = cos(yaw / 2) * cos(pitch / 2) * cos(roll / 2) + sin(yaw / 2) * sin(pitch / 2) * sin(roll / 2);
    quaternion = [q1; q2; q3; q4];
    quaternion = quaternion / norm(quaternion);
end