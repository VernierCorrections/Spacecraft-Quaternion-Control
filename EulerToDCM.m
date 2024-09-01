function DCM = EulerToDCM(yaw, pitch, roll)
    roll_matrix = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
    pitch_matrix = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
    yaw_matrix = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
    DCM = roll_matrix * (pitch_matrix * (yaw_matrix * [1 0 0; 0 1 0; 0 0 1]));
end
