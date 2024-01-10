function R = vrrotvec2mat(rotation_vector)
    % rotation_vector: 4-element vector [axis_x, axis_y, axis_z, angle]

    % Normalize the rotation axis
    axis_vector = rotation_vector(1:3);
    axis_vector = axis_vector / norm(axis_vector);

    % Extract the angle
    angle = rotation_vector(4);

    % Rodrigues' rotation formula
    K = skew(axis_vector);
    R = eye(3) + sin(angle) * K + (1 - cos(angle)) * (K^2);
end

function K = skew(v)
    % Skew-symmetric matrix for a 3D vector
    K = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end