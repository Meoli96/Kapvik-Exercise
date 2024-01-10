function rotated_axis = rot_axis(alpha)
    % This function rotates the reference axis (Z) around the rotation axis (X) by
    % the given angle alpha (in radians).

    % Reference axis
    reference_axis = [0, 0, 1];
    
    % Rotation axis (perpendicular to reference axis)
    rotation_axis = [1, 0, 0];
    
    % Normalize the rotation axis
    rotation_axis = rotation_axis / norm(rotation_axis);
    
    % Construct the rotation matrix
    rotation_matrix = vrrotvec2mat([rotation_axis, alpha]);
    
    % Rotate the reference axis
    rotated_axis = (rotation_matrix * reference_axis')';
end