function Ij = inertia_joint(m, d)
    % This function computes the inertia matrix for a point mass at
    % distance d from the com of the link and distance r from the rotation
    % axis --- we assume we are along the principal axis
    Ij = eye(3,3).*(m*d^2); 
end