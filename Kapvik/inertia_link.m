function I = inertia_link(m, l, R, r)
    % This function computes the inertia matrix of a link modeled as a
    % hollow cylinder of length l, mass m, outer and inner radius R, r.
    
    % !!! The inertia matrix is computed as central principal, 
    % where X along the link, Y and Z perpendicular to the link.

    Ix = m*(R^2 + r^2)/2;
    Iyz = m*(R^2 + r^2)/4 + m*l^2/12;
    I = [Ix, 0, 0; 0,Iyz, 0; 0,0, Iyz];
end
