function Icm = inertia_off(Ia, m, Pc)
    % This function computes the Inertia matrix of a body in its CoM given its
    % Inertia matrix about a point A, its mass and the position of the
    % center of mass of the body wrt to A.
    Icm  = Ia - m*(Pc'*Pc*eye(3) - Pc*Pc');
end