function offset = com_off(m1, pos1, m2, pos2)
    % This function computes the offset from pos1 of the CoM of m1 and m2
    if (m1+m2 ~= 0)
        offset = (m1*pos1 + m2*pos2)/(m1+m2);
    else 
        offset = zeros(3, 1);
    end
end