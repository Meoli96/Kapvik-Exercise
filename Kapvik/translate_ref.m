function [q_t] = translate_ref(q, ref1)
    % This function translates q in the nominal reference frame
    % ref0 = [0,0,0,0] (deployed arm along x) into ref1 (written in ref0),
    % where ref1 is assumed to be the new [0,0,0,0]
    
    assert(length(q) == length(ref1), "q and ref1 should have the same dimension");
    q_t = q - ref1;
end

