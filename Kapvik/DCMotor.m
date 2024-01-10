classdef DCMotor 


    properties
    % Motor parameters
    tau_max
    i_max
    Jm
    b
    thetad_max
    end

    methods
        function obj = DCMotor(tau_max, i_max, Jm, b, thetad_max)
            obj.tau_max = tau_max;
            obj.i_max = i_max;
            obj.Jm = Jm;
            obj.b = b;
            obj.thetad_max = thetad_max;
        end

        function [qdd_max] = get_qdd_max(obj)
            qdd_max = obj.tau_max / obj.Jm; % !!! Here im assuming the motor is the only source of inertia,
                                            % so we are at high gear ratio
        end
        
    end
end