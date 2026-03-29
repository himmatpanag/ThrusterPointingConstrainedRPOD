function ppOut = ReduceThrusterMomentArm(problemParameters,factor)
    ppOut = problemParameters;
    
    for ii = 1:ppOut.dynamics.numEngines 
        d = ppOut.dynamics.engineLocationBody(:,ii); 
        u = ppOut.dynamics.thrustDirectionBody(:,ii); 
    end 
end 