function CVX_Params = WriteOCP_ParamsToCVX(CVX_Params,problemParameters)

    CVX_Params.rInit = problemParameters.x0(1:3);
    CVX_Params.rFinal = problemParameters.xf(1:3);
    CVX_Params.Omega = problemParameters.dynamics.A(4,5)/2;
    CVX_Params.simTimeHours = problemParameters.transferTime/3600;
    CVX_Params.aMax = problemParameters.u_max;
    CVX_Params.vInit = problemParameters.x0(4:6);
    CVX_Params.vFinal = problemParameters.xf(4:6);

    CVX_Params.coneConstraintActive = false; 
    
    CVX_Params.coneAngle = problemParameters.constraint.alpha0;

end 
