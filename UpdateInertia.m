function [problemParameters, inertiaNewFinal] = UpdateInertia(problemParameters,multiplier)
    originalInertia = problemParameters.dynamics.inertia(2,2);
    initialI = originalInertia*multiplier;
    problemParameters.dynamics.inertia = problemParameters.dynamics.inertia*multiplier;
    problemParameters.dynamics.inertiaInverse = problemParameters.dynamics.inertiaInverse/multiplier;
    inertiaNewFinal = [initialI,originalInertia];

end