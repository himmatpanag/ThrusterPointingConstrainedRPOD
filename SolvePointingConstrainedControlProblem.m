function solution = SolvePointingConstrainedControlProblem(problemParameters, solverParameters)
    % Author: Himmat Panag
    % Solves a single instance of the Optimal Control Problem
    % Not allowed to point at the origin
    % TODO: Turn this into a mex file
    % Date: 4 March 2024

    [lam0,~] = fsolve(@costFunction,solverParameters.initialCostateGuess,...
        solverParameters.fSolveOptions,...
        solverParameters.tSpan, problemParameters.x0, problemParameters.xf,...
        problemParameters.u_max,solverParameters.rho,...
        problemParameters.constraint,...
        problemParameters.dynamics,solverParameters.odeOptions,false);

    solution = emptySolutionStruct(problemParameters,solverParameters);
    solution.newCostateGuess = lam0;
    if solverParameters.getTraj
        optimalTraj = costFunction(solution.newCostateGuess,...            
            solverParameters.tSpan, problemParameters.x0, problemParameters.xf,...
            problemParameters.u_max,solverParameters.rho,...
            problemParameters.constraint,...
            problemParameters.dynamics,solverParameters.odeOptions,true);
        
        X_minU = optimalTraj.X;
        error = optimalTraj.err; 
        solution.solutionFound = abs(error)<solverParameters.stateConvergeneTolerance; 
        solution.finalStateError = error;
        solution.t = optimalTraj.t;
        solution.x = X_minU;
        [S,delta,uDir,phiMin,thetaMin] = GetControlProfile(X_minU,solverParameters.rho,problemParameters.constraint,problemParameters.dynamics);
        solution.switchFunction = S; 
        solution.throttle = delta;
        solution.uDir = uDir;
        solution.theta = thetaMin;
        solution.phi = phiMin;
        solution.rho = solverParameters.rho;
        solution.constraint = problemParameters.constraint;
    end 
end

function [S,delta,uDir,phiMin,thetaMin] = GetControlProfile(X_minU,rho,constraint,dynamics)
    S = zeros(size(X_minU,1),1);
    delta = zeros(size(X_minU,1),1);
    uDir = zeros(size(X_minU,1),3);
    phiMin= zeros(size(X_minU,1),1);
    thetaMin= zeros(size(X_minU,1),1);
    %     uMinUnrotated = zeros(size(X_minU,1),3);    
    if size(X_minU,2) <=12
        lambdaVIdxs =  10:12; 
    else
        lambdaVIdxs =  11:13; 
    end 
    for ii = 1:size(X_minU,1)
        [S(ii),delta(ii),u1,~,~,phiMin(ii),thetaMin(ii)] = ChooseOptimalPointingDirection(X_minU(ii,1:3)',X_minU(ii,lambdaVIdxs)',X_minU(ii,7),X_minU(ii,end),rho,dynamics,constraint);
        uDir(ii,:) = u1';
        %uMinUnrotated(ii,:) = u2';
    end 
end 

function [S,throttle,uDir,uMinUnrotated,a,phiMin,thetaMin] = ChooseOptimalPointingDirection(pos,lambda_v,m,lambda_m,rho,dynamics,constraint)
    uMinUnrotated=[]; a = []; phiMin=nan; thetaMin = nan; 
    
    r1 = pos(1); r2 = pos(2); r3 = pos(3);
    rNorm = norm(pos(1:3));
    r12 = sqrt(r1^2 + r2^2);

    switch constraint.type
        case POINTING_CONSTRAINT_TYPE.NONE
            uDir = -lambda_v/norm(lambda_v); % optimal uDir if no constraint

        otherwise 
            %% Origin pointing constraint            
            if constraint.type == POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE
                if rNorm < constraint.targetRadius
                    alpha = pi/2;
                else
                    alpha = constraint.angleFunc(rNorm);
                end 
            else 
                alpha = constraint.alpha0;
            end 
            
            % Compute a1,a2,a3 (lambdaV^T Psi(r))
            % Construct rotation matrix from x',y',z' frame to x,y,z frame. Use quaternions to avoid singularities.
            % z' axis is aligned with r.
            % to get x,y,z frame: rotate by gamma (angle between z and r) about cross(r,z));
            
            gamma = acos(r3/rNorm); % dot(z,r)/norm(r); z = [0;0;1]
            n = -[r2;-r1;0]/r12; % cross(X(1:3),z)/norm(cross(X(1:3),z));
            q0 = cos(gamma/2);
            quatVec = n.*sin(gamma/2);
            skewSymmetric = @(v) [0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
            RotMatrix = ((q0^2 - dot(quatVec,quatVec)) * eye(3) + 2.*quatVec*quatVec' + 2*q0*skewSymmetric(quatVec));
            a = lambda_v'*RotMatrix;
            
            % Projection Onto Cone Method(a,alpha);
            uFree = -a'/norm(a);
            z=[0;0;1];
            mu = acos(dot(uFree,z));
            if mu > alpha % constraint inactive
                uDir = -lambda_v/norm(lambda_v); % optimal uDir if no constraint
                uMinUnrotated = uFree;
            else
                rotVector = cross(z,uFree);
                rotVector=rotVector/norm(rotVector);
                uMinUnrotated = RodriguesRotation(uFree,alpha-mu,rotVector);
                uDir = RotMatrix*uMinUnrotated;
            end
            
            [~,thetaMin,phiMin] = CartesianToSpherical(uMinUnrotated);


    end 
    S = -1-dynamics.exhaustVelocity/m*dot(lambda_v,uDir)+lambda_m; % Switch function
    switch dynamics.regularizationMethod
        case 'L2_Norm'
            % S = S*dynamics.maxThrust/dynamics.exhaustVelocity;
            throttle = 0.5*(1 + S./sqrt(S^2+rho^2) ); % Throttle
        case 'HyperbolicTangentSmoothing'
            throttle = 0.5*(1+tanh(S./rho)); % Throttle
    end
end 

function g = ComputeDerivativesOfControlTerm(lambda_v,u,rVec)
    r = norm(rVec);
    rSq = r^2; 
    rCubed = rSq*r;
    r1 = rVec(1); r2 = rVec(2); r3 = rVec(3);    
    r12Sq = r1^2 + r2^2;
    r12_4 = r12Sq^2;

    % First term, d/dr(r3/norm(r))
    partialCosGamma = [-r1*r3;-r2*r3;r12Sq]./rCubed; % no n term here
    gFirstTerm = dot(lambda_v,u).*partialCosGamma;

    % Second term
    cg = r3/r; % cosgamma;
    ndotu = r2*u(1) - r1*u(2);
    ndotlambda_v = r2*lambda_v(1) - r1*lambda_v(2);
    gSecondTerm = [-2*r1*(1-cg)*ndotu*ndotlambda_v/r12_4 + ... % ndotu*ndotlamdba cause -ns to cance;
        (1-cg)*(-u(2)*ndotlambda_v -lambda_v(2)*ndotu)/r12Sq + ...
        -partialCosGamma(1)*ndotu*ndotlambda_v/r12Sq;
        -2*r2*(1-cg)*ndotu*ndotlambda_v/r12_4 + ...
        (1-cg)*(u(1)*ndotlambda_v + lambda_v(1)*ndotu)/r12Sq + ...
        -partialCosGamma(2)*ndotu*ndotlambda_v./r12Sq;
        -partialCosGamma(3)*ndotu*ndotlambda_v./r12Sq]; 

    % Third term
    k = cross(u,lambda_v);
    gThirdTerm = -[-(r2*k(1) - r1*k(2))*r1/rCubed - k(2)/r;... % put minus here becaus n is now -n. 
        -(r2*k(1) - r1*k(2))*r2/rCubed + k(1)/r;...
        -(r2*k(1) - r1*k(2))*r3/rCubed];

    g = gFirstTerm + gSecondTerm + gThirdTerm; 

end 

function Xdot = ThreeDimConstrainedOptimalControlProblem(t,X,dynamics,u_max,rho,constraint)
    pos = X(1:3);
    switch dynamics.type
        case 'None' % These cases don't have mass yet!
            lambda = X(7:12);
            lambda_v = X(10:12);            
            %warning('Add mass state to dynamics!')
        case 'Linear'
            lambda = X(8:13);
            lambda_v = X(11:13);
            m = X(7);
            lambda_m=X(14);
    end 
    
    switch constraint.type
        case POINTING_CONSTRAINT_TYPE.NONE
            [~,delta,uDir,~,~,~,~] = ChooseOptimalPointingDirection(pos,lambda_v,m,lambda_m,rho,dynamics,constraint);            
            dudr = [0;0;0];            
        case POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE
            [~,delta,uDir,uMinPrime,~,~,~] = ChooseOptimalPointingDirection(pos,lambda_v,m,lambda_m,rho,dynamics,constraint);
            dudr = ComputeDerivativesOfControlTerm(lambda_v,uMinPrime,pos);
        case POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE
            
            if (norm(pos) < constraint.targetRadius)||(pos'*pos-constraint.targetRadius^2<0)
                constraintAngle = pi/2;
                dalphadr=[0;0;0];
            else
                constraintAngle = constraint.angleFunc(norm(pos)); % Find constraint at current position
                dalphadr = constraint.angleFuncDerivative(pos); % This should be 3x1
            end 
            [~,delta,uDir,uMinPrime,a,phiMin,thetaMin] = ChooseOptimalPointingDirection(pos,lambda_v,m,lambda_m,rho,dynamics,constraint);
            
            sigmaOpt = (pi/2+phiMin)/(pi-constraintAngle); % sigma \in [0,1]
            
            g = ComputeDerivativesOfControlTerm(lambda_v,uMinPrime,X(1:3));
            dudr = g + ...
                (-sigmaOpt) .* dalphadr .* dot(a,[-sin(phiMin)*cos(thetaMin);
                                                  -sin(phiMin)*sin(thetaMin);
                                                         cos(phiMin)      ]);
    end 

    switch dynamics.type
        case 'None' % xDot = Thrust, toy problem
            u = delta*uDir*u_max;
            fr = X(4:6);
            fv = zeros(3,1);

            dfdx = zeros(6,6);
            dfdx(1:3,4:6) = eye(3);

            Xdot = [fr; fv+u; -dfdx'*lambda + [zeros(3,1); -delta*u_max*dudr]]; 

        case 'Linear' % xDot = Ax + B*Thrust, B = [0x3,Ix3];
            f = dynamics.A*X(1:6);
            mDot = -dynamics.maxThrust/dynamics.exhaustVelocity*delta;
            u = delta*uDir*dynamics.maxThrust./m;
            lambda_mdot = delta*dynamics.maxThrust.*dot(lambda_v,uDir)/(m^2);   
            Xdot = [f(1:3); f(4:6) + u; mDot; -dynamics.A'*lambda + [-delta*dynamics.maxThrust*dudr/m; zeros(3,1)];lambda_mdot];

    end 
end 

function trajOut = costFunction(lam0_guess,tspan, x0, xf,u_max,rho,constraint,dynamics,opts_ode,getSol)
    [t,X] = ode45(@ThreeDimConstrainedOptimalControlProblem,...
        tspan,[x0; lam0_guess],opts_ode,dynamics,u_max,rho,constraint);

    err = [X(end,1:6)-xf',X(end,14)]; 

    if getSol 
        trajOut.err = err; 
        trajOut.t = t;
        trajOut.X = X;
    else
        trajOut = err; 
    end 
end

% function v = uSpherical(phi,theta) 
%     v = [cos(phi)*cos(theta);cos(phi)*sin(theta);sin(phi)]; 
% end 
function solStruct = emptySolutionStruct(problemParameters,solverParameters)
    solStruct = struct();
    solStruct.solutionFound = false; 
    solStruct.t = [];
    solStruct.x = [];
    solStruct.uDir = [];
    solStruct.throttle = [];
    solStruct.switchFunction = [];
    solStruct.problemParameters = problemParameters;
    solStruct.solverParameters = solverParameters;
end 