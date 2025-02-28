function solution = OLD_Solve6DOFPointingConstrainedControlProblem(problemParameters, solverParameters, skipSol)
    % Author: Himmat Panag
    % Solves a single instance of the 6DOF Optimal Control Problem
    % Not allowed to point at the origin
    % TODO: Turn this into a mex file
    % Date: 18 Oct 2024
    if nargin < 3, skipSol = false; end
    if isempty(gcp), parpool(8); end

    [initialGuess,remainingCostates] = ChooseCostatesToIgnore( ...
        problemParameters.dynamics,solverParameters.initialCostateGuess);
    if skipSol % We may want to get experimental with guesses of lam0, This spits out the error with a particular lam0, without doing any solving
        lam0=initialGuess; exitFlag = -1;
    else
        [lam0,~,exitFlag] = fsolve(@costFunction,initialGuess,...
            solverParameters.fSolveOptions,...
            solverParameters.tSpan, problemParameters.x0, problemParameters.xf,...
            problemParameters.p0,problemParameters.pf,...
            problemParameters.w0,problemParameters.wf,...
            solverParameters.rho,...
            problemParameters.constraint,...
            problemParameters.dynamics,remainingCostates, ...ß
            solverParameters.odeOptions,false);
    end
    solution = emptySolutionStruct(problemParameters,solverParameters);
    solution.newCostateGuess = CombineIgnoredCostates(problemParameters.dynamics, lam0, remainingCostates); 
    if solverParameters.getTraj
        [initialGuess,remainingCostates] = ChooseCostatesToIgnore( ...
            problemParameters.dynamics, solution.newCostateGuess);
        optimalTraj = costFunction(initialGuess,...            
            solverParameters.tSpan, problemParameters.x0, problemParameters.xf,...
            problemParameters.p0,problemParameters.pf,...
            problemParameters.w0,problemParameters.wf,...
            solverParameters.rho,...
            problemParameters.constraint,...
            problemParameters.dynamics,remainingCostates, ...
            solverParameters.odeOptions,true);
        
        X_minU = optimalTraj.X;
        error = optimalTraj.err; 
        solution.solutionFound = exitFlag>0;
        solution.finalStateError = error;
        solution.t = optimalTraj.t;
        solution.x = X_minU;
        solution.throttle = zeros(problemParameters.dynamics.numEngines,numel(solution.t));
        solution.eta = zeros(problemParameters.dynamics.numEngines,numel(solution.t));
        solution.switchFunction = zeros(problemParameters.dynamics.numEngines,numel(solution.t));
        solution.torqueInertialFrame = zeros(3,numel(solution.t),1);
        solution.thrustTranslationalFrame = zeros(3,numel(solution.t),1);
        for ii = 1:numel(solution.t)
            optimalSolStruct = SixDimConstrainedOptimalControlProblem(solution.t(ii),...
                X_minU(ii,:)',problemParameters.dynamics,solverParameters.rho,...
                problemParameters.constraint,false);
            solution.throttle(:,ii) = optimalSolStruct.throttles;
            solution.eta(:,ii) = optimalSolStruct.etas;
            solution.switchFunction(:,ii) = optimalSolStruct.switchFunctions;
            solution.torqueInertialFrame(:,ii) = optimalSolStruct.totalTorqueInertialFrame;
            solution.thrustTranslationalFrame(:,ii) = optimalSolStruct.totalThrustTranslationalFrame;
        end       
    end 
end
function [initialGuess,remainingCostates] = ChooseCostatesToIgnore(dynamics, costateGuess)
    if (dynamics.engineConfiguration==THRUSTER_CONFIGURATION.CG_ALIGNED_6 ||dynamics.engineConfiguration==THRUSTER_CONFIGURATION.CG_ALIGNED_CANTED_8) && (dynamics.attitudeActuator==ATTITUDE_CONTROL_TYPE.NONE)
                % In this case the attitude 
                % dynamics is not controllable. The solver tries to vary the p
                % and omega costates but they have no affect on the cost
                % function so it is unable to converge. We limit the fsolve
                % parameters to only the first 7 of the initialCostateGuess and
                % the solver easily converges to the optimal solution
                initialGuess = costateGuess(1:7);
                remainingCostates =  costateGuess(8:end);
    else 
                initialGuess = costateGuess;
                remainingCostates = [];
    end
end
function lam0 = CombineIgnoredCostates(dynamics, costateGuess, costatesToIgnore)
    if (dynamics.engineConfiguration==THRUSTER_CONFIGURATION.CG_ALIGNED_6 || dynamics.engineConfiguration==THRUSTER_CONFIGURATION.CG_ALIGNED_CANTED_8) && (dynamics.attitudeActuator==ATTITUDE_CONTROL_TYPE.NONE)
        lam0 = [costateGuess;costatesToIgnore];
    else
        lam0 = costateGuess;
    end
end

function Xdot = SixDimConstrainedOptimalControlProblem(t,X,dynamics,rho,constraint,returnDynamics)
if nargin < 6
    returnDynamics = true;
end
    if ~isreal(X)||any(isnan(X))
        stop = true;
    end
    % states 1-6 are pos vel
    % state 7 is mass
    % states 8-13 are MRP and angular velocity
    % states 14-26 are corresponding costates. (14-19 pos vel, 20 mass,
    % 21-26 angular)
    r = X(1:3);
    v = X(4:6);
    m = X(7);
    p = X(8:10);
    w = X(11:13);
    lambda_r = X(14:16);
    lambda_v = X(17:19);
    lambda_m = X(20);
    lambda_p = X(21:23);
    lambda_w = X(24:26);

    p2 = p'*p;
    pCross = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    temp = (1+p2)^2;
    D_Temp = lambda_w'*dynamics.inertiaInverse; % cross product is bilinear so product rule distributes

    % Translational dynamics are in a rotating reference frame
    DCM_BodyToInertial = eye(3) + (8*pCross*pCross - 4*(1-p2)*pCross)/temp;
    DCM_InertialToLVLH = [cos(dynamics.frameRotationRate*t) sin(dynamics.frameRotationRate*t) 0;
                          -sin(dynamics.frameRotationRate*t) cos(dynamics.frameRotationRate*t) 0;
                          0 0 1];
    DCM_BodyToTranslationalFrame = DCM_InertialToLVLH*DCM_BodyToInertial;

    eta = ones(dynamics.numEngines,1); 
    constraintAngle = zeros(dynamics.numEngines,1);
    dalphadr=zeros(3,dynamics.numEngines,1); 
    % Compute the constraint function, eta
    if constraint.type ~= POINTING_CONSTRAINT_TYPE.NONE
        etaPrime = zeros(dynamics.numEngines,1);
        dirEngineRelTargetBodyFrame = zeros(3,dynamics.numEngines);
        distEngineToTarget = zeros(1,dynamics.numEngines);

        for ii = 1:dynamics.numEngines
            if constraint.simplified
                posEngineLVLH = r; 
            else
                posEngineLVLH = DCM_BodyToTranslationalFrame*dynamics.engineLocationBody(:,ii) + r;
            end 
            distEngineToTarget(ii) = norm(posEngineLVLH);
            if (distEngineToTarget(ii) < constraint.targetRadius+eps(constraint.targetRadius))
                constraintAngle(ii) = pi/2;      
                dalphadr(:,ii) = sign(posEngineLVLH)*1e3;
            else
                constraintAngle(ii) = constraint.angleFunc(distEngineToTarget(ii));      
                dalphadr(:,ii) = constraint.angleFuncDerivative(posEngineLVLH);
            end

            posEngineRelTargetBodyFrame  = DCM_BodyToTranslationalFrame'*posEngineLVLH;
            dirEngineRelTargetBodyFrame(:,ii)  = posEngineRelTargetBodyFrame./distEngineToTarget(ii);

            constraintFunc = 1/constraint.targetRadius *(distEngineToTarget(ii)*cos(constraintAngle(ii)) - ...
                dot(dynamics.thrustDirectionBody(:,ii),posEngineRelTargetBodyFrame));
                
            eta(ii) = 1/2 * (1 + tanh(constraintFunc/constraint.epsilon));
            etaPrime(ii) = 1/(2*constraint.epsilon)*(1-(tanh(constraintFunc/constraint.epsilon))^2);
            % If the constraint function is negative, the constraint should be INACTIVE and eta = 1 
        end 
    end

    % Compute the switch functions and throttles
    SwitchFunctions = zeros(dynamics.numEngines,1);
    deltaI = zeros(dynamics.numEngines,1);
    totalTorque = zeros(3,1); totalThrustLVLH = zeros(3,1); mDot = 0;
    for ii = 1:dynamics.numEngines
        r_cross_v = cross(dynamics.engineLocationBody(:,ii),dynamics.thrustDirectionBody(:,ii));
        % Compute the switch functions
        SwitchFunctions(ii) = (-1 + eta(ii) * (lambda_m - dynamics.exhaustVelocity/m * lambda_v'*DCM_BodyToTranslationalFrame*dynamics.thrustDirectionBody(:,ii) + ...
            - dynamics.exhaustVelocity*dynamics.initialMass/m*D_Temp*r_cross_v));
        % Compute the controls
        deltaI(ii) = 1/2 * (1+tanh(SwitchFunctions(ii)/rho));

        mDot = mDot - dynamics.maxThrust(ii)/dynamics.exhaustVelocity*deltaI(ii)*eta(ii);
        % mDot = mDot - dynamics.exhaustVelocity*deltaI*eta(ii);
        totalThrustLVLH = totalThrustLVLH + deltaI(ii)*eta(ii)*dynamics.maxThrust(ii)*DCM_BodyToTranslationalFrame*dynamics.thrustDirectionBody(:,ii);
        totalTorque = totalTorque + dynamics.maxThrust(ii)*1e3*deltaI(ii)*eta(ii)*r_cross_v;

        if ~isreal(deltaI)
            stop = true;
        end
    end

    optimalControlTorque = [0;0;0];
    switch dynamics.attitudeActuator
        case ATTITUDE_CONTROL_TYPE.CONTROL_TORQUE
            optimalControlTorque = (-dynamics.inertiaInverse'*lambda_w)./dynamics.torqueCostMultiplier;
            % for ii = 1:3
            %     optimalControlTorque(ii) = max(min(optimalControlTorque(ii),dynamics.maxControlTorque),-dynamics.maxControlTorque);
            % end
    end

    % Compute the state and costate dynamics
    switch dynamics.type
        case 'Linear' % xDot = Ax + B*Thrust, B = [0x3,Ix3];
            fDynamics = dynamics.A*X(1:6);
            lambda_rvDot = -dynamics.A'*[lambda_r; lambda_v];
    end
    lambda_rDot = lambda_rvDot(1:3);  
    rDot = fDynamics(1:3);
    vDot = fDynamics(4:6) + totalThrustLVLH/m;
    B_Temp = 0.25 * ((1 + p2) .* eye(3) + 2 * (pCross*pCross + pCross));
    % B_Temp = 0.25 * ((1 + p2) .* eye(3) + (pCross*pCross + pCross)); %
    % OLD ONE! Error in pCross term?
    pDot = B_Temp * w; % Testing pdot %pDot2 = .5 * (.5*(1-p2)*eye(3) + pCross + p*p')*w
    omegaDot = dynamics.inertiaInverse*(dynamics.initialMass/m * (totalTorque+optimalControlTorque) - cross(w,dynamics.inertia*w));

    % Derivative of the MRP dynamics term in the Hamiltonian
    lp1Dot = -.5*lambda_p'*[p(1), p(2), p(3); p(2), -p(1), -1; p(3), 1, -p(1)]*w;
    lp2Dot = -.5*lambda_p'*[-p(2), p(1), 1; p(1), p(2), p(3); -1, p(3), -p(2)]*w;
    lp3Dot = -.5*lambda_p'*[-p(3), -1, p(1); 1, -p(3), p(2); p(1), p(2), p(3)]*w;
    % symbolicDerivatives.lambda_pDotfp(w,p,lambda_p). < This won't match anymore

    % Derivatives of the translational control term in the Hamiltonian (contains p)
    [D_p1,D_p2,D_p3] = DerivativeDCM_BodyToInertialWRT_p(p(1),p(2),p(3));
    C1_Temp = lambda_v'*DCM_InertialToLVLH * D_p1/m;
    C2_Temp = lambda_v'*DCM_InertialToLVLH * D_p2/m;
    C3_Temp = lambda_v'*DCM_InertialToLVLH * D_p3/m;
    
    F1 = D_p1'*DCM_InertialToLVLH'*r;
    F2 = D_p2'*DCM_InertialToLVLH'*r;
    F3 = D_p3'*DCM_InertialToLVLH'*r;
    for ii = 1:dynamics.numEngines
        lp1Dot = lp1Dot - deltaI(ii)*eta(ii)*C1_Temp*dynamics.maxThrust(ii)*dynamics.thrustDirectionBody(:,ii);
        lp2Dot = lp2Dot - deltaI(ii)*eta(ii)*C2_Temp*dynamics.maxThrust(ii)*dynamics.thrustDirectionBody(:,ii);
        lp3Dot = lp3Dot - deltaI(ii)*eta(ii)*C3_Temp*dynamics.maxThrust(ii)*dynamics.thrustDirectionBody(:,ii);
        % -symbolicDerivatives.lambda_pDotControl(p,lambda_v'*DCM_InertialToLVLH, deltaI(ii)*eta(ii)*dynamics.maxThrust(ii)/m, dynamics.thrustDirectionBody(:,ii));
    end  
    
    % Compute derivatives of cosntraint terms here!
    if constraint.type ~= POINTING_CONSTRAINT_TYPE.NONE
        F_Temp = lambda_v'*DCM_BodyToTranslationalFrame/m;
        K_Temp = dynamics.initialMass/m * D_Temp;
        M_Temp = lambda_m/dynamics.exhaustVelocity;
        for ii = 1:dynamics.numEngines
            % eta is a function of p and r. deta/dr and deta/dp both have
            % the following constant coefficient (Gi)
            G_i = deltaI(ii)*dynamics.maxThrust(ii) * ( ...
                    F_Temp*dynamics.thrustDirectionBody(:,ii) + ... % This is the translational term
                    K_Temp*cross(dynamics.engineLocationBody(:,ii),dynamics.thrustDirectionBody(:,ii)) + ... % this is the rotational term
                    -M_Temp); % this is the mass term
            G_i_Temp = etaPrime(ii)/constraint.targetRadius*G_i;
            uivi = dynamics.thrustDirectionBody(:,ii)'*dirEngineRelTargetBodyFrame(:,ii);
            lp1Dot = lp1Dot - G_i_Temp*(F1'*dirEngineRelTargetBodyFrame(:,ii)*cos(constraintAngle(ii)) ...
                - F1'*dynamics.thrustDirectionBody(:,ii) ...
                - distEngineToTarget(ii)*sin(constraintAngle(ii))*dalphadr(:,ii)'*D_p1*dynamics.thrustDirectionBody(:,ii));
            lp2Dot = lp1Dot - G_i_Temp*(F2'*dirEngineRelTargetBodyFrame(:,ii)*cos(constraintAngle(ii)) ...
                - F2'*dynamics.thrustDirectionBody(:,ii) ...
                - distEngineToTarget(ii)*sin(constraintAngle(ii))*dalphadr(:,ii)'*D_p2*dynamics.thrustDirectionBody(:,ii));
            lp3Dot = lp1Dot - G_i_Temp*(F3'*dirEngineRelTargetBodyFrame(:,ii)*cos(constraintAngle(ii)) ...
                - F3'*dynamics.thrustDirectionBody(:,ii)...
                - distEngineToTarget(ii)*sin(constraintAngle(ii))*dalphadr(:,ii)'*D_p3*dynamics.thrustDirectionBody(:,ii));

            lambda_rDot = lambda_rDot - G_i_Temp * ...
                (DCM_BodyToTranslationalFrame*dirEngineRelTargetBodyFrame(:,ii)*cos(constraintAngle(ii)) ...
                - distEngineToTarget(ii)*sin(constraintAngle(ii))*dalphadr(:,ii) ...
                - DCM_BodyToTranslationalFrame * dynamics.thrustDirectionBody(:,ii));
            if ~isreal(lambda_rDot)||any(isnan(lambda_rDot))
                stop = true;
            end
        end
    end
    lambda_pDot = [lp1Dot; lp2Dot; lp3Dot];
    
    % Derivatives of the terms containing w in the Hamiltonian
    fw_w1Dot = D_Temp*(cross([1;0;0],dynamics.inertia*w) + cross(w,dynamics.inertia(1:3,1)));
    fw_w2Dot = D_Temp*(cross([0;1;0],dynamics.inertia*w) + cross(w,dynamics.inertia(1:3,2)));
    fw_w3Dot = D_Temp*(cross([0;0;1],dynamics.inertia*w) + cross(w,dynamics.inertia(1:3,3)));

    lambda_wDot = -B_Temp'*lambda_p + ... % derivative of the attitude dynamics term (contains w)
        [fw_w1Dot; fw_w2Dot; fw_w3Dot]; % derivative of the rotational dynamics terms (-w X (Iw))
    lambda_mDot = lambda_v'*totalThrustLVLH/(m^2) + lambda_w'*dynamics.inertiaInverse*totalTorque*dynamics.initialMass/(m^2);
    
    % symbolicDerivatives.lambda_wDot(w,p,lambda_w,lambda_p, dynamics.inertia,dynamics.inertiaInverse); symbolicDerivatives.lambda_wDot2(w,p,lambda_w,lambda_p, dynamics.inertia,dynamics.inertiaInverse)
    if returnDynamics
        Xdot = [rDot; vDot; mDot; pDot; omegaDot;...
            lambda_rDot; lambda_rvDot(4:6); lambda_mDot; lambda_pDot; lambda_wDot];
        % -dynamics.A'*lambda + [-delta*dynamics.maxThrust*dudr/m; zeros(3,1)];lambda_mdot];
        if ~isreal(Xdot)||any(isnan(Xdot))%||any(lambda_rDot>20)
            stop = true;
        end
    else % return the switch functions, constraints, throttles, etc.
        Xdot.throttles = deltaI;
        Xdot.etas = eta;
        Xdot.switchFunctions = SwitchFunctions;
        Xdot.totalTorqueInertialFrame = totalTorque+optimalControlTorque;
        Xdot.totalThrustTranslationalFrame = totalThrustLVLH;
    end
     
end 

function trajOut = costFunction(lam0_guess,tspan, x0, xf, p0, pf, ...
    w0, wf,rho,constraint,dynamics,costatesToIgnore,opts_ode,getSol)
    
    lambda_0 = CombineIgnoredCostates(dynamics, lam0_guess, costatesToIgnore); 
    useShadowSet = true;
    if useShadowSet
        maxSetChanges = 100; tOut = 0; ii = 0;
        opts_ode = odeset(opts_ode, Event=@MRP_InShadowSet);
        TSPAN = tspan; X0 = [x0; p0; w0;lambda_0];
        t = []; X = [];
        while (ii<maxSetChanges) && (tOut(end) < tspan(end))
            % Solve one arc
            [tOut,XOut] = ode45(@SixDimConstrainedOptimalControlProblem,...
                TSPAN,X0,opts_ode,dynamics,rho,constraint);
            
            % Combine trajectory arcs
            t = [t; tOut]; 
            X = [X; XOut];

            % Prepare ODE for next arc
            TSPAN = [tOut(end),tspan(end)];
            X0 = XOut(end,:); 
            normFinalMRP = norm(X0(8:10));
            if normFinalMRP > .2
                X0(8:10) = -X0(8:10)./(normFinalMRP^2); % Switch from shadow set to main set 
            end 
        end
        if ii>=maxSetChanges
            warning('Max number of shadow set switches');
        end
    else
        [t,X] = ode45(@SixDimConstrainedOptimalControlProblem,...
            tspan,[x0; p0; w0;lambda_0],opts_ode,dynamics,rho,constraint);
    end
    % states 1-6 are pos vel
    % state 7 is mass
    % states 8-10 are MRP and 11-13 are angular velocity
    % states 14-26 are corresponding costates. (14-19 pos vel, 20 mass,
    % 21-26 angular)
    % Compute error from shadow set and normal set
    normOriginal = norm(X(end,8:10)-pf');
    MRP_Error = X(end,8:10)-pf';
    if normOriginal > 0.2 % stop switching errors
        shadowX_Final = -X(end,8:10)/(norm(X(end,8:10))^2);
        if normOriginal > norm(shadowX_Final-pf')
            MRP_Error = shadowX_Final-pf';
        end
    end 

    % Scale errors. pos, vel are in km, km/sec. So 1e-6 corresponds to
    % accuracy of 1mm. Want angVel to have accuracy of .1mm/sec (1e-7)
    % MRP = n*tan(theta/4). Angular error of 1degree is 0.0044
    % ang vel is in rad/s. ang rate error of .05deg/s is 8.7266e-04
    velScale = 10;
    MRP_Scale = 1e-6/.0044;
    omega_Scale = 1e-6/(8.7266e-04);
    if dynamics.finalAttitudeFree
        MRP_Error = X(end,21:23); % costates = 0 
        MRP_Scale = 1;
    end
    scale = 100.*[1,1,1,velScale,velScale,velScale,MRP_Scale,MRP_Scale,MRP_Scale,...
        omega_Scale,omega_Scale,omega_Scale,1];
    errorUnscaled = [X(end,1:6)-xf',MRP_Error,X(end,11:13)-wf',X(end,20)]; 
    err = errorUnscaled.*scale;

    if getSol 
        trajOut.err = err; 
        trajOut.t = t;
        trajOut.X = X;
    else
        trajOut = err; 
    end 
end

function [value,isterminal,direction] = MRP_InShadowSet(~,X,dynamics,rho,constraint)
    value = norm(X(8:10))-1-0.5; % norm goes over 1.5
    isterminal = 1;
    direction = 1;
end

function solStruct = emptySolutionStruct(problemParameters,solverParameters)
    solStruct = struct();
    solStruct.solutionFound = false; 
    solStruct.t = [];
    solStruct.x = [];
    solStruct.problemParameters = problemParameters;
    solStruct.solverParameters = solverParameters;
end 

function [D_p1,D_p2,D_p3] = DerivativeDCM_BodyToInertialWRT_p(p1,p2,p3)
    D_p1 = [[                                                                                                       (32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3,                        (8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3,                        (8*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1*p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3]
            [(8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3,                                                                                             (32*p1^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3, (4*p1^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1^2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3]
            [(8*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3, (8*p1^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p1^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3,                                                                                             (32*p1^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3] 
            ];
    D_p2 =  [[                                                                                            (32*p2^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3, (8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3, (8*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p2^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3]
            [                       (8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3,                                                                                                        (32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3,                        (8*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p2^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3]
            [(4*p2^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3, (8*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p2^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1*p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3,                                                                                             (32*p2^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3]
             ];
    D_p3 =  [[                                                                                            (32*p3^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p2^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3, (4*p3^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3, (8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3]
            [(8*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p3^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3,                                                                                             (32*p3^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p1^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3, (8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3]
            [                       (8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3,                        (8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3,                                                                                                        (32*p1^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p2^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3]
             ];
end