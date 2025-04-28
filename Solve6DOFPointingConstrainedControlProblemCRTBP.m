function solution = Solve6DOFPointingConstrainedControlProblemCRTBP(problemParameters, solverParameters, skipSol) %Done with CRTBP
    % Author: Marin Hubert
    % Solves a single instance of the 6DOF Optimal Control Problem with
    % CRTBP dynamics
    % Not allowed to point at the origin
    % Date: 16 Apr 2025
    if nargin < 3, skipSol = false; end
    if isempty(gcp), parpool(8); end
    problemParameters.dynamics.finalT = solverParameters.tSpan(end);
    [initialGuess,remainingCostates] = ChooseCostatesToIgnore( ...
        problemParameters.dynamics,solverParameters.initialCostateGuess);
    if skipSol % We may want to get experimental with guesses of lam0, This spits out the error with a particular lam0, without doing any solving
        lam0=initialGuess; exitFlag = -1;
    else
        [lam0,~,exitFlag] = fsolve(@costFunction,initialGuess,...
            solverParameters.fSolveOptions,...
            solverParameters.tSpan, problemParameters.xT0, problemParameters.x0, problemParameters.xf,...
            problemParameters.p0,problemParameters.pf,...
            problemParameters.w0,problemParameters.wf,...
            solverParameters.rho,...
            problemParameters.constraint,...
            problemParameters.dynamics,remainingCostates, ...
            solverParameters.odeOptions,false);
    end
    solution = emptySolutionStruct(problemParameters,solverParameters);
    solution.newCostateGuess = CombineIgnoredCostates(problemParameters.dynamics, lam0, remainingCostates); 
    if solverParameters.getTraj
        [initialGuess,remainingCostates] = ChooseCostatesToIgnore( ...
            problemParameters.dynamics, solution.newCostateGuess);
        optimalTraj = costFunction(initialGuess,...            
            solverParameters.tSpan, problemParameters.xT0, problemParameters.x0, problemParameters.xf,...
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
        if problemParameters.dynamics.sundman.useTransform
            solution.t = optimalTraj.X(:,39); 
            solution.simTime = optimalTraj.t;
        else
            solution.t = optimalTraj.t;
        end
        solution.x = X_minU;
        solution.throttle = zeros(problemParameters.dynamics.numEngines,numel(solution.t));
        solution.eta = zeros(problemParameters.dynamics.numEngines,numel(solution.t));
        solution.constraint = zeros(problemParameters.dynamics.numEngines,numel(solution.t));
        solution.switchFunction = zeros(problemParameters.dynamics.numEngines,numel(solution.t));
        solution.torqueInertialFrame = zeros(3,numel(solution.t),1);
        solution.thrustTranslationalFrame = zeros(3,numel(solution.t),1);
        for ii = 1:numel(solution.t)
            optimalSolStruct = SixDimConstrainedOptimalControlProblem(solution.t(ii),...
                X_minU(ii,:)',problemParameters.dynamics,solverParameters.rho,...
                problemParameters.constraint,false);
            solution.throttle(:,ii) = optimalSolStruct.throttles;
            solution.eta(:,ii) = optimalSolStruct.etas;
            solution.constraint(:,ii) = optimalSolStruct.constraint;
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
        
        if (dynamics.sundman.useTransform) && (~dynamics.sundman.useSimplified)
            initialGuess = costateGuess([1:13,20]);
            remainingCostates =  costateGuess(14:19);
        else
            initialGuess = costateGuess(1:13);
            remainingCostates =  costateGuess(14:end);
        end
    else 
        initialGuess = costateGuess;
        remainingCostates = [];
    end
end

function lam0 = CombineIgnoredCostates(dynamics, costateGuess, costatesToIgnore)
    if (dynamics.engineConfiguration==THRUSTER_CONFIGURATION.CG_ALIGNED_6 || dynamics.engineConfiguration==THRUSTER_CONFIGURATION.CG_ALIGNED_CANTED_8) && (dynamics.attitudeActuator==ATTITUDE_CONTROL_TYPE.NONE)
        if (dynamics.sundman.useTransform) && (~dynamics.sundman.useSimplified)
            lam0 = [costateGuess(1:13);costatesToIgnore;costateGuess(end)];
        else
            lam0 = [costateGuess;costatesToIgnore];
        end
    else
        lam0 = costateGuess;
    end
end

function Xdot = SixDimConstrainedOptimalControlProblem(t,X,dynamics,rho,constraint,returnDynamics)
if nargin < 6
    returnDynamics = true;
end
    % states 1-6 are pos vel of target
    % states 7-12 are rel pos vel of spacecraft
    % state 13 is mass
    % states 14-19 are MRP and angular velocity
    % states 20-38 are corresponding costates. (20-31 pos vel target/rel, 32 mass,
    % 33-38 angular)
    pos_T = X(1:3);
    pos = X(7:9);
    m = X(13);
    p = X(14:16);
    w = X(17:19);
    lambda_v_T = X(23:25);
    lambda_v = X(29:31);
    lambda_m = X(32);
    lambda_w = X(36:38);
    if dynamics.sundman.useTransform
        t = X(39); 
    end 

    p2 = p'*p;
    pCross = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    temp = (1+p2)^2;
    D_Temp = lambda_w'*dynamics.inertiaInverse; % cross product is bilinear so product rule distributes
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    % Translational and CRTBP dynamics are in a rotating reference frame
    D = eye(3) + (8*pCross*pCross - 4*(1-p2)*pCross)/temp;
    G = [cos(dynamics.frameRotationRate*t) sin(dynamics.frameRotationRate*t) 0;
                          -sin(dynamics.frameRotationRate*t) cos(dynamics.frameRotationRate*t) 0;
                          0 0 1];
    Phi = G*D;

    eta = ones(dynamics.numEngines,1); constraintFunc = ones(dynamics.numEngines,1);
    etaPrime= zeros(dynamics.numEngines,1);
    % Compute the constraint function, eta and etaPrime
    if (constraint.type == POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE || constraint.type== POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE)
        for ii = 1:dynamics.numEngines
            d = dynamics.engineLocationBody(:,ii);
            u = dynamics.thrustDirectionBody(:,ii);
            R = constraint.targetRadius;
            rB = Phi'*pos+d;
            if rB'*u >= 0
                constraintFunc(ii) = 1/R^2*(pos'*pos + 2*pos'*Phi*d+d'*d-R^2 - (rB'*u)^2);
            else
                constraintFunc(ii) =  1/R^2*(pos'*pos + 2*pos'*Phi*d+d'*d-R^2);
            end
       
            eta(ii) = 1/2 * (1 + tanh(constraintFunc(ii)/constraint.epsilon));
            etaPrime(ii) = 1/(2*constraint.epsilon)*(1-(tanh(constraintFunc(ii)/constraint.epsilon))^2);
            % If the constraint function is positive, the constraint should be INACTIVE and eta = 1 
        end 

    elseif constraint.type == POINTING_CONSTRAINT_TYPE.ELLIPSOIDAL
        for ii = 1:dynamics.numEngines
            d = dynamics.engineLocationBody(:,ii);
            u = dynamics.thrustDirectionBody(:,ii);
            
            %Ellipsoid properties
            a1 = constraint.targetAxisx;
            a2 = constraint.targetAxisy;
            a3 = constraint.targetAxisz;

            sigma = diag([1/(a1^2),1/(a2^2),1/(a3^2)]);

            U = constraint.U;
            
            c0_target = [constraint.offsetx;constraint.offsety;constraint.offsetz];
            
            % Compute ellipsoid matrix A
            A = U * sigma * U';
            
            % Formulate the quadratic equation: (x0 - t*d)' * A * (x0 - t*d) = 1.
            a = (Phi*u)' * A * (Phi*u);
            b = -2 * (((pos + Phi*d) - c0_target)' * A * (Phi*u));
            c = (((pos + Phi*d) - c0_target)' * A * ((pos + Phi*d) - c0_target)) - 1;
            
            rB = Phi'*A'*((pos+Phi*d)-c0_target);
            
            if rB'*u >= 0
                constraintFunc(ii) = (-(b^2)+4*a*c)*a1^2.5; % -Delta, if positive then constraint not violated, eta = 1, Scaled by semi axis squared
            else
                constraintFunc(ii) = (4*a*c)*a1^2.5; % -Delta, smoothed function so that a1, Scaled by semi axis squared
            end
            eta(ii) = 1/2 * (1 + tanh(constraintFunc(ii)/constraint.epsilon));
            etaPrime(ii) = 1/(2*constraint.epsilon)*(1-(tanh(constraintFunc(ii)/constraint.epsilon))^2);
            % If the constraint function is positive, the constraint should be INACTIVE and eta = 1 
        end
     end

    % Compute the switch functions and throttles
    SwitchFunctions = zeros(dynamics.numEngines,1);
    delta = zeros(dynamics.numEngines,1);
    totalTorque = zeros(3,1); totalThrustLVLH = zeros(3,1); mDot = 0; H = 0;
    for ii = 1:dynamics.numEngines
        r_cross_v = cross(dynamics.engineLocationBody(:,ii),dynamics.thrustDirectionBody(:,ii));
        % Compute the switch functions and controls
        SwitchFunctions(ii) = (-1 + eta(ii) * (lambda_m - dynamics.exhaustVelocity/m * lambda_v'*Phi*dynamics.thrustDirectionBody(:,ii) + ...
            - dynamics.exhaustVelocity*D_Temp*r_cross_v));
        delta(ii) = 1/2 * (1+tanh(SwitchFunctions(ii)/rho));

        mDot = mDot - dynamics.maxThrust(ii)/dynamics.exhaustVelocity*delta(ii)*eta(ii);
        H = H + dynamics.maxThrust(ii)/dynamics.exhaustVelocity*delta(ii);
        totalThrustLVLH = totalThrustLVLH + delta(ii)*eta(ii)*dynamics.maxThrust(ii)*Phi*dynamics.thrustDirectionBody(:,ii);
        totalTorque = totalTorque + dynamics.maxThrust(ii)*1e3*delta(ii)*eta(ii)*r_cross_v;   
    end

    optimalControlTorque = [0;0;0];
    switch dynamics.attitudeActuator
        case ATTITUDE_CONTROL_TYPE.CONTROL_TORQUE
            optimalControlTorque = (-dynamics.inertiaInverse'*lambda_w)./dynamics.torqueCostMultiplier;
            %S = -dynamics.inertiaInverse'*lambda_w;
            %optimalControlTorque = S./sqrt(S.^2 + rho^2) .* dynamics.maxControlTorque;
            % for ii = 1:3
            %     optimalControlTorque(ii) = max(min(optimalControlTorque(ii),dynamics.maxControlTorque),-dynamics.maxControlTorque);
            % end
    end

    % Compute the state and costate dynamics
    switch dynamics.type                  
        case 'CRTBP' %Xdot = Non-linear
            x_SunL2 = dynamics.x1shift;
            x_EarthL2 = dynamics.x2shift;

            r1T = [X(1)+x_SunL2; X(2); X(3)]; %dynamics.x_sun-L2
            r2T = [X(1)+x_EarthL2; X(2); X(3)]; %dynamics.x_earth-L2
            mu_1 = dynamics.mu_1; %Sun
            mu_2 = dynamics.mu_2; %Earth
            x_OL2 =dynamics.x_OL2 ; % distance barycenter to L2
            W = dynamics.frameRotationRate;

            fDynamics_target = [X(4); X(5); X(6); 
                2*W*X(5)+((W^2)*X(1)*x_OL2) - (mu_1*((r1T(1))/(norm(r1T)^3))) - (mu_2*(r2T(1)/(norm(r2T)^3)));...
                -2*W*X(4)+((W^2)*X(2)) - (mu_1*(r1T(2)/(norm(r1T)^3))) - (mu_2*(r2T(2)/(norm(r2T)^3)));...
                - (mu_1*(r1T(3)/(norm(r1T)^3))) - (mu_2*(r2T(3)/(norm(r2T)^3)))];


            fDynamics = [X(10); X(11); X(12); 
                2*W*X(11)+((W^2)*X(7)) + (mu_1*(((r1T(1))/(norm(r1T)^3))-((r1T(1)+X(7))/(norm(r1T+pos)^3))))+(mu_2*(((r2T(1))/(norm(r2T)^3))-((r2T(1)+X(7))/(norm(r2T+pos)^3))));...
                -2*W*X(10)+((W^2)*X(8)) + (mu_1*(((r1T(2))/(norm(r1T)^3))-((r1T(2)+X(8))/(norm(r1T+pos)^3))))+(mu_2*(((r2T(2))/(norm(r2T)^3))-((r2T(2)+X(8))/(norm(r2T+pos)^3))));...
                (mu_1*(((r1T(3))/(norm(r1T)^3))-((r1T(3)+X(9))/(norm(r1T+pos)^3))))+(mu_2*(((r2T(3))/(norm(r2T)^3))-((r2T(3)+X(9))/(norm(r2T+pos)^3))))
            ];
            
    end

    rDot_T = fDynamics_target(1:3);
    vDot_T = fDynamics_target(4:6);
    rDot = fDynamics(1:3);
    vDot = fDynamics(4:6) + totalThrustLVLH/m;  
    B_Temp = 0.25 * ((1 + p2) .* eye(3) + 2 * (pCross*pCross + pCross));
    pDot = B_Temp * w; % Testing pdot %pDot2 = .5 * (.5*(1-p2)*eye(3) + pCross + p*p')*w
    omegaDot = dynamics.inertiaInverse*(totalTorque+  optimalControlTorque - cross(w,dynamics.inertia*w));
    %[lambdaDot, E_p, E_r, E_t] = CostateDerivativesSymbolic6DOF(t,X,dynamics,delta,Phi,eta,etaPrime,constraint);

    if constraint.type == POINTING_CONSTRAINT_TYPE.ELLIPSOIDAL
        [lambdaDot, E_p, E_r, E_t] = CostateDerivativesSymbolic6DOFEllipsoidal(t,X,dynamics,delta,Phi,eta,etaPrime,constraint);
    else
        [lambdaDot, E_p, E_r, E_t] = CostateDerivativesSymbolic6DOF(t,X,dynamics,delta,Phi,eta,etaPrime,constraint);
    end 
    
    % gcf; for ii = 1:3
    %     subplot(3,2,ii); plot(t,E_p(ii),'.','HandleVisibility','off'); hold on; grid on;
    %     subplot(3,2,ii+3); plot(t,E_r(ii),'.','HandleVisibility','off'); hold on; grid on;
    % end
    
    H = H + X(20:22)'*rDot_T + lambda_v_T'*vDot_T + X(26:28)'*rDot + lambda_v'*vDot + ...
        lambda_m*mDot + X(33:35)'*pDot + lambda_w'*omegaDot;
 

    if returnDynamics
        if dynamics.sundman.useTransform
            timeDilationTerms = 1-dynamics.sundman.theta.*exp(-SwitchFunctions.^2);
            tPrime = prod(timeDilationTerms);
            if dynamics.sundman.useSimplified
                Xdot = [rDot_T; vDot_T; rDot; vDot; mDot; pDot; omegaDot;lambdaDot;1].*tPrime;
            else
                lambda_t = X(40);
                tPrimeProdExcept_ith = tPrime./timeDilationTerms;
                dtPrimedp = [0;0;0]; dtPrimedr = [0;0;0];
                dtPrimedt = 0; dtPrimedm = 0;
                dH_olddt = 0;
                H = H + lambda_t;
                for ii = 1:dynamics.numEngines
                    lpu = lambda_v'*Phi*dynamics.thrustDirectionBody(:,ii);
                    [lpu_t,lpu_p] = SundmanDerivatives(t,dynamics.thrustDirectionBody(1,ii),dynamics.thrustDirectionBody(2,ii),dynamics.thrustDirectionBody(3,ii), ...
                        p(1),p(2),p(3),lambda_v(1),lambda_v(2),lambda_v(3),dynamics.frameRotationRate);
                    coeff_dSdx = 2*dynamics.sundman.theta*SwitchFunctions(ii)*...
                        exp(-SwitchFunctions(ii)^2) * tPrimeProdExcept_ith(ii);
                    dtPrimedp = dtPrimedp + coeff_dSdx * (-dynamics.exhaustVelocity)/m *...
                        (etaPrime(ii)*lpu*E_p(:,ii) + eta(ii) * lpu_p); 
                    dtPrimedr = dtPrimedr + coeff_dSdx * (-dynamics.exhaustVelocity)/m * etaPrime(ii)*lpu *E_r(:,ii);
                    dSidt = (-dynamics.exhaustVelocity)/m *(etaPrime(ii)*lpu*E_t(ii) + eta(ii) * lpu_t);
                    dH_olddt = dH_olddt + dynamics.maxThrust(ii)/dynamics.exhaustVelocity * delta(ii)*dSidt;
                    dtPrimedt = dtPrimedt + coeff_dSdx * dSidt;
                    dtPrimedm = dtPrimedm + coeff_dSdx * dynamics.exhaustVelocity/(m^2)*eta(ii)*lpu;
                end
                lambda_tPrime = -H*dtPrimedt - dH_olddt*tPrime;
                
                % lambda_rTPrime = lambdaDot(1:3).*tPrime + H*dtPrimedr;
                % lambda_vTPrime = lambdaDot(4:6).*tPrime; 
                lambda_rPrime = lambdaDot(7:9).*tPrime + H*dtPrimedr;
                lambda_vPrime = lambdaDot(10:12).*tPrime;
                lambda_mPrime = lambdaDot(13).*tPrime + H*dtPrimedm;
                lambda_pPrime = lambdaDot(14:16).*tPrime + H*dtPrimedp;
                lambda_wPrime = lambdaDot(17:19).*tPrime;
                
                Xdot = [
                    rDot.*tPrime; vDot.*tPrime; mDot.*tPrime; pDot.*tPrime; omegaDot.*tPrime;
                    lambda_rPrime; lambda_vPrime; lambda_mPrime; lambda_pPrime; lambda_wPrime;
                    tPrime; lambda_tPrime];
            end 
        else
            Xdot = [rDot_T; vDot_T; rDot; vDot; mDot; pDot; omegaDot; lambdaDot];
        end 
    else % For constructing the solution structure. 
        % Return the switch functions, constraints, throttles, anything else you want to log.
        Xdot.throttles = delta;
        Xdot.etas = eta;
        Xdot.switchFunctions = SwitchFunctions;
        Xdot.totalTorqueInertialFrame = totalTorque+optimalControlTorque;
        Xdot.totalThrustTranslationalFrame = totalThrustLVLH;
        Xdot.Hamiltonian = H;
        Xdot.constraint = constraintFunc;
    end
end 

function [lpu_t,lpu_p] = SundmanDerivatives(t,u1,u2,u3,p1,p2,p3,lambda_v1,lambda_v2,lambda_v3,W)
    lpu_t = u1*(lambda_v1*(W*sin(W*t)*((8*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^2 - 1) + W*cos(W*t)*((8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^2)) - lambda_v2*(W*sin(W*t)*((8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^2) - W*cos(W*t)*((8*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^2 - 1))) - u2*(lambda_v1*(W*sin(W*t)*((8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^2) + W*cos(W*t)*((8*p1^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^2 - 1)) - lambda_v2*(W*sin(W*t)*((8*p1^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^2 - 1) - W*cos(W*t)*((8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^2))) - u3*(lambda_v1*(W*sin(W*t)*((8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^2) - W*cos(W*t)*((8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (p1*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^2)) + lambda_v2*(W*sin(W*t)*((8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (p1*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^2) + W*cos(W*t)*((8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^2)));
    lpu_p = [   u2*(lambda_v3*((8*p1^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p1^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) + lambda_v1*(sin(W*t)*((32*p1^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3) + cos(W*t)*((8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3)) + lambda_v2*(cos(W*t)*((32*p1^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3) - sin(W*t)*((8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3))) - u3*(lambda_v1*(sin(W*t)*((8*p1^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p1^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) - cos(W*t)*((8*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1*p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3)) - lambda_v3*((32*p1^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3) + lambda_v2*(cos(W*t)*((8*p1^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p1^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) + sin(W*t)*((8*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1*p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3))) + u1*(lambda_v3*((8*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3) + lambda_v1*(cos(W*t)*((32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3) + sin(W*t)*((8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3)) + lambda_v2*(cos(W*t)*((8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3) - sin(W*t)*((32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3)))
                u3*(lambda_v3*((32*p2^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3) + lambda_v1*(cos(W*t)*((8*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p2^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) + sin(W*t)*((8*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p2^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3)) - lambda_v2*(sin(W*t)*((8*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p2^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) - cos(W*t)*((8*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p2^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3))) - u1*(lambda_v3*((8*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p2^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) - lambda_v1*(cos(W*t)*((32*p2^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3) + sin(W*t)*((8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3)) + lambda_v2*(sin(W*t)*((32*p2^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3) - cos(W*t)*((8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3))) + u2*(lambda_v3*((8*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p1*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p2^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1*p2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3) + lambda_v1*(cos(W*t)*((8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3) + sin(W*t)*((32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3)) + lambda_v2*(cos(W*t)*((32*p1^2*p2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3) - sin(W*t)*((8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p2^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3)))
                u1*(lambda_v3*((8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3) + lambda_v1*(sin(W*t)*((8*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p3^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) + cos(W*t)*((32*p3^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p2^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3)) + lambda_v2*(cos(W*t)*((8*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p3^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 - (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) - sin(W*t)*((32*p3^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p2^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3))) + u2*(lambda_v3*((8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3) - lambda_v1*(cos(W*t)*((8*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p3^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) - sin(W*t)*((32*p3^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p1^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3)) + lambda_v2*(sin(W*t)*((8*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^2 + (4*p1^2 + 4*p2^2 + 4*p3^2 - 4)/(p1^2 + p2^2 + p3^2 + 1)^2 - (4*p3^2*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p1*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) + cos(W*t)*((32*p3^3)/(p1^2 + p2^2 + p3^2 + 1)^3 - (16*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 + (32*p1^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3))) + u3*(lambda_v3*((32*p1^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3 + (32*p2^2*p3)/(p1^2 + p2^2 + p3^2 + 1)^3) + lambda_v1*(cos(W*t)*((8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3) + sin(W*t)*((8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3)) + lambda_v2*(cos(W*t)*((8*p2)/(p1^2 + p2^2 + p3^2 + 1)^2 - (8*p1*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p2*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3 + (4*p1*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3) - sin(W*t)*((8*p1)/(p1^2 + p2^2 + p3^2 + 1)^2 + (8*p2*p3)/(p1^2 + p2^2 + p3^2 + 1)^2 - (32*p1*p3^2)/(p1^2 + p2^2 + p3^2 + 1)^3 - (4*p2*p3*(4*p1^2 + 4*p2^2 + 4*p3^2 - 4))/(p1^2 + p2^2 + p3^2 + 1)^3)))
            ];
end

function trajOut = costFunction(lam0_guess,tspan, xT0, x0, xf, p0, pf, ...
    w0, wf,rho,constraint,dynamics,costatesToIgnore,opts_ode,getSol)
    
    lambda_0 = CombineIgnoredCostates(dynamics, lam0_guess, costatesToIgnore); 
    endTimeReached = false;
    if dynamics.sundman.useTransform
        opts_ode = odeset(opts_ode, Event=@BothEvents);
        tspan = tspan*2000; 
        if dynamics.sundman.useSimplified
            X0 = [xT0; x0; p0; w0;lambda_0;0];
        else 
            X0 = [xT0; x0; p0; w0;lambda_0(1:(end-1));0;lambda_0(end)];
        end
    else
        X0 = [xT0; x0; p0; w0; lambda_0]; %added m0 here, have to ask Himmat+added problemParameters.initialMass,... before calling cost function
        opts_ode = odeset(opts_ode, Event=@MRP_InShadowSet);
    end
    useShadowSet = true;
    if useShadowSet
        maxSetChanges = 100; tOut = 0; ii = 0;
        TSPAN = tspan; 
        t = []; X = [];
        while (ii<maxSetChanges) && (tOut(end) < tspan(end)) && ~endTimeReached
            % Solve one arc
            [tOut,XOut,te,ye,ie] = ode45(@SixDimConstrainedOptimalControlProblem,...
                TSPAN,X0,opts_ode,dynamics,rho,constraint);
            if ie == 1
                endTimeReached = true;
            end
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
            ii=ii+1;
        end
        if ii>=maxSetChanges
            warning('Max number of shadow set switches');
        end
    else
        [t,X] = ode45(@SixDimConstrainedOptimalControlProblem,...
            tspan,[xT0; x0; p0; w0;lambda_0],opts_ode,dynamics,rho,constraint);
    end
    % states 1-6 are pos vel
    % state 7 is mass
    % states 8-10 are MRP and 11-13 are angular velocity
    % states 14-26 are corresponding costates. (14-19 pos vel, 20 mass,
    % 21-26 angular)
    % Compute error from shadow set and normal set
    if dynamics.sundman.useTransform
        xODE_Final = ye;
    else
        xODE_Final = X(end,:);
    end
    normOriginal = norm(xODE_Final(14:16)-pf');
    MRP_Error = xODE_Final(14:16)-pf';
    if normOriginal > 0.2 % stop switching errors
        shadowX_Final = -xODE_Final(14:16)/(norm(xODE_Final(14:16))^2);
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
        MRP_Error = xODE_Final(33:35); % costates = 0 
        MRP_Scale = 1;
    end
    %scale = 100.*[1,1,1,velScale,velScale,velScale,MRP_Scale,MRP_Scale,MRP_Scale,...
        %omega_Scale,omega_Scale,omega_Scale,1];
    scale = 100.*[1,1,1,velScale,velScale,velScale,MRP_Scale,MRP_Scale,MRP_Scale,...
        omega_Scale,omega_Scale,omega_Scale,1,1,1,1,1,1,1];
    errorUnscaled = [xODE_Final(7:12)-xf',MRP_Error,xODE_Final(17:19)-wf',xODE_Final(20:25),xODE_Final(32)]; 
    % Want costates on final target pos,vel to be 0; want final costate for
    % mass 0
    err = errorUnscaled.*scale;

    if getSol 
        trajOut.err = err; 
        trajOut.t = t;
        trajOut.X = X;
    else
        trajOut = err; 
    end 
end
function [value,isterminal,direction] = BothEvents(t,X,dynamics,rho,constraint)
    [value1,isterminal1,direction1] = FinalTimeReachedEvent(t,X,dynamics,rho,constraint);
    [value2,isterminal2,direction2] = MRP_InShadowSet(t,X,dynamics,rho,constraint);
    value = [value1,value2];
    isterminal = [isterminal1,isterminal2];
    direction = [direction1,direction2];
end
function [value,isterminal,direction] = FinalTimeReachedEvent(t,X,dynamics,rho,constraint)
    value = X(39) - dynamics.finalT;
    if value > 0
        stop = true;
    end
    isterminal = 1;
    direction = 1;
end

function [value,isterminal,direction] = MRP_InShadowSet(~,X,dynamics,rho,constraint)
    value = norm(X(14:16))-1-0.5; % norm goes over 1.5
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