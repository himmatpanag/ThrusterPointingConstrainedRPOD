function TestConstrainedOptimaLPointingDirection
    
    % Projection method

    [problemParameters, solverParameters] = ConstrainedApproachTestCondition(12);
    problemParameters.constraint.alpha0=90*pi/180;

    N = 100;
    errors = nan(3,100);
    for ii = 1:N
        lambda_v=randn(3,1)*5;
        pos=randn(3,1)*3;
        
        
        [~,~,uDir,uMinUnrotated,a,phiMin,thetaMin] = ChooseOptimalPointingDirection(pos,lambda_v,0,0,0, ...
            problemParameters.dynamics,problemParameters.constraint,0);
        uDirProjection = ProjectionOntoConeMethod(a',problemParameters.constraint.alpha0);

        errors(:,ii) = uMinUnrotated - uDirProjection;
    
    end
    figure; plot(errors');

end

function uDir = ProjectionOntoConeMethod(a,alpha)
    uFree = -a/norm(a);
    z=[0;0;1];
    gamma = acos(dot(uFree,z));
    if gamma > alpha
        uDir = uFree;
    else
        n = cross(z,uFree);
        n=n/norm(n);
        uDir = RodriguesRotation(uFree,alpha-gamma,n);
    end
end
function rotatedVector = RodriguesRotation(vector,theta,axis)
    % Compute the cross product between the axis and the vector
    crossProduct = cross(axis, vector);
    
    % Compute the dot product between the axis and the vector
    dotProduct = dot(axis, vector);
    
    % Compute the rotated vector using the Rodrigues rotation formula
    rotatedVector = vector * cos(theta) + crossProduct * sin(theta) + axis * dotProduct * (1 - cos(theta));
end

function [S,throttle,uDir,uMinUnrotated,a,phiMin,thetaMin] = ChooseOptimalPointingDirection(pos,lambda_v,m,lambda_m,rho,dynamics,constraint,t)
    % Code snipped from SolvePointingConstrainedControlProblem

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
                    alpha = constraint.thetaFunc(rNorm);
                end 
            else 
                alpha = constraint.alpha0;
            end 
            
            % Compute a1,a2,a3 (lambdaV^T Psi(r))
            % Construct rotation matrix from x',y',z' frame to x,y,z frame. Use quaternions to avoid singularities.
            % z' axis is aligned with r.
            % to get x,y,z frame: rotate by gamma (theta between z and r) about cross(r,z));
            
            gamma = acos(r3/rNorm); % dot(z,r)/norm(r); z = [0;0;1]
            n = -[r2;-r1;0]/r12; % cross(X(1:3),z)/norm(cross(X(1:3),z));
            q0 = cos(gamma/2);
            quatVec = n.*sin(gamma/2);
            skewSymmetric = @(v) [0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
            RotMatrix = ((q0^2 - dot(quatVec,quatVec)) * eye(3) + 2.*quatVec*quatVec' + 2*q0*skewSymmetric(quatVec));
            a = lambda_v'*RotMatrix;
            a1 = a(1); a2 = a(2); a3 = a(3);
            
            %% Find the four Critical points of the Hamiltonian. 
            % Note (E = lambda_v^T uDir) is the critical term, as no other terms in the Hamiltonian depent on uDir
            % uSpherical = @(phi,theta) [cos(phi)*cos(theta);cos(phi)*sin(theta);sin(phi)]; 
            E = zeros(5,1); phi = zeros(5,1); theta = zeros(5,1);
            % Common thetas
            if ~isreal(a1)||~isreal(a2)
                wow =3;
            end 
            delta = atan2(a2,a1); 
            if delta > 0 
                gamma = delta - pi;
            else 
                gamma = delta + pi;
            end

            % Edge points of domain, phi in [-pi/2,pi/2-alpha], theta in [-pi,pi] 
            phi(1) = -pi/2; 
            E(1) = -a3; % phiLowerLimit = -pi/2; theta = has two possible values that result in dH/Phi = 0, but all values of theta will give the same value of uSpherical
            
            phi(2) = pi/2 - alpha; theta(2) = delta; % two options here!
            E(2) = dot(a,uSpherical(phi(2),theta(2)));
            phi(3) = pi/2 - alpha; theta(3) = gamma; % 2nd option
            E(3) = dot(a,uSpherical(phi(2),theta(3)));
        
            % Points where the gradient of the Hamiltonian is zero
            R = sqrt(a1^2 + a2^2);
            phi(4) = atan(a3/R); theta(4) = delta; 
            phi(5) = -phi(4); theta(5) = gamma;

            % Check phi3, phi4 are in the domain
            if (phi(4) > pi/2 - alpha) || (phi(4) == -pi/2)
                E(4) = Inf; 
            else 
                E(4) = dot(a,uSpherical(phi(4),theta(4)));
            end
            
            if (phi(5) > pi/2 - alpha) || (phi(5) == -pi/2)
                E(5) = Inf;
            else 
                E(5) = dot(a,uSpherical(phi(5),theta(5)));
            end
        
            %% Find minimizer of Hamiltonian and corresponding u
            [~,imin] = min(E);
            phiMin = phi(imin);
            thetaMin = theta(imin);
            uMinUnrotated = uSpherical(phiMin,thetaMin); % This is in the (x',y',z') frame
        
            uDir = RotMatrix*uMinUnrotated; % Rotate to (x,y,z) frame
    end 
    %S = -1-dot(lambda_v,uDir); % Switch function
    S = -1-dynamics.exhaustVelocity/m*dot(lambda_v,uDir)+lambda_m; % Switch function
    switch dynamics.regularizationMethod
        case 'L2_Norm'
%             S = S*dynamics.maxThrust/dynamics.exhaustVelocity;
            throttle = 0.5*(1 + S./sqrt(S^2+rho^2) ); % Throttle
        case 'HyperbolicTangentSmoothing'
            throttle = 0.5*(1+tanh(S./rho)); % Throttle
    end
end 

function v = uSpherical(phi,theta) 
    v = [cos(phi)*cos(theta);cos(phi)*sin(theta);sin(phi)]; 
end 