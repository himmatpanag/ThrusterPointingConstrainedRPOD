function Derive6DOF_EOM
    syms r v p w lambda_r lambda_v lambda_p lambda_w [3,1] real
    syms lambda_m m W s t theta eta R epsilon T c delta real 
    syms consts 
    syms D [3,3] real 
    syms G [3,3] real 
    syms I Iinv [3,3] real
    syms u tau [3,1] real
    syms d [3,1] real
    
    A = zeros(6,6); A(1:3,4:6) = eye(3); 
    A = sym(A); W2 = W*W; A(4,1) = 3*W2; A(4,5) = 2*W; A(5,4)=-2*W; A(6,3)=-W2;   

    normFunc = @(x) sqrt(x'*x);
    p2Temp = p'*p;
    pCross = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    temp = (1+p2Temp)^2;
    
    % Translational dynamics are in a rotating reference frame
    D = eye(3) - 4*(1-p2Temp)/temp*pCross + 8/temp * pCross*pCross;
    G = [cos(W*t) sin(W*t) 0;
          -sin(W*t) cos(W*t) 0;
          0 0 1];
    Phi = G*D;

    oldConstraintMethod = false;
    if oldConstraintMethod
        rL = Phi*d + r;
        rB = Phi'*r + d;
        a = asin(R/normFunc(rL));
        ca = cos(a);
        E = 1/R * (normFunc(u)*ca-rB'*u);
    else
        rB = Phi'*r+d;
        E1 = r'*r + 2*r'*Phi*d+d'*d-R^2 - (rB'*u)^2;
        E2 = r'*r + 2*r'*Phi*d+d'*d-R^2; %% TESTING THIS!
        %if rB'*u >= 0 % engine is pointed in the half plane pointed towards the target
            % Want constraint function be negative of Delta. 
            E=E1;
        %else
        %    E=E2;
        %end
    end
    eta = 1/2*(1+tanh(E/epsilon));

    rDot = v;
    vDot = A(4:6,:)*[r;v];
    vDotControls = T/m * delta * eta * Phi * u;
    mDot = -T/c * delta * eta;
    pDot = 0.25 * ((1 + p2Temp) * eye(3) + 2 * pCross*pCross + 2*pCross) * w;
    wDot = (Iinv*cross(w,I*w));
    wDotControls = T*delta*eta*Iinv*cross(d,u) + Iinv*tau;

    H_etaTerms = eta *(lambda_v' * Phi * u/m +...
        -lambda_m/(c) + ...
        + lambda_w'*Iinv*cross(d,u))*T*delta;
    etaCoeff = (lambda_v' * Phi * u/m +...
        -lambda_m/(c) + ...
        + lambda_w'*Iinv*cross(d,u))*T*delta;

    %% H_rTerms
    H_rTerms = lambda_v'*vDot;
    
    %% H_vTerms
    H_vTerms = lambda_r'*rDot + lambda_v'*A(4:6,:)*[r;v];
    
    %% H_mTerms
    H_mTerms = lambda_v'*vDotControls;

    %% H_pTerms
    H_pTerms = lambda_v'*vDotControls + lambda_m*mDot +lambda_p'*pDot + ...
        + lambda_w'*(wDotControls);
    %% H_wTerms
    H_wTerms = lambda_p'*pDot + lambda_w'*wDot;

    %% Sundman terms
    lpu = lambda_v'*Phi*u 
    diff(lpu,t)
    diff(E1,t)
    diff(E2,t)
    [diff(lpu,p1);diff(lpu,p2);diff(lpu,p3)]
    [diff(lpu,r1);diff(lpu,r2);diff(lpu,r3)]
    tPrime = 1-theta*exp(S^2)

    %% Derivatives - one time
    fprintf('lambda_rDot = [\n')
    disp(-[diff(H_rTerms,r1);
    diff(H_rTerms,r2);
    diff(H_rTerms,r3)]);
    fprintf(']\n');

    fprintf('lambda_vDot = [\n')
    disp(-[diff(H_vTerms,v1);
    diff(H_vTerms,v2);
    diff(H_vTerms,v3)]);
    fprintf('];\n');

    fprintf('lambda_pDot = [\n')
    disp(-[diff(lambda_p'*pDot,p1);
    diff(lambda_p'*pDot,p2);
    diff(lambda_p'*pDot,p3)]);
    fprintf(']\n');

    %% lambda_wDot
    fprintf('lambda_wDot = [\n')
    disp(-[diff(H_wTerms,w1);
    diff(H_wTerms,w2);
    diff(H_wTerms,w3)]);
    fprintf(']\n');

    %% Derivatives recurring
    fprintf('lambda_rDot = lambda_rDot + etaCoeff*etaPrime(ii)*[\n')
    disp(-[diff(E1,r1);
    diff(E1,r2);
    diff(E1,r3)]);
    fprintf('];\n');

    % lambda_pDot 
    fprintf('lambda_pDot = lambda_pDot + etaCoeff*etaPrime(ii)*[\n')
    disp(-[ diff(E1,p1);
    diff(E1,p2);
    diff(E1,p3)]);
    fprintf('];\n');

    % if rB'*u < 0
    fprintf('lambda_rDot = lambda_rDot + etaCoeff*etaPrime(ii)*[\n')
    disp(-[diff(E2,r1);
    diff(E2,r2);
    diff(E2,r3)]);
    fprintf('];\n');

    % lambda_pDot 
    fprintf('lambda_pDot = lambda_pDot + etaCoeff*etaPrime(ii)*[\n')
    disp(-[ diff(E2,p1);
    diff(E2,p2);
    diff(E2,p3)]);
    fprintf('];\n');
    
    % Copy this into the code for lambdaMDot \/
    lambda_mDot = T/(m^2) * delta(ii)*eta(ii)*lambda_v'*Phi*u;

    fprintf('lambda_pDot_Part2 = T/m * delta(ii) *eta(ii)*[\n')
    disp(-[diff( lambda_v'* Phi * u ,p1);
        diff( lambda_v'* Phi * u ,p2);
        diff( lambda_v'* Phi * u ,p3)]);
    fprintf(']\n');

    %% END 

    

end