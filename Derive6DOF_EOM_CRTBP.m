function Derive6DOF_EOM_CRTBP
    syms r_T v_T r v p w lambda_rT lambda_vT lambda_r lambda_v lambda_p lambda_w [3,1] real
    syms lambda_m m W s t eta R epsilon T c delta pNorm1 real 
    syms x_OL2 x_SunL2 x_EarthL2 mu_1 mu_2 real
    syms consts 
    syms D [3,3] real 
    syms G [3,3] real 
    syms I Iinv [3,3] real
    syms u tau [3,1] real
    syms d [3,1] real
    syms c0_target a [3,1] real
    syms theta_x theta_y theta_z real
    sympref('AbbreviateOutput', false)

    %% CW Equations in Rotating frame

    normFunc = @(x) sqrt(x'*x);
    p2Temp = p'*p;
    pCross = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    temp = (1+p2Temp)^2;
    
    % Translational dynamics are in a rotating reference frame: MRP (D) +
    % Rotation about z-axis (G)
    D = eye(3) - 4*(1-p2Temp)/temp*pCross + 8/temp * pCross*pCross;
    G = [cos(W*t) sin(W*t) 0;
          -sin(W*t) cos(W*t) 0;
          0 0 1];
    Phi = G*D;

    %% Ellipsoidal Constraint    
    Rx = [1, 0, 0;
          0, cos(theta_x), -sin(theta_x);
          0, sin(theta_x), cos(theta_x)];

    Ry = [cos(theta_y), 0, sin(theta_y);
          0, 1, 0;
          -sin(theta_y), 0, cos(theta_y)];

    Rz = [cos(theta_z), -sin(theta_z), 0;
          sin(theta_z), cos(theta_z), 0;
          0, 0, 1];

    U = Rx * Ry * Rz;  % Ellipsoid rotation in the target frame
    sigma = diag([1/(a(1)^2) 1/(a(2)^2) 1/(a(3)^2)]);

    % Compute the ellipsoid matrix B
    B = U * sigma * U';

    % Compute the anchor point in LVLH frame.
    x0 = (r + Phi*d) - c0_target;

    % Formulate the quadratic equation: (x0 - t*d)' * B * (x0 - t*d) = 1.
    rB = Phi'*B'*((r+Phi*d)-c0_target);
    E1 = (-((-2 * (x0' * B * (Phi*u)))^2) + 4*((Phi*u)' * B * (Phi*u))*((x0' * B * x0) - 1))*a1^2;
    E2 = (4*((Phi*u)' * B * (Phi*u))*((x0' * B * x0) - 1))*a1^2;

    %if rB'*u >= 0
            E = E1; % -Delta, if positive then constraint not violated, eta = 1
    %else
            %E = E2; % -Delta, smoothed function so that a1
    %end

    eta = 1/2*(1+tanh(E/epsilon));

    %% CRTBP quantities
    r1T = r_T+[x_SunL2;0;0];
    r2T = r_T+[x_EarthL2;0;0];

    %% Dynamics equations for CRTBP
    r_TDot = v_T;
    v_TDot = [2*W*v_T(2)+((W^2)*r_T(1)*x_OL2) - (mu_1*((r_T(1)+x_SunL2)/(norm(r_T+[x_SunL2;0;0])^3))) - (mu_2*((r_T(1)+x_EarthL2)/(norm(r_T+[x_EarthL2;0;0])^3)));
                -2*W*v_T(1)+((W^2)*r_T(2)) - (mu_1*(r_T(2)/(norm(r_T+[x_SunL2;0;0])^3))) - (mu_2*(r_T(2)/(norm(r_T+[x_EarthL2;0;0])^3)));
                - (mu_1*(r_T(3)/(norm(r_T+[x_SunL2;0;0])^3))) - (mu_2*(r2T(3)/(norm(r_T+[x_EarthL2;0;0])^3)))];
    rDot = v;
    vDot = [2*W*v(2)+((W^2)*r(1)) + (mu_1*(((r_T(1)+x_SunL2)/(norm(r_T+[x_SunL2;0;0])^3))-((r_T(1)+x_SunL2+r(1))/(norm(r_T+[x_SunL2;0;0]+r)^3))))+(mu_2*(((r_T(1)+x_EarthL2)/(norm(r_T+[x_EarthL2;0;0])^3))-((r_T(1)+x_EarthL2+r(1))/(norm(r_T+[x_EarthL2;0;0]+r)^3))));
                -2*W*v(1)+((W^2)*r(2)) + (mu_1*(((r_T(2))/(norm(r_T+[x_SunL2;0;0])^3))-((r_T(2)+r(2))/(norm(r_T+[x_SunL2;0;0]+r)^3))))+(mu_2*(((r_T(2))/(norm(r_T+[x_EarthL2;0;0])^3))-((r_T(2)+r(2))/(norm(r_T+[x_EarthL2;0;0]+r)^3))));
                (mu_1*(((r_T(3))/(norm(r_T+[x_SunL2;0;0])^3))-((r_T(3)+r(3))/(norm(r_T+[x_SunL2;0;0]+r)^3))))+(mu_2*(((r_T(3))/(norm(r_T+[x_EarthL2;0;0])^3))-((r_T(3)+r(3))/(norm(r_T+[x_EarthL2;0;0]+r)^3))))];

    vDotControls = T/m * delta * eta * Phi * u; % control term for thrust
    mDot = -T/c * delta * eta;
    pDot = 0.25 * ((1 + p2Temp) * eye(3) + 2 * pCross*pCross + 2*pCross) * w;
    wDot = (Iinv*cross(w,I*w));
    wDotControls = T*delta*eta*Iinv*cross(d,u) + Iinv*tau;

    %% Part of Hamiltonian depending on eta
    H_etaTerms = eta *(lambda_v' * Phi * u/m +...
        -lambda_m/(c) + ...
        + lambda_w'*Iinv*cross(d,u))*T*delta;
    etaCoeff = (lambda_v' * Phi * u/m +...
        -lambda_m/(c) + ...
        + lambda_w'*Iinv*cross(d,u))*T*delta;

    %% H_rT Terms
    H_rTTerms = lambda_vT'*v_TDot;
    
    %% H_vT Terms
    H_vTTerms = lambda_rT'*r_TDot + lambda_vT'*v_TDot;
    
    %% H_rTerms
    H_rTerms = lambda_v'*vDot;
    
    %% H_vTerms
    H_vTerms = lambda_r'*rDot + lambda_v'*vDot;
    
    %% H_mTerms (with 1/m)
    H_mTerms = lambda_v'*vDotControls;

    %% H_pTerms
    H_pTerms = lambda_v'*vDotControls + lambda_m*mDot +lambda_p'*pDot + ...
        + lambda_w'*(wDotControls);
    %% H_wTerms
    H_wTerms = lambda_p'*pDot + lambda_w'*wDot;

    %% Derivatives - one time
    fprintf('lambda_rTDot = [\n')
    disp(-[diff(H_rTTerms,r_T(1));
    diff(H_rTerms,r_T(2));
    diff(H_rTerms,r_T(3))]);
    fprintf(']\n');

    fprintf('lambda_vTDot = [\n')
    disp(-[diff(H_vTTerms,v_T(1));
    diff(H_vTTerms,v_T(2));
    diff(H_vTTerms,v_T(3))]);
    fprintf(']\n');
    
    fprintf('lambda_rDot = [\n')
    disp(-[diff(H_rTerms,r(1));
    diff(H_rTerms,r(2));
    diff(H_rTerms,r(3))]);
    fprintf(']\n');

    fprintf('lambda_vDot = [\n')
    disp(-[diff(H_vTerms,v(1));
    diff(H_vTerms,v(2));
    diff(H_vTerms,v(3))]);
    fprintf('];\n');

    fprintf('lambda_pDot = [\n')
    disp(-[diff(lambda_p'*pDot,p(1));
    diff(lambda_p'*pDot,p(2));
    diff(lambda_p'*pDot,p(3))]);
    fprintf(']\n');

    fprintf('lambda_wDot = [\n')
    disp(-[diff(H_wTerms,w(1));
    diff(H_wTerms,w(2));
    diff(H_wTerms,w(3))]);
    fprintf(']\n');

    %% Derivatives recurring
    fprintf('lambda_rDot = lambda_rDot + etaCoeff*etaPrime(ii)*[\n')
    disp(-[diff(E1,r(1));
    diff(E1,r(2));
    diff(E1,r(3))]);
    fprintf('];\n');
    E_r1 = -[diff(E1,r(1));diff(E1,r(2));diff(E1,r(3))];

    % lambda_pDot 
    fprintf('lambda_pDot = lambda_pDot + etaCoeff*etaPrime(ii)*[\n')
    disp(-[ diff(E1,p(1));
    diff(E1,p(2));
    diff(E1,p(3))]);
    fprintf('];\n');

    E_p1 = -[diff(E1,p(1));diff(E1,p(2));diff(E1,p(3))];

    % if rB_ellips'*u < 0
    fprintf('lambda_rDot = lambda_rDot + etaCoeff*etaPrime(ii)*[\n')
    disp(-[diff(E2,r(1))]);
    diff(E2,r(2));
    diff(E2,r(3));
    fprintf('];\n');
    E_r2 = -[diff(E2,r(1));diff(E2,r(2));diff(E2,r(3))];


    % lambda_pDot 
    fprintf('lambda_pDot = lambda_pDot + etaCoeff*etaPrime(ii)*[\n')
    disp(-[ diff(E2,p(1));
    diff(E2,p(2));
    diff(E2,p(3))]);
    fprintf('];\n');
    E_p2 = -[diff(E1,p(1));diff(E1,p(2));diff(E1,p(3))];

    % eta Coef
    disp([])

    E_p1 = subs(E_p1, (p1^2 + p2^2 + p3^2 + 1), pNorm1);
    E_p2 = subs(E_p2, (p1^2 + p2^2 + p3^2 + 1), pNorm1);
    E_r1 = subs(E_r1, (p1^2 + p2^2 + p3^2 + 1), pNorm1);
    E_r2 = subs(E_r2, (p1^2 + p2^2 + p3^2 + 1), pNorm1);


    % % Copy this into the code for lambdaMDot \/
    % lambda_mDot = T/(m^2) * delta(ii)*eta(ii)*lambda_v'*Phi*u;
    % % 
    % fprintf('lambda_pDot_Part2 = T/m * delta(ii) *eta(ii)*[\n')
    % disp(-[diff( lambda_v'* Phi * u ,p1);
    %      diff( lambda_v'* Phi * u ,p2);
    %     diff( lambda_v'* Phi * u ,p3)]);
    %  fprintf(']\n');

    %% END 
    % Write full expression for E_r1, E_p1 to a text file
    outputStr = evalc('disp(E_p1)');
    fid = fopen('E_p1_fullExpression.txt','w');
    fprintf(fid, '%s', outputStr);
    fclose(fid);
    
    outputStr = evalc('disp(E_r1)');
    fid = fopen('E_r1_fullExpression.txt','w');
    fprintf(fid, '%s', outputStr);
    fclose(fid);
    
    % Write full expression for E_r2, E_p2 to a text file
    outputStr = evalc('disp(E_r2)');
    fid = fopen('E_r2_fullExpression.txt','w');
    fprintf(fid, '%s', outputStr);
    fclose(fid);
    
    outputStr = evalc('disp(E_p2)');
    fid = fopen('E_p2_fullExpression.txt','w');
    fprintf(fid, '%s', outputStr);
    fclose(fid);

end