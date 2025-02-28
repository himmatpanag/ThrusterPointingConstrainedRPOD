function Compare6DOF_SphericalConstraints
% Compare various constraint functions formulations for a spherical constraint.

N = 1000;
Constraint1 = zeros(N,1);
Constraint2 = zeros(N,1);
R = 2; d = [1;0;0]; uBody = d; Psi = eye(3); 
t= 0; Omega = .001;
for ii = 1:N
    randPosDir = rand(3,1);
    r = randPosDir./vecnorm(randPosDir) * 4;
    randOrient = rand(3,1); randOrient = randOrient./vecnorm(randOrient);
    randAngle = (rand*360 - 180)*pi/180;
    p = randOrient*tan(randAngle/4);
    
    Psi = RotationMatrix(p,t,Omega);

    % Solve system of algebraic eqns using solve. 
    syms t
    line = % vector eqn here (symbolic expression in terms of t)
    ellipsoid function: 
    eqn for line 
    eqn for ellipse 
    sol = solve(ellipsoid evaluated at line,t)
    if isreal(sol) && t > 0 
        constraint is active \
    else 
        inactive
    end

    % Test your derivation here: 
    Constraint2(ii) = DiscrimantFunction(r,d,uBody,R,Psi);
end 
if ~(nnz(sign(Constraint2)==sign(Constraint1)) == N)
    stop = true;
end

figure; grid on; hold on; 
plot(1:N,sign(Constraint1),'-.','DisplayName','\DotProd')
plot(1:N,sign(Constraint2),'-.','DisplayName','\Delta')

%% Compute derivatives
step = 0.001;
derivative_p1 = zeros(N,1);
derivative_p2 = zeros(N,1);
derivative_p3 = zeros(N,1);
derivative_r1 = zeros(N,1);
derivative_r2 = zeros(N,1);
derivative_r3 = zeros(N,1);
for ii = 1:N
    randPosDir = rand(3,1);
    r1 = randPosDir./vecnorm(randPosDir) * 4;
    randOrient = rand(3,1); randOrient = randOrient./vecnorm(randOrient);
    randAngle = (rand*360 - 180)*pi/180;
    p1 = randOrient*tan(randAngle/4);
    Psi1 = RotationMatrix(p1,t,Omega);
    Psi2 = Psi1;

    r2 = r1 + step*[1;0;0];
    derivative_r1(ii) = ComputeNumericalDerivative(r1,Psi1,r2,Psi2,d,uBody,R,step);
    r2 = r1 + step*[0;1;0];
    derivative_r2(ii) = ComputeNumericalDerivative(r1,Psi1,r2,Psi2,d,uBody,R,step);
    r2 = r1 + step*[0;0;1];
    derivative_r3(ii) = ComputeNumericalDerivative(r1,Psi1,r2,Psi2,d,uBody,R,step);
    
    r2 = r1;
    p2 = p1 + step*[1;0;0]; Psi2 = RotationMatrix(p2,t,Omega);
    derivative_p1(ii) = ComputeNumericalDerivative(r1,Psi1,r2,Psi2,d,uBody,R,step);
    p2 = p1 + step*[0;1;0]; Psi2 = RotationMatrix(p2,t,Omega);
    derivative_p2(ii) = ComputeNumericalDerivative(r1,Psi1,r2,Psi2,d,uBody,R,step);
    p2 = p1 + step*[0;0;1]; Psi2 = RotationMatrix(p2,t,Omega);
    derivative_p3(ii) = ComputeNumericalDerivative(r1,Psi1,r2,Psi2,d,uBody,R,step);
end

figure; 
subplot(2,3,1); grid on; hold on; plot(1:N, derivative_r1);
subplot(2,3,2); grid on; hold on; plot(1:N, derivative_r2);
subplot(2,3,3); grid on; hold on; plot(1:N, derivative_r3);
subplot(2,3,4); grid on; hold on; plot(1:N, derivative_p1);
subplot(2,3,5); grid on; hold on; plot(1:N, derivative_p2);
subplot(2,3,6); grid on; hold on; plot(1:N, derivative_p3);

end 
function derivative = ComputeNumericalDerivative(r1,Psi1,r2,Psi2,d,uBody,R,step)
    Delta1 = DiscrimantFunction(r1,d,uBody,R,Psi1);
    Delta2 = DiscrimantFunction(r2,d,uBody,R,Psi2);
    derivative = (Delta2-Delta1)/step;
end

function E = DotProductConstraint(r,d,uBody,R,Psi)
    rL = Psi*d+r;
    rB = Psi'*r+d;
    alpha = asin(R/norm(rL));
    E = 1/R * (norm(rB)*cos(alpha)-rB'*uBody);
end 

function Delta = DiscrimantFunction(r,d,uBody,R,Psi)
    rB = Psi'*r+d;
    if rB'*uBody >= 0 % engine is pointed in the half plane pointed towards the target
        % Want constraint function be negative of Delta. 
        Delta = r'*r + 2*r'*Psi*d+d'*d-R^2 - (rB'*uBody)^2;
    else
        Delta = r'*r + 2*r'*Psi*d+d'*d-R^2; %% TESTING THIS!
    end
end 

function Psi = RotationMatrix(p,t,Omega)
    p2 = p'*p;
    pCross = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    temp = (1+p2)^2;
    
    % Translational dynamics are in a rotating reference frame
    DCM_BodyToInertial = eye(3) + (8*pCross*pCross - 4*(1-p2)*pCross)/temp;
    DCM_InertialToLVLH = [cos(Omega*t) sin(Omega*t) 0;
                          -sin(Omega*t) cos(Omega*t) 0;
                          0 0 1];
    Psi = DCM_InertialToLVLH*DCM_BodyToInertial;
end