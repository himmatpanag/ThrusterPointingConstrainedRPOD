function [problemParameters, solverParameters] = ConstrainedApproachTestCondition(caseNum,engineType)

% Author: Himmat Panag
% Description: Common code to define problem and solver parameters for
% constraint optimal transfer problems. Use this by adding different cases.
    
    % Default options 
    if nargin < 1, caseNum = 1; end 
    if nargin < 2, engineType = 1; end 
    problemParameters.dynamics.type = 'None';
    problemParameters.dynamics.regularizationMethod = 'HyperbolicTangentSmoothing';
    problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.NONE;

    solverParameters.initialCostateGuess = [];
    solverParameters.rho = 0.5;
    solverParameters.odeOptions = odeset('RelTol',1e-10,'AbsTol',1e-12); % ode
    solverParameters.fSolveOptions = optimoptions('fsolve','Display','iter','MaxFunEvals',1e3,...
        'MaxIter',3e2,'TolFun',1e-12,'TolX',1e-9,...
        'UseParallel',true); % fsolve    
    
    solverParameters.finalRho = 1e-5;
    solverParameters.stateConvergeneTolerance = 1e-8;
    solverParameters.getTraj = true;         
 
    % Engine parameters
    switch engineType 
        case 1 % ION Givers uMax of .2mm/s^2, IAW advice from Aero Corp
            ThrustNewtons = .025; % 0.236; 
            m = 100; % 250; 
            Isp = 3200; 
        case 2 % Bipropellant scaled by crew dragon thrust to weight ratio. Crew dragon has 18 of these. Apollo service module has 12, 400N thrusters for a 5.5tonne vehicle. There is clearly some leeway in design choice/redundancy here. These are RCS thrusters, likely wouldn't use a giant 40kN engine for RPOD at mm/sec speeds
            ThrustNewtons = 3;
            m = 100; % 250; 
            Isp = 300; 
        case 3 % Monopropellant
            ThrustNewtons = 2;
            m = 100; % 250; 
            Isp = 270; 
        case 4 % Cold gas
            ThrustNewtons = .33;
            m = 100; % 250; 
            Isp = 90; 
    end 

    g = 9.8;
    c = Isp*g;
    problemParameters.initialMass = m; 
    problemParameters.u_max = ThrustNewtons/m;
    problemParameters.dynamics.maxThrust = ThrustNewtons;
    problemParameters.dynamics.maxEngineFlowRate = problemParameters.u_max/c; 
    problemParameters.dynamics.exhaustVelocity = c; 

switch caseNum
    case {0,1,2,3,4}
        problemParameters.x0 = [.010;0;0;0;0;0];
        problemParameters.xf = [.004;0;0;0;0;0];    
        problemParameters.u_max = 0.05;        
        problemParameters.transferTime = 40;
        solverParameters.tSpan = linspace(0,problemParameters.transferTime,1000);
        switch caseNum
            case 0 % No constraint
                problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.NONE;
            case 1 % No Dynamics, no angle variation 
                problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE;
                problemParameters.constraint.alpha0 = 10*pi/180; 
        
            case 2 % No dynamics, make constraint stricter as you approach the origin
                problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE; % alpha0/(|r|+1)^j
                alpha0 = pi/4; j = 1;
                problemParameters.constraint.alpha0 = pi/4;
                problemParameters.constraint.angleFunc = @(rNorm) alpha0/((rNorm+1)^j);
                problemParameters.constraint.angleFuncDerivative = @(r) -j.*alpha0 .* r./((norm(r)+1)^(j+1));
        
            case 3 % no dynamics, spherical constraint function 
                % make constraint stricter as you approach the origin (mimic spherical target)
                problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE; 
                R = 1;         
                problemParameters.constraint.angleFunc = @(rNorm) asin(R/rNorm);
                problemParameters.constraint.angleFuncDerivative = @(r) -R.*r./(r'*r*sqrt(r'*r-R^2));        
                problemParameters.constraint.targetRadius = R;
        
        end
    case {11,12,13,14}
       % Clohessy Wiltshire linear dynamics dockings parmeters
        %mu = 398600.4415; % km2/s2
        %radEarth = 6378.1363; alt = 550; rad = radEarth+alt;
        %w = sqrt(mu./rad^3); w2 = w^2; 
        %A = zeros(6,6); A(1:3,4:6) = eye(3); A(4,1) = 3*w2; A(4,5) = 2*w; A(5,4)=-2*w; A(6,3)=-w2;

        % Parameters below taken from AE508 project, N. Ortalao paper?
        w = 0.001060922896439; w2 = w^2; A = zeros(6,6); A(1:3,4:6) = eye(3); A(4,1) = 3*w2; A(4,5) = 2*w; A(5,4)=-2*w; A(6,3)=-w2;        
        problemParameters.dynamics.A = A;    

        problemParameters.dynamics.type = 'Linear';
        problemParameters.dynamics.A = A;        
        problemParameters.x0 = [0;.010;0;0;0;0;m];
        problemParameters.xf = [0;.004;0;0;0;0];                
        problemParameters.dynamics.maxThrust = problemParameters.dynamics.maxThrust/1e3; % Convert to km
        problemParameters.u_max = problemParameters.u_max/1e3; % Convert to km
        problemParameters.dynamics.exhaustVelocity = problemParameters.dynamics.exhaustVelocity/1e3;  % Convert to km
        problemParameters.transferTime = 25*60;

        solverParameters.tSpan = [0,problemParameters.transferTime];
        % rVec = problemParameters.x0(1:3)-problemParameters.xf(1:3); rVec = rVec/norm(rVec);
        %solverParameters.initialCostateGuess = 1e-3*[rand(3,1);rVec;rand(1,1)];
        solverParameters.initialCostateGuess = 1e-3*rand(7,1);

        switch caseNum
            case 11 % Clohessy wiltshire dynamics
            case 12
                problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE;
                problemParameters.constraint.alpha0 = 10*pi/180; 
            case 13
                problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE; 
                R = 1/1e3; % convert to km
                problemParameters.constraint.angleFunc = @(rNorm) asin(R/rNorm);
                problemParameters.constraint.angleFuncDerivative = @(r) -R.*r./(r'*r*sqrt(r'*r-R^2));        
                problemParameters.constraint.targetRadius = R;
            case 14                
                problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE; 
                R = 2/1e3; % convert to km
                problemParameters.constraint.angleFunc = @(rNorm) asin(R/rNorm);
                problemParameters.constraint.angleFuncDerivative = @(r) -R.*r./(r'*r*sqrt(r'*r-R^2));        
                problemParameters.constraint.targetRadius = R;
                problemParameters.dynamics.regularizationMethod = 'L2_Norm';
        end 
    case {110,120,130,140}
        [problemParameters, solverParameters] = ConstrainedApproachTestCondition(caseNum/10);
        problemParameters.dynamics.type = 'LinearNoMass';        
        problemParameters.x0 = problemParameters.x0(1:6);        

end

end 