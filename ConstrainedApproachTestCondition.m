function [problemParameters, solverParameters] = ConstrainedApproachTestCondition(caseNum,engineType,thrusterConfig)

% Author: Himmat Panag
% Description: Common code to define problem and solver parameters for
% constraint optimal transfer problems. Use this by adding different cases.
    
    % Default options 
    if nargin < 1, caseNum = 1; end 
    if nargin < 2, engineType = 1; end 
    if nargin < 3, thrusterConfig = THRUSTER_CONFIGURATION.CG_ALIGNED_6; end 

    problemParameters.dynamics.type = 'None';
    problemParameters.dynamics.regularizationMethod = 'HyperbolicTangentSmoothing';
    problemParameters.dynamics.finalAttitudeFree = false;
    problemParameters.dynamics.sundman.useTransform = false;
    problemParameters.dynamics.sundman.theta = 0.9;
    problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.NONE;

    solverParameters.initialCostateGuess = [];
    solverParameters.rho = 0.5;
    solverParameters.odeOptions = odeset('RelTol',1e-7,'AbsTol',1e-12); % ode
    solverParameters.fSolveOptions = optimoptions('fsolve','Display','iter','MaxFunEvals',1e3,...
        'MaxIter',3e1,'TolFun',1e-12,'TolX',1e-12,'StepTolerance',1e-12,...
        'UseParallel',true); % fsolve    
    
    solverParameters.finalRho = 1e-5;
    solverParameters.stateConvergeneTolerance = 1e-8;
    solverParameters.getTraj = true;         
 
    % Engine parameters
    switch engineType 
        case 1 % ION Givers uMax of .2mm/s^2, IAW advice from Aero Corp
            ThrustNewtons = .025; % 0.236; 
            Isp = 3200; 
        case 2 % Bipropellant scaled by crew dragon thrust to weight ratio. 
            % Crew dragon has 18 of these. Apollo service module has 12, 
            % 400N thrusters for a 5.5tonne vehicle. There is some 
            % leeway in design choice/redundancy here. These are RCS thrusters, 
            % likely wouldn't use a giant 40kN engine for RPOD at mm/sec speeds
            ThrustNewtons = 3;
            Isp = 300; 
        case 3 % Monopropellant
            ThrustNewtons = 2;
            Isp = 270; 
        case 4 % Cold gas
            ThrustNewtons = .33;
            Isp = 90; 
        case 5 % POPSAT-HIP1 1.1mN
            ThrustNewtons =  0.0011; % POPSAT-HIP1 1.1mN
            Isp = 43; %s
    end 

    m = 100; % 250; 
    g = 9.8;
    c = Isp*g;
    problemParameters.initialMass = m; 
    problemParameters.u_max = ThrustNewtons/m;
    problemParameters.dynamics.maxThrust = ThrustNewtons;
    problemParameters.dynamics.maxEngineFlowRate = problemParameters.u_max/c; 
    problemParameters.dynamics.exhaustVelocity = c; 
    problemParameters.dynamics.attitudeActuator = ATTITUDE_CONTROL_TYPE.NONE;

switch caseNum
    case {0,1,2,3,4}
        problemParameters.x0 = [.010;0;0;0;0;0];
        problemParameters.xf = [.004;0;0;0;0;0];    
        problemParameters.u_max = 0.05;        
        transferTime = 40;
        solverParameters.tSpan = linspace(0,transferTime,1000);
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

        % Parameters below taken from AE508 project, N. Ortalano paper
        w = 0.001060922896439; w2 = w^2; A = zeros(6,6); A(1:3,4:6) = eye(3); A(4,1) = 3*w2; A(4,5) = 2*w; A(5,4)=-2*w; A(6,3)=-w2;        
        problemParameters.dynamics.type = 'Linear';
        problemParameters.dynamics.A = A;        
        problemParameters.dynamics.frameRotationRate = w;
        problemParameters.x0 = [0;.010;0;0;0;0;m];
        problemParameters.xf = [0;.004;0;0;0;0];                
        problemParameters.dynamics.maxThrust = problemParameters.dynamics.maxThrust/1e3; % Convert to km
        problemParameters.u_max = problemParameters.u_max/1e3; % Convert to km
        problemParameters.dynamics.exhaustVelocity = problemParameters.dynamics.exhaustVelocity/1e3;  % Convert to km
        transferTime = 25*60;

        solverParameters.tSpan = [0,transferTime];
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
        [problemParameters, solverParameters] = ConstrainedApproachTestCondition(caseNum/10,engineType);
        problemParameters.dynamics.type = 'LinearNoMass';        
        problemParameters.x0 = problemParameters.x0(1:6); 
    case {1011,1012,1013,1014} % 6dof problems with similar conditions to the transfers in the JSR paper 
        [problemParameters, solverParameters] = ConstrainedApproachTestCondition(caseNum-1000,engineType,thrusterConfig);
        transferTime = 48;
        solverParameters.tSpan = [0, transferTime];

        [numEngines, locations, thrustDirection] = GetEngineLocations(thrusterConfig);
        problemParameters.dynamics.engineConfiguration = thrusterConfig;
        problemParameters.dynamics.numEngines = numEngines;
        problemParameters.dynamics.maxThrust = repmat(problemParameters.dynamics.maxThrust,numEngines,1);
        problemParameters.dynamics.engineLocationBody = locations/1e3;
        problemParameters.dynamics.thrustDirectionBody = thrustDirection;
        problemParameters.dynamics.inertia = diag([36,37,38]); % kg.km^2
        problemParameters.dynamics.inertiaInverse = inv(problemParameters.dynamics.inertia);
        problemParameters.dynamics.initialMass = problemParameters.initialMass;
        problemParameters.constraint.epsilon = 0.5;
        problemParameters.constraint.simplified = false;
        switch thrusterConfig
            case {THRUSTER_CONFIGURATION.CG_ALIGNED_6,THRUSTER_CONFIGURATION.CG_ALIGNED_CANTED_8}
            otherwise
                problemParameters.dynamics.attitudeActuator = ATTITUDE_CONTROL_TYPE.THRUSTERS;
        end

        problemParameters.p0 = [0;0;0];
        problemParameters.pf = [0;0;0];
        problemParameters.w0 = [0;0;0];
        problemParameters.wf = [0;0;0];
    case {1100,1101} % Attitude rotations only
        [problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011,engineType,thrusterConfig);
        problemParameters.xf(1:6) = problemParameters.x0(1:6);
        solverParameters.tSpan = [0, 20];
        solverParameters.initialCostateGuess = 1e-4.*[ 0.000000000000548   0.000000000009525  -0.000000000000206   0.000000000003950   0.000000000238142  -0.000000000017221   0.000194845229529 -0.037211240026859   0.000000000000000  -0.000000000000000  -0.893093076017197  -0.000000000000000  -0.000000000000002]';
        switch caseNum
            case 1100 % 60 degrees
                problemParameters.pf = [1;0;0]*tan(60*pi/180/4); % = n*tan(theta/4), where n is unit vector, theta is angle of rotation. 
            case 1101 % 180 degrees
                problemParameters.pf = [1;0;0]; % = n*tan(theta/4), where n is unit vector, theta is angle of rotation. 
        end
    case {2011,2012,2013,2014,2100,2101} % direct control torque 
        [problemParameters, solverParameters] = ConstrainedApproachTestCondition(caseNum-1000,engineType,thrusterConfig);
        problemParameters.dynamics.attitudeActuator = ATTITUDE_CONTROL_TYPE.CONTROL_TORQUE;
        problemParameters.dynamics.torqueCostMultiplier = 1; % typical fuel consumption should be < 1g. We want this multiplier to be 100x less than the fuel consumption
        problemParameters.dynamics.maxControlTorque = .5;
    case 2200 % Replicate results in R. Bommenna 2024, Scitech paper 
        [problemParameters, solverParameters] = ConstrainedApproachTestCondition(2011,5,thrusterConfig);
        problemParameters.x0 = [0;.10;0;0;0;0;50];
        problemParameters.xf = [0;0;0;0;0;0];   
        k = 1.771761600123494; I1 = 269.038*k; I2 = 268.912*k; I3 = 269.038*k;
        problemParameters.dynamics.inertia = diag([I1;I2;I3]); % kg.km^2
        problemParameters.dynamics.inertiaInverse = inv(problemParameters.dynamics.inertia);
        problemParameters.dynamics.torqueCostMultiplier = 1; 
        solverParameters.tSpan = [0, 80*60];
        [w,A] = GetCW_RotationRate(398600,6778);
        problemParameters.dynamics.A = A;        
        problemParameters.dynamics.frameRotationRate = w;
        problemParameters.dynamics.initialMass = 50; 
        solverParameters.rho = 9.550049507968265e-04; % k = 1.771761600123494;
        lam0guess = [0.000899424805008596	-6.76862240275099e-05	-2.07344652578406e-88	0.319437133775403	0.363529484608779	1.16014260940288e-84	-0.000135377674371161	3.13645966157262e-84	-2.44693986173122e-84	0.0529321450201394	3.65106073176034e-86	-9.63216174770913e-87	-0.00868811217900040]';
        solverParameters.initialCostateGuess = lam0guess;
        solverParameters.initialCostateGuess(8:10) = lam0guess(11:13); % p
        solverParameters.initialCostateGuess(11:13) = lam0guess(8:10); % omega
        %solverParameters.initialCostateGuess(21:23) = lam0guess(24:26); % lambdap %solverParameters.initialCostateGuess(24:26) = lam0guess(21:23); % lambdaw
    case 3000 % Reaction wheel formulations

end

end 

function [w,A] = GetCW_RotationRate(mu,rad)
        w = sqrt(mu./rad^3); w2 = w^2; 
        A = zeros(6,6); A(1:3,4:6) = eye(3); 
        A(4,1) = 3*w2; A(4,5) = 2*w; A(5,4)=-2*w; A(6,3)=-w2;
end