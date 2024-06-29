function PlotsForSphericalTargetPaper()
% Function generates plots for the spherical target RPOD in LEO 
parallelFSolveOptions = optimoptions('fsolve','Display','iter','MaxFunEvals',1e3,...
    'MaxIter',3e2,'TolFun',1e-12,'TolX',1e-9,...
    'UseParallel',true); % fsolve    

%% Diagram showing the set of feasible control regions
r = 2;
saveDir = [pwd,'/Resuls'];%Users/himmatpanag/Documents/UIUC/Papers/2024 IEEE/JSR/';
od = pwd; 
hFig = figure("Name",'ConstraintSet','Units','normalized','Position',[0.1689 0.3090 0.2692 0.4993]);
Plot3D_PointingConstraintSphericalTarget(gca,[3.8;0;0],r);
hFig.Children(2).View = [19.1976, 8.0594];

%% Linear (HCW) Dynamics, unconstrained
%engineType = 1; % ION
engineType = 2; % BIPROPELLANT
%engineType = 3; % MONOPROPELLANT
[problemParameters, solverParameters] = ConstrainedApproachTestCondition(11,engineType);
solution.solutionFound = false; 
solverParameters.initialCostateGuess =  [-0.000061909304126   1.683061783958727  -0.000000000000181  -1.449486254206033  40.355191913670460  -0.000000000007311   0.000100989532481]';
problemParameters.transferTime = 48;
solverParameters.tSpan = [0, problemParameters.transferTime];

while any(~solution.solutionFound)
    solverParameters.initialCostateGuess = [rand(3,1)/1e3;0;1;0;1]*1e-1;

    solution = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
    % unconstrainedInitialRho = PlotSolution.summary(solution);
end 
UnconstrainedShortTimeRhoSweep = SweepSolutions(solution,'rho',1e-4,true);
PlotSolution.SweepShort(UnconstrainedShortTimeRhoSweep)

%% Add cosntant constraint angle
solverParameters = UnconstrainedShortTimeRhoSweep(1).solverParameters;
problemParameters = UnconstrainedShortTimeRhoSweep(1).problemParameters;
solverParameters.rho = 0.5; 
problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE;
problemParameters.constraint.alpha0 = 2*pi/180;
sol2degrhoLarge = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
PlotSolution.summary(sol2degrhoLarge)

% angle sweep to 60 deg
angleSweep2To30LargeRho = SweepSolutions(sol2degrhoLarge,'Angle',[2:5:27,30]*pi/180);
angleSweepTo40LargeRho = SweepSolutions(angleSweep2To30LargeRho(end),'Angle',[30:5:60]*pi/180);
angleSweepTo60LargeRho = SweepSolutions(angleSweepTo40LargeRho(end),'Angle',[40:.5:60]*pi/180);
angleSweepTo89LargeRho = SweepSolutions(angleSweepTo60LargeRho(end),'Angle',[60:5:89]*pi/180);
angleSweepTo90LargeRho = SweepSolutions(angleSweepTo89LargeRho(end),'Angle',[89:3:90]*pi/180);
angleSweepTo90LargeRho = SweepSolutions(angleSweepTo89LargeRho(end),'Angle',[89:.1:100]*pi/180);

%% Get 40 and 60 Deg solutions
sol40Deg = SweepSolutions(angleSweepTo40LargeRho(end),'rho',1e-4);
sol60DegLargeRho = angleSweepTo60LargeRho(end);
sol60DegLargeRho.constraint.alpha0 = 60*pi/180;
sol60Deg = SweepSolutions(sol60DegLargeRho,'rho',1e-4);

PlotSolution.Sweep(angleSweepTo40LargeRho,'Angle')
PlotSolution.Sweep(angleSweepTo60LargeRho,'Angle')
PlotSolution.Sweep(angleSweepTo89LargeRho,'Angle')
PlotSolution.Sweep(angleSweepTo89LargeRho(20:30),'Angle')

constraint85DegRhoSweep = SweepSolutions(angleSweepTo89LargeRho(27),'rho',1e-4,true);
PlotSolution.summary(constraint85DegRhoSweep(end))
PlotSolution.Sweep(constraint85DegRhoSweep)

PlotSolution.Sweep(angleSweepTo90LargeRho,'Angle')
constraint90DegRhoSweep = SweepSolutions(angleSweepTo90MediumRho(end),'rho',1e-4,true);
PlotSolution.Sweep(constraint90DegRhoSweep)
PlotSolution.summary(constraint90DegRhoSweep(end))

rhoSweepAngle20 = SweepSolutions(angleSweep2To30LargeRho(end-9),'rho',1e-4,true);
numTraj = 12;
constantConstraintAngleRhoSweep(1,1:numTraj) = rhoSweepAngle20(floor(linspace(1,min(numel(rhoSweepAngle20),numTraj),numTraj)));
constantConstraintAngleRhoSweep(2,1:numTraj) = constraint40DegRhoSweep(floor(linspace(1,min(numel(constraint40DegRhoSweep),numTraj),numTraj)));
constantConstraintAngleRhoSweep(3,1:numTraj) = constraint60DegRhoSweep(floor(linspace(1,min(numel(constraint60DegRhoSweep),numTraj),numTraj)));
constantConstraintAngleRhoSweep(4,1:numTraj) = constraint80DegRhoSweep(floor(linspace(1,min(numel(constraint80DegRhoSweep),numTraj),numTraj)));
constantConstraintAngleRhoSweep(5,1:numTraj) = constraint85DegRhoSweep(floor(linspace(1,min(numel(constraint85DegRhoSweep),numTraj),numTraj)));
constantConstraintAngleRhoSweep(6,1:numTraj) = constraint90DegRhoSweep(floor(linspace(1,min(numel(constraint90DegRhoSweep),numTraj),numTraj)));

%% Comparison to direct method, constant angle constraint
engineType = 2; % BIPROPELLANT
[problemParameters, solverParameters] = ConstrainedApproachTestCondition(11,engineType);
problemParameters.transferTime = 48;
problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE;
problemParameters.constraint.alpha0 = 2*pi/180;

CVX_Params = CW_RPO_TestCondition;
CVX_Params = WriteOCP_ParamsToCVX(CVX_Params,problemParameters);
CVX_Params.numSteps = 200;

[eta, x, u] = OptimalApproachTrajCW(CVX_Params);
CVX_SolUnconstrained = WriteCVX_SolStruct(x,eta,u,CVX_Params);
PlotTrajectorySummary(eta,x,u,CVX_Params,figure);

CVX_Params.thrustPointingConstraint.prevTraj = x; 
CVX_Params.thrustPointingConstraint.prevControl = u;
CVX_Params.thrustPointingConstraint.prevEta = eta;

CVX_Params.thrustPointingConstraint.active = true;
CVX_Params.thrustPointingConstraint.useLinearApprox = true; 
CVX_Params.trustRegion.useTrustRegion = false;
CVX_Params.trustRegion.eta = 0;

time = linspace(0,CVX_Params.simTimeHours*3600,CVX_Params.numSteps);

constraintAngle = 40*pi/180;
CVX_Params.thrustPointingConstraint.angle = constraintAngle;
iterTol = 1e-8;
N=6;
for ii = 1:N
    fprintf('Starting iteration %d\n',ii)
    CVX_Params.thrustPointingConstraint.prevTraj = x; 
    CVX_Params.thrustPointingConstraint.prevControl = u;
    CVX_Params.thrustPointingConstraint.prevEta = eta;
    [eta, x, u] = OptimalApproachTrajCW(CVX_Params);
    if max(vecnorm(CVX_Params.thrustPointingConstraint.prevTraj(1:3,:)-x(1:3,:))) < iterTol, break, end 
    CVX_Params.trustRegion.useTrustRegion = true;
end
CVX_Sol40Deg = WriteCVX_SolStruct(x,eta,u,CVX_Params);

constraintAngle = 60*pi/180;
CVX_Params.thrustPointingConstraint.angle = constraintAngle;
for ii = 1:N
    fprintf('Starting iteration %d\n',ii)
    CVX_Params.thrustPointingConstraint.prevTraj = x; 
    CVX_Params.thrustPointingConstraint.prevControl = u;
    CVX_Params.thrustPointingConstraint.prevEta = eta;
    [eta, x, u] = OptimalApproachTrajCW(CVX_Params);
    if max(vecnorm(CVX_Params.thrustPointingConstraint.prevTraj(1:3,:)-x(1:3,:))) < iterTol, break, end 
end
CVX_Sol60Deg = WriteCVX_SolStruct(x,eta,u,CVX_Params);

constraintAngle = 70*pi/180;
CVX_Params.thrustPointingConstraint.angle = constraintAngle;
for ii = 1:N
    fprintf('Starting iteration %d\n',ii)
    CVX_Params.thrustPointingConstraint.prevTraj = x; 
    CVX_Params.thrustPointingConstraint.prevControl = u;
    CVX_Params.thrustPointingConstraint.prevEta = eta;
    [eta, x, u] = OptimalApproachTrajCW(CVX_Params);
    if max(vecnorm(CVX_Params.thrustPointingConstraint.prevTraj(1:3,:)-x(1:3,:))) < iterTol, break, end 
end
CVX_Sol70Deg = WriteCVX_SolStruct(x,eta,u,CVX_Params);

constraintAngle = 80*pi/180;
CVX_Params.thrustPointingConstraint.angle = constraintAngle;
for ii = 1:N
    fprintf('Starting iteration %d\n',ii)
    CVX_Params.thrustPointingConstraint.prevTraj = x; 
    CVX_Params.thrustPointingConstraint.prevControl = u;
    CVX_Params.thrustPointingConstraint.prevEta = eta;
    [eta, x, u] = OptimalApproachTrajCW(CVX_Params);
    if max(vecnorm(CVX_Params.thrustPointingConstraint.prevTraj(1:3,:)-x(1:3,:))) < iterTol, break, end 
end
CVX_Sol80Deg = WriteCVX_SolStruct(x,eta,u,CVX_Params);

%% Plots to include and comments. 
% 40 degrees - requires out of plane acceleration to maintain constraint and full throttle,
% not possible to compensate with an out of plane acceleration unless the
% step size is very small. Results in the constraint being violated for a
% small period (while the control sweeps through the target) and the
% control stays mostly in plane. 

% 60 degrees - Smaller violation of constraint as the single continuous
% burn starts to split into two discrete burns.

% 70 degrees - the final burn is split into two burns (in plane) which the
% cvx method approximates well. As the constraint angle is increased beyond
% this, more successsive approxiamtions are required for convergence. 
    
% 80 degrees - able to converge if you step through 70deg. The differing 

alphaStr = ['alpha = ',num2str(angles(jj)),' deg'];
[hFig1,hFig2,hFig3,hFig4,hFig5,hFig6] = CreateFigures(figVis, alphaStr);
iterTol = 1e-8; % .1cm

[eta, x, u] = OptimalApproachTrajCW(CVX_Params);
N=6;
for ii = 1:N
    fprintf('Starting iteration %d\n',ii)
    CVX_Params.thrustPointingConstraint.prevTraj = x; 
    CVX_Params.thrustPointingConstraint.prevControl = u;
    CVX_Params.thrustPointingConstraint.prevEta = eta;
    [eta, x, u] = OptimalApproachTrajCW(CVX_Params);
    
    strName = ['Iter',num2str(ii)];
    PlotTrajectorySummary(eta,x,u,CVX_Params,hFig1);
    PlotTrajXY(time,x,hFig4,strName);
    PlotThrustPointingConstraint(time, x,u,CVX_Params.thrustPointingConstraint.angle,CVX_Params.aMax, strName, hFig6);
    PlotChangeInSolution(time,x,eta, CVX_Params,strName,hFig5);

    if max(vecnorm(CVX_Params.thrustPointingConstraint.prevTraj(1:3,:)-x(1:3,:))) < iterTol
        break            
    end 

end
fprintf('Converged after %d iterations\n',ii)

hfConvexComparison = figure(Name='ConvexComparison',Units='normalized',Position=[0.3175 0.2138 0.5813 0.6670]); 
solution = constantConstraintAngleRhoSweep(2,end);
% ii = ii+1; if ii==3, for jj=1:2, ax=subplot(2,2,jj); ax.ColorOrderIndex = ax.ColorOrderIndex + 1; xlabel('Time (s)'); end, end 
ax1 = subplot(2,2,1); grid on; hold on; titles = {'Indirect x','Indirect y','Indirect z'};
titles2 = {'Convex x','Convex y','Convex z'};
solution.x(:,1:6) = solution.x(:,1:6)*1e3;
for ii = 1:3, plot(solution.t,solution.x(:,ii),'-','LineWidth',1,'DisplayName',titles{ii}); 
    ReduceColorOrderIndex(ax1); plot(time,CVX_Sol40Deg.x(:,ii),'--','LineWidth',2,'DisplayName',titles2{ii});
end 
title('Trajectory'); ylabel('Position (m)'); legend('show','Location','west'); xlabel('Time (s)');

PlotSolution.ThrustProfile(solution, subplot(2,2,2),'Indirect');
PlotSolution.ThrustProfile(CVX_Sol40Deg, subplot(2,2,2),'Convex40Deg',true);
annotation("textarrow",[.79,.84],[.8,.8],'Color','r')
legend('show','Location','south');
% Plot mini box
ax2 = axes('Position',[.64 .7 .15 .2]); box on; grid on; hold on;
idx1 = find(solution.t>41,1); idx2 = find(solution.t>42.8,1);
plot(solution.t(idx1:idx2), solution.throttle(idx1:idx2),'LineWidth',2)
idx1 = find(time>41,1); idx2 = find(time>42.8,1);
stairs(time(idx1:idx2), CVX_Sol40Deg.throttle(idx1:idx2),'LineWidth',2)

ax3 = subplot(2,2,3:4); grid on; hold on; 
throttleOn1 = solution.throttle > 0.1; 
angsAll = 90-solution.phi*180/pi;
angsAll(~throttleOn1) = 0;
plot(solution.t,angsAll,'--','LineWidth',2.5);

throttleOn2 = CVX_Sol40Deg.throttle > 0.1; 
angs = CVX_Sol40Deg.thrustDirAngle;
angs(~throttleOn2) = 0; 
stairs(CVX_Sol40Deg.t,angs,'LineWidth',2)

t1 = 44; t2 = 46;
axes(ax3); rectangle('Position', [44, 5, 3, 40], 'EdgeColor', 'r', 'LineWidth', 2); 
legend({'Indirect','Convex'});
title('Plume angle from target');xlabel('Time (s)'); ylabel('Angle (deg)');
% Plot mini box
ax2 = axes('Position',[.4 .2 .3 .2]); box on; grid on; hold on;
axes(ax2)
idx1 = find(solution.t>t1,1); idx2 = find(solution.t>t2,1);
plot(solution.t(idx1:idx2), angsAll(idx1:idx2),'--','LineWidth',2.5)
idx1 = find(time>t1,1); idx2 = find(time>t2,1);
stairs(time(idx1:idx2), angs(idx1:idx2),'LineWidth',2)

% Compare z component of control vector
hFigZComponent = figure('Name','ConvexControlZComponent'); grid on; hold on; 
plot(solution.t,solution.uDir(:,3).*0.03,'LineWidth',2,'DisplayName','Indirect'); % 3m/s^2
stairs([CVX_Sol40Deg.t, 48],[CVX_Sol40Deg.u(3,:), CVX_Sol40Deg.u(3,end)],'LineWidth',2,'DisplayName','Convex')
ylim([-20e-3,1e-3]); legend('show',Location='north');
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)')
t1 = 43; t2 = 49;
ax2 = axes('Position',[.2 .2 .5 .5]); box on; grid on; hold on;
idx1 = find(solution.t>t1,1); idx2 = find(solution.t>t2,1); if isempty(idx2),idx2 = numel(solution.t); end
plot(solution.t(idx1:idx2),solution.uDir((idx1:idx2),3).*0.03,'LineWidth',2); % 3m/s^2
hold on; idx1 = find(time>t1,1); idx2 = find(time>t2,1); if isempty(idx2),idx2 = numel(CVX_Sol40Deg.t); end
axes(ax2); stairs([CVX_Sol40Deg.t(idx1:idx2), 48],[CVX_Sol40Deg.u(3,(idx1:idx2)), CVX_Sol40Deg.u(3,end)],'LineWidth',2)
ylim([-2e-6,1e-6]);

% Compare multiple convex and indirect methods
hfConvexSweep = figure('Name','ConvexSweep'); grid on; hold on; ax1 = gca;
xlabel('Time (s)'); ylabel('Angle (deg)');
solutions = {UnconstrainedShortTimeRhoSweep(end), constantConstraintAngleRhoSweep(2,end),...
    constantConstraintAngleRhoSweep(3,end), constantConstraintAngleRhoSweep(4,end)};
convexSols = {CVX_SolUnconstrained, CVX_Sol40Deg, CVX_Sol60Deg, CVX_Sol80Deg};
ax2 = axes('Position',[.34 .6 .25 .3]); box on; grid on; hold on;
namesSol = {'Indirect Free','Indirect 40deg','Indirect 60deg','Indirect 80deg'};
convexNames = {'Convex Free','Convex 40deg','Convex 60deg','Convex 80deg'};
for ii = 1:4
    solution = solutions{ii};
    CVX_Sol = convexSols{ii};
    
    throttleOn1 = solution.throttle > 0.1; 
    angsAll = 90-solution.phi*180/pi;
    angsAll(~throttleOn1) = 0;
    axes(ax1);
    plot(solution.t,angsAll,'--','LineWidth',2,'DisplayName',namesSol{ii});
    ReduceColorOrderIndex(gca)
    throttleOn2 = CVX_Sol.throttle > 0.1; 
    angs = CVX_Sol.thrustDirAngle;
    angs(~throttleOn2) = 0; 
    stairs(CVX_Sol.t,angs,'LineWidth',1,'DisplayName',convexNames{ii})

    axes(ax2); 
    t1 = 1; t2 = 8;
    idx1 = find(solution.t>t1,1); idx2 = find(solution.t>t2,1);
    plot(solution.t(idx1:idx2), angsAll(idx1:idx2),'--','LineWidth',2)
    idx1 = find(CVX_Sol.t>t1,1); idx2 = find(CVX_Sol.t>t2,1);
    ReduceColorOrderIndex(ax2);
    stairs(CVX_Sol.t(idx1:idx2), angs(idx1:idx2),'LineWidth',1)
end 

ylim([145,160]);
axes(ax1); legend('show'); 
rectangle('Position', [1, 140, 7, 25], 'EdgeColor', 'r', 'LineWidth', 2); 
axes(ax2);

%% Spherical constraint
problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE;
problemParameters = UpdateSphereCircleRadius(problemParameters,.5/1000);
solution.solutionFound = false; 
solverParameters.initialCostateGuess = constraint40DegRhoSweep(1).newCostateGuess;
problemParameters.dynamics.regularizationMethod = 'HyperbolicTangentSmoothing';
sphericalTargetRadius0_5_RhoLarge = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);

sphericalTargetRadius0_5_To2RhoLarge = SweepSolutions(sphericalTargetRadius0_5_RhoLarge,'Radius',[.5:.5:2]/1000);
for ii = 1:numel(sphericalTargetRadius0_5_To2RhoLarge)
    temp = SweepSolutions(sphericalTargetRadius0_5_To2RhoLarge(ii),'rho',1e-4);
    sphericalTargetSmallRadiusSweepSmallRho(1:15,ii) = temp;
end

PlotSolution.summary(sphericalTargetRadius0_5_To2RhoLarge(end));
sphericalTargetRadius2_RhoSweep = SweepSolutions(sphericalTargetRadius0_5_To2RhoLarge(end),'rho',1e-4);
PlotSolution.summary(sphericalTargetRadius2_RhoSweep(end));
PlotSolution.Sweep(sphericalTargetRadius2_RhoSweep,'rho');

sphericalTargetRadiusSweepLargeRho = SweepSolutions(sphericalTargetRadius0_5_To2RhoLarge(end),'Radius',[2:.2:3.6]/1000);
PlotSolution.Sweep(sphericalTargetRadiusSweepLargeRho,'Radius');

for ii = 1:numel(sphericalTargetRadiusSweepLargeRho)
    temp = SweepSolutions(sphericalTargetRadiusSweepLargeRho(ii),'rho',1e-4);
    sphericalTargetRadiusSweepSmallRho(1:15,ii) = temp;
end

sphericalTargetRadiusLargestSweepSmallRho = SweepSolutions(sphericalTargetRadiusSweepSmallRho(end,end),'Radius',[3.6:.05:3.9]/1000);

sphericalTargetRadius3_4_RhoSweep = SweepSolutions(sphericalTargetRadiusSweepLargeRho(end),'rho',1e-4);
PlotSolution.Sweep(sphericalTargetRadius3_4_RhoSweep,'rho');

sphericalTargetRadius3_87_RhoSweep = SweepSolutions(sphericalTargetRadiusSweepLargestRho(end-5),'rho',1e-4);
PlotSolution.Sweep(sphericalTargetRadius3_87_RhoSweep,'rho');
PlotSolution.summary(sphericalTargetRadius3_87_RhoSweep(end))

sphericalTargetRadius3_9_RhoSweep = SweepSolutions(sphericalTargetRadiusSweepLargestRho(end-2),'rho',1e-4);

% Try again with l2 norm regularization
t = sphericalTargetRadius3_4_RhoSweep(1);
t.problemParameters.dynamics.regularizationMethod = 'L2_Norm';
t=RerunSolution(t);
sphericalTargetRadius3_4_RhoSweep_L2Norm = SweepSolutions(t,'rho',1e-5);
PlotSolution.summary(sphericalTargetRadius3_4_RhoSweep(end));
PlotSolution.summary(sphericalTargetRadius3_4_RhoSweep_L2Norm(end));

% Try to get to R=4 again 
testR4 = SweepSolutions(sphericalTargetRadiusSweepLargestRho(end),'Radius',[3.9:.01:3.95]/1000);
sphericalR3_9_rhoSweep = SweepSolutions(testR4(1),'rho',1e-4);
sphericalR4_rhoFinal = SweepSolutions(sphericalTargetRadius3_9_RhoSweep(end),'Radius',[3.95:.01:3.97,3.975:.005:3.995]/1000);

%% PLOTS IN PAPER
saveFigFcn(hfConvexSweep,saveDir);
saveFigFcn(hfConvexComparison,saveDir);
saveFigFcn(hFigZComponent,saveDir);

hf1 = PlotSolution.summaryShort(UnconstrainedShortTimeRhoSweep(end));
hf1.Position=[1000         812         870         525];
hf1.Children(1).Position = [ 0.750070859456635   0.408698781838317   0.142528735632184   0.036190476190476];
hf1.Name = 'unconstrained';
saveFigFcn(hf1,saveDir)

hf2 = figure(Name='trajAndPlumes'); 
ax = gca;grid on; hold on; 
solution = UnconstrainedShortTimeRhoSweep(end);
solution.x(:,1:6)=solution.x(:,1:6)*1e3;
plot3(solution.x(:,1), solution.x(:,2), solution.x(:,3), 'LineWidth',2, 'DisplayName',['Unconstrained']);  
PlotSolution.addThrustDirection(solution,gca,true)
for ii = [2,3,4,6]
    sol = constantConstraintAngleRhoSweep(ii,end);
    sol.x(:,1:6)=sol.x(:,1:6)*1e3;
   plot3(sol.x(:,1),sol.x(:,2), sol.x(:,3), ...
       'LineWidth',2, 'DisplayName',['\alpha = ', num2str(round(sol.problemParameters.constraint.alpha0*180/pi)), ' deg']); 
    PlotSolution.addThrustDirection(sol,gca,true)
end
plot3(0,0,0,'r.','MarkerSize',100,'DisplayName','Target Spacecraft')
plot3(0,10,0,'b.','MarkerSize',60,'DisplayName','x_0 Chaser Initial')
plot3(0,4,0,'k.','MarkerSize',60,'DisplayName','x_f Chaser Final')
legend('show','Location','best')
axis equal, axis tight; xlabel('R-bar (m)'), ylabel('V-bar (m)');
xlim([-1,4]), ylim([-.5,11])
ax.View = [-90,90];
hf2.Position = [113         219        1273         535];
saveDir = '/Users/himmatpanag/Documents/UIUC/Papers/2024 IEEE/JSR/';
saveFigFcn(hf2,saveDir)

hf3 = figure(Name='throttleAndPlumeAngleRotationSpeedComparison'); 
ii = 0;
for solution = [UnconstrainedShortTimeRhoSweep(end),constantConstraintAngleRhoSweep([2,4,6],end)']
    ii = ii +1; if ii==3, for jj=1:2, ax=subplot(1,2,jj); ax.ColorOrderIndex = ax.ColorOrderIndex +1; xlabel('Time (s)'); end, end 
    solution.x(:,1:6) = solution.x(:,1:6)*1e3;
    PlotSolution.ThetaPhi(solution,subplot(1,2,1),subplot(1,2,2))
end
for jj = 1:2, subplot(1,2,jj); xlabel('Time (s)'); end
hf3.Position = [512   334   891   501];
legend('show','Location','best');
saveFigFcn(hf3,saveDir)

hf4 = figure(Name='hyperbolicsmoothing');
for solution = constantConstraintAngleRhoSweep(4,[1,3,5,9,12])
    PlotSolution.ThrustProfile(solution,gca,['\rho=',num2str(solution.rho)]);
end
summary = GetSolutionSummary(solution);
xlabel('Time (s)'); ylabel('Throttle \delta^*'); title('');
legend('show','Location','best')
saveFigFcn(hf4,saveDir)

hf5 = figure("Name",'masscons');
for solution = [UnconstrainedShortTimeRhoSweep(end),constantConstraintAngleRhoSweep([2,3,4,6],end)']
    PlotSolution.MassConsumption(solution,gca);
end
xlabel('Time (s)'); title('');
saveFigFcn(hf5,saveDir)

hf6 = figure("Name",'constraintConeActive');
sol = constantConstraintAngleRhoSweep(2,end); grid on; hold on;
sol.x(:,1:6)=sol.x(:,1:6)*1e3;
plot3(sol.x(:,1),sol.x(:,2), sol.x(:,3), ...
   'LineWidth',2, 'DisplayName',['\alpha = ', num2str(round(sol.problemParameters.constraint.alpha0*180/pi)), ' deg']); 
PlotSolution.ThrustConeDirection(sol,gca,false,1)
plot3(0,0,0,'r.','MarkerSize',100,'DisplayName','Target Spacecraft')
plot3(0,10,0,'b.','MarkerSize',60,'DisplayName','x_0 Chaser Initial')
plot3(0,4,0,'k.','MarkerSize',60,'DisplayName','x_f Chaser Final')
axis equal, axis tight; 
xlabel('R-bar (m)'), ylabel('V-bar (m)');  zlabel('H-bar (m)')
ax = gca; ax.View = [45  20];
t = gcf; 
%t.Children(1).Location = 'northoutside';
t.Children(1).Label.String = 'Time (s)';
% title('Fuel optimal trajectory and engine plume directions for \alpha = 40 deg')
t.Position = [882         548        1152         603];
legend('show','Location','best'); 
saveFigFcn(hf6,saveDir)

hf7 = figure(Name='fuelConsumption'); grid on; hold on; 
constraintAngles = [0,20,40,60,80,85,90];ii=0;
for solution = [UnconstrainedShortTimeRhoSweep(end),constantConstraintAngleRhoSweep(:,end)']
    ii = ii+1;
    fuelConsumption(ii) = -(solution.x(end,7)-solution.x(1,7))*1e3;
end
plot(constraintAngles,fuelConsumption,'-*','LineWidth',2);
xlabel('Constraint Angle (degrees)'); ylabel('Fuel consumption (g)')
% title('Total fuel consumption versus thruster constraint angle')
saveFigFcn(hf7,saveDir)

%% Spherical plots
hsph1 = figure(Name='targetrad3_9'); ii = 0;
for sol = [sphericalTargetRadius2_RhoSweep(end), sphericalTargetRadius2_RhoSweep(end), ...
        sphericalTargetRadius3_9_RhoSweep(end),sphericalTargetRadius3_9_RhoSweep(end)]
    ii = ii+1;
    ax = subplot(2,2,ii); grid on; hold on; 
    sol.x(:,1:6)=sol.x(:,1:6)*1e3;
    plot3(sol.x(:,1),sol.x(:,2), sol.x(:,3), ...
       'LineWidth',2, 'DisplayName','Trajectory'); 
    PlotSolution.ThrustConeDirection(sol,gca,false,1)
    plot3(0,0,0,'r.','MarkerSize',100,'DisplayName','Target Spacecraft')
    plot3(0,10,0,'b.','MarkerSize',60,'DisplayName','x_0 Chaser Initial')
    plot3(0,4,0,'k.','MarkerSize',60,'DisplayName','x_f Chaser Final')
    axis equal, axis tight; 
    xlabel('R-bar (m)'), ylabel('V-bar (m)');  zlabel('H-bar (m)')
    s = PlotSphereConstraint(ax,sol.problemParameters.constraint.targetRadius*1e3,[0;0;0]);
    set(s,'EdgeAlpha',0);
    title([num2str(sol.problemParameters.constraint.targetRadius*1e3),'m spherical target constraint']);
    if ii == 1
        t = gcf; 
        ax.View = [4.8,6.2];
        t.Children(1).Location = 'southoutside';
        t.Children(1).Label.String = 'Time (s)';
        t.Position = [ 1020         157        1152         971];
        legend('show','Location','best')
        title(['                                                                                                  ',ax.Title.String]);
    else 
        delete(t.Children(1));
        ax.View = [ 109.9010   26.9764];
    end
    if ii==2
        ax.View = [101.8663   42.5437];
        title('');
    elseif ii==3
        ax.View = [11,10];
        title(['                                                                                                  ',ax.Title.String]);
    elseif ii==4
        title('')
    end 
end
saveFigFcn(hsph1,saveDir)

hsph2 = figure(Name='sphericalMassCon'); grid on; hold on; 
ii=0; clear fuelConsumption; clear targetRadii
for solution = [sphericalTargetRadiusSweepSmallRho(end,:),sphericalTargetRadiusLargestSweepSmallRho(end,[2:2:14]),sphericalR4_rhoFinal(2:2:10)]%sphericalTargetRadius3_9_RhoSweep(end),sphericalTargetRadius3_9_RhoSweep(end)]
    ii = ii+1;
    targetRadii(ii) = solution.problemParameters.constraint.targetRadius*1e3;
    fuelConsumption(ii) = -(solution.x(end,7)-solution.x(1,7))*1e3;
end
plot(targetRadii,fuelConsumption,'-*','LineWidth',2,'DisplayName','Spherical Pointing Constraint');
massConsumptionUnconstrained = (UnconstrainedShortTimeRhoSweep(end).x(1,7)-UnconstrainedShortTimeRhoSweep(end).x(end,7))*1e3;
plot(   targetRadii([1,ii]),ones(2,1)*massConsumptionUnconstrained,'r-','LineWidth',1,'DisplayName','Unconstrained');
xlabel('Target Radius (m)'); ylabel('Fuel consumption (g)')
% title('Total fuel consumption versus target radius pointing constraint')
legend('show','Location','best')
saveFigFcn(hsph2,saveDir)

hsph3 = figure(Name='sphericalPlumeAngleVsDistance');
grid on; hold on; ax1 = gca; fig2 = figure; ax2 = gca; ii =1;
for solution = [UnconstrainedShortTimeRhoSweep(end),sphericalTargetSmallRadiusSweepSmallRho(end,[3,5]),sphericalTargetRadiusSweepSmallRho(end,[7,10]),...
        sphericalR4_rhoFinal([1,10])]%sphericalTargetRadius3_9_RhoSweep(end),sphericalTargetRadius3_9_RhoSweep(end)]
    axes(ax1);
    allowableAngs=PlotSolution.GetAllowableAngles(solution);
    if ii ==1, handVis = 'on';  ReduceColorOrderIndex(ax1); else, handVis='off';end
    plot(solution.t,allowableAngs,'r:','LineWidth',.5,'DisplayName','Pointing Constraint','HandleVisibility',handVis);
    
    PlotSolution.ThetaPhi(solution,ax2,ax1)
    
    ii = ii +1;
end
close(fig2); axes(ax1); legend('show','Location','best'); xlabel('Time (s)')
title(''); ylabel('\phi^{*}, polar angle from target (deg)')
saveFigFcn(hsph3,saveDir)

hsph4 = figure(Name='CostatesAndSwitchFunction',Units='pixels',Position=[476 432 708 434]);
for solution = [UnconstrainedShortTimeRhoSweep(end),sphericalTargetSmallRadiusSweepSmallRho(end,5),sphericalTargetRadiusSweepSmallRho(end,10),...
        sphericalR4_rhoFinal([1,10])]%sphericalTargetRadius3_9_RhoSweep(end),sphericalTargetRadius3_9_RhoSweep(end)]
    PlotSolution.CostatesSwitchFunction(solution)
end
saveFigFcn(hsph4,saveDir)

% Print converged costates
solsToDisplay=[RerunSolution(UnconstrainedShortTimeRhoSweep(end));
    sol40Deg(end);
    sol60Deg(end);
constantConstraintAngleRhoSweep([4,6],end) ];
solsToDisplay = RerunSolution(solsToDisplay); % Double check everything is converged
PlotSolution.PrintConvergedCostates(solsToDisplay);

solsToDisplay=[UnconstrainedShortTimeRhoSweep(end),...
    sphericalTargetSmallRadiusSweepSmallRho(end,3),...
    sphericalTargetRadiusSweepSmallRho(end,7),...
        sphericalR4_rhoFinal([1,10]) ];
solsToDisplay = RerunSolution(solsToDisplay); % Double check everything is converged
PlotSolution.PrintConvergedCostates(solsToDisplay)

