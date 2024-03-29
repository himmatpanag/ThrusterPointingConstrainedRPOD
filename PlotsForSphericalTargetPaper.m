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
angleSweep2To30LargeRho = SweepSolutions(sol2degrhoLarge,'Angle',[2:1:30]*pi/180);
angleSweepTo40LargeRho = SweepSolutions(angleSweep2To30LargeRho(end),'Angle',[30:.5:40]*pi/180);
angleSweepTo60LargeRho = SweepSolutions(angleSweepTo40LargeRho(end),'Angle',[40:.5:60]*pi/180);
angleSweepTo89LargeRho = SweepSolutions(angleSweepTo60LargeRho(end),'Angle',[60:1:89]*pi/180);
angleSweepTo90LargeRho = SweepSolutions(angleSweepTo89LargeRho(end),'Angle',[89:1:90]*pi/180);

PlotSolution.Sweep(angleSweepTo40LargeRho,'Angle')
constraint40DegRhoSweep = SweepSolutions(angleSweepTo60LargeRho(1),'rho',1e-4,true);
PlotSolution.Sweep(constraint40DegRhoSweep,'rho')
PlotSolution.Sweep(angleSweepTo60LargeRho,'Angle')
PlotSolution.Sweep(angleSweepTo89LargeRho,'Angle')
PlotSolution.Sweep(angleSweepTo89LargeRho(20:30),'Angle')

constraint60DegRhoSweep = SweepSolutions(angleSweepTo89LargeRho(1),'rho',1e-4,true);
constraint80DegRhoSweep = SweepSolutions(angleSweepTo89LargeRho(end-8),'rho',1e-4,true);

constraint85DegRhoSweep = SweepSolutions(angleSweepTo89LargeRho(27),'rho',1e-4,true);
PlotSolution.summary(constraint85DegRhoSweep(end))
PlotSolution.Sweep(constraint85DegRhoSweep)

PlotSolution.Sweep(angleSweepTo90LargeRho,'Angle')
constraint90DegRhoSweep = SweepSolutions(angleSweepTo90LargeRho(end),'rho',1e-4,true);
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
title('Optimal approach trajectories and engine plume directions')
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
    PlotSolution.ThrustProfile(solution,gca);
end
summary = GetSolutionSummary(solution);
title(['Throttle profile versus time for ',summary.constraintString, ' decreasing \rho']);
xlabel('Time (s)'); ylabel('Throttle \delta^*')
saveFigFcn(hf4,saveDir)

hf5 = figure("Name",'masscons');
for solution = [UnconstrainedShortTimeRhoSweep(end),constantConstraintAngleRhoSweep([2,3,4,6],end)']
    PlotSolution.MassConsumption(solution,gca);
end
xlabel('Time (s)')
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
title('Fuel optimal trajectory and engine plume directions for \alpha = 40 deg')
t.Position = [882         548        1152         603];
legend('show','Location','best')
saveFigFcn(hf6,saveDir)

hf7 = figure(Name='fuelConsumption'); grid on; hold on; 
constraintAngles = [0,20,40,60,80,85,90];ii=0;
for solution = [UnconstrainedShortTimeRhoSweep(end),constantConstraintAngleRhoSweep(:,end)']
    ii = ii+1;
    fuelConsumption(ii) = -(solution.x(end,7)-solution.x(1,7))*1e3;
end
plot(constraintAngles,fuelConsumption,'-*','LineWidth',2);
xlabel('Constraint Angle (degrees)'); ylabel('Mass (g)')
title('Total fuel consumption versus thruster constraint angle')
saveFigFcn(hf7,saveDir)

%% Spherical plots
hsph1 = figure(Name='targetrad3_9'); ii = 0;
for sol = [sphericalTargetRadius2_RhoSweep(end), sphericalTargetRadius3_9_RhoSweep(end)]
    ii = ii+1;
    ax = subplot(1,2,ii); grid on; hold on; 
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
        ax.View = [45  20];
        %t.Children(1).Location = 'northoutside';
        t.Children(1).Label.String = 'Time (s)';
        t.Position = [882         548        1152         603];
        legend('show','Location','best')
    else 
        delete(t.Children(1));
        ax.View = [7.9356   18.6533];
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
xlabel('Target Radius (m)'); ylabel('Mass (g)')
title('Total fuel consumption versus target radius pointing constraint')
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
saveFigFcn(hsph3,saveDir)

hsph3 = figure(Name='variousRPOD_Trajectories');

% %% Transfer time sweep
% transferTimes = 48:5:203; 
% 
% timeSweep = SweepSolutions(solution,'Time',transferTimes);
% results = load("Results/CW_ConstrainedRPO/Bipropellant/unconstrainedTOF_timeSweep48To198_LargeRho.mat");
% timeSweep = results.timeSweep;
% PlotSolution.Sweep(timeSweep);
% 
% %% Sweep the longest one to finalRho = 1e-3;
% unconstrainedFinalRho = SweepSolutions(timeSweep(end),'rho',1e-3);
% PlotSolution.Sweep(unconstrainedFinalRho);
% 
% unconstrainedFinalRhoShortTime = SweepSolutions(timeSweep(1),'rho',1e-3);
% PlotSolution.Sweep(unconstrainedFinalRhoShortTime);
% 
% save('unconstrainedTOF_48sec_rhoSweep',"unconstrainedFinalRhoShortTime")
% save('unconstrainedTOF_198sec_rhoSweep',"unconstrainedFinalRho")
% save('unconstrainedTOF_timeSweep48To198_LargeRho',"timeSweep")
% 
% % Load results
% results = load("Results/CW_ConstrainedRPO/Bipropellant/unconstrainedTOF_198sec_rhoSweep.mat");
% unconstrainedFinalRho = results.unconstrainedFinalRho;
% 
% results = load("Results/CW_ConstrainedRPO/Bipropellant/unconstrainedTOF_48sec_rhoSweep.mat");
% unconstrainedFinalRhoShortTime = results.unconstrainedFinalRhoShortTime;
% PlotSolution.summaryShort(unconstrainedFinalRhoShortTime(end))
% for ii = 2:4, subplot(2,2,ii); xlabel('Time (sec)'); end 
% 
% RerunSolution(unconstrainedFinalRhoShortTime(1))
% 
% %% Linear HCW dynamics with constant constraint angle of 45 degrees
% % [problemParameters, solverParameters] = ConstrainedApproachTestCondition(12);
% % Use problem from above
% 
% % Add constraint angle of 15deg
% solverParameters = unconstrainedFinalRho(1).solverParameters;
% problemParameters = unconstrainedFinalRho(1).problemParameters;
% solverParameters.rho = 0.5; 
% problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE;
% problemParameters.constraint.alpha0 = 2*pi/180;
% sol2degrhoLarge = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
% PlotSolution.summary(sol2degrhoLarge)
% 
% % angle sweep to 30 deg
% angleSweep2To30LargeRho = SweepSolutions(sol2degrhoLarge,'Angle',[2:1:30]*pi/180);
% PlotSolution.Sweep(angleSweep2To30LargeRho,'Angle')
% 
% % Try Angle sweep to 40 deg. Got to 29. 
% initAngle = angleSweep2To30LargeRho(end).problemParameters.constraint.alpha0*180/pi;
% angleSweep30To40LargeRho = SweepSolutions(angleSweep2To30LargeRho(end),'Angle', [initAngle:.1:40]*pi/180);
% results = load("Results/CW_ConstrainedRPO/Bipropellant/constantAngleTOF_198sec_angleSweep30To40LargeRho.mat");
% angleSweep28To39LargeRho = results.newSols;
% PlotSolution.Sweep(angleSweep28To31LargeRho,'Angle')
% 
% % Sweep a few of these to finalRho
% idxsToSweep = 1:5:30; N = numel(idxsToSweep);
% for ii = 1:N
%     sols = SweepSolutions(angleSweep2To30LargeRho(idxsToSweep(ii)),'rho',1e-3,false);
%     N2 = numel(sols); idxsToKeep = floor(linspace(1,N2,20));
%     constrainedSolRhoSweep(ii,1:20) = sols(idxsToKeep);
% end
% PlotSolution.Sweep(constrainedSolRhoSweep2(end,:),'rho')
% PlotSolution.Sweep(constrainedSolRhoSweep2(3,:),'rho')
% save('constantAngleTOF_48sec_23To28DegRhoSweep',"constrainedSolRhoSweep")
% save('constantAngleTOF_198sec_2To30DegLargeRhoOnly',"angleSweep2To30LargeRho")
% 
% results = load("Results/CW_ConstrainedRPO/Bipropellant/constantAngleTOF_48sec_23To28DegRhoSweep.mat");
% t = results.constrainedSolRhoSweep;
% % Find non empty structs
% sz = size(t); idxs = [];
% for ii = 1:sz(1), jIdx = 0; for jj = 1:sz(2), if ~isempty(t(ii,jj).solutionFound), jIdx = jIdx + 1; idxs(ii,jIdx) = jj; end, end, end
% constrainedSolRhoSweep=t;
% % Plot 1! constraint angle of 28 deg 
% constIdx = 6; 
% solutionToPlot = constrainedSolRhoSweep(constIdx,idxs(constIdx,20));
% PlotSolution.summaryShort(solutionToPlot); summary = GetSolutionSummary(solutionToPlot);
% for ii = 3:4, subplot(2,2,ii); xlabel('Time (sec)'); end 
% subplot(2,2,1:2); title(['Optimal trajectory, ',summary.constraintString])
% 
% % Plot 2! throttle profile
% hf = figure; grid on; hold on; 
% idxsToPlot = floor(linspace(1,19,7)); idxs(constIdx,1:20)
% for ii = idxsToPlot    
%     solution = constrainedSolRhoSweep(constIdx,idxs(constIdx,ii));
%     plot(solution.t, solution.throttle, 'DisplayName',['\rho = ',num2str(solution.rho)]);
% end 
% xlabel('Time (sec)'); title(['Throttle profile vs \rho, ',summary.constraintString])
% legend('show','Location','best')
% 
% % Plot 3! 
% hf = figure; 
% solution = unconstrainedFinalRhoShortTime(end); solution.x(:,1:6) = solution.x(:,1:6)*1e3;
% subplot(2,2,1); grid on; hold on; plot3(solution.x(:,1), solution.x(:,2), solution.x(:,3), 'LineWidth',2, 'DisplayName',['alpha unconstrained']);  title('Optimal Trajectories')
% PlotSolution.PlumeAngle(solution,subplot(2,2,2));
% PlotSolution.SwitchFunction(solution,subplot(2,2,3));
% PlotSolution.MassConsumption(solution,subplot(2,2,4));
% finalRhoIdx = [25,48,29,20,20,20];
% for ii = 1:6
%     solStruct2Plot(ii) = constrainedSolRhoSweep(ii,idxs(ii,finalRhoIdx(ii)));
% end 
% PlotSolution.SweepShort(solStruct2Plot,'Angle',hf, true)
% subplot(2,2,1); legend('hide'); for ii = 2:4, subplot(2,2,ii); xlabel('Time (sec)'); end 
% 
% 
% % PlotSolution.SweepShort(constrainedSolRhoSweep(constIdx,idxs(constIdx,1:20)),'rho')
% % PlotSolution.SweepShort(constrainedSolRhoSweep(3,idxs(3,20:25)),'rho')
% % 
% % constIdx = 1; PlotSolution.SweepShort(sol2Plot,'rho'); 
% 
% 
% %% Spherical Constraints
% % Plot the constraint angle vs time to get an idea of what the sphere should be
% hfConstraintAngleFig = figure; grid on; hold on; 
% for targetRad = [.5, 1, 1.5, 2]
%     problemParameters = UpdateSphereCircleRadius(problemParameters,targetRad);
%     N=100; Rs = linspace(2,10,N); for ii=1:N, constraintAngVsDist(ii)= problemParameters.constraint.angleFunc(Rs(ii));end
%     plot(Rs,constraintAngVsDist*180/pi,'DisplayName',['Radius ',num2str(targetRad)]);
% end
% xlabel('Distance from target (m)'); legend('show'); ylabel('Constraint angle (deg)'); title('Constraint angle for different spherical target radii')
% 
% % Set parameters from fixed constraint angle problem 
% problemParameters = angleSweep28To39LargeRho(end).problemParameters;
% solverParameters = angleSweep28To39LargeRho(end).solverParameters;
% solverParameters.rho = 0.5; 
% problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE;
% problemParameters = UpdateSphereCircleRadius(problemParameters,1.5/1000);
% solution.solutionFound = false; 
% solverParameters.initialCostateGuess = [angleSweep28To39LargeRho(end).newCostateGuess; rand(1,1)*1e-5];
% problemParameters.dynamics.regularizationMethod = 'HyperbolicTangentSmoothing';
% 
% solR1_5_rhoLarge = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
% PlotSolution.summary(solR1_5_rhoLarge)
% rhoSweepRadius1_5 = SweepSolutions(solR2_rhoLargeRadius,'rho',1e-3,false);
% PlotSolution.summary(rhoSweepRadius1_5(end))
% 
% % Now try with L2 norm regularization method Radius = 1
% problemParameters.dynamics.regularizationMethod = 'L2_Norm';
% solverParameters.rho = 0.1; 
% problemParameters = UpdateSphereCircleRadius(problemParameters,1/1000);
% solverParameters.initialCostateGuess = rhoSweepRadius1_5(end).newCostateGuess;
% solR1_rhoLargeL2_NormSmoothing = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
% rhoSweepR1_L2_NormSmoothing = SweepSolutions(solR1_rhoLargeL2_NormSmoothing,'rho',1e-3,false);
% PlotSolution.Sweep(rhoSweepR1_L2_NormSmoothing);
% 
% % Radius sweep to 2
% radiusSweep = SweepSolutions(solR1_rhoLargeL2_NormSmoothing,'Radius',[1:0.1:2]/1000,false);
% PlotSolution.Sweep(radiusSweep);
% 
% % rho sweep R2
% rhoSweepMediumRadius = SweepSolutions(radiusSweep(end),'rho',1e-4,false);
% PlotSolution.Sweep(rhoSweepMediumRadius);
% PlotSolution.summary(rhoSweepMediumRadius(end))
% rhoSweepMediumRadiusSmallRho = SweepSolutions(rhoSweepMediumRadius(end),'rho',1e-4,false);
% sols = [rhoSweepMediumRadius,rhoSweepMediumRadiusSmallRho];
% PlotSolution.summary(rhoSweepMediumRadiusSmallRho(end))
% 
% save('sphericalTOF_198sec_radiusSweep_5To1_5_LargeRhoOnly',"radiusSweep")
% save('sphericalTOF_198sec_rhoSweep_targetRadius1_5_SingularArc',"sols")
% 
% save('sphericalTOF_198sec_L2Norm_rhoSweep_targetRadius1_9_SingularArc',"rhoSweepMediumRadius")
% save('sphericalTOF_198sec_L2Norm_LargeRho_RadiusSweep',"radiusSweep")
% 
% 
% % This shows a singular arc!^ 
% 
% %% Try spherical with reduceed TOF
% constIdx = 6; 
% solutionToPlot = constrainedSolRhoSweep(constIdx,idxs(constIdx,20));
% newSol = RerunSolution(solutionToPlot);
% PlotSolution.summary(newSol)
% problemParameters = newSol.problemParameters;
% solverParameters = newSol.solverParameters;
% problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE;
% problemParameters = UpdateSphereCircleRadius(problemParameters,0.5/1000);
% solverParameters.initialCostateGuess = newSol.newCostateGuess;
% solTOF48_rhoLargeRadius_5 = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
% PlotSolution.summary(solTOF48_rhoLargeRadius_5);
% 
% problemParameters = constrainedSolRhoSweep2(end,1).problemParameters;
% solverParameters = constrainedSolRhoSweep2(end,1).solverParameters;
% problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE;
% problemParameters = UpdateSphereCircleRadius(problemParameters,0.5/1000);
% 
% solverParameters.initialCostateGuess = constrainedSolRhoSweep2(1).newCostateGuess;
% solTOF48_rhoLargeRadius_5 = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
% PlotSolution.summary(solTOF48_rhoLargeRadius_5)
% solTOF48_rhoSmallRadius_5 = SweepSolutions(solTOF48_rhoLargeRadius_5,'rho',1e-3);
% save('sphericalTOF_48sec_rhoSweepTargetRadius_0_5',"solTOF48_rhoSmallRadius_5")
% 
% results = load("Results/CW_ConstrainedRPO/Bipropellant/sphericalTOF_48sec_rhoSweepTargetRadius_0_5.mat");
% solTOF48_rhoSmallRadius_5 = results.solTOF48_rhoSmallRadius_5;
% 
% PlotSolution.summary(constrainedSolRhoSweep2(1)); PlotSolution.summary(constrainedSolRhoSweep2(end))
% PlotSolution.SweepShort(solTOF48_rhoSmallRadius_5)
% PlotSolution.summaryShort(solTOF48_rhoSmallRadius_5(end))
% for ii = 3:4, subplot(2,2,ii); xlabel('Time (sec)'); end 
% 
% 
% % Another singular arc!
% solTOF48_rhoLargeRadius_1_5;
% solTOF48_rhoSmallRadius_1_5 = SweepSolutions(solTOF48_rhoLargeRadius_1_5,'rho',1e-3,false);
% 
% %% Ion engine solutions 
% angleSweep = SweepSolutions(solution,'Angle',[15:.1:45]*pi/180);
% PlotSolution.summary(newSols(1));
% 
% rhoSweep25Deg = SweepSolutions(newSols(20),'rho',1e-4);
% rhoSweep30Deg = SweepSolutions(newSols(31),'rho',1e-4);
% PlotSolution.Sweep(rhoSweep,'rho')
% 
% PlotSolution.Sweep(newSols,'angle')
% 
% rhoAngleSweep35 = SweepSolutions(newSols(21),'rhoAngle',[1e-3,40*pi/180],false);
% PlotSolution.Sweep(rhoAngleSweep,'angle')
% PlotSolution.summary(rhoAngleSweep(60))
% 
% rhoAngleSweep30Deg = SweepSolutions(rhoAngleSweep(80),'rho',1e-4);
% PlotSolution.Sweep(rhoAngleSweep,'angle')
% RerunSolution(rhoAngleSweep30Deg(end));
% PlotSolution.summary(rhoSweep25Deg(end))
% PlotSolution.summary(rhoAngleSweep30Deg(end))
% 
% % Try smaller alpha steps?
% angleSweepto25_35 = SweepSolutions(newSols(21),'Angle',[25:.1:35]*pi/180);
% 
% % Try random guesses
% solution = solutionUnconstrainedFinalRho(1);
% solution.solutionFound = false; 
% problemParameters = solution.problemParameters;
% solverParameters = solution.solverParameters;
% solverParameters.rho = 0.5; 
% problemParameters.constraint.type = POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE;
% problemParameters.constraint.alpha0 = 45*pi/180;
% iter = 0; iterMax = 100;
% solverParameters.stateConvergeneTolerance = 1e-8;
% while any(~(solution.solutionFound)) && iter<iterMax
%     solverParameters.initialCostateGuess = [rand(3,1)/1e4;rand(3,1)/10];
%     solution = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
%     iter = iter+1;
% end
% PlotSolution.summary(solution);
% 
% rhoSweep45DegNew = SweepSolutions(solution,'rho',1e-3);
% rhoSweep35Deg = SweepSolutions(solution,'rho',1e-3);
% PlotSolution.Sweep(rhoSweep35Deg); 
% PlotSolution.Sweep(rhoSweep35Deg(1:10)); 
% 
% rhoSweep35DegNoDynamicStepSize = SweepSolutions(solution,'rho',1e-3,false);
% PlotSolution.Sweep(rhoSweep35DegNoDynamicStepSize); 
% 
% PlotSolution.summary(rhoSweep45Deg(end))
% PlotSolution.Sweep(rhoSweep45Deg); 
% rhoSweep45DegToSmallerRho = SweepSolutions(rhoSweep45Deg(end),'rho',1e-4);
% PlotSolution.Sweep(rhoSweep45DegToSmallerRho); 
% 
% % IT CONVERGED!^^ Try 60 degrees for lolz
% % Try increase 30deg constraint when rho =1e-3
% angleSweep31DegFrom30DegFinalRho = SweepSolutions(rhoAngleSweep(20),'Angle',[27.25,30]*pi/180);
% PlotSolution.Sweep(rhoAngleSweep,'angle')
% angleSweepTo30DegMediumRho = SweepSolutions(rhoAngleSweep(20),'Angle',[27.25:0.2:35]*pi/180);
% 
% %% Linear HCW dynamics with spherical target of R = 2 
% [problemParameters, solverParameters] = ConstrainedApproachTestCondition(14);
% useExistingSolution = true; 
% if useExistingSolution
%     load('solutionSphericalCW_ApproachR2.mat');
% else 
%     solverParameters.initialCostateGuess = [0.072508101952911  -0.016008226036710  -0.000966157781467   1.270347522843222  -0.049121252997545  -0.006030148702710   rand(1,1)*1e-5]';
%     solution = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
%     while solverParameters.rho > solverParameters.finalRho 
%         solverParameters.rho = solverParameters.rho*0.9;
%         solverParameters.initialCostateGuess = solution.newCostateGuess;
%         solution = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
%     end 
% end 
% hfHCW_SphR2 = PlotSolution.summary(solution);
% 
% solverParameters.rho=solution.rho;
% 
% %% Linear HCW dynamics, trajectories approaching sphericalTarget
% soution4 = solution; 
% 
% 
% %% Extend to CRTBP dynamics and JWST orbit
% 
% % %% Constrained, spherical target R = 1; 
% % [problemParameters, solverParameters] = ConstrainedApproachTestCondition(13);
% % solverParameters.initialCostateGuess = [0.059946692386909  -0.005254239642207  -0.005254239729821   1.197573082530436  -0.014646044993121  -0.014646045237169]';
% % solverParameters.rho = solverParameters.finalRho*10;
% % solution = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
% % PlotSolution.summary(solution)
% % while solverParameters.rho > solverParameters.finalRho 
% %     solverParameters.rho = solverParameters.rho*0.9;
% %     solverParameters.initialCostateGuess = solution.newCostateGuess;
% %     solution = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
% % end 
% 
% %% SaveFigs
% cd(saveDir); 
% saveas(hf,'solSummaryToy_SphericalR1','fig'); saveas(hf,'solSummaryToy_SphericalR1','png'); 
% saveas(hFig,'constraint','fig'); saveas(hFig,'constraint','png'); 
% saveas(hfHCW_SphR2,'solSummaryCW_SphericalR2','fig'); saveas(hfHCW_SphR2,'solSummaryCW_SphericalR2','png'); 
% cd(od);
% 
% %% Unconstrained trajectory WITH DYNAMICS
% 
% 
% %% Constant angle 45degree trajectory with dynamics 
% 
% %% spherical target trajectory with dynamics
% 
% %% Toy dynamics, constrained problem
% plotToyDynamics = false;
% if plotToyDynamics
%     useExistingSolution = true; 
%     if useExistingSolution
%         load("solutionSphericalApproachR1.mat");
%         solverParameters.rho = solution.rho;
%     else 
%         solverParameters.rho = 0.5;
%         solution.solutionFound = true;     
%         solution.newCostateGuess = [0.059946692386909  -0.005254239642207  -0.005254239729821   1.197573082530436  -0.014646044993121  -0.014646045237169]';
%         
%         while solverParameters.rho > solverParameters.finalRho 
%             solverParameters.rho = solverParameters.rho*0.9;
%             solverParameters.initialCostateGuess = solution.newCostateGuess;
%             solution = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
%         end 
%     end
%     hf = figure('Units','normalized','Position',[ 0.3406    0.1762    0.6594    0.7047]);
%     PlotSolution.summaryShort(solution,hf);
% end

end 