
function PlotsFor6DOF_ConstraintPaper
%% This function contains results generated for the 6DOF RPOD paper 

% Optimal Six-DOF Trajectories for Proximity Operations with Thruster 
% Pointing Pointing Constraints using Indirect Optimization

saveDir = '/Users/himmatpanag/Documents/UIUC/Papers/2025 AAS SFM AAC/Figs/';

engineType = 3; % MONOPROPELLANT
% engineType = 4; % COLD GAS
[problemParameters, solverParameters] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_6);
[pp_CG8, ~] = ConstrainedApproachTestCondition(1011,engineType,THRUSTER_CONFIGURATION.CG_ALIGNED_CANTED_8);

load("ASC_and_JSR_Results.mat")

%% First plot figures showing the transfer
hf1 = figure('Name','EngineConfigs1');
ax1 = subplot(1,2,1);
PlotSolution.InitialOrientation(problemParameters,ax1); legend('hide');
ax2 = subplot(1,2,2);
PlotSolution.InitialOrientation(pp_CG8,ax2);
saveFigFcn(hf1,saveDir);

%% Figure showing engine 5 plume active 
hUnconstEng5 = figure('Name','UnconstEng5Active');
grid on; hold on; ax1 = gca;
solution = solutionCG_AlignedUnconstrainedrhoSweep(end);
plot3(solution.x(:,1)*1e3,solution.x(:,2)*1e3,solution.x(:,3)*1e3,'k','LineWidth',2,'DisplayName','Unconstrained Trajectory')
plot3(0,0,0,'m.','MarkerSize',80,'DisplayName','Target Location')
plot3(x(1,1), x(1,2), x(1,3),'bo','MarkerSize',10,'LineWidth',2,'DisplayName','Chaser Initial')
plot3(x(end,1), x(end,2), x(end,3),'bx','MarkerSize',13,'LineWidth',2,'DisplayName','Chaser Final')
customPlot.cubeOpacity=0.0;
customPlot.coneHeightFactor=1.5;
customPlot.coneOffOpacity=1;
customPlot.cubeLengthScale=1;
customPlot.handVis = 'on'; 
PlotSolution.OrientationAtTime(solution,0,ax1,customPlot);
customPlot.handVis = 'off'; 
PlotSolution.OrientationAtTime(solution,solution.t(end),ax1,customPlot);
hUnconstEng5.Position = [  1914         333        1150         642];
ax1.Legend.Position=[ 0.1000    0.4459    0.2483    0.3752];
ax1.View=[ 43.1606   22.8059]; ylabel('Along Track (m)'); xlabel('Radial (m)')
zlabel('Out of Plane (m)');
saveFigFcn(hUnconstEng5,saveDir);

%% Comparison figure
hf2 = figure('Name','3DOF_Comparison');
ax1 = subplot(2,2,3); grid on; hold on; 
sol3DOF = sphericalTargetRadius2_RhoSweep(end);
sol6DOF = solCG6_Constraint2_Torque_FreeMRP_VerySmallRho(end);
titles = {'x','y','z'};
for ii = 1:3
    plot(sol3DOF.t,1e3.*sol3DOF.x(:,ii),'-','LineWidth',1,'DisplayName',[titles{ii},'_{3DOF}'])
    ReduceColorOrderIndex(ax1); 
    plot(sol6DOF.t,1e3.*sol6DOF.x(:,ii),'--','LineWidth',1,'DisplayName',[titles{ii},'_{6DOF}'])
end
xlabel('Time (s)'); ylabel('Position (m)'); legend('show','Location','best')

ax2 = subplot(2,2,2);
PlotSolution.ThetaPhi(sol3DOF,ax2,ax2); xlabel('Time (s)');
allowableAngs=PlotSolution.GetAllowableAngles(sol3DOF);
plot(sol3DOF.t,allowableAngs,'k:','LineWidth',.5);
ylabel('Plume Angle (deg)'); legend('Azimuth Angle','Polar Angle','Minimum Polar Angle'); 
legend('Location','southwest'); title('');

ax3 = subplot(2,2,4); grid on; hold on; 
e=PlotSolution.GetEulerAngles(sol6DOF);
PlotSolution.TimeSeriesThrottleOnOff(e,sol6DOF,{'Yaw','Pitch','Roll'},ax3);
xlabel('Time (s)');ylabel('Euler Angle (deg)')

ax4 = subplot(2,2,1); grid on; hold on; 
sol3D_km = sol3DOF; sol3D_km.x = sol3DOF.x*1e3; xlabel('Radial (m)')
ylabel('Along Track (m)')
PlotSolution.addTrajectory(sol3D_km,ax4);
ax4.View = [90,-90];
s = PlotSphereConstraint(ax4,sol3DOF.problemParameters.constraint.targetRadius*1e3,[0;0;0]);
set(s,'EdgeAlpha',0);
saveFigFcn(hf2,saveDir);

%% Figure Plot trajectory showing rotations and engine plumes 
hf3 = figure('Name','TrajRotations');
customPlot.cubeOpacity=0;
customPlot.coneHeightFactor=1.3;
customPlot.coneOffOpacity=0.05;
customPlot.cubeLengthScale=.6;
customPlot.handVis='on';
ax = gca; grid on; hold on;
sol = solCG6_R35_TorqueRho(end); 
plotTimes = [0,7,16,25,39,48];
plot3(sol.x(:,1)*1e3,sol.x(:,2)*1e3,sol.x(:,3)*1e3,'b--','LineWidth',1,'DisplayName','Trajectory')
plot3(sol.x(1,1)*1e3,sol.x(1,2)*1e3,sol.x(1,3)*1e3,'b.','MarkerSize',10,'DisplayName','start')
plot3(sol.x(end,1)*1e3,sol.x(end,2)*1e3,sol.x(end,3)*1e3,'b.','MarkerSize',10,'DisplayName','end')
PlotSolution.OrientationAtTime(sol,plotTimes,ax,customPlot);
view([-90,90]); legend('hide')
ylabel('Along Track (m)'); xlabel('Radial (m)')
saveFigFcn(hf3,saveDir);

%% Plots of rotations, engine plumes and constraint for QUAD chart
%sol= solCG6_R35_TorqueRho(end);
sol= freeMRPR32_KRE(end); 

customPlot.arrowSpacing = 1.5;
customPlot.arrowLength=3;
hf3Quad1 = figure('Name','TrajRotationsQuadR35'); grid on; hold on;
plot3(sol.x(:,1)*1e3,sol.x(:,2)*1e3,sol.x(:,3)*1e3,'b--','LineWidth',1.5,'DisplayName','Trajectory');
PlotSolution.addThrustDirection6DOF(sol,gca,customPlot)
s = PlotSphereConstraint(gca,sol.problemParameters.constraint.targetRadius*1e3,[0;0;0]);
axis equal;view([-90,90]); legend('hide')
customPlot.cubeOpacity=0.0;
customPlot.coneHeightFactor=1.3;
customPlot.coneOffOpacity=0.08;
customPlot.cubeLengthScale=.6;
customPlot.handVis='off';
customPlot.showArrows=false;
plotTimes = [0,16,25,39,48];
PlotSolution.OrientationAtTime(sol,plotTimes,gca,customPlot);
plot3(sol.x(1,1)*1e3,sol.x(1,2)*1e3,sol.x(1,3)*1e3,'b.','MarkerSize',10,'DisplayName','start','HandleVisibility','off')
plot3(sol.x(end,1)*1e3,sol.x(end,2)*1e3,sol.x(end,3)*1e3,'b.','MarkerSize',10,'DisplayName','end','HandleVisibility','off')
view([-90,90]); %legend('hide')
ylabel('Along Track (m)'); xlabel('Radial (m)')

%% Quad chart 2 - Multiple traj on same fig
hf3Quad2 = figure('Name','TrajRotationsQuadR35'); grid on; hold on; ax=gca;
sols2Plot = [seedA_RadKER_34(end);seedB_RadKER_34(end);seedC_RadKER(end);seedD_RadKER(end);
    seedE_RadKER(end);seedF_RadKER(end);seedG_RadKER(end);seedH_RadKER(end);]';
customPlot.arrowSpacing = 1;
customPlot.arrowLength=1;handVis='on';
for sol = sols2Plot
    plot3(sol.x(:,1)*1e3,sol.x(:,2)*1e3,sol.x(:,3)*1e3,'b--','LineWidth',1.5,'DisplayName','Trajectories','HandleVisibility',handVis);
    plot3(sol.x(1,1)*1e3,sol.x(1,2)*1e3,sol.x(1,3)*1e3,'b.','MarkerSize',10,'DisplayName','start','HandleVisibility','off')
    plot3(sol.x(end,1)*1e3,sol.x(end,2)*1e3,sol.x(end,3)*1e3,'b.','MarkerSize',10,'DisplayName','end','HandleVisibility','off')
    if strcmp(handVis,'on'),handVis='off';end

    customPlot.enginesToPlot = 5;
    customPlot.arrowSpacing = 1;
    customPlot.arrowLength=3;
    PlotSolution.addThrustDirection6DOF(sol,gca,customPlot)
    
    customPlot.enginesToPlot = [1,2,3,4,6];
    customPlot.arrowLength=.3;
    customPlot.arrowSpacing = 3;
    %PlotSolution.addThrustDirection6DOF(sol,gca,customPlot)
    customPlot.coneOffOpacity=0.04;

    %PlotSolution.OrientationAtTime(sol,30,gca,customPlot);
end
s = PlotSphereConstraint(gca,sol.problemParameters.constraint.targetRadius*1e3,[0;0;0]);
s.LineStyle=':';
axis equal; xlim([-4,4]);zlim([-4,4]); ylim([-4,10.5])

customPlot.cubeOpacity=0.0;
customPlot.coneHeightFactor=1.3;
customPlot.coneOffOpacity=01;
customPlot.cubeLengthScale=.7;
customPlot.handVis='on';
customPlot.showArrows=false;
PlotSolution.OrientationAtTime(sol,0,ax,customPlot);

customPlot.cubeLengthScale=.7;
customPlot.coneOffOpacity=0;
customPlot.handVis='off';
for jj = [1,3,6,5,7,8]
    sol=sols2Plot(jj);
    PlotSolution.OrientationAtTime(sol,32,ax,customPlot);
end
ax.View=[207.8128   10.9537];ylabel('Along Track (m)'); xlabel('Radial (m)')
zlabel('Out of Plane (m)'); title('Thruster Constrained Optimal Proximity Operations in LEO');
saveFigFcn(hf3Quad2,saveDir)

%% Mass consumption figures
hf4 = figure('Name','MassConsumptionAll'); grid on; hold on; 
PlotSolution.MassConsumption(solutionCG_Aligned_Constrained2SmallEps_RhoSweep(end),gca,'Constrained, Without Attitude Control, Fixed Final Orientation')
PlotSolution.MassConsumption(solCG6_R2_TorqueRho(end),gca,'Constrained, With Attitude Control, Fixed Final Orientation')
PlotSolution.MassConsumption(sol6DOF,gca,'Constrained, With Attitude Control, Free Final Orientation')
%PlotSolution.MassConsumption(solutionCG_AlignedUnconstrainedrhoSweep(end),gca,'Unconstrained, Without Attitude Control, Fixed Final Orientation')
PlotSolution.MassConsumption(sol3DOF(end),gca,'Constrained 3DOF, (No Attitude Rate Limits)')
PlotSolution.MassConsumption(rhoKappaSweepCG_Aligned_ControlTorque(end),gca,'Unconstrained, With Attitude Control, Fixed Final Orientation')
xlabel('Time (s)'); title('');
saveFigFcn(hf4,saveDir);

hf4a = figure('Name','MassConsumptionRad'); grid on; hold on; 
sols2plot = [rhoKappaSweepCG_Aligned_ControlTorque(end); 
    solCG6_R1_TorqueRho(end);
    solCG6_R2_TorqueRho(end);
    solCG6_R3_TorqueRho(end);
    solCG6_R35_TorqueRho(end);
    solCG6_R38_TorqueKRadERSmallR(end);];
Cols = GetColors(numel(sols2plot));
for ii = 1:numel(sols2plot)
    sol = sols2plot(ii);
    if ii==1
        lab = 'Unconstrained';
    else
        lab=['Constraint Radius = ',num2str(sol.problemParameters.constraint.targetRadius*1e3),'m'];
    end
    PlotSolution.MassConsumption(sol(end),gca,lab,Cols(ii,:))
end 

xlabel('Time (s)'); title('');
saveFigFcn(hf4a,saveDir);

%% 6Dof trajectory plot with multiple trajectories 
hf5=figure('Name','6DOF_Trajectories'); grid on; hold on;
ax = gca;
solution = solutionCG_AlignedUnconstrainedrhoSweep(end);
plot3(0,0,0,'m.','MarkerSize',80,'DisplayName','Target Location')
PlotSphereConstraint(gca,2,[0;0;0])
x = [solution.problemParameters.x0(1:6)';solution.problemParameters.xf']*1e3;
%plot3(x(1,1), x(1,2), x(1,3),'bo','MarkerSize',10,'DisplayName','x_0 Chaser Initial')
plot3(x(end,1), x(end,2), x(end,3),'ko','MarkerSize',10,'DisplayName','x_f Chaser Final')
plot3(solution.x(:,1)*1e3,solution.x(:,2)*1e3,solution.x(:,3)*1e3,'LineWidth',2,'DisplayName','Unconstrained Trajectory')
solution = solutionCG_Constraint2CheaperTorque_Sr_Se(end);
plot3(solution.x(:,1)*1e3,solution.x(:,2)*1e3,solution.x(:,3)*1e3,'LineWidth',2,'DisplayName','Constrained with Attitude Control')
solution = solutionCG_Aligned_Constrained2SmallEps_RhoSweep(end);
plot3(solution.x(:,1)*1e3,solution.x(:,2)*1e3,solution.x(:,3)*1e3,'LineWidth',2,'DisplayName','Constrained without Attitude Control')
customPlot.cubeOpacity=0.05;
customPlot.coneHeightFactor=1.25;
customPlot.coneOffOpacity=0.2;
customPlot.cubeLengthScale=1;
customPlot.cubeLengthScale=.8;
customPlot.handVis = 'off'; 

PlotSolution.OrientationAtTime(solutionCG_AlignedUnconstrainedrhoSweep(end),0,ax,customPlot)
PlotSolution.OrientationAtTime(solution,[31],ax,customPlot)
PlotSolution.OrientationAtTime(solutionCG_Constraint2CheaperTorque_Sr_Se(end),[33],ax,customPlot)
xlim([-3,3]); ylim([-1,12])
view([-90,90]); 
ylabel('Along Track (m)'); xlabel('Radial (m)')
saveFigFcn(hf5,saveDir);

%% Plot of trajectories without attitude control 
hf6 = figure('Name','6DOF_Trajectories_NoAttitudeControl'); grid on; hold on;
ax = gca;
sol = solutionCG_AlignedUnconstrainedrhoSweep(end);
sols2Plot = [solutionCG_Aligned_ConstrainedR05_VerySmallrhoSweep(end),...
    solutionCG_Aligned_ConstrainedR1_rhoSweep(end),...
    solutionCG_Aligned_ConstrainedR15_Se_Sr(end),...
    solutionCG_Aligned_ConstrainedR25_Se_Sr(end),...
    solutionCG_Aligned_ConstrainedR3_Se_Sr(end)];%...solutionCG_Aligned_ConstrainedR34_Se_Sr(end)
x = sol.x*1e3;
plot3(0,0,0,'m.','MarkerSize',20,'DisplayName','Target Location')
plot3(x(1,1), x(1,2), x(1,3),'bo','MarkerSize',10,'LineWidth',2,'DisplayName','Chaser Initial')
plot3(x(end,1), x(end,2), x(end,3),'bx','MarkerSize',13,'LineWidth',2,'DisplayName','Chaser Final')
%sols2Plot = sols2Plot([5:-1:1]);
C = linspecer(numel(sols2Plot)+1);
C = GetColors(7);
jj=1;
plot3(sol.x(:,1)*1e3,sol.x(:,2)*1e3,sol.x(:,3)*1e3,'Color',C(jj,:),'LineWidth',2,'DisplayName','Unconstrained');
for sol = sols2Plot
    jj=jj+1;
    plot3(sol.x(:,1)*1e3,sol.x(:,2)*1e3,sol.x(:,3)*1e3,'Color',C(jj,:),'LineWidth',2,'DisplayName',['R = ',num2str(sol.problemParameters.constraint.targetRadius*1e3),'m']);
    ReduceColorOrderIndex(ax);
    % Plot circle of radius targetRadius
    if jj==2;
        p = patch(1e3*sol.problemParameters.constraint.targetRadius*cos(0:0.01:2*pi),1e3*sol.problemParameters.constraint.targetRadius*sin(0:0.01:2*pi),C(jj,:),'FaceAlpha',0,'HandleVisibility','off','LineWidth',1,...
        'EdgeColor',C(jj,:));
        r2=1e3*sol.problemParameters.constraint.targetRadius;
    else
        r1=r2;
        r2=1e3*sol.problemParameters.constraint.targetRadius;
        p = PlotSolution.Annulus(r1,r2,C(jj,:));
        p.FaceAlpha = 0; %p.LineStyle="none";
    end
    H=hatchfill(p,'single',jj*65,15); H.Color = C(jj,:); H.HandleVisibility = 'off';
end
jj=jj+1;
xlabel('Radial (m)'); ylabel('Along Track (m)'); 
ax.View = [90,90]; axis equal
xlim([-4,4]); ylim([-4,10.5])
hf6.Position = [1 1 1015 697];legend('show','Location','east'); 
saveFigFcn(hf6,saveDir); 

%% Animation
PlotSolution.SixDOF_Traj_Animated(solutionCG_Constraint2CheaperTorque_Sr_Se(end))

%% Plot of engine 5 constraint for increasing radius target (without control torque)
hf7 = figure('Name','Engine5Constraint'); grid on; hold on;
ax = gca;
sols2Plot2 = [solutionCG_AlignedUnconstrainedrhoSweep(end),sols2Plot];
for kk = 2:numel(sols2Plot2), radii(kk-1)=sols2Plot2(kk).problemParameters.constraint.targetRadius*1e3; end
PlotSolution.PlumeAngleSixDOF(sols2Plot2,ax,5);
legend('Unconstrained',['R = ',num2str(radii(1)),'m'],['R = ',num2str(radii(2)),'m'],['R = ',num2str(radii(3)),'m'],['R = ',num2str(radii(4)),'m'],['R = ',num2str(radii(5)),'m'])
title('Trajectories without attitude control, Engine 5 plume angle from target')
saveFigFcn(hf7,saveDir); 

%% Plot of Yaw Angle, angular velocity and torque for increasing Radius
sols2plot = [rhoKappaSweepCG_Aligned_ControlTorque(end); 
    solCG6_R1_TorqueRho(end);
    %solCG6_R15_TorqueRho(end);
    solCG6_R2_TorqueRho(end);
    %solCG6_R25_TorqueRho(end);
    solCG6_R3_TorqueRho(end);
    solCG6_R35_TorqueRho(end); 
    %solCG6_R35_TorqueKER(end);
    %solCG6_R38_TorqueKER(end)
    ];
hf8 = figure('Name','RotationComparison');
C = GetColors(numel(sols2plot));

for ii = 1:numel(sols2plot)
    sol = sols2plot(ii); 
    if ii==1, lab = 'Unconstrained'; else
        radius = sol.problemParameters.constraint.targetRadius*1e3;
        lab = ['R = ',num2str(radius),'m']; 
    end
    ax1 = subplot(3,1,1); grid on; hold on;
    e=PlotSolution.GetEulerAngles(sol);xlabel('Time (s)');ylabel('Yaw (deg)')
    PlotSolution.TimeSeriesThrottleOnOff(e(1,:),sol,{lab},ax1,C(ii,:));
    subplot(3,1,2); grid on; hold on;
    plot(sol.t,180/pi.*sol.x(:,13),'Color',C(ii,:),'LineWidth',2);xlabel('Time (s)'); ylabel('Angular velocity (deg/s)')
    subplot(3,1,3); grid on; hold on;
    plot(sol.t,sol.torqueInertialFrame(3,:),'Color',C(ii,:),'LineWidth',2); xlabel('Time (s)'); ylabel('Torque (Nm)')
end
axes(ax1); ylim([-5,45]); saveFigFcn(hf8,saveDir); 
saveFigSeparateAxes(hf8,saveDir); 

%% Plot costate continuity for a single result
sol = solCG6_R3_TorqueRho(end);
hf9 = figure('Name','Costates_R3'); 
stateLabels = PlotSolution.GetStateLabels(sol);
subplot(2,2,1); for ii = [13+[1,2,4,5]], plot(sol.t,sol.x(:,ii),"DisplayName",stateLabels{ii},'LineWidth',2); hold on; grid on; end, legend('show','Location','best'); xlabel('Time (s)')
subplot(2,2,3);for ii = [13+[3,6]], plot(sol.t,sol.x(:,ii),"DisplayName",stateLabels{ii},'LineWidth',2); hold on; grid on; end, legend('show','Location','northwest'); xlabel('Time (s)')
subplot(2,2,4);for ii = [13+[8,9,11,12]], plot(sol.t,sol.x(:,ii),"DisplayName",stateLabels{ii},'LineWidth',2); hold on; grid on; end, legend('show','Location','best'); xlabel('Time (s)')
subplot(2,2,2);for ii = [13+[7,10,13]], plot(sol.t,sol.x(:,ii),"DisplayName",stateLabels{ii},'LineWidth',2); hold on; grid on; end, legend('show','Location','best'); xlabel('Time (s)')
MagInset(hf9,subplot(2,2,3), [34,34.5,1.2e-15,2e-15],[22,47,-12e-14,-6e-14],{'SE','NE'});
saveFigFcn(hf9,saveDir); 
saveFigSeparateAxes(hf9,saveDir); 

%% Print converged costates
sols2PrintNoControl = [solutionCG_AlignedUnconstrainedrhoSweep(end),...
    solutionCG_Aligned_ConstrainedR1_rhoSweep(end),...
    solutionCG_Aligned_ConstrainedR25_VE_Sr(end),...
    solutionCG_Aligned_ConstrainedR3_Se_Sr(end)];%...solutionCG_Aligned_ConstrainedR34_Se_Sr(end)

PlotSolution.PrintConvergedCostates(sols2PrintNoControl);

sols2Print = [rhoKappaSweepCG_Aligned_ControlTorque(end); 
    solCG6_R1_TorqueRho(end);
    solCG6_R2_TorqueRho(end);
    solCG6_R3_TorqueRho(end)];

PlotSolution.ThrustProfileAllEngines(solCG6_R3_TorqueRho(end))
PlotSolution.ConvergedCostateTrace(solCG6_R3_TorqueRho)

for ii=2:4
    sol = sols2Print(ii);
    k = SweepSolutions(sol, ...
        'epsilon',1e-2,true);
    soleps(ii) = k(end);
    ii=ii+1;
end
soleps(1)=sols2Print(1);
PlotSolution.PrintConvergedCostates(soleps);

%% Torque switch function plot 
% Plot with kappa sweep
hf12 = figure('Name','KappaSweep');
N = numel(solCG6_R3_TorqueKappa);
numTraj = min(5,N); 
solsToPlot = round(linspace(1,N,numTraj));
[xVals, xLab, symb,sweepType] = PlotSolution.DetectSweep(solCG6_R3_TorqueKappa(solsToPlot));
Cols = GetColors(numTraj);
for ii = 1:numTraj
    idx = solsToPlot(ii);
    solution = solCG6_R3_TorqueKappa(idx);            
    subplot(1,3,1); grid on; hold on;xlabel('Radial (m)'); ylabel('Along Track (m)'); 
    C = Cols(ii,:);
    plot3(solution.x(:,1)*1e3, solution.x(:,2)*1e3, solution.x(:,3)*1e3, 'LineWidth',2,'Color',C, 'DisplayName',...
                        [symb,' = ',num2str(xVals(ii))],'HandleVisibility','off');
    subplot(1,3,2); grid on; hold on; xlabel('Time (s)'); ylabel('Torque (Nm)')
    plot(solution.t,solution.torqueInertialFrame(3,:), 'LineWidth',2,'Color',C,'DisplayName',...
                        [symb,' = ',num2str(xVals(ii))]);legend('show','Location','northwest')
    subplot(1,3,3); grid on; hold on; xlabel('Time (s)'); ylabel('Switch function (\lambda_{\omega}^TI^{-1})_3');
    S = PlotSolution.GetTorqueSwitchFunction(solution);
    plot(solution.t,S(3,:), 'LineWidth',2,'Color',C, 'DisplayName',...
                        [symb,' = ',num2str(xVals(ii))]);%legend('show','Location','northwest')
end
subplot(1,3,1); 
legend('show','location','northwest')
plot3(0,10,0,'ko','MarkerSize',10,'LineWidth',2,'DisplayName','Chaser Initial')
plot3(0,4,0,'kx','MarkerSize',13,'LineWidth',2,'DisplayName','Chaser Final')
saveFigFcn(hf12,saveDir); 
saveFigSeparateAxes(hf12,saveDir); 

%% Rho Sweep 
hf13 = [figure('Name','RhoSweepA'),figure('Name','RhoSweepB'),figure('Name','RhoSweepC')];
N = numel(solCG6_R3_TorqueRho);
numTraj = min(5,N); 
solsToPlot = round(linspace(1,N,numTraj));
[xVals, xLab, symb,sweepType] = PlotSolution.DetectSweep(solCG6_R3_TorqueRho(solsToPlot));
Cols = GetColors(numTraj);
eng2Plot = [1,2,5]; numEnginesToPlot =3;
for ii = 1:numTraj
    C = Cols(ii,:);
    idx = solsToPlot(ii);
    solution = solCG6_R3_TorqueRho(idx); 
    for jj = 1:numEnginesToPlot 
        figure(hf13(jj)); grid on; hold on;
        engIdx = eng2Plot(jj);
        xlabel('Time (s)'); 
        yyaxis left
        plot(solution.t,solution.throttle(engIdx,:), '-','Color',C,'LineWidth',2,'Marker','none', 'DisplayName',...
                        [symb,' = ',num2str(xVals(ii))]);
        ylabel('Throttle')
        yyaxis right 
        semilogy(solution.t,solution.switchFunction(engIdx,:)','--','Color',C,'LineWidth',1,'Marker','none','HandleVisibility','off');
        ylabel('Switch Function')
        title(['Engine ',num2str(engIdx)]);
        if jj==2, legend('show','Location','northeast'); end
        if ii==numTraj
            PlotSolution.ShadeConstraintRegion(solution,engIdx,'r',false);
        end
        title('');
    end
end
saveFigFcn(hf13,saveDir); 


%% Plot of engine 5 constraint for increasing radius target (with control torque)
hf10 = figure('Name','Engine5ConstraintTorque'); grid on; hold on;
ax = gca;
sols2Plot2 = [solutionCG_AlignedUnconstrainedrhoSweep(end),sols2Plot];
for kk = 2:numel(sols2Plot2), radii(kk-1)=sols2Plot2(kk).problemParameters.constraint.targetRadius*1e3; end
PlotSolution.PlumeAngleSixDOF(sols2Plot2,ax,5);
legend('Unconstrained',['R = ',num2str(radii(1)),'m'],['R = ',num2str(radii(2)),'m'],['R = ',num2str(radii(3)),'m'],['R = ',num2str(radii(4)),'m'],['R = ',num2str(radii(5)),'m'])
title('Trajectories without attitude control, Engine 5 plume angle from target')

