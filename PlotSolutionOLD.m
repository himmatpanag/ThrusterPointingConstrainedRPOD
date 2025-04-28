classdef PlotSolutionOLD
% take solution struct from SolvePointingConstrainedControlProblem and plot results. 

methods(Static)

    function hf = summaryShort(solution,figIn,rotated)
        if nargin < 2, hf = figure; else, hf = figIn; end
        if nargin < 3, rotated = false; end 
        summary = GetSolutionSummary(solution); 
        if rotated
            ax1 = subplot(2,2,[1,3]); 
            ax2 = subplot(2,2,2);
        else
            ax1 = subplot(2,2,[1,2]); 
            ax2 = subplot(2,2,3);
        end
        if strcmp(solution.problemParameters.dynamics.type,'Linear')
            solution.x(:,1:6) = solution.x(:,1:6)*1e3; xlabel('x (m)'); ylabel('y (m)')
        else
            xlabel('x)'); ylabel('y')
        end 
        PlotSolutionOLD.addTrajectory(solution,ax1);
                 
        switch solution.constraint.type
            case POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE
                s = PlotSphereConstraint(ax1,solution.problemParameters.constraint.targetRadius*1e3,[0;0;0]);
                set(s,'EdgeAlpha',0);
                title('Optimal trajectory with spherical target constraint')
        end 
        t = gca; % t.View = [-2.0180   41.0092];
        t.View = [-90,-90];
        % plot switch function and throttle versus time 
        legend('show', 'Location','best'); 
        
        axes(ax2); yyaxis left        
        PlotSolutionOLD.SwitchFunction(solution,ax2)        
        yyaxis right  
        PlotSolutionOLD.ThrustProfile(solution,ax2)
        title('Switch function and throttle')

        PlotSolutionOLD.PlumeAngle(solution, subplot(2,2,4));

    end 

    function hf = Sweep(solutions,sweepType)
        if nargin < 2 
            sweepType = 'rho';
        end 
        numTraj = 7; 
        N = numel(solutions);
        solsToPlot = round(linspace(1,N,numTraj));
        hf = figure; 
        for ii = 1:numTraj
            idx = solsToPlot(ii);
            solution = solutions(idx);            
            ax = subplot(3,2,[1,2]);  grid on; hold on;
            summary = GetSolutionSummary(solution); 

            switch sweepType
                case 'rho'
                    plot3(solution.x(:,1), solution.x(:,2), solution.x(:,3), 'LineWidth',2, 'DisplayName',['rho = ',num2str(solution.rho)]);
                    title(['Optimal trajectory \rho sweep, ', summary.constraintString])
                case 'Angle'
                    plot3(solution.x(:,1), solution.x(:,2), solution.x(:,3), 'LineWidth',2, 'DisplayName',['alpha = ',num2str(solution.problemParameters.constraint.alpha0*180/pi)]);
                case 'Radius'
                    plot3(solution.x(:,1), solution.x(:,2), solution.x(:,3), 'LineWidth',2, 'DisplayName',['Rad = ',num2str(solution.problemParameters.constraint.targetRadius*1000)]);
            end 
            ax.View = [-90,-90]; legend('show','Location','best')            
            PlotSolutionOLD.PlumeAngle(solution, subplot(3,2,3));
            PlotSolutionOLD.SwitchFunction(solution,subplot(3,2,4));
            PlotSolutionOLD.ThrustProfile(solution,subplot(3,2,6));

            ax = subplot(3,2,5); grid on; hold on; 
            plot(solution.t,solution.uDir(:,1).*solution.throttle,'DisplayName','controlX')
            ax.ColorOrderIndex = mod(ax.ColorOrderIndex-1,7)+1; 
            plot(solution.t,solution.uDir(:,2).*solution.throttle,'--','DisplayName','controlY')
            % ax.ColorOrderIndex = mod(ax.ColorOrderIndex-1,7)+1; 
            %plot(solution.t,solution.uDir(:,3).*solution.throttle,'DisplayName','controlZ')
        end
    end 

    function hf = Costates(solution,axIn)
        if nargin < 2 
            hf = figure;
        else 
            axes(axIn);
        end
        if size(solution.x,2)==26
            tits = {'\lambda_{r1}','\lambda_{r2}','\lambda_{r3}','\lambda_{v1}','\lambda_{v2}','\lambda_{v3}','\lambda_{m}','\lambda_{p1}','\lambda_{p2}','\lambda_{p3}','\lambda_{\omega_1}','\lambda_{\omega_2}','\lambda_{\omega_3}'};
            for ii = 1:13
                subplot(5,3,ii);
                grid on; hold on;
                plot(solution.t,solution.x(:,ii+13)); title(tits{ii})
                xlabel('Time (s)')
            end
        else
            tits = {'\lambda_{r1}','\lambda_{r2}','\lambda_{r3}','\lambda_{v1}','\lambda_{v2}','\lambda_{v3}','\lambda_{m}'};
            for ii = 1:7
                subplot(4,2,ii);
                grid on; hold on;
                plot(solution.t,solution.x(:,ii+7)); title(tits{ii})
                xlabel('Time (s)')
            end
        end
    end

    function hf = States(solution,figIn)
        if nargin < 2 
            hf = figure;
        else 
            hf = figIn;
        end
        tits = {'r_1','r_2','r_3','v_1','v_2','v_3','m'};
        for ii = 1:7
            subplot(4,2,ii);
            grid on; hold on;
            plot(solution.t,solution.x(:,ii)); title(tits{ii})
            xlabel('Time (s)')
        end
    end

    function hf = SweepShort(solutions,sweepType, hf, massPlot)
        if nargin < 4, massPlot = false; end
        if nargin < 3, hf =figure; end
        if nargin < 2 
            sweepType = 'rho';
        end 
        numTraj = min(7,numel(solutions)); 
        N = numel(solutions);
        solsToPlot = round(linspace(1,N,numTraj));        
        for ii = 1:numTraj
            idx = solsToPlot(ii);
            solution = solutions(idx);            
            ax = subplot(2,2,1);  grid on; hold on;
            summary = GetSolutionSummary(solution); 
            if strcmp(solution.problemParameters.dynamics.type,'Linear')
                solution.x(:,1:6) = solution.x(:,1:6)*1e3; xlabel('x (m)'); ylabel('y (m)')
            else
                xlabel('x)'); ylabel('y')
            end 

            switch sweepType
                case 'rho'
                    plot3(solution.x(:,1), solution.x(:,2), solution.x(:,3), 'LineWidth',2, 'DisplayName',['rho = ',num2str(solution.rho)]);
                    title(['Optimal trajectory \rho sweep, ', summary.constraintString])
                    legString = 'hide';
                case 'Angle'
                    plot3(solution.x(:,1), solution.x(:,2), solution.x(:,3), 'LineWidth',2, 'DisplayName',['alpha = ',num2str(solution.problemParameters.constraint.alpha0*180/pi)]);                    
                    legString = 'show';
            end 
            ax.View = [-90,-90]; legend('show','Location','best')            
            ax = subplot(2,2,2); PlotSolutionOLD.PlumeAngle(solution,ax); legend(legString);
            PlotSolutionOLD.SwitchFunction(solution,subplot(2,2,3));

            ax = subplot(2,2,4);
            if massPlot
                PlotSolutionOLD.MassConsumption(solution,ax);
            else
                PlotSolutionOLD.ThrustProfile(solution,ax);
            end 

        end
    end

    % Implement slider plot for summaryShort
    function hf = summary(solution,figIn)
        if nargin < 2 
            hf = figure;
        else 
            hf = figIn;
        end

        ax = subplot(3,2,[1,2]);         
        PlotSolutionOLD.addTrajectory(solution,ax);
        ax.View = [-90,-90];

        switch solution.constraint.type
            case POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE
                s = PlotSphereConstraint(ax,solution.problemParameters.constraint.targetRadius,[0;0;0]);
                set(s,'EdgeAlpha',0);
                title('Optimal trajectory with spherical target constraint')
        end                
        
        % plot switch function and throttle versus time 
        ax = subplot(3,2,4); yyaxis left        
        PlotSolutionOLD.SwitchFunction(solution,ax)        
        yyaxis right  
        PlotSolutionOLD.ThrustProfile(solution,ax)
        title('Switch function and throttle')

        PlotSolutionOLD.PlumeAngle(solution, subplot(3,2,3));

        subplot(3,2,5); grid on; hold on; 
        plot(solution.t,solution.uDir(:,1).*solution.throttle,'DisplayName','controlX')
        plot(solution.t,solution.uDir(:,2).*solution.throttle,'DisplayName','controlY')
        plot(solution.t,solution.uDir(:,3).*solution.throttle,'DisplayName','controlZ')
        legend('show','Location','best')

        PlotSolutionOLD.ControlDirectionAngle(solution, subplot(3,2,6));        
    end 

    function addTrajectory(solution,ax)
        axes(ax);
        title('Optimal trajectory')
        grid on; hold on;            
        plot3(solution.x(1,1), solution.x(1,2), solution.x(1,3),'b.','MarkerSize',60,'DisplayName','x_0 Chaser Initial')
        plot3(solution.x(end,1), solution.x(end,2), solution.x(end,3),'k.','MarkerSize',60,'DisplayName','x_f Chaser Final')
        plot3(solution.x(:,1), solution.x(:,2), solution.x(:,3), 'LineWidth',2, 'DisplayName','Trajectory');
        if isempty(ax.XLabel.String); xlabel('x'); ylabel('y'); zlabel('z'); end
        view(3);
        axis equal;
        PlotPoint.Target([0,0,0]);
        PlotSolutionOLD.addThrustDirection(solution,gca);        
    end

    function PlumeAngle(solution, ax)
        summary = GetSolutionSummary(solution); 
        axes(ax)
        % plot plume angle from origin versus time
        angle = zeros(size(solution.throttle));
        for ii = 1:numel(solution.t)
            angle(ii) = acos(dot(solution.x(ii,1:3),solution.uDir(ii,:))/(norm(solution.x(ii,1:3))*norm(solution.uDir(ii,:))));
        end 
        
        plot(solution.t, angle*180/pi,'LineWidth',2, 'DisplayName',summary.constraintString);
        grid on; hold on;
        if solution.problemParameters.constraint.type == POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE
            allowableAngs=PlotSolutionOLD.GetAllowableAngles(solution);
            plot(solution.t, allowableAngs ,'r--','DisplayName','Allowable angle')
        end
        xlabel('Time (s)'); title('Plume angle from origin');
        ylabel('angle (deg)')
        legend('show',  'Location','best');
    end 

    function allowableAngs=GetAllowableAngles(solution)
        if solution.problemParameters.constraint.type == POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE
            dist = vecnorm(solution.x(:,1:3)');
            for ii = 1:numel(dist)
                allowableAngs(ii) = 180/pi*solution.problemParameters.constraint.angleFunc(dist(ii));
            end
        elseif solution.problemParameters.constraint.type == POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE
            allowableAngs = ones(numel(solution.t),1)*solution.problemParameters.constraint.alpha*180/pi;
        else 
            allowableAngs = nan(numel(solution.t),1);
        end
    end

    function ControlDirectionAngle(solution, ax)
        summary = GetSolutionSummary(solution); 
        axes(ax)
        % plot plume angle from origin versus time
        angle = zeros(size(solution.throttle));
        for ii = 1:numel(solution.t)
            angle(ii) = acos(dot(solution.x(ii,1:3),solution.uDir(ii,:))/(norm(solution.x(ii,1:3))*norm(solution.uDir(ii,:))));
        end 
        grid on; hold on;
        plot(solution.t, angle*180/pi, 'DisplayName',summary.constraintString);
        xlabel('Time (s)'); title('Control direction angle from origin');
        ylabel('angle (deg)')
        legend('show',  'Location','best');
    end 

    function MassConsumption(solution,ax)        
        axes(ax); grid on; hold on; title('Fuel consumption')
        xlabel('t'); ylabel('Fuel consumption (g)')        
        summary = GetSolutionSummary(solution); 
        massConsumption = -1000*(solution.x(:,7)-solution.x(1,7));
        plot(solution.t,massConsumption,'LineWidth',2,'DisplayName',summary.constraintString);
        legend('show',  'Location','best'); t = colororder;
        t2=text(solution.t(end),(massConsumption(end)),[num2str(round(massConsumption(end))), ...
            'g \rightarrow'],'Color',t(ax.ColorOrderIndex-1,:),'FontSize',12,'HorizontalAlignment','right');
    end 

    function ThrustProfile(solution,ax,legStr,convex)
        if nargin < 3 
            legStr = 'Throttle';
        end
        if nargin < 4
            convex = false; 
        end
        if convex
            plotFunc = @stairs;
            legStr = 'Convex';
        else 
            plotFunc = @plot;
        end 
        axes(ax); grid on; hold on; 
        title('Throttle')
        plotFunc(solution.t, solution.throttle,'LineWidth',2, 'DisplayName',legStr);
        ylabel('throttle');
        xlabel('Time (s)'); %legend('show',  'Location','best');
    end 

    function [theta, phi] = GetOptimalThetaPhiFromSolution(solution)
        theta = solution.t;
        phi = solution.t;
        for ii = 1:numel(theta)
            pos = solution.x(ii,1:3);
            r1 = pos(1); r2 = pos(2); r3 = pos(3);
            rNorm = norm(pos(1:3));
            r12 = sqrt(r1^2 + r2^2);  
            gamma = acos(r3/rNorm); % dot(z,r)/norm(r); z = [0;0;1]
            n = -[r2;-r1;0]/r12; % cross(X(1:3),z)/norm(cross(X(1:3),z));
            q0 = cos(gamma/2);
            quatVec = n.*sin(gamma/2);
            skewSymmetric = @(v) [0,-v(3),v(2);v(3),0,-v(1);-v(2),v(1),0];
            RotMatrix = ((q0^2 - dot(quatVec,quatVec)) * eye(3) + 2.*quatVec*quatVec' + 2*q0*skewSymmetric(quatVec));
            [~,theta(ii),phi(ii)] = PlotSolutionOLD.CartesianToSpherical(RotMatrix'*solution.uDir(ii,:)');
        end
    end

    function [r,theta,phi] = CartesianToSpherical(vec)
        x = vec(1); y = vec(2); z = vec(3);
        rxy = sqrt(x^2 + y^2);
        r = sqrt(x^2 + y^2 + z^2);
        phi = atan(z/rxy);
        theta = atan2(y,x);
    end

    function ThetaPhi(solution,axTheta,axPhi,convexSol)
        if solution.constraint.type == POINTING_CONSTRAINT_TYPE.NONE
            [theta, phi] = PlotSolutionOLD.GetOptimalThetaPhiFromSolution(solution);
            solution.theta=theta;
            solution.phi = phi;
        end
        
        if nargin < 4 
            convexSol = false;
        end
        if ~convexSol
            summary = GetSolutionSummary(solution); 
            displayStr = summary.constraintString;
            plotfunc = @plot;
        else 
            displayStr = 'Convex';
            plotfunc = @stairs;
        end
        axes(axTheta); grid on; hold on; 
        title('\theta^*')
        throttleOn = solution.throttle > .1;
        thetaThrottleOn=wrapTo180(solution.theta*180/pi);
        thetaThrottleOff=thetaThrottleOn;
        thetaThrottleOn(~throttleOn)=nan;
        thetaThrottleOff(throttleOn)=nan;

        plotfunc(solution.t, thetaThrottleOn,'-','LineWidth',3, 'DisplayName',displayStr);
        ReduceColorOrderIndex(axTheta);
        plotfunc(solution.t, thetaThrottleOff,'--','LineWidth',1,'HandleVisibility','off');

        ylabel('azimuth angle from target (deg)');
        xlabel('t'); %legend('show',  'Location','best');

        axes(axPhi); grid on; hold on; 
        title('\phi^*')

        phiThrottleOn=90-solution.phi*180/pi;
        phiThrottleOn(~throttleOn)=nan;
        phiThrottleOff=90-solution.phi*180/pi;
        phiThrottleOff(throttleOn)=nan;
        
        plotfunc(solution.t, phiThrottleOn,'-','LineWidth',3, 'DisplayName', displayStr);
        ReduceColorOrderIndex(axPhi);
        plotfunc(solution.t, phiThrottleOff,'--','LineWidth',1,'HandleVisibility','off');

        ylabel('polar angle from target (deg)');
        xlabel('t'); %legend('show',  'Location','best');

    end 

    function SwitchFunction(solution,ax)
        axes(ax);
        title('Switch Function')
        grid on; hold on;
        plot(solution.t, solution.switchFunction,'LineWidth',2, 'DisplayName','Switch Function');
        ylabel('switch function'); 
    end

    function CostatesSwitchFunction(solution)
        subplot(2,2,[2,4]); grid on; hold on; 
        summary = GetSolutionSummary(solution); 
        displayStr = summary.constraintString;
        plot(solution.t, solution.switchFunction,'LineWidth',2, 'DisplayName',displayStr);
        title('Switch Function'); xlabel('Time (s)')
        subplot(2,2,1); grid on; hold on; xlabel('Time (s)')
        tits = {'\lambda_{r1}','\lambda_{r2}','\lambda_{r3}'};
        styles = {'--','-',':'};
        for ii = 2%1:3
            plot(solution.t,solution.x(:,ii+7),styles{ii},'LineWidth',1.5, 'DisplayName',displayStr);
            %if ii<3, ReduceColorOrderIndex(gca); end
        end
        legend('show','Location','northwest');
        title(tits{ii})
        tits = {'\lambda_{v1}','\lambda_{v2}','\lambda_{v3}','\lambda_{m}'};
        subplot(2,2,3); grid on; hold on; xlabel('Time (s)')
        for ii = 2%1:3
            plot(solution.t,solution.x(:,ii+10),styles{ii},'LineWidth',1.5)%,'DisplayName',tits{ii})
            %if ii<3, ReduceColorOrderIndex(gca); end
        end
        title(tits{ii})
        %legend('show','Location','best')
    end 

    function addThrustDirection(solution,ax,useTrajColor)
        if nargin < 3
            useTrajColor = false; 
        end
        axes(ax);
        axisLength = max(ax.XLim(2)-ax.XLim(1),ax.YLim(2)-ax.YLim(1));

        % Make time steps linearly spaced
        numTimeSteps = 300;
        times = linspace(solution.t(1),solution.t(end),numTimeSteps);
        
        sol.x = interpn(solution.t,solution.x,times)';
        if ~isfield(solution,'uDir')
            solution.uDir = (solution.thrustTranslationalFrame./vecnorm(solution.thrustTranslationalFrame))';
            sol.throttle = interpn(solution.t,vecnorm(solution.thrustTranslationalFrame),times)';
        else
            sol.throttle = interpn(solution.t,solution.throttle,times)';
        end
        sol.uDir = interpn(solution.t,solution.uDir,times)';
  
        % look at every 30 indices and plot the thrust direction if throttle is greater than 0.1
        numArrows = 20;
        idxStep = 10;
        idxs = find(sol.throttle>0.1);
%         idxs = idxs(floor(linspace(1,numel(idxs),numArrows)));
        idxs = idxs(1:idxStep:end);        
        arrowLength = axisLength/13;
        if useTrajColor
            ax.ColorOrderIndex = ax.ColorOrderIndex - 1;
            quiver3(sol.x(idxs,1), sol.x(idxs,2), sol.x(idxs,3),...
                -arrowLength.*sol.uDir(idxs,1), ...
                -arrowLength.*sol.uDir(idxs,2), ...
                -arrowLength.*sol.uDir(idxs,3),...
                0,'LineWidth',1.5,'MaxHeadSize',.15,'HandleVisibility','off')
%             arrow3(sol.x(idxs,1:3),sol.x(idxs,1:3)-1.*sol.uDir(idxs,1:3),'b')
        else
            quiver3(sol.x(idxs,1), sol.x(idxs,2), sol.x(idxs,3),...
                -arrowLength.*sol.throttle(idxs).*sol.uDir(idxs,1), ...
                -arrowLength.*sol.throttle(idxs).*sol.uDir(idxs,2), ...
                -arrowLength.*sol.throttle(idxs).*sol.uDir(idxs,3),...
                2,'-r','LineWidth',1,'MaxHeadSize',1,'DisplayName','Engine Plume')
        end
    end

    function ThrustConeDirection(sol,ax,useTrajColor,arrowLengthFactor)
        if nargin < 4
            arrowLengthFactor=2;
        end
        if nargin < 3
            useTrajColor = false; 
        end
        axes(ax);
        axisLength = max(ax.XLim(2)-ax.XLim(1),ax.YLim(2)-ax.YLim(1));

        idxs = find(sol.throttle>0.8);
     
        arrowLength = axisLength/arrowLengthFactor;
        if useTrajColor
            ax.ColorOrderIndex = ax.ColorOrderIndex - 1;
            quiver3(sol.x(idxs,1), sol.x(idxs,2), sol.x(idxs,3),...
                -arrowLength.*sol.uDir(idxs,1), ...
                -arrowLength.*sol.uDir(idxs,2), ...
                -arrowLength.*sol.uDir(idxs,3),...
                0,'LineWidth',1.5,'MaxHeadSize',.15,'HandleVisibility','off')
%             arrow3(sol.x(idxs,1:3),sol.x(idxs,1:3)-1.*sol.uDir(idxs,1:3),'b')
        else
            quiverc2wcmap(sol.x(idxs,1), sol.x(idxs,2), sol.x(idxs,3),...
                -arrowLength.*sol.throttle(idxs).*sol.uDir(idxs,1), ...
                -arrowLength.*sol.throttle(idxs).*sol.uDir(idxs,2), ...
                -arrowLength.*sol.throttle(idxs).*sol.uDir(idxs,3),sol.t(idxs))
%             quiver3(sol.x(idxs,1), sol.x(idxs,2), sol.x(idxs,3),...
%                 -arrowLength.*sol.throttle(idxs).*sol.uDir(idxs,1), ...
%                 -arrowLength.*sol.throttle(idxs).*sol.uDir(idxs,2), ...
%                 -arrowLength.*sol.throttle(idxs).*sol.uDir(idxs,3),...
%                 2,'-r','LineWidth',1,'MaxHeadSize',1,'DisplayName','Engine Plume')
        end
    end

    function PrintConvergedCostates(solsToDisplay)
        titleRow = 'Costate';
        rows = {'$\lambda_{r_1}$', '$\lambda_{r_2}$',... 
            '$\lambda_{r_3}$', '$\lambda_{v_1}$', '$\lambda_{v_2}$', '$\lambda_{v_3}$',...
            '$\lambda_m$'};
        for ii = 1:numel(solsToDisplay)
            solution = solsToDisplay(ii);
            if solution.constraint.type == POINTING_CONSTRAINT_TYPE.NONE
                titleRow = [titleRow, ' & Unconstrained'];
            elseif solution.constraint.type == POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE
                titleRow = [titleRow, ' & $\alpha=',num2str(solution.constraint.alpha0*180/pi),'^{\circ}$'];
            elseif solution.constraint.type == POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE
                titleRow = [titleRow, ' & $R=',num2str(solution.constraint.targetRadius*1e3),'$m'];
            end 
            if ii == numel(solsToDisplay), titleRow = [titleRow, '\\']; end 
        end 
        disp(titleRow)
        
        disp('\hline')
        for ii = 1:7
            row = rows{ii};
            for jj = 1:numel(solsToDisplay)
                solution = solsToDisplay(jj);
                row = [row,' & ',num2str(solution.newCostateGuess(ii),'%.9g')];
            end 
            if ii < 7, row = [row,'\\']; end
            disp(row);
        end 
    end 
    
    function PlotOrientationChaserRelativeToTranslationalFrame(p,pos,t,problemParameters,ax)
        % Plotting the orientation of the chaser in the translational frame is misleading, since the target may be rotating with some angular velocity relative to thte inertial frame and it is not clear what this rate is. 
        % If we asssume the target is not rotating relative to the inertial frame, then by the end of the trajectory, it will have a slightly different orientation (w.r.t.) to the translational frame. 
        DCM_BodyToInertial = mrp2DCM(p);
        DCM_InertialToLVLH = DCM_InertialToRotating(problemParameters.dynamics.frameRotationRate,t);
        DCM_BodyToLVLH = DCM_InertialToLVLH*DCM_BodyToInertial;

        % Plot a cube of length 2 centred at pos and oriented according to DCM_BodyToLVLH
        % Define the vertices of the cube
        cubeLength = 2;
        vertices = cubeLength/2*[1,1,1;1,1,-1;1,-1,1;1,-1,-1;-1,1,1;-1,1,-1;-1,-1,1;-1,-1,-1];
         % Rotate the vertices
        vertices = (DCM_BodyToLVLH*vertices')';
        % Translate the vertices
        vertices = vertices + pos';
        % Define the faces of the cube
        faces = [1,2,4,3;5,6,8,7;1,2,6,5;3,4,8,7;1,3,7,5;2,4,8,6];
        % Plot the cube
        axes(ax);
        hold on;
        patch('Vertices',vertices,'Faces',faces,'FaceColor','none','EdgeColor','k','LineWidth',1.5,'DisplayName','Chaser');
        % Plot the engine plume directions as small cones and label/colour them
        numEngines = problemParameters.dynamics.numEngines; 
        C = linspecer(numEngines);
        for ii = 1:numEngines
            posEngine = pos + DCM_BodyToLVLH*1e3*problemParameters.dynamics.engineLocationBody(:,ii);
            plumeDir = -DCM_BodyToLVLH*problemParameters.dynamics.thrustDirectionBody(:,ii);
            if problemParameters.dynamics.engineConfiguration == THRUSTER_CONFIGURATION.RCS_CANTED
                coneAng = pi/12;
            else 
                coneAng = pi/7;
            end 
            s = PlotSolutionOLD.PlotCone(ax,coneAng,C(ii,:),cubeLength/5,plumeDir,posEngine);
            s.DisplayName = ['Engine ',num2str(ii)];
            % sz = size(s.XData);
            % row = dataTipTextRow('Engine ',repelem(num2str(ii),20,1)  );
            % s.DataTipTemplate.DataTipRows(end+1) = row;
            % datatip(s,s.XData(1),s.YData(2),23)
        end
        legend('show',"Location","best")
        axis equal; xlabel('x'); ylabel('y'); zlabel('z')
        ax.View = [32 30];
    end

    function s = PlotCone(ax,alpha,color,height,unitVec,origin)
        axes(ax);
        % alpha = half angle of cone
        % color = face color of cone

        heights = linspace(0,height,2);
        thetas = linspace(-pi,pi,20);
        [R,T] = meshgrid(heights,thetas);
        Z = R + origin(3); 
        X = R.*cos(T).*tan(alpha)+ origin(1);
        Y = R.*sin(T).*tan(alpha)+ origin(2);
        s = surf(X,Y,Z);
        set(s,'FaceAlpha',1);
        set(s,'LineStyle','--');
        set(s,'EdgeAlpha',0);
        set(s,'FaceColor',color);

        rotAngle = 180/pi * acos(dot(unitVec,[0;0;1]));   
        if ~(rotAngle < pi/180) 
            if rotAngle > 179
                rotVec = [1;0;0];
            else
                rotVec = cross(unitVec,[0;0;1]);
            end
            if ~isreal(rotAngle)
                rotAngle = 180;
            end
            rotate(s,rotVec,-rotAngle,origin);
        end
    end

    function InitialOrientation(solution,ax)
        if nargin <2 
            figure; ax = gca;
        end
        if ~isfield(solution,'problemParameters')
            solution.problemParameters = solution;
        end
        axes(ax);
        PlotSolutionOLD.PlotOrientationChaserRelativeToTranslationalFrame(solution.problemParameters.p0,solution.problemParameters.x0(1:3)*1e3,0,solution.problemParameters,ax);
    end

    function hFig = ThrustProfileAllEngines(solution)
        % Plot the thrust profiles of each engine, create a subplot for each engine
        hFig = figure;
        numEngines = solution.problemParameters.dynamics.numEngines;
        % create grid of subplots based on number of engines
        numCols = ceil(sqrt(numEngines));
        numRows = ceil(numEngines/numCols);
        for ii = 1:numEngines
            ax(ii) = subplot(numRows,numCols,ii); grid on; hold on;
            xlabel('Time (s)'); ylabel('Throttle');
            plot(solution.t,solution.throttle(ii,:), 'LineWidth',2,'DisplayName','\delta');
            plot(solution.t,solution.eta(ii,:), '--','LineWidth',3,'DisplayName','\eta');
            yyaxis right
            plot(solution.t,solution.switchFunction(ii,:), 'DisplayName','Switch Function');
            title(['Engine ',num2str(ii)]); ylabel('Switch Function')
            yyaxis left
            if ii==1, legend('show','Location','best'); end
        end
        linkaxes(ax)
    end

    function SwitchFunctionAnalysis(solution,ii,ax)
        if nargin < 3; figure; ax = gca; end
        if nargin < 2, ii = 1; end
        axes(ax); grid on; hold on; 
        dynamics = solution.problemParameters.dynamics;
        plot(solution.t,solution.switchFunction(ii,:),'DisplayName',['Total Engine ',num2str(ii)]);
        plot(solution.t,solution.eta(ii,:)'.* solution.x(:,20)-1,'DisplayName','\lambda_m - 1 term')
        
        
        r_cross_v = cross(dynamics.engineLocationBody(:,ii),dynamics.thrustDirectionBody(:,ii));
        lastTerm = zeros(numel(solution.t),1);
        lmbdav_Term = zeros(numel(solution.t),1);
        
        % Compute the switch functions
        for jj = 1:numel(solution.t)
            p = solution.x(jj,8:10)'; p2 = p'*p;
            pCross = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
            temp = (1+p2)^2; t = solution.t(jj);
            DCM_BodyToInertial = eye(3) - 4*(1-p2)/temp*pCross + 8/temp * (p*p');
            DCM_InertialToLVLH = [cos(dynamics.frameRotationRate*t) sin(dynamics.frameRotationRate*t) 0;
                                  -sin(dynamics.frameRotationRate*t) cos(dynamics.frameRotationRate*t) 0;
                                  0 0 1];
            DCM_BodyToTranslationalFrame = DCM_InertialToLVLH*DCM_BodyToInertial;
            lastTerm(jj) = -solution.eta(ii,jj).*dynamics.exhaustVelocity*dynamics.initialMass/solution.x(jj,7) * solution.x(jj,24:26)*dynamics.inertiaInverse*r_cross_v;

            lmbdav_Term(jj) = -solution.eta(ii,jj).*solution.x(jj,17:19)*DCM_BodyToTranslationalFrame*dynamics.thrustDirectionBody(:,ii) * dynamics.exhaustVelocity./solution.x(jj,7);
        end
        plot(solution.t,lmbdav_Term,'DisplayName','\lambda_v term')
        plot(solution.t,lastTerm,'DisplayName','\lambda_\omega term')
        legend('show','Location','best'); xlabel('Time (s)'); ylabel('SF Components'); title(['Breakdown engine ',num2str(ii)])
    end

    function SwitchFunctionAnalysisAllEngines(solution)
        numEngines = solution.problemParameters.dynamics.numEngines;
        % create grid of subplots based on number of engines
        figure;
        numCols = ceil(sqrt(numEngines));
        numRows = ceil(numEngines/numCols);
        for ii = 1:numEngines
            ax = subplot(numRows,numCols,ii);
            PlotSolutionOLD.SwitchFunctionAnalysis(solution,ii,ax);
            if ii > 1
                axes(ax); legend('hide');
            end
        end
    end

    function hf=SixDOF_Traj(solution)
        hf=figure('Name','6DOF Trajectory'); grid on; hold on;
        PlotSolutionOLD.InitialOrientation(solution,gca);
        plot3(0,0,0,'m.','MarkerSize',80,'DisplayName','Target Location')
        x = [solution.problemParameters.x0(1:6)';solution.problemParameters.xf']*1e3;
        plot3(x(1,1), x(1,2), x(1,3),'bo','MarkerSize',10,'DisplayName','x_0 Chaser Initial')
        plot3(x(end,1), x(end,2), x(end,3),'ko','MarkerSize',10,'DisplayName','x_f Chaser Final')
        plot3(solution.x(:,1)*1e3,solution.x(:,2)*1e3,solution.x(:,3)*1e3,'DisplayName','OptimalTraj');
    
        switch solution.problemParameters.constraint.type
            case POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE
                s = PlotSphereConstraint(gca,solution.problemParameters.constraint.targetRadius*1e3,[0;0;0]);
                set(s,'EdgeAlpha',0);
                title('Optimal trajectory with spherical target constraint')
            case POINTING_CONSTRAINT_TYPE.ELLIPSOIDAL
                s = PlotEllipsoidConstraint(gca,solution.problemParameters);
                set(s,'EdgeAlpha',0);
                title('Optimal trajectory with ellipsoidal target constraint')
        end
    end

  function hf=SixDOF_TrajCRTBP(solution)
        hf=figure('Name','6DOF Trajectory'); grid on; hold on;
        PlotSolution.InitialOrientation(solution,gca);
        plot3(0,0,0,'m.','MarkerSize',20,'DisplayName','Target Location')
        x = [solution.problemParameters.x0(1:6)';solution.problemParameters.xf']*1e3;
        plot3(x(1,1), x(1,2), x(1,3),'bo','MarkerSize',10,'DisplayName','x_0 Chaser Initial')
        plot3(x(end,1), x(end,2), x(end,3),'ko','MarkerSize',10,'DisplayName','x_f Chaser Final')
        plot3(solution.x(:,7)*1e3,solution.x(:,8)*1e3,solution.x(:,9)*1e3,'DisplayName','OptimalTraj')
    
        if solution.problemParameters.constraint.type == POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE || solution.problemParameters.constraint.type == POINTING_CONSTRAINT_TYPE.ORIGIN_CONSTANT_ANGLE
            PlotSphereConstraint(gca,solution.problemParameters.constraint.targetRadius*1e3,[0;0;0])
        
        elseif solution.problemParameters.constraint.type == POINTING_CONSTRAINT_TYPE.ELLIPSOIDAL
            PlotEllipsoidConstraint(gca,solution.problemParameters)
        end
   end


    function PlumeAngleSixDOF(solution,ax, engineNum)
        if nargin < 2 
            figure; ax = gca;
        end
        axes(ax); grid on; hold on;
        if nargin < 3
            engineNum = 1:solution.problemParameters.dynamics.numEngines;
        end
        C = linspecer(numel(engineNum));
        % Compute the plume angle from the origin
        for jj = engineNum
            angle = zeros(numel(solution.t),1);
            for ii = 1:numel(solution.t)
                p = solution.x(ii,8:10)';
                DCM_BodyToInertial = mrp2DCM(p);
                DCM_InertialToLVLH = DCM_InertialToRotating(solution.problemParameters.dynamics.frameRotationRate,solution.t(ii));
                DCM_BodyToLVLH = DCM_InertialToLVLH*DCM_BodyToInertial;
                vecEngineToTargetBodyFrame = DCM_BodyToLVLH'*solution.x(ii,1:3)' + solution.problemParameters.dynamics.engineLocationBody(:,jj);
                
                angle(ii) = acos(dot(solution.problemParameters.dynamics.thrustDirectionBody(:,jj),vecEngineToTargetBodyFrame)/...
                    (norm(vecEngineToTargetBodyFrame)))*180/pi;
            end
            engineOn = solution.throttle(jj,:) > .8; angleOn = angle; angleOn(~engineOn) =NaN;
            angleOff = angle; angleOff(engineOn) = NaN;
            plot(solution.t,angleOn,'Color',C(jj,:),'LineWidth',2,'DisplayName',['Engine ',num2str(jj)]);
            plot(solution.t,angleOff,'--','Color',C(jj,:),'DisplayName',['Engine ',num2str(jj)],'HandleVisibility','off');
        end
        xlabel('Time (s)'); ylabel('Plume Angle (deg)'); title('Plume Angle from Target')
        if solution.problemParameters.constraint.type == POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE
            allowableAngs=PlotSolutionOLD.GetAllowableAngles(solution);
            plot(solution.t, allowableAngs ,'r--','DisplayName','Allowable angle')
        end
        legend('show','Location','best')
    end
    function AttitudeMRP(sol,ax)
        if nargin < 2, figure; ax = gca; end
        axes(ax);grid on; hold on; 
        for ii = 1:3
            plot(sol.t,sol.x(:,ii+7),'DisplayName',['MRP ',num2str(ii)])
            ReduceColorOrderIndex(ax);
            plot(0,sol.problemParameters.p0(ii),'x','DisplayName',['MRP ',num2str(ii),' Initial']);
            ReduceColorOrderIndex(ax);
            plot(sol.t(end),sol.problemParameters.pf(ii),'o','DisplayName',['MRP ',num2str(ii),' Initial']);
        end 
        legend('show','Location','best'); ylabel('MRP'); xlabel('Time (s)');
    end 
    function AngularRates(sol,ax)
        if nargin < 2, figure; ax = gca; end
        axes(ax);AxBruh = 'xyz'; grid on; hold on; 
        for ii = 1:3
            plot(sol.t,180/pi.*sol.x(:,ii+10),'DisplayName',['\omega_',AxBruh(ii)])
            ReduceColorOrderIndex(ax);
            plot(0,180/pi.*sol.problemParameters.w0(ii),'x','DisplayName',['\omega_',AxBruh(ii),' Initial']);
            ReduceColorOrderIndex(ax);
            plot(sol.t(end),180/pi.*sol.problemParameters.wf(ii),'o','DisplayName',['\omega_',AxBruh(ii),' Initial']);
        end 
        legend('show','Location','best')
        ylabel('Deg/s'); xlabel('Time (s)');
    end 
    function Torque(sol,ax)
        if nargin < 2, figure; ax = gca; end
        axes(ax);AxBruh = 'xyz'; grid on; hold on; 
        for ii = 1:3
            plot(sol.t,sol.torqueInertialFrame(ii,:),'DisplayName',['\omega_',AxBruh(ii)])
        end 
        legend('show','Location','best')
        ylabel('Torque Nm'); xlabel('Time (s)');
    end 
    function RotationalSummary(sol)
        figure; 
        PlotSolutionOLD.AttitudeMRP(sol,subplot(3,1,1));
        PlotSolutionOLD.AngularRates(sol,subplot(3,1,2));
        PlotSolutionOLD.Torque(sol,subplot(3,1,3));
    end 

    function ConvergedCostateTrace(sols)
        N = numel(sols);
        NLambda = numel(sols(1).newCostateGuess);
        xVals = zeros(N,1);
        plotFun = @plot;
        if sols(1).solverParameters.rho ~= sols(3).solverParameters.rho % This is a rho sweep
            for ii = 1:N, xVals(ii) = sols(ii).solverParameters.rho; end 
            xLab = 'Throttle smoothing parameter \rho'; 
            plotFun = @semilogx;
        elseif isfield(sols(1).problemParameters.constraint,'targetRadius') && (sols(1).problemParameters.constraint.targetRadius ~= sols(3).problemParameters.constraint.targetRadius)
            for ii = 1:N, xVals(ii) = 1000*sols(ii).problemParameters.constraint.targetRadius; end 
            xLab = 'Target Radius (m)'; 
        elseif isfield(sols(1).problemParameters.constraint,'epsilon') && sols(1).problemParameters.constraint.epsilon ~= sols(3).problemParameters.constraint.epsilon
            for ii = 1:N, xVals(ii) = sols(ii).problemParameters.constraint.epsilon; end 
            xLab = 'Constraint smoothing parameter \epsilon'; 
            plotFun = @semilogx;
        elseif sols(1).problemParameters.dynamics.maxThrust(1) ~=sols(4).problemParameters.dynamics.maxThrust(1)
            for ii = 1:N, xVals(ii) = sols(ii).problemParameters.dynamics.maxThrust(end); end 
            xLab = 'Thrust Value'; 
        elseif sols(1).problemParameters.dynamics.inertia(2,2) ~=sols(4).problemParameters.dynamics.inertia(2,2)
            for ii = 1:N, xVals(ii) = sols(ii).problemParameters.dynamics.inertia(2,2); end 
            xLab = 'Inertia'; 
        end
        costateGuesses = zeros(NLambda,N);
        converged = false(N,1);
        for ii =1:N
            costateGuesses(:,ii) = sols(ii).newCostateGuess;
            converged(ii) = ~any(~sols(ii).solutionFound);
        end
        figure; allIters = 1:N;
        for ii =1:NLambda
            subplot(2,2,min(floor(ii/4)+1,4));
            g = costateGuesses(ii,:);
            a1 = plotFun(xVals(converged),g(converged),'-o','DisplayName',['\lambda_{',num2str(ii),'}']);
            row = dataTipTextRow('Iter ',allIters(converged));
            a1.DataTipTemplate.DataTipRows(end+1) = row;
            grid on;  hold on; 
            if any(~converged)
                ReduceColorOrderIndex(gca);
                a2 = plotFun(xVals(~converged),g(~converged),'--','DisplayName',['\lambda_',num2str(ii)],'HandleVisibility','off');
                row2 = dataTipTextRow('Iter ',allIters(~converged));
                a2.DataTipTemplate.DataTipRows(end+1) = row2;
            end
            legend('show','Location','best'); xlabel(xLab);
        end
        for ii = 1:4, ax(ii) = subplot(2,2,ii); end 
        linkaxes(ax,'x');
    end

    function TranslationalSummary(solution)
        % Plot pos, vel, Thrust on three different subplots
        figure;
        legs = 'xyz';
        subplot(3,1,1); grid on; hold on;
        for jj = 1:3
            plot(solution.t,solution.x(:,jj)*1e3,'DisplayName',legs(jj));
            ReduceColorOrderIndex(gca);
            plot(0,solution.problemParameters.x0(jj)*1e3,'x','DisplayName',[legs(jj),' Initial']);
            ReduceColorOrderIndex(gca);
            plot(solution.t(end),solution.problemParameters.xf(jj)*1e3,'o','DisplayName',[legs(jj),' Final']);
        end
        legend('show','Location','best'); ylabel('Position (m)'); xlabel('Time (s)');
        subplot(3,1,2); grid on; hold on;
        for jj = 1:3
            plot(solution.t,solution.x(:,jj+3)*1e3,'DisplayName',['v_',legs(jj)]);
            ReduceColorOrderIndex(gca);
            plot(0,solution.problemParameters.x0(jj+3)*1e3,'x','DisplayName',['v_',legs(jj),' Initial']);
            ReduceColorOrderIndex(gca);
            plot(solution.t(end),solution.problemParameters.xf(jj+3)*1e3,'o','DisplayName',['v_',legs(jj),' Final']);
        end
        legend('show','Location','best'); ylabel('Velocity (m/s)'); xlabel('Time (s)');
        subplot(3,1,3); grid on; hold on;
        for ii = 1:3
            plot(solution.t,1e3.*solution.thrustTranslationalFrame(ii,:),'DisplayName',['F',legs(ii)])
        end 
        legend('show','Location','best')
        ylabel('Force N'); xlabel('Time (s)');
    end

    function CostBreakdown(sol)
        figure; grid on; hold on; 
        thrustCost = zeros(numel(sol.t),1);
        for ii = 1:sol.problemParameters.dynamics.numEngines
            thrustCost = thrustCost + cumtrapz(sol.t,sol.throttle(ii,:).*sol.eta(ii,:))'.*sol.problemParameters.dynamics.maxThrust(ii)/sol.problemParameters.dynamics.exhaustVelocity;
        end
        massCost = -(sol.x(:,7)-sol.x(1,7));
        
        torqueCost = cumtrapz(sol.t,(vecnorm(sol.torqueInertialFrame)).^2).*sol.problemParameters.dynamics.torqueCostMultiplier;
        plot(sol.t,thrustCost,'DisplayName','Thrust Cost');
        plot(sol.t,massCost,'DisplayName','Mass Cost');
        plot(sol.t,torqueCost,'DisplayName','Torque Cost');
        legend('show','Location','best');
        xlabel('Time'); title('Cost of control torque vs thrust');
        warning('Not suitable for RCS thrusters yet!'); % Need to separate the RW torque cost from the thruster torques
    end
end 
end 