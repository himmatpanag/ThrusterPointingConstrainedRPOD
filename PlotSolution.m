classdef PlotSolution
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
        PlotSolution.addTrajectory(solution,ax1);
                 
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
        PlotSolution.SwitchFunction(solution,ax2)        
        yyaxis right  
        PlotSolution.ThrustProfile(solution,ax2)
        title('Switch function and throttle')

        PlotSolution.PlumeAngle(solution, subplot(2,2,4));

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
            PlotSolution.PlumeAngle(solution, subplot(3,2,3));
            PlotSolution.SwitchFunction(solution,subplot(3,2,4));
            PlotSolution.ThrustProfile(solution,subplot(3,2,6));

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
        tits = {'\lambda_{r1}','\lambda_{r2}','\lambda_{r3}','\lambda_{v1}','\lambda_{v2}','\lambda_{v3}','\lambda_{m}'};
        for ii = 1:7
            subplot(4,2,ii);
            grid on; hold on;
            plot(solution.t,solution.x(:,ii+7)); title(tits{ii})
            xlabel('Time (s)')
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
            ax = subplot(2,2,2); PlotSolution.PlumeAngle(solution,ax); legend(legString);
            PlotSolution.SwitchFunction(solution,subplot(2,2,3));

            ax = subplot(2,2,4);
            if massPlot
                PlotSolution.MassConsumption(solution,ax);
            else
                PlotSolution.ThrustProfile(solution,ax);
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
        PlotSolution.addTrajectory(solution,ax);
        ax.View = [-90,-90];

        switch solution.constraint.type
            case POINTING_CONSTRAINT_TYPE.ORIGIN_VARIABLE_ANGLE
                s = PlotSphereConstraint(ax,solution.problemParameters.constraint.targetRadius,[0;0;0]);
                set(s,'EdgeAlpha',0);
                title('Optimal trajectory with spherical target constraint')
        end                
        
        % plot switch function and throttle versus time 
        ax = subplot(3,2,4); yyaxis left        
        PlotSolution.SwitchFunction(solution,ax)        
        yyaxis right  
        PlotSolution.ThrustProfile(solution,ax)
        title('Switch function and throttle')

        PlotSolution.PlumeAngle(solution, subplot(3,2,3));

        subplot(3,2,5); grid on; hold on; 
        plot(solution.t,solution.uDir(:,1).*solution.throttle,'DisplayName','controlX')
        plot(solution.t,solution.uDir(:,2).*solution.throttle,'DisplayName','controlY')
        plot(solution.t,solution.uDir(:,3).*solution.throttle,'DisplayName','controlZ')
        legend('show','Location','best')

        PlotSolution.ControlDirectionAngle(solution, subplot(3,2,6));        
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
        PlotSolution.addThrustDirection(solution,gca);        
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
            allowableAngs=PlotSolution.GetAllowableAngles(solution);
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
            [~,theta(ii),phi(ii)] = PlotSolution.CartesianToSpherical(RotMatrix'*solution.uDir(ii,:)');
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
            [theta, phi] = PlotSolution.GetOptimalThetaPhiFromSolution(solution);
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
        sol.throttle = interpn(solution.t,solution.throttle,times)';
        sol.x = interpn(solution.t,solution.x,times)';
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
     

end 
end 