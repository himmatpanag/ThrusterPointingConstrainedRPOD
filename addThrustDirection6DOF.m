function addThrustDirection6DOF(solution,ax,customPlot)
        if nargin < 3
            customPlot.arrowLength = 1;
            customPlot.arrowSpacing = 1; % seconds
            customPlot.enginesToPlot = 1:solution.problemParameters.dynamics.numEngines;
        end
        axes(ax);
        axisLength = max(ax.XLim(2)-ax.XLim(1),ax.YLim(2)-ax.YLim(1));

        % Make time steps linearly spaced
        numTimeSteps = 300;
        times = linspace(solution.t(1),solution.t(end),numTimeSteps);
        h=times(2)-times(1); hIdx = max(1,customPlot.arrowSpacing/h);
        
        sol.x = interpn(solution.t,solution.x,times)';
        sol.throttle = interpn(solution.t,solution.throttle',times);
        numEngines=solution.problemParameters.dynamics.numEngines;
        C = linspecer(numEngines);
        for ii = customPlot.enginesToPlot
            kk = 1; kkMax = 5; 
            throttleOn = sol.throttle(ii,:)>0.1; 
            throttleOnIdx = find(throttleOn,1,'first');
            clear idxOn idxOff
            while (kk < kkMax) && (~isempty(throttleOnIdx))% max number of shaded regions to plot
                idxOn(kk) = throttleOnIdx;
                throttleOff = throttleOnIdx + find(~throttleOn(idxOn(kk):end),1,'first')-1;
                if isempty(throttleOff)
                    idxOff(kk) = numel(times);
                else
                    idxOff(kk) = throttleOff;
                end
                throttleOn(idxOn(kk):idxOff(kk)) = false;

                % Plot arrows
                idxs=round([idxOn(kk),(idxOn(kk)+hIdx):hIdx:(idxOff(kk)-hIdx),idxOff(kk)]);
                plumeDir = zeros(3,numel(idxs));
                for jj = 1:numel(idxs)
                    idx = idxs(jj);
                    p = sol.x(idx,8:10)';
                    DCM_BodyToInertial = mrp2DCM(p);
                    DCM_InertialToLVLH = DCM_InertialToRotating(solution.problemParameters.dynamics.frameRotationRate,times(idx));
                    DCM_BodyToLVLH = DCM_InertialToLVLH*DCM_BodyToInertial;
                    plumeDir(:,jj) = -DCM_BodyToLVLH*solution.problemParameters.dynamics.thrustDirectionBody(:,ii);
                end
                quiver3(sol.x(idxs,1)*1e3, sol.x(idxs,2)*1e3, sol.x(idxs,3)*1e3,...
                    customPlot.arrowLength.*plumeDir(1,:)', ...
                    customPlot.arrowLength.*plumeDir(2,:)', ...
                    customPlot.arrowLength.*plumeDir(3,:)',...
                    0,'Color',C(ii,:)',...
                    'LineWidth',1.5,'MaxHeadSize',.15,'HandleVisibility','off')

                % Find next region throttle is on 
                throttleOnIdx = find(throttleOn,1,'first');
                kk = kk+1;
            end
        end

    end