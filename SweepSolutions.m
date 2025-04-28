function newSols = SweepSolutions(initialSolution,type,values,dynamicStepSize,oldSweep)
if nargin < 5
    oldSweep = false; 
end
if nargin < 4
    dynamicStepSize = true;
end
rhoMin = 0.5;

solverParameters = initialSolution.solverParameters;
problemParameters = initialSolution.problemParameters;
solution = initialSolution;
solverParameters.initialCostateGuess = solution.newCostateGuess;   

dynamicRhoSpeedUp = false;
if contains(type,'rho')
    rate = 0.8;    
    finalRho = values(1);
    N = ceil(log(finalRho/solverParameters.rho)/log(rate));
    if dynamicStepSize
        dynamicRhoSpeedUp = true;
    end
elseif contains(type,'epsilon')
    rate = 0.7;    
    final_epsilon = values(1);
    N = ceil(log(final_epsilon/problemParameters.constraint.epsilon)/log(.99));
    if dynamicStepSize
        dynamicRhoSpeedUp = true;
    end
elseif contains(type,'kappa')
    rate = 0.95;    
    finalKappa = values(1);
    N = ceil(log(finalKappa/problemParameters.dynamics.torqueCostMultiplier)/log(rate));
    if dynamicStepSize
        dynamicRhoSpeedUp = true;
    end
else 
    %if ismatrix(values) && ~isvector(values)
        % strictly 2D matrix
        N = size(values,1);
    %else
        %N = numel(values);
%end

end

ii=0;
numFails = 0;
iter = 1; prevStepPass = false; targetValue = 0; prevValue = 0;
while ii < N+1
    if ii == 0
        newSols(iter) = RerunSolution(initialSolution);
        prevStepPass = true;      
        ii = ii+1;
    end

    switch type 
        case 'FinalX_Position'
            if prevStepPass, prevValue = problemParameters.xf(1); end
            targetValue = values(ii);            
            stepSize = (values(ii)-prevValue)/(numFails+1);
            problemParameters.xf(1) = prevValue + stepSize;
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
        case 'FinalY_Position'
            if prevStepPass, prevValue = problemParameters.xf(2); end
            targetValue = values(ii);            
            stepSize = (values(ii)-prevValue)/(numFails+1);
            problemParameters.xf(2) = prevValue + stepSize;
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
        case 'FinalZ_Position'
            if prevStepPass, prevValue = problemParameters.xf(3); end
            targetValue = values(ii);            
            stepSize = (values(ii)-prevValue)/(numFails+1);
            problemParameters.xf(3) = prevValue + stepSize;
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
        % case 'FinalPosVel'
        %     if prevStepPass, prevValue = problemParameters.xf; end
        %     targetValue = values;            
        %     stepSize = (values-prevValue)/(numFails+1);
        %     problemParameters.xf = prevValue + stepSize;
        %     fprintf('prevValue %f\t stepSize\t%f newValue%f\t',norm(prevValue),norm(stepSize),norm(prevValue+stepSize));
        case 'FinalMRP'
            if prevStepPass, prevValue = problemParameters.pf; end
            targetValue = values{ii};            
            stepSize = (values-prevValue)/(numFails+1);
            problemParameters.pf = prevValue + stepSize;
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',norm(prevValue),norm(stepSize),norm(prevValue+stepSize));
        case 'Time'
            if prevStepPass, prevValue = problemParameters.transferTime; end
            targetValue = values(ii);            
            stepSize = (values(ii)-prevValue)/(numFails+1);
            problemParameters.transferTime = prevValue + stepSize;
            solverParameters.tSpan = [0,values(ii)];
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
        case 'Angle'
            if prevStepPass, prevValue = problemParameters.constraint.alpha0; end    
            targetValue = values(ii);            
            stepSize = (targetValue-prevValue)/(numFails+1);
            problemParameters.constraint.alpha0 = prevValue + stepSize;
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue*180/pi,stepSize*180/pi,(prevValue+stepSize)*180/pi);
        case 'Radius'
            if prevStepPass, prevValue = problemParameters.constraint.targetRadius; end             
            targetValue = values(ii);             
            stepSize = (targetValue-prevValue)/(numFails+1);
            problemParameters = UpdateSphereCircleRadius(problemParameters,prevValue + stepSize);            
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue*1e3,stepSize*1e3,(prevValue+stepSize)*1e3);
        case 'SemiAxis_x'
            if prevStepPass, prevValue = problemParameters.constraint.targetAxisx; end                
            targetValue = values(ii);             
            stepSize = (targetValue - prevValue) / (numFails+1);
            problemParameters.constraint.targetAxisx = prevValue + stepSize;          
            fprintf('Axis X: prevValue %f\t stepSize %f\t newValue %f\t', prevValue, stepSize, (prevValue+stepSize));
        case 'SemiAxis_y'
            if prevStepPass, prevValue = problemParameters.constraint.targetAxisy; end                
            targetValue = values(ii);             
            stepSize = (targetValue - prevValue) / (numFails+1);
            problemParameters.constraint.targetAxisy = prevValue + stepSize;          
            fprintf('Axis Y: prevValue %f\t stepSize %f\t newValue %f\t', prevValue, stepSize, (prevValue+stepSize));
        case 'SemiAxis_z'
            if prevStepPass, prevValue = problemParameters.constraint.targetAxisz; end                
            targetValue = values(ii);             
            stepSize = (targetValue - prevValue) / (numFails+1);
            problemParameters.constraint.targetAxisz = prevValue + stepSize;          
            fprintf('Axis Z: prevValue %f\t stepSize %f\t newValue %f\t', prevValue, stepSize, (prevValue+stepSize));
        case 'Ellipsoid1' % Scale all ellipsoid axes at same time and rate
            if prevStepPass
                prevValue = problemParameters.constraint.targetAxisx;
                prevValuey = problemParameters.constraint.targetAxisy;
                prevValuez = problemParameters.constraint.targetAxisz;
            end
            targetValue = values(ii);
            stepSize = (targetValue-prevValue)/(numFails+1);
            newVal = (prevValue + stepSize);
            factorIncrease = newVal/prevValue;
            problemParameters.constraint.targetAxisx = newVal;
            problemParameters.constraint.targetAxisy = prevValuey*factorIncrease;
            problemParameters.constraint.targetAxisz = prevValuez*factorIncrease;
            fprintf('Axis X: prevValue %f\t stepSize %f\t newValue %f\t', prevValue, stepSize, (prevValue+stepSize));
          
        case 'Ellipsoid2' % Scale all ellipsoid axes individually toward their targets
            if prevStepPass
                prevValueX = problemParameters.constraint.targetAxisx;
                prevValueY = problemParameters.constraint.targetAxisy;
                prevValueZ = problemParameters.constraint.targetAxisz;

                prevValue = prevValueX;
            end
        
            targetValuex = values(ii,1);  
            targetValuey = values(ii,2);  
            targetValuez = values(ii,3);  
        
            % Compute incremental step sizes separately.
            stepSizeX = (targetValuex - prevValueX) / (numFails + 1);
            stepSizeY = (targetValuey - prevValueY) / (numFails + 1);
            stepSizeZ = (targetValuez - prevValueZ) / (numFails + 1);
            
            % Update the constraint targets.
            problemParameters.constraint.targetAxisx = prevValueX + stepSizeX;
            problemParameters.constraint.targetAxisy = prevValueY + stepSizeY;
            problemParameters.constraint.targetAxisz = prevValueZ + stepSizeZ;
            
            % For the overall code, use the X axis as the reference:
            targetValue = targetValuex;
            stepSize  = stepSizeX;
        
            % Diagnostics.
            fprintf('Axis X: prevValue %f\t stepSize %f\t newValue %f\t\n', prevValueX*1e3, stepSizeX*1e3, (prevValueX + stepSizeX)*1e3);
            fprintf('Axis Y: prevValue %f\t stepSize %f\t newValue %f\t\n', prevValueY*1e3, stepSizeY*1e3, (prevValueY + stepSizeY)*1e3);
            fprintf('Axis Z: prevValue %f\t stepSize %f\t newValue %f\t\n', prevValueZ*1e3, stepSizeZ*1e3, (prevValueZ + stepSizeZ)*1e3);

            if ii >= N
                break
            end

        case 'rho'
            if prevStepPass, prevValue = solverParameters.rho; end             
            targetValue = prevValue*rate;
            stepSize = (targetValue-prevValue)/(numFails+1);
            solverParameters.rho = prevValue + stepSize; 
            if prevStepPass && (prevValue <= finalRho)
                break
            end
        case 'epsilon'
            if prevStepPass, prevValue = problemParameters.constraint.epsilon; end
            targetValue = prevValue*rate;
            stepSize = (targetValue-prevValue)/(numFails+1);
            problemParameters.constraint.epsilon = prevValue + stepSize;
            if prevStepPass && (prevValue <= final_epsilon)
                break
            end
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
        case 'rhoAngle'
            if prevStepPass, prevValue = solverParameters.rho; prevValueAng = problemParameters.constraint.alpha0; end                         

            step = interpn([1,N],[initialSolution.problemParameters.constraint.alpha0, values(2)],ii) - prevValueAng;
            stepSize = step/(numFails+1);   
            problemParameters.constraint.alpha0 = prevValueAng + stepSize; 
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValueAng*180/pi,stepSize*180/pi,(prevValueAng+stepSize)*180/pi);

            targetValue = prevValue*rate;
            stepSize = (targetValue-prevValue)/(numFails+1);
            solverParameters.rho = prevValue + stepSize;             
        case 'rhoepsilon'
            if prevStepPass, prevValue = solverParameters.rho; prevValueEps = problemParameters.constraint.epsilon; end             
            if numel(values) > 1, finalEps = values(2); else, finalEps = finalRho; end
            targetValue = prevValue*rate; 
            stepSize = (targetValue-prevValue)/(numFails+1);

            targetValueEps = prevValueEps*rate;
            stepSizeEps = (targetValueEps-prevValueEps)/(numFails+1);
            if (prevValueEps >= finalEps) % only update parameters if target has not been reached
                problemParameters.constraint.epsilon = prevValueEps + stepSizeEps;
            end
            if (prevValue >= finalRho) 
                solverParameters.rho = prevValue + stepSize; 
            end

            if prevStepPass && (prevValue < finalRho) && (prevValueEps < finalEps)
                break
            end
        case 'kappa'
            if prevStepPass, prevValue = problemParameters.dynamics.torqueCostMultiplier; end    
            targetValue = prevValue*rate;
            stepSize = (targetValue-prevValue)/(numFails+1);
            if (prevValue >= finalKappa) % only update parameters if target has not been reached
                problemParameters.dynamics.torqueCostMultiplier = prevValue + stepSize;
            end

            if prevStepPass && (prevValue < finalKappa)
                break
            end
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
        case 'rhokappa'
            if prevStepPass, prevValue = solverParameters.rho; prevValueKappa = problemParameters.dynamics.torqueCostMultiplier; end    
            if numel(values) > 1, finalKappa = values(2); else, finalKappa = finalRho; end
            targetValue = prevValue*rate;
            stepSize = (targetValue-prevValue)/(numFails+1);

            targetValueKappa = prevValueKappa*rate;
            stepSizeKappa = (targetValueKappa-prevValueKappa)/(numFails+1);
            if (prevValueKappa >= finalKappa) % only update parameters if target has not been reached
                problemParameters.dynamics.torqueCostMultiplier = prevValueKappa + stepSizeKappa;
            end
            if (prevValue >= finalRho) 
                solverParameters.rho = prevValue + stepSize; 
            end

            if prevStepPass && (prevValue < finalRho) && (prevValueKappa < finalKappa)
                break
            end
        case 'rhoepsilonkappa'
            if prevStepPass
                prevValue = solverParameters.rho; 
                prevValueKappa = problemParameters.dynamics.torqueCostMultiplier; 
                prevValueEps = problemParameters.constraint.epsilon; 
            end    
            if numel(values) > 1, finalKappa = values(2); finalEps = values(3); else, finalKappa = finalRho; finalEps = finalRho; end
            targetValue = prevValue*rate;
            stepSize = (targetValue-prevValue)/(numFails+1);

            targetValueKappa = prevValueKappa*rate;
            stepSizeKappa = (targetValueKappa-prevValueKappa)/(numFails+1);
            targetValueEps = prevValueEps*rate;
            stepSizeEps = (targetValueEps-prevValueEps)/(numFails+1);
            if (prevValueEps >= finalEps) % only update parameters if target has not been reached
                problemParameters.constraint.epsilon = prevValueEps + stepSizeEps;
            end
            if (prevValueKappa >= finalKappa) % only update parameters if target has not been reached
                problemParameters.dynamics.torqueCostMultiplier = prevValueKappa + stepSizeKappa;
            end
            if (prevValue >= finalRho) 
                solverParameters.rho = prevValue + stepSize; 
            end

            if prevStepPass && (prevValue < finalRho) && (prevValueKappa < finalKappa) && (prevValueEps < finalEps)
                break
            end

        case 'ThrusterConfig' % quick and dirty implementation of thrust value 
            if prevStepPass, prevValue = problemParameters.dynamics.maxThrust(end); end             
            targetValue = values(ii);
            stepSize = (targetValue-prevValue)/(numFails+1);
            problemParameters.dynamics.maxThrust(1:6) = problemParameters.dynamics.maxThrust(1)+problemParameters.dynamics.maxThrust(end) - (prevValue + stepSize);            
            problemParameters.dynamics.maxThrust(7:18) = (prevValue + stepSize);            
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue*1e3,stepSize*1e3,(prevValue+stepSize)*1e3);
        case 'Thrust'
            if prevStepPass, prevValue = problemParameters.dynamics.maxThrust(end); end             
            targetValue = values(ii);
            stepSize = (targetValue-prevValue)/(numFails+1);
            for jj=1:numel(problemParameters.dynamics.maxThrust)
                problemParameters.dynamics.maxThrust(jj) = (prevValue + stepSize);
            end
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue*1e3,stepSize*1e3,(prevValue+stepSize)*1e3);
        case 'ExhaustVelocity'
            if prevStepPass, prevValue = problemParameters.dynamics.exhaustVelocity; end             
            targetValue = values(ii);
            stepSize = (targetValue-prevValue)/(numFails+1);
            problemParameters.dynamics.exhaustVelocity=(prevValue + stepSize);
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
        case 'Inertia'
            if prevStepPass
                prevValue = problemParameters.dynamics.inertia(2,2); 
                prevI = problemParameters.dynamics.inertia; 
                prevInverse = problemParameters.dynamics.inertiaInverse; 
            end 
            targetValue = values(ii);
            stepSize = (targetValue-prevValue)/(numFails+1);
            newVal = (prevValue + stepSize);
            factorIncrease = newVal/prevValue;
            problemParameters.dynamics.inertia = factorIncrease.* prevI;
            problemParameters.dynamics.inertiaInverse = prevInverse./factorIncrease;
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
        case 'ControlTorqueCost'
            if prevStepPass, prevValue = problemParameters.dynamics.torqueCostMultiplier; end    
            targetValue = values(ii);            
            stepSize = (targetValue-prevValue)/(numFails+1);
            problemParameters.dynamics.torqueCostMultiplier = prevValue + stepSize;
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
    end     

    fprintf('iterIdx = %d\trho=%f\n',iter,solverParameters.rho);
    %if numel(solverParameters.initialCostateGuess) > 10
        if oldSweep 
            solveFunc = @OLD_Solve6DOFPointingConstrainedControlProblem;
        elseif strcmp(solution.problemParameters.dynamics.type, 'CRTBP')
            solveFunc = @Solve6DOFPointingConstrainedControlProblemCRTBP;
        else
            solveFunc = @Solve6DOFPointingConstrainedControlProblem;
        end
        
        if (solverParameters.rho < .2) || (strcmp(type,'Radius')) || (strcmp(type,'kappa')) || (strcmp(type,'Ellipsoid1')) || (strcmp(type,'Ellipsoid2'))
            solverParameters.fSolveOptions.MaxIterations = 30;
        else
            solverParameters.fSolveOptions.MaxIterations = 300;

        end
        %solverParameters.fSolveOptions.MaxIterations = 400;
        solverParameters.fSolveOptions.MaxFunctionEvaluations = 2000;
        %  optimoptions('fsolve','Display','iter','MaxFunEvals',1e3,...
        % 'MaxIter',3e1,'TolFun',1e-12,'TolX',1e-12,'StepTolerance',1e-12,...
        % 'UseParallel',true); % fsolve    
    
    %else 
        %solveFunc = @SolvePointingConstrainedControlProblem;
    %end 
    if iter > 1 && isfield(solution,'finalStateError') 
        % compare two different costate guesses - use smoothness of the
        % homotopy paths to pick a new costate guess based on previous step
        guess1 = newSols(iter).newCostateGuess; % solution.newCostateGuess;
        solverParameters.initialCostateGuess = guess1;
        % if false && (norm(prevPassedStepSize) > 0)
        %     sol1 = solveFunc(problemParameters,solverParameters,true);
        %     guess2 = newSols(iter).newCostateGuess + (newSols(iter).newCostateGuess-newSols(iter-1).newCostateGuess)*norm(stepSize)/norm(prevPassedStepSize);
        %     solverParameters.initialCostateGuess = guess2;
        %     sol2 = solveFunc(problemParameters,solverParameters,true);
        %     if nosrm(sol1.finalStateError) < norm(sol2.finalStateError)
        %         solverParameters.initialCostateGuess = guess1;
        %     end
        % end
    end 

    solution = solveFunc(problemParameters,solverParameters); 

    debug = false; 
    if debug % Put a breakpoint in here to plot latest solutions
        sol = newSols(1);
        PlotSolution.ConvergedCostateTrace(newSols(1:(end-1)))

        PlotSolution.Sweep6DOF(newSols(1:(end-1)))
        PlotSolution.RotationalSummary(sol);PlotSolution.ThrustProfileAllEngines(sol) 
        PlotSolution.SixDOF_Traj(sol)
        PlotSolution.OrientationAtTime(sol,48,gca)

        PlotSolution.PlotOrientationChaserRelativeToTranslationalFrame(sol.x(end,8:10)',sol.x(end,1:3)'*1e3,sol.t(end),sol.problemParameters,gca)
        
        sol.solverParameters.rho = 1e-4;
        s2 = RerunSolution(sol); 
        sol = s2; sol.solverParameters.initialCostateGuess=sol.newCostateGuess;
        PlotSolution.CostBreakdown(sol)
    end

    if isfield(solution,'finalStateError')
        costateDelta = solution.newCostateGuess-newSols(iter).newCostateGuess;
        [maxVal,maxIdx] = max(abs(costateDelta));
        maxPctChange = maxVal./abs(newSols(iter).newCostateGuess(maxIdx))* 100;

        costateDeltaNormPct = norm(costateDelta)/norm(newSols(iter).newCostateGuess);
        solPass = (norm(solution.finalStateError)<1e-7) || ...
            ((norm(solution.finalStateError)<1e-4) && (costateDeltaNormPct < 1e-2) && maxPctChange < 0.1 && solverParameters.rho < 3e-2) || ...
            ((norm(solution.finalStateError)<1e-6) && ( ~any(~solution.solutionFound)));% || ...
            %((norm(solution.finalStateError)<1e-4) && ( ~any(~solution.solutionFound))) || ...
            
    else
        solPass = ~any(~solution.solutionFound);
    end
    if ~solPass && dynamicStepSize % 1e-7 % less than 0.1mm error
        warning('Solution did not converge');        
        numFails = numFails+1;
        prevStepPass = false;            
        if dynamicRhoSpeedUp
            rate = min(.999,rate+.1);        
        end
    else 
        iter = iter+1;
        if prevStepPass && (abs(stepSize)>=abs(targetValue-prevValue))
            ii = ii+1;
        end 
        prevStepPass = true;
        numFails = max(0,numFails-1); % successful step!
        newSols(iter) = solution;
        prevPassedStepSize = stepSize;
        if dynamicRhoSpeedUp
            rate = max(rhoMin,rate-.05);        
        end
    end 
end 
end