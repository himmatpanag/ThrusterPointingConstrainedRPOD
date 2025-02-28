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
    rate = 0.99;    
    finalRho = values(1);
    N = ceil(log(finalRho/solverParameters.rho)/log(rate));
    if dynamicStepSize
        dynamicRhoSpeedUp = true;
    end
elseif contains(type,'epsilon')
    rate = 0.9;    
    final_epsilon = values(1);
    N = ceil(log(final_epsilon/problemParameters.constraint.epsilon)/log(rate));
    if dynamicStepSize
        dynamicRhoSpeedUp = true;
    end
else 
    N = numel(values);
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


        case 'ThrusterConfig' % quick and dirty implementation of thrust value 
            if prevStepPass, prevValue = problemParameters.dynamics.maxThrust(end); end             
            targetValue = values(ii);
            stepSize = (targetValue-prevValue)/(numFails+1);
            problemParameters.dynamics.maxThrust(1:6) = problemParameters.dynamics.maxThrust(1)+problemParameters.dynamics.maxThrust(end) - (prevValue + stepSize);            
            problemParameters.dynamics.maxThrust(7:18) = (prevValue + stepSize);            
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue*1e3,stepSize*1e3,(prevValue+stepSize)*1e3);
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
    if numel(solverParameters.initialCostateGuess) > 10
        if oldSweep 
            solveFunc = @OLD_Solve6DOFPointingConstrainedControlProblem;
        else
            solveFunc = @Solve6DOFPointingConstrainedControlProblem;
        end
        if solverParameters.rho < .01
            solverParameters.fSolveOptions.MaxIterations = 30;
        else
            solverParameters.fSolveOptions.MaxIterations = 300;
        end
        solverParameters.fSolveOptions.MaxFunctionEvaluations = 2000;
        %  optimoptions('fsolve','Display','iter','MaxFunEvals',1e3,...
        % 'MaxIter',3e1,'TolFun',1e-12,'TolX',1e-12,'StepTolerance',1e-12,...
        % 'UseParallel',true); % fsolve    
    
    else 
        solveFunc = @SolvePointingConstrainedControlProblem;
    end 
    if iter > 1 && isfield(solution,'finalStateError') 
        % compare two different costate guesses - use smoothness of the
        % homotopy paths to pick a new costate guess based on previous step
        guess1 = newSols(iter).newCostateGuess; % solution.newCostateGuess;
        solverParameters.initialCostateGuess = guess1;
        if false && (norm(prevPassedStepSize) > 0)
            sol1 = solveFunc(problemParameters,solverParameters,true);
            guess2 = newSols(iter).newCostateGuess + (newSols(iter).newCostateGuess-newSols(iter-1).newCostateGuess)*norm(stepSize)/norm(prevPassedStepSize);
            solverParameters.initialCostateGuess = guess2;
            sol2 = solveFunc(problemParameters,solverParameters,true);
            if norm(sol1.finalStateError) < norm(sol2.finalStateError)
                solverParameters.initialCostateGuess = guess1;
            end
        end
    end 

    solution = solveFunc(problemParameters,solverParameters);

    debug = false; 
    if debug % Put a breakpoint in here to plot latest solutions
        sol = newSols(end-1);
        PlotSolution.ConvergedCostateTrace(newSols(1:(end-1)))
        PlotSolution.RotationalSummary(sol); PlotSolution.ThrustProfileAllEngines(sol)
    end

    if isfield(solution,'finalStateError')
        solPass = (norm(solution.finalStateError)<1e-8)|| ( ~any(~solution.solutionFound));
    else
        solPass = ~any(~solution.solutionFound);
    end
    if ~solPass && dynamicStepSize % 1e-7 % less than 0.1mm error
        warning('Solution did not converge');        
        numFails = numFails+1;
        prevStepPass = false;            
        if dynamicRhoSpeedUp
            rate = min(.99,rate+.1);        
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
            rate = max(rhoMin,rate-.02);        
        end
    end 
end 
end