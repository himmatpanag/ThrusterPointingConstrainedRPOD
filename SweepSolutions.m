function newSols = SweepSolutions(initialSolution,type,values,dynamicStepSize)
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
    rhoRate = 0.9;    
    finalRho = values(1);
    N = ceil(log(finalRho/solverParameters.rho)/log(rhoRate));
    if dynamicStepSize
        dynamicRhoSpeedUp = true;
    end
elseif contains(type,'epsilon')
    epsilonRate = 0.9;    
    final_epsilon = values(1);
    N = ceil(log(final_epsilon/problemParameters.constraint.epsilon)/log(epsilonRate));
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
        case 'FinalPosition'
            if prevStepPass, prevValue = problemParameters.xf(1); end
            targetValue = values(ii);            
            stepSize = (values(ii)-prevValue)/(numFails+1);
            problemParameters.xf(1) = prevValue + stepSize;
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
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
            targetValue = prevValue*rhoRate;
            stepSize = (targetValue-prevValue)/(numFails+1);
            solverParameters.rho = prevValue + stepSize; 
            if targetValue*rhoRate < finalRho
                break
            end
        case 'epsilon'
            if prevStepPass, prevValue = problemParameters.constraint.epsilon; end
            targetValue = prevValue*epsilonRate;
            stepSize = (targetValue-prevValue)/(numFails+1);
            problemParameters.constraint.epsilon = prevValue + stepSize;
            if targetValue*epsilonRate < final_epsilon
                break
            end
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValue,stepSize,(prevValue+stepSize));
        case 'rhoAngle'
            if prevStepPass, prevValue = solverParameters.rho; prevValueAng = problemParameters.constraint.alpha0; end                         

            step = interpn([1,N],[initialSolution.problemParameters.constraint.alpha0, values(2)],ii) - prevValueAng;
            stepSize = step/(numFails+1);   
            problemParameters.constraint.alpha0 = prevValueAng + stepSize; 
            fprintf('prevValue %f\t stepSize\t%f newValue%f\t',prevValueAng*180/pi,stepSize*180/pi,(prevValueAng+stepSize)*180/pi);

            targetValue = prevValue*rhoRate;
            stepSize = (targetValue-prevValue)/(numFails+1);
            solverParameters.rho = prevValue + stepSize;             
    end     

    fprintf('iterIdx = %d\trho=%f\n',ii,solverParameters.rho);
    if numel(solverParameters.initialCostateGuess) > 10
        solution = Solve6DOFPointingConstrainedControlProblem(problemParameters,solverParameters);
    else 
        solution = SolvePointingConstrainedControlProblem(problemParameters,solverParameters);
    end 
    if norm(solution.x(end,1:6)'-solution.problemParameters.xf) > 1e-6 && dynamicStepSize % 1e-8 % less than 0.1mm error
        warning('Solution did not converge');        
        numFails = numFails+1;
        prevStepPass = false;            
        if dynamicRhoSpeedUp
            rhoRate = min(.95,rhoRate+.1);        
        end
    else 
        iter = iter+1;
        if prevStepPass && (abs(stepSize)>=abs(targetValue-prevValue))
            ii = ii+1;
        end 
        prevStepPass = true;
        numFails = max(0,numFails-1); % successful step!
        newSols(iter) = solution;
        solverParameters.initialCostateGuess = solution.newCostateGuess;        
        if dynamicRhoSpeedUp
            rhoRate = max(rhoMin,rhoRate-.1);        
        end
    end 

end 
end