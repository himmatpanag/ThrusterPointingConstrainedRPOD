function [numEngines, locations, thrustDirection] = GetEngineLocations(engineConfiguration)
   
    switch engineConfiguration 
        case THRUSTER_CONFIGURATION.CG_ALIGNED_6  % in this case the locations of the engines are unimportant
            % for simplicity we put them at the centre of each face.
            % This spacecraft does not have any control attitude authority 
            numEngines = 6;
            locations = .5.*[eye(3),-eye(3)];
            thrustDirection = -locations./vecnorm(locations);
        case THRUSTER_CONFIGURATION.CG_ALIGNED_CANTED_8  % Put 8 engines, one on each corner and aligned with the CG
            % This spacecraft does not have any control attitude authority 
            numEngines = 8;
            corners1 = .5.*[1,1,1,1;
                       1,1,-1,-1;
                    1,-1,-1,1];
            locations = [corners1,-corners1];
            thrustDirection = -locations./vecnorm(locations);
        case THRUSTER_CONFIGURATION.RCS_12    % In this case we have 12 orthogonal thrusters so that (6 pairs of 2). 
            % Firing one in the pair will result in a translational and
            % rotational acceleration. Firing both in the pair will cause a translational acceleration only
            numEngines = 12;
            corners = .5.*[1,1,-1,-1;
                       1,-1,1,-1;
                       1,-1,-1,1];
            locations = repmat(corners,1,3);
            thrustDirection = zeros(3,12); mm = 1;
            for jj = 1:3
                for ii = 1:4
                    thrustDirection(jj,mm) = -locations(jj,mm);
                    mm = mm + 1; 
                end
            end
        case THRUSTER_CONFIGURATION.CG_6_RCS_12
            [numEngines1, locations1, thrustDirection1] = GetEngineLocations(THRUSTER_CONFIGURATION.CG_ALIGNED_6);
            [numEngines2, locations2, thrustDirection2] = GetEngineLocations(THRUSTER_CONFIGURATION.RCS_12);
            numEngines = numEngines1 + numEngines2; 
            locations = [locations1,locations2];
            thrustDirection = [thrustDirection1,thrustDirection2];
            
        case THRUSTER_CONFIGURATION.RCS_14_CANTED
            [~, locations, thrustDirection] = GetEngineLocations(THRUSTER_CONFIGURATION.RCS_12);
            numEngines = 14; 
            locations = [locations,.5.*[-1;1;1],.5.*[-1;-1;-1]];
            thrustDirection = [thrustDirection,[1;0;0],[1;0;0]];
            for ii = [3,4,13,14]
                r = cross(locations(:,ii),thrustDirection(:,ii));
                thrustDirection(:,ii) = RodriguesRotation(thrustDirection(:,ii),30*pi/180,r);
            end
            % 40 degree cant angles in the docking axis.
        case THRUSTER_CONFIGURATION.RCS_CANTED      
            % Put 6 thrusters on two corners of cube on the positive y face. 
            numEngines = 18;
            corners = .5.*[1,1,-1,-1;
                       1,-1,1,-1;
                       1,-1,-1,1];
            locations = repmat(corners(:,[1,3]),1,3);
            thrustDirection = zeros(3,6); mm = 1;
            for jj = 1:3
                for ii = 1:2
                    thrustDirection(jj,mm) = -locations(jj,mm);
                    mm = mm + 1; 
                end
            end
            % Put 12 canted thrusters on the negative y face
            corners2 = .5.*[[1;-1;1],[-1;-1;-1],[1;-1;-1],[-1;-1;1]];
            thrusDir = [0;1;0];
            for jj = 1:4
                for ii = 1:3
                    locations = [locations,corners2(:,jj)];
                    angle = 40*pi/180;
                    if ii ==1
                        thrustDirection = [thrustDirection,-[corners2(1,jj)*sin(angle);corners2(2,jj)*cos(angle);0]];
                    elseif ii == 2 
                        thrustDirection = [thrustDirection,-[0;corners2(2,jj)*cos(angle);corners2(3,jj)*sin(angle)]];
                    else 
                        thrustDirection = [thrustDirection,RodriguesRotation(thrusDir,30*pi/180,cross(corners2(:,jj),thrusDir))];
                    end 
                end 
            end
    end 
end 