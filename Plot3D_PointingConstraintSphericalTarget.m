function hFig = Plot3D_PointingConstraintSphericalTarget(ax,posChaser,r,uFree)
if nargin < 4
    uFree = [];
end

if nargin <3
    r=2;
end

    if nargin < 2
        posChaser = 1.5*[3;1;0.5];
    end

    if nargin < 1
        % Create a figure and axes
        hFig = figure('Name','Pointing Constraint');
        ax = gca; grid on; hold on; axis equal;
    else 
        axes(ax); grid on; hold on; axis equal;
    end

    edgeColour = 'none';
    N = 200;    

    % Get circle on target which corresponds to plume boundary
    theta = linspace(0,2*pi,N);
    a = (r^2/norm(posChaser));
    chaserDir = posChaser./norm(posChaser);
    aVec = chaserDir .* a;
    randVec = rand(3,1);
    randVec =randVec./norm(randVec);
    v1 = cross(randVec,chaserDir);
    v1 = v1./norm(v1);

    v2 = cross(v1,chaserDir);
    v2 = v2./norm(v2);
    
    AB = sqrt(r^2-a^2); 
    circle = aVec + AB.*cos(theta).*v1 + AB.*sin(theta).*v2;
    
    xC = circle(1,:);
    yC = circle(2,:);
    zC = circle(3,:);
 
    % Project the mirror segment onto the surface of the sphere
    for ii = 1:length(xC)
        [xCProj(ii),yCProj(ii),zCProj(ii)] = ProjectOntoSphere(xC(ii),yC(ii),zC(ii),posChaser,1);
        [NegxCProj(ii),NegyCProj(ii),NegzCProj(ii)] = ProjectOntoSphere(xC(ii),yC(ii),zC(ii),posChaser,-1);
    end

    % Plot sphere of radius 1 centred at posChaser, but remove the projected segment.
    [x,y,z] = sphere(5*N);
    xSphere = x + posChaser(1);
    ySphere = y + posChaser(2);   
    zSphere = z + posChaser(3);   

    % remove the projected segment
    rads = linspace(0,2,N);
    for ii = 1:size(x,1)
        for jj = 1:size(x,2)
            plume = -[x(ii,jj);y(ii,jj);z(ii,jj)];
            [xPlane,yPlane,zPlane] = ProjectVectorOntoPlane(plume,posChaser,aVec,chaserDir);
            if (plume'*(-posChaser) > 0) && (norm([xPlane;yPlane;zPlane]) < r)
                xSphere(ii,jj) = NaN;
                ySphere(ii,jj) = NaN;
                zSphere(ii,jj) = NaN;
            end             
        end
    end 

    if false % ~isempty(uFree)
        % Plot the Hamiltonian on the surface of the sphere. 
        for ii = 1:size(x,1)
            for jj = 1:size(x,2)
                u = [x(ii,jj);y(ii,jj);z(ii,jj)];
                H(ii,jj) = dot(-uFree,u); % uFree = -lambdaV/|lambdaV|
            end
        end 
        surf(xSphere,ySphere,zSphere,H,"EdgeColor",edgeColour,'FaceAlpha',0.7,'DisplayName','Available Control Set')
        h = colorbar;
        set(get(h,'label'),'string','Hamiltonian on sphere');
        quiver3(posChaser(1),posChaser(2),posChaser(3),2*uFree(1),2*uFree(2),2*uFree(3),...
            '-k','LineWidth',1,'MaxHeadSize',1,'DisplayName','Optimal choice of u')
    end 

    for ii = 1:N+1
        for jj = 1:N
            xC_PlumeLimit = xC(jj);
            yC_PlumeLimit = yC(jj);
            zC_PlumeLimit = zC(jj);
            [xCProjCone(ii,jj),yCProjCone(ii,jj),zCProjCone(ii,jj)] = ProjectOntoSphere(xC_PlumeLimit,yC_PlumeLimit,zC_PlumeLimit,posChaser,-(ii-1)/N);

            projectedCone = [xC_PlumeLimit;yC_PlumeLimit;zC_PlumeLimit]*(1-(ii-1)/N) + posChaser*(ii-1)/N;
            xProjectedCone(ii,jj) = projectedCone(1);
            yProjectedCone(ii,jj) = projectedCone(2);
            zProjectedCone(ii,jj) = projectedCone(3);
        end
    end 

    %% All plotting
    plot3(0,0,0,'mx','DisplayName','Target centre','MarkerSize',20);
    plot3(posChaser(1),posChaser(2),posChaser(3),'rx','DisplayName','Chaser','MarkerSize',20)
    surf(r.*x,r.*y,r.*z,'EdgeColor',edgeColour,'FaceColor', 'c','FaceAlpha',0.2,'DisplayName',['Spherical target r=',num2str(r)]);  
    h = light; 
    h.Position = posChaser*2;

    % plot sphere with constant face colour
    surf(xSphere,ySphere,zSphere,'EdgeColor',edgeColour,'FaceColor','b','FaceAlpha',0.3,'DisplayName','Available Control Set');  
    
    % plot the circle
    %plot3(xC,yC,zC,'Color',[0.8,1,0.8],'LineWidth',2,'HandleVisibility','off','DisplayName','Plume Boundary on Target');
    % shade the circle in light red 
    %fill3(xC,yC,zC,[1,0.8,0.8],'FaceAlpha',0.1,'HandleVisibility','off');
    
    % plot the projected circle
    plot3(NegxCProj,NegyCProj,NegzCProj,'Color','r','LineWidth',2,'DisplayName','Boundary of Available Control Set');
    plot3(xCProj,yCProj,zCProj,'Color','c','LineWidth',2,'DisplayName','Spherical target projected onto control set');    
    
    surf(xCProjCone,yCProjCone,zCProjCone,'EdgeColor',edgeColour,'FaceColor','w','FaceAlpha',0.5,'HandleVisibility','off')
    % plot projected cone in light green
    surf(xProjectedCone,yProjectedCone,zProjectedCone,'EdgeColor',edgeColour,'FaceColor',[0.8,1,0.8],'FaceAlpha',0.4,...
    'DisplayName','Plume restricted region');

    view([42,20]);

    xlabel('x'); ylabel('y'); zlabel('z')
    title('Thrust pointing constrained control set with spherical target')
    legend('show','Location','best')


end 

function [x,y,z] = ProjectVectorOntoPlane(vector,originVector,planeOrigin,planeNormal)
    n = planeNormal;
    m = planeOrigin;
    r = originVector;
    plumeDirFree = vector;

    t_AlongLine = (-n'*(r-m))/(n'*plumeDirFree);
    PlumeProjectedPosGlobal = r+t_AlongLine*plumeDirFree; % position vector on plane of curve. 
    
    x = PlumeProjectedPosGlobal(1);
    y = PlumeProjectedPosGlobal(2);
    z = PlumeProjectedPosGlobal(3);        

end

function [x,y,z] = ProjectOntoSphere(x,y,z,centre,radius)
    % Project the points (x,y,z) onto the sphere of radius radius centred at centre

    % Find the vector from the centre of the sphere to the point
    v = [x;y;z] - centre;

    % Find the length of the vector
    vLength = sqrt(v(1)^2 + v(2)^2 + v(3)^2);

    % Find the unit vector in the direction of v
    vUnit = v/vLength;

    % Find the point on the sphere
    x = centre(1) + radius*vUnit(1);
    y = centre(2) + radius*vUnit(2);
    z = centre(3) + radius*vUnit(3);

end