function s = PlotEllipsoidConstraint(ax,parameters)
    if nargin < 2
        c0 = [0;0;0];
    end
    axes(ax);
    hold on; 
    % Ellipsoid
    a1 = parameters.constraint.targetAxisx * 1e3;  % in m
    a2 = parameters.constraint.targetAxisy * 1e3;
    a3 = parameters.constraint.targetAxisz * 1e3;

    % c0 = [ parameters.constraint.offsetx
    %    parameters.constraint.offsety
    %    parameters.constraint.offsetz ] * 1e3;    % also in m
    % 
    
    % Plot the ellipsoid surface in LVLH frame
    [u_grid, v_grid] = meshgrid(linspace(0, 2*pi, 50), linspace(0, pi, 25));
    x = a1 * sin(v_grid) .* cos(u_grid);
    y = a2 * sin(v_grid) .* sin(u_grid);
    z = a3 * cos(v_grid);
    
    % Preallocate arrays for the transformed surface points.
    X = zeros(size(x)); 
    Y = zeros(size(x)); 
    Z = zeros(size(x));
    
    % Rotate each point using U_total and then translate by c0_LVLH.
    for i = 1:size(x,1)
        for j = 1:size(x,2)
            pt = [x(i,j); y(i,j); z(i,j)];
            pt_rot = parameters.constraint.U * pt;  % rotate the ellipsoid point
            % Translate to the ellipsoid center in LVLH:
            X(i,j) = pt_rot(1) + parameters.constraint.offsetx;
            Y(i,j) = pt_rot(2) + parameters.constraint.offsety;
            Z(i,j) = pt_rot(3) + parameters.constraint.offsetz;
        end
    end
    
    s = surf(X, Y, Z, 'FaceColor',[0,153,255]./255,'FaceAlpha',0.3,'DisplayName',['Constrained Ellipsoid']);
    axis equal
    axis auto
    set(s,'FaceAlpha',0.3);
    set(s,'LineStyle','--');
    set(s,'EdgeAlpha',0.3);
    title('Rotated Ellipsoid in LVLH Frame');
    grid on;

end
