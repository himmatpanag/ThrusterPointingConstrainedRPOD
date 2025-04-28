function dydt = crtbp_dynamics(y, xL, mu_Earth, mu_Sun, r1shift, r2shift, w)
    dydt = zeros(12,1); 
    
    % Extract absolute motion (r)
    r = y(1:3);
    v = y(4:6);
    
    % Extract relative motion (z)
    z = y(7:9);
    dz = y(10:12);

    % Compute r1T and r2T
    r1T = [r(1) + r1shift; r(2); r(3)];
    r2T = [r(1) + r2shift; r(2); r(3)];
    
    r_1TR = r1T + z; 
    r_2TR = r2T + z; 

    % Absolute motion equations (dr/dt)
    dydt(1:3) = v;
    dydt(4) = (w^2)*(r(1) + xL) + 2*w*v(2) ...
        - mu_Sun * ((r(1) + r1shift) / norm(r1T)^3) ...
        - mu_Earth * ((r(1) + r2shift) / norm(r2T)^3);
    
    dydt(5) = -2*w*v(1) + (w^2) * r(2) ...
        - mu_Sun * (r(2) / norm(r1T)^3) ...
        - mu_Earth * (r(2) / norm(r2T)^3);
    
    dydt(6) = - mu_Sun * (r(3) / norm(r1T)^3) ...
        - mu_Earth * (r(3) / norm(r2T)^3);

    % Relative motion equations (dz/dt) using results from target position
    dydt(7:9) = dz;
    dydt(10) = (w^2)*z(1) + 2*w*dz(2) ...
        + mu_Sun * ((r1T(1) / norm(r1T)^3) - (r1T(1) + z(1)) / norm(r_1TR)^3) ...
        + mu_Earth * ((r2T(1) / norm(r2T)^3) - (r2T(1) + z(1)) / norm(r_2TR)^3);

    dydt(11) = -2*w*dz(1) + (w^2) * z(2) ...
        + mu_Sun * ((r1T(2) / norm(r1T)^3) - (r1T(2) + z(2)) / norm(r_1TR)^3) ...
        + mu_Earth * ((r2T(2) / norm(r2T)^3) - (r2T(2) + z(2)) / norm(r_2TR)^3);

    dydt(12) = mu_Sun * ((r1T(3) / norm(r1T)^3) - (r1T(3) + z(3)) / norm(r_1TR)^3) ...
        + mu_Earth * ((r2T(3) / norm(r2T)^3) - (r2T(3) + z(3)) / norm(r_2TR)^3);
end