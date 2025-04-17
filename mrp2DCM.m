function DCM = mrp2DCM(p)
    p2 = p'*p;
    pCross = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    temp = (1+p2)^2;

    % Translational dynamics are in a rotating reference frame
    DCM = eye(3) - 4*(1-p2)/temp*pCross + 8/temp * pCross^2;

end