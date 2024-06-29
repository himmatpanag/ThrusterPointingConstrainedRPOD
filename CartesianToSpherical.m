function [r,theta,phi] = CartesianToSpherical(vec)
    x = vec(1); y = vec(2); z = vec(3);
    rxy = sqrt(x^2 + y^2);
    r = sqrt(x^2 + y^2 + z^2);
    phi = atan(z/rxy);
    theta = atan2(y,x);
end