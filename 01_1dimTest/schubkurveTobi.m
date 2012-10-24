function [schub]=schubkurveTobi(u)
    % return is Force
    % input 0 - 511
    % output is ThrustForce in z Direction of the body frame
    % u=302 is hoovering
    schub=2.371190582616025*10^(-5)*u^2+0.004587809331818*u+0.419806660877117;
end