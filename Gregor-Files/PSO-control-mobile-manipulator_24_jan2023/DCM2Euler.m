
function EulerAngles=DCM2Euler(R)
      % Euler 321 notacija 
      % R rotacijska matrika podana kot v knjigi AMS [cos sin; -sin cos]

    ax = atan2(R(2,3),R(3,3));  % 
    ay = - asin(R(1,3));
    az = atan2(R(1,2),R(1,1));
    EulerAngles=[ax, ay, az];
end