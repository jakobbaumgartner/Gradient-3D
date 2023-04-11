function d = distPointLine(x0, y0, x1, y1, x2, y2)
    % Calculates the distance between a point and a line defined by two points
    % (x1,y1) and (x2,y2).
    
    numerator = abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1));
    denominator = sqrt((x2-x1)^2 + (y2-y1)^2);
    d = numerator / denominator;
    
end
