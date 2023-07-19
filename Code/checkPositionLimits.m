function [jointsWithinLimits, isWithinLimits] = checkPositionLimits(position)

    combinedLimits = [ 
        -2.7437 2.7437 ; 
        -1.7837 1.7837 ;
        -2.9007 2.9007 ;
        -3.0421 -0.1518 ;
        -2.8065 2.8065 ;
        0.5445 4.5169 ;
        -3.0159 3.0159
    ];

    isWithinLimits = position >= combinedLimits(:, 1) & position <= combinedLimits(:, 2);
    
    jointsWithinLimits = (position <= combinedLimits(:, 1)) .* combinedLimits(:, 1) ...
                       + (position >= combinedLimits(:, 2)) .* combinedLimits(:, 2) ...
                       + (position > combinedLimits(:, 1)) .* (position < combinedLimits(:, 2)) .* position;

end