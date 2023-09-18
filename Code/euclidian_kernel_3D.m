function K = euclidian_kernel_3D(m, n, p)

    % LINEARDISTANCETRANSFORM3D Generates a 3D linear distance transform kernel.
    %
    %   K = LINEARDISTANCETRANSFORM3D(m, n, p) generates a 3D linear distance transform kernel with size
    %   m x n x p. The values in the kernel represent the linear distance from the center of the kernel,
    %   with the maximum value at the center decreasing linearly towards the edges.
    %
    %   Input arguments:
    %   - m, n, p: Size of the kernel in the x, y, and z directions, respectively.
    %
    %   Output argument:
    %   - K: 3D linear distance transform kernel.

    [x,y,z] = meshgrid(-floor(m/2):floor(m/2), -floor(n/2):floor(n/2), -floor(p/2):floor(p/2));
    dist = sqrt(x.^2 + y.^2 + z.^2); % Calculate the Euclidean distance to the center
    maxDist = sqrt((m/2)^2 + (n/2)^2 + (p/2)^2); % Calculate the maximum possible distance

    K = (maxDist - dist) / maxDist; % Generate the kernel by subtracting the distances from the maximum distance
    K = {max(0, K)}; % Remove negative values which might occur due to flooring in meshgrid

end
