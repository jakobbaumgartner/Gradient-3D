function K = gaussian_kernel_3d(m, n, p, sigma)

    % GAUSSIANFILTER3D Generates a 3D Gaussian filter kernel with standard deviation sigma.
    %
    %   K = GAUSSIANFILTER3D(m, n, p, sigma) generates a 3D Gaussian filter kernel with size
    %   m x n x p and standard deviation sigma. The output kernel is normalized so that the sum
    %   of its elements is equal to 1.
    %
    %   Input arguments:
    %   - m, n, p: Size of the kernel in the x, y, and z directions, respectively.
    %   - sigma: Standard deviation of the Gaussian function.
    %
    %   Output argument:
    %   - K: 3D Gaussian filter kernel.
    
    [x,y,z] = meshgrid(-floor(m/2):floor(m/2), -floor(n/2):floor(n/2), -floor(p/2):floor(p/2));
    K = exp(-(x.^2 + y.^2 + z.^2)/(2*sigma^2)) / (2*pi*sigma^2)^1.5;
    K = {K / sum(K(:))}; % normalize the kernel

end
