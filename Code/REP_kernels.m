function [kernels] = REP_kernels()

    % set directional kernels
    sigma = 11;
    kernel_length = 61;
    kernel_width = 9;
    kernel_height = kernel_width;
    
    % form kernels
    kernel_x = directional_kernel_3d('x', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;
    kernel_y = directional_kernel_3d('y', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;
    kernel_z = directional_kernel_3d('z', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, 'gaussian')/1000;
    
    kernels = {kernel_x, kernel_y, kernel_z};


end