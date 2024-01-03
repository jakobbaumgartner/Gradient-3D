function [kernels] = REP_kernels(type)

    % REP_kernels - Generate directional kernels for a 3D image processing task
    %
    % Usage:
    %    [kernels] = REP_kernels(type)
    %
    % Description:
    %    This function generates directional kernels for a 3D image processing task.
    %    It creates directional kernels along the x, y, and z axes using Gaussian functions.
    %
    % Dependencies:
    %   The directional_kernel_3d function is used internally to generate the kernels. It is
    %   expected to be defined elsewhere in your codebase with the following signature:
    %   directional_kernel_3d(direction, length, sigma1, width, sigma2, height, sigma3, type)
    %
    % Inputs:
    %    type: 'linear' or 'gaussian'
    %
    % Outputs:
    %    kernels - A cell array containing the generated directional kernels.

    type

    % set directional kernels
    sigma = 2
    kernel_length = 10 % floored to even number
    kernel_width = 5
    kernel_height = kernel_width
    
    % form kernels
    kernel_x = directional_kernel_3d('x', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, type);
    kernel_y = directional_kernel_3d('y', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, type);
    kernel_z = directional_kernel_3d('z', kernel_length, sigma, kernel_width, sigma, kernel_height, sigma, type);

%     figure()
%     scatter(1:1:(length(kernel_x(5,:,5))),kernel_x(5,:,5))
%     hold on
%     plot(1:1:(length(kernel_x(5,:,5))),kernel_x(5,:,5))

%     
    kernels = {kernel_x, kernel_y, kernel_z};


end