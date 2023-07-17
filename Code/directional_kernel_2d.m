function [kernel] = directional_kernel_2d(direction, kernel_length, kernel_width, kernel_sigma, kernel_type)

    % x - direction
    % ------------------------------------

    kernel = zeros(kernel_size, kernel_size);
    center = ceil(kernel_size/2); % mid of the kernel


    if kernel_type == 'box'

        % prepare directional weights
        weights = [1:1:center center+1 -flip(1:1:center)] / (center+1); 

        % extend weights to kernel width
        kernel = ones(kernel_width,1) * weights






    elseif kernel_type == 'gaussian'

        % The standard deviation is usually set to be (Size/2)/3
        sigma = floor(kernel_width/2)/3; 

        [X,Y] = meshgrid(-(center-1):(center-1));
        kernel = exp(-(X.^2) / (2* sigma^2));
        kernel = kernel / sum(kernel(:)) % Normalize the kernel

    end







end