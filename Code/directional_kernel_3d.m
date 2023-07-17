function [kernel] = directional_kernel_3d(direction, kernel_size, kernel_sigma, kernel_num_layers, kernel_layers_sigma, kernel_type)

    % check inputs
    % =========================================================================

    % check that number of layers is odd
    if mod(kernel_num_layers, 2) == 0
        error('Number of layers must be odd');
    end

    % check that kernel size is odd
    if mod(kernel_size, 2) == 0
        error('Kernel size must be odd');
    end

    % check that direction is: 'x', 'y' or 'z'
    if ~strcmp(direction, 'x') && ~strcmp(direction, 'y') && ~strcmp(direction, 'z')
        error('Direction must be: ''x'', ''y'' or ''z''');
    end

    % check that kernel type is: 'gaussian', 'linear' or 'box'
    if ~strcmp(kernel_type, 'gaussian') && ~strcmp(kernel_type, 'linear') && ~strcmp(kernel_type, 'box')
        error('Kernel type must be: ''gaussian'', ''linear'' or ''box''');
    end

    % check that kernel sigma is a scalar
    if ~isscalar(kernel_sigma)
        error('Kernel sigma must be a scalar');
    end

    % check that kernel layers sigma is a scalar
    if ~isscalar(kernel_layers_sigma)
        error('Kernel layers sigma must be a scalar');
    end


    % create kernel
    % =========================================================================
    
    % x kernel
    if strcmp(direction, 'x')
        kernel = zeros(kernel_size, kernel_size, kernel_num_layers);
        for i = 1:kernel_num_layers
            kernel(:, :, i) = directional_kernel_2d('x', kernel_size, kernel_sigma, kernel_type);
        end
    end

    % y kernel

    % z kernel
    






    





end