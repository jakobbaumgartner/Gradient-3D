function [kernels] = REP_kernels(varargin)

    % REP_kernels - Generate directional kernels for a 3D image processing task
    %
    % Usage:
    %    [kernels] = REP_kernels(varargin)
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
    %    varargin: optional input arguments
    %       - type: 'linear' or 'gaussian'
    %       - sigma: optional, default is 2
    %       - kernel_length: optional, default is 10
    %       - kernel_width: optional, default is 5
    %       - kernel_height: optional, default is kernel_width
    %
    % Outputs:
    %    kernels - A cell array containing the generated directional kernels.


    % Create an input parser object
    p = inputParser;

    % Define the default values for the optional parameters
    defaultSigma = 2;
    defaultKernelLength = 10;
    defaultKernelWidth = 5;
    defaultKernelHeight = 5;

    % Add the required and optional parameters to the input parser
    addParameter(p, 'type', 'linear', @ischar);
    addParameter(p, 'sigma', defaultSigma, @isnumeric);
    addParameter(p, 'kernel_length', defaultKernelLength, @isnumeric);
    addParameter(p, 'kernel_width', defaultKernelWidth, @isnumeric);
    addParameter(p, 'kernel_height', defaultKernelHeight, @isnumeric);

    % Parse the input arguments
    parse(p, varargin{:});

    % Extract the values from the input parser
    type = p.Results.type;
    sigma = p.Results.sigma;
    kernel_length = p.Results.kernel_length;
    kernel_width = p.Results.kernel_width;
    kernel_height = p.Results.kernel_height;

    % form kernels
    kernel_x = directional_kernel_3d('x', kernel_length, sigma, kernel_width, 0, kernel_height, 0, type);
    kernel_y = directional_kernel_3d('y', kernel_length, sigma, kernel_width, 0, kernel_height, 0, type);
    kernel_z = directional_kernel_3d('z', kernel_length, sigma, kernel_width, 0, kernel_height, 0, type);

    kernels = {kernel_x/max(kernel_x(:)), kernel_y/max(kernel_y(:)), kernel_z/max(kernel_z(:))};

end