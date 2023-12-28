function [kernel3D] = directional_kernel_3d(direction, kernel_length, sigma, kernel_width, sigma_width, kernel_height, sigma_height, kernel_type)

   % FORM KERNELS 
   % ---------------------------------------------------------------------

    center = floor(kernel_length/2); % mid of the kernel

   if strcmp('linear',kernel_type)

        % prepare directional weights
        kernel = [1:1:center -flip(1:1:center)] / (center); 

        % extend weights to kernel width
        center_width = floor(kernel_width/2);
        width_extender = [1:1:center_width center_width flip(1:1:center_width)]/(center_width);

        % generate 2D kernel
        kernel2D = width_extender' * kernel;

        % extend weight to kernel height
        center_height = floor(kernel_height/2);
        height_extender = [1:1:center_height center_height flip(1:1:center_height)]/(center_height);
        
        % create each layer
        kernel3D = repmat(kernel2D, 1, 1, length(height_extender)) .* reshape(height_extender, 1, 1, []);


    elseif strcmp('gaussian',kernel_type)
      
        % generate gaussian weights
        kernel = exp(-(1:center).^2/(2*sigma^2)) / (sigma * sqrt(2*pi)); 
        
        % generate symmetric kernel
        kernel = [flip(kernel) -kernel];
        
        % prepare gaussian weights for width
        center_width = floor(kernel_width/2);
        width_extender = exp(-(1:center_width).^2/(2*sigma_width^2)); 
        width_extender = [flip(width_extender) exp(-(center_width).^2/(2*sigma_width^2)) width_extender];
        
        % generate 2D kernel
        kernel2D = width_extender' * kernel;
        
        % prepare gaussian weights for height
        center_height = floor(kernel_height/2);
        height_extender = exp(-(1:center_height).^2/(2*sigma_height^2));
        height_extender = [flip(height_extender) exp(-(center_height).^2/(2*sigma_height^2)) height_extender];

        % create each layer
        kernel3D = repmat(kernel2D, 1, 1, length(height_extender)) .* reshape(height_extender, 1, 1, []);

   end

   % STACK KERNELS IN PICKED DIRECTION
   % ---------------------------------------------------------------------
    
    % x kernel
    if strcmp(direction, 'x')

    	% create each layer
        kernel3D = repmat(kernel2D, 1, 1, length(height_extender)) .* reshape(height_extender, 1, 1, []);
     

    % y kernel
    elseif strcmp(direction, 'y')
       
        % rotate layer by 90 degrees
        kernel2D = rot90(kernel2D,-1);

    	% create each layer
        kernel3D = repmat(kernel2D, 1, 1, length(height_extender)) .* reshape(height_extender, 1, 1, []);

    % z kernel
    elseif strcmp(direction, 'z')
        % kernel width and height should be equal (imagine a column pushing
        % down on robot)

        % reshape arrays for bsxfun operation
        reshaped_kernel2D = reshape(kernel2D, [1 size(kernel2D)]);
        reshaped_height_extender = reshape(height_extender, [length(height_extender) 1 1]);
        
        % perform element-wise multiplication using bsxfun
        kernel3D = bsxfun(@times, reshaped_kernel2D, reshaped_height_extender);


    end

    end




    





