function [kernel3D] = directional_kernel_3d(direction, kernel_length, sigma, kernel_width, sigma_width, kernel_height, sigma_height, kernel_type)

   % FORM KERNELS 
   % ---------------------------------------------------------------------

    center = floor(kernel_length/2); % mid of the kernel

   if strcmp('linear',kernel_type)

        % prepare directional weights
        kernel = [1:1:center 0 -flip(1:1:center)] / (center); 

        % extend weights to kernel width
        center_width = ceil(kernel_width/2);
        
        % if sigma_width is 0, then use a constant weight for the width
        if (sigma_width == 0)
            width_extender = [ones(1,center_width) 1 ones(1,center_width)];
        else
            width_extender = [1:1:center_width center_width flip(1:1:center_width)]/(center_width);
        end


        % generate 2D kernel
        kernel2D = width_extender' * kernel;

        % extend weight to kernel height
        center_height = ceil(kernel_height/2);

        % if sigma_height is 0, then use a constant weight for the height
        if (sigma_height == 0)
            height_extender = [ones(1,center_height) 1 ones(1,center_height)];
        else
            height_extender = [1:1:center_height center_height flip(1:1:center_height)]/(center_height);
        end

        
        % create each layer
        kernel3D = repmat(kernel2D, 1, 1, length(height_extender)) .* reshape(height_extender, 1, 1, []);


    elseif strcmp('gaussian',kernel_type)
      
        % generate gaussian weights
        kernel = exp(-(1:center).^2/(2*sigma^2)) / (sigma * sqrt(2*pi)); 
        
        % generate symmetric kernel
        kernel = [flip(kernel) 0 -kernel];
        
        % prepare gaussian weights for width
        center_width = ceil(kernel_width/2);

        % if sigma_width is 0, then use a constant weight for the width
        if (sigma_width == 0)
            width_extender = [ones(1,center_width) 1 ones(1,center_width)];
        else
            width_extender = exp(-(1:center_width).^2/(2*sigma_width^2)); 
            width_extender = [flip(width_extender) center_width width_extender];
        end
        
        % generate 2D kernel
        kernel2D = width_extender' * kernel;
        
        % prepare gaussian weights for height
        center_height = ceil(kernel_height/2);

        % if sigma_height is 0, then use a constant weight for the height
        if (sigma_height == 0)
            height_extender = [ones(1,center_height) 1 ones(1,center_height)];
        else
            height_extender = exp(-(1:center_height).^2/(2*sigma_height^2));
            height_extender = [flip(height_extender) center_height height_extender];
        end


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




    





