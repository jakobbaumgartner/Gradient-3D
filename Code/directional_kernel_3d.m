function [kernel3D] = directional_kernel_3d(direction, kernel_length, kernel_sigma, kernel_width, kernel_width_sigma, kernel_height, kernel_height_sigma, kernel_type)

    % create kernel
    % =========================================================================

    center = floor(kernel_length/2); % mid of the kernel
    
    % x kernel
    if strcmp(direction, 'x')
        if strcmp('linear',kernel_type)
    
            % prepare directional weights
            kernel = [1:1:center 0 -flip(1:1:center)] / (center+1); 
    
            % extend weights to kernel width
            center_width = floor(kernel_width/2);
            width_extender = [1:1:center_width center_width+1 flip(1:1:center_width)]/(center_width+1);
    
            % generate 2D kernel
            kernel2D = width_extender' * kernel;
    
            % extend weight to kernel height
            center_height = floor(kernel_height/2);
            height_extender = [1:1:center_height center_height+1 flip(1:1:center_height)]/(center_height+1);
            
            % create each layer
            kernel3D = repmat(kernel2D, 1, 1, length(height_extender)) .* reshape(height_extender, 1, 1, []);

    
        elseif strcmp('gaussian',kernel_type)
    
    
            % define the standard deviation (sigma)
            sigma = 1;
            
            % generate gaussian weights
            kernel = exp(-(1:center).^2/(2*sigma^2)); 
            
            % generate symmetric kernel
            kernel = [flip(kernel) 1 kernel] / sum([flip(kernel) 1 kernel]);
            
            % prepare gaussian weights for width
            center_width = floor(kernel_width/2);
            width_extender = exp(-(1:center_width).^2/(2*sigma^2)); 
            width_extender = [flip(width_extender) 1 width_extender] / sum([flip(width_extender) 1 width_extender]);
            
            % generate 2D kernel
            kernel2D = width_extender' * kernel;
            
            % prepare gaussian weights for height
            center_height = floor(kernel_height/2);
            height_extender = exp(-(1:center_height).^2/(2*sigma^2));
            height_extender = [flip(height_extender) 1 height_extender] / sum([flip(height_extender) 1 height_extender]);
            
            % initiate kernel matrix
            kernel3D = zeros(kernel_width, kernel_length, kernel_height);
            
            % create each layer
            kernel3D = repmat(kernel2D, 1, 1, length(height_extender)) .* reshape(height_extender, 1, 1, []);



        end


     end


    % y kernel

    % z kernel
    







    end




    





