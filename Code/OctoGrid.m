classdef OctoGrid < handle
    % OCTOGRID class defines 3D occupancy grid structure and methods

    properties
        grid = []; % Property to store the occupancy grid
        length = 0;
        width = 0;
        height = 0;
        resolution = 0;
    end

    methods
        function obj = OctoGrid(length, width, height, resolution)
            % Constructor for OctoGrid class
            %
            % length - length in meters
            % width - width in meters
            % height - height in meters
            % resolution - number of squares per meter
            %
            % Initialize the occupancy grid with zeros based on the given dimensions and resolution

            obj.length = length;
            obj.width = width;
            obj.height = height;
            obj.resolution = resolution;

            obj.grid = zeros(round(length)*resolution, round(width)*resolution, round(height)*resolution);

        end

        function showGridITV(obj, grid)

            % Display the occupancy grid (Image Toolbox Viewer)
            % Uses Image Processing Toolbox
            volshow(grid);
      
        end

      
        function showGridVoxel(obj, grid, varargin)
            % Displays a voxel grid 
            %
            % showGridVoxel(obj, grid, varargin)
            %
            % Inputs:
            %   - obj: an instance of the OctoGrid class
            %   - grid: a 3D array representing the voxel grid
            %   - min_display_value (optional): the minimum value to display in the grid (default: 0.5)
            %   - space_resolution (optional): the resolution of the grid in space (default: 10)
            %   - heightColorMap (optional): a flag indicating whether to use a height-based color map (default: false)
            %   - color (optional): the color to use for the voxels (default: [0 0 0])
            %   - floor (optional): a flag indicating whether to display the floor surface (default: false)
            %
            % The `min_display_value` argument can be used to set a threshold for the
            % values in the grid. Any values below this threshold will not be displayed.
            % The default value for this argument is 0.5.
            %
            % The `space_resolution` argument can be used to set the resolution of the
            % grid in space. A higher resolution will result in a more detailed display.
            % The default value for this argument is 10.
            %
            % The `heightColorMap` argument can be used to enable a height-based color
            % map, which will color the voxels based on their height in the grid.
            %
            % The `color` argument can be used to set the color of the voxels. This can
            % be a character string to specify a color or an RGB triplet to specify a
            % custom color. If a scalar value is provided, it will be interpreted as a
            % grayscale value where 0 is black and 1 is white.
            %
            % The floor argument can be used to control whether to display the floor
            % surface. If set to true, a floor surface will be displayed. The default
            % value is false.
        
            % Set default values for optional input arguments
            default_min_display_value = 0.5;
            default_space_resolution = 10;
            color_floor = [33/256, 33/256, 33/256];

        
            % Create an input parser object
            p = inputParser;
            addOptional(p, 'min_display_value', default_min_display_value, @isnumeric);
            addOptional(p, 'space_resolution', default_space_resolution, @isnumeric);
            addOptional(p, 'heightColorMap', false, @islogical);
            addOptional(p, 'color', [48/256, 123/256, 242/256], @(x)validateattributes(x, {'numeric'}, {'numel',3}));
            addOptional(p, 'floor', false, @islogical);
            parse(p, varargin{:});
        
            % Get the values of the parsed input arguments
            min_display_value = p.Results.min_display_value;
            space_resolution = p.Results.space_resolution;
            heightColorMap = p.Results.heightColorMap;
            color = p.Results.color;
            show_floor = p.Results.floor;
        
            % Loop through every element of the space matrix
            for x = 1:size(grid, 1)
                for y = 1:size(grid, 2)
                    for z = 1:size(grid, 3)
                        % Get the start position of the voxel (bottom left corner)
                        start = [(x-1), (y-1), (z-1)];

                        if heightColorMap
                            % Get the value of the current voxel
                            voxel_value = grid(x,y,z);
                            % Define the scalar value
                            value = z/size(grid,3);

                            % Map the value to an HSV color
                            hue = (1 - value) * 0.55;
                            saturation = 1;
                            value = 1;
                            color = hsv2rgb([hue saturation value]);

                        else
                            % Get the value of the current voxel
                            voxel_value = grid(x,y,z);
                        end
        
                        % Draw the voxel as a cube if its value is above the 
                        % min_display_value threshold
                        if voxel_value > min_display_value
                            % Draw a cube at the position defined by start, with 
                            % dimensions [1 1 1] and color black ('k'). The value 
                            % of the voxel is used to set the color of the cube.
                            voxel(start * space_resolution, [1 1 1] * space_resolution, color, voxel_value);
                        end
                        
                        % Keep the same plot for subsequent voxel cubes
                        hold on
                    end
                end
            end
        
            % Set the fixed axis limits
            axis([0, size(grid,1), 0, size(grid,2),0,size(grid,3)]*space_resolution)

            % Display the floor surface if enabled
            if show_floor
                % Define the X and Y coordinates for the floor surface
                [X, Y] = meshgrid(0:10*space_resolution:space_resolution*size(grid,1), 0:10*space_resolution:space_resolution*size(grid,2));
                
                % Define the Z coordinates for the floor surface (elevation)
                Z = zeros(size(X));
                
                % Plot the floor surface
                surf(X, Y, Z, 'FaceColor', color_floor,'FaceAlpha', 0.1);
            end
        
            % Set the default line of sight for the 3D plot
            view(3)

            % add labels
            xlabel('X')
            ylabel('Y')
            zlabel('Z')
        end


        function showGridVol3D(obj, grid, varargin)

            % Displays a 3D grid using the vol3d function
            % If the optional input argument heightColorMap is true ('heightColorMap', true), the height of the grid is used to color the voxels
            % grid: a 3D matrix representing the grid to be displayed

            % Parse the optional input argument
            p = inputParser;
            addOptional(p, 'heightColorMap', false, @islogical);
            addOptional(p, 'floor', false, @islogical);

            parse(p, varargin{:});
            heightColorMap = p.Results.heightColorMap;
            show_floor = p.Results.floor;

            color_floor = [33/256, 33/256, 33/256];

        
            grid_size = size(grid);
        
            % If selected by the user, color the voxels based on their height
            if heightColorMap
                [~, ~, z] = meshgrid(1:grid_size(2), 1:grid_size(1), 1:grid_size(3));
                grid = grid .* z / grid_size(3);
            end
            
            hold on
            H = vol3d('CData', grid);

            % Display the floor surface if enabled
            if show_floor
                % Define the X and Y coordinates for the floor surface
                [X, Y] = meshgrid(0:10:size(grid,1), 0:10:size(grid,2));

                
                % Define the Z coordinates for the floor surface (elevation)
                Z = zeros(size(X));
                
                % Plot the floor surface
                surf(X, Y, Z, 'FaceColor', color_floor,'FaceAlpha', 0.1);

            end
                    
            view([-15.5 49.3])

            % add labels
            xlabel('X')
            ylabel('Y')
            zlabel('Z')

           
        end

        function showSlice(obj,slice)
            
            % showSlice - Display a 2D matrix grid of values as a surface.
            %
            %   showSlice(slice) displays a 2D matrix grid of values as a surface plot.
            %   The input argument "slice" is a 2D matrix representing the grid.
            %
            %   Example:
            %       grid = [1 2 3; 4 5 6; 7 8 9];
            %       showSlice(grid);
            

            % Create x and y coordinates for the grid
            [x, y] = meshgrid(1:size(slice, 2), 1:size(slice, 1));
            
            % Plot the grid as a surface
            surf(x, y, slice);
            
            % Set colormap for coloring the surface
            colormap jet
            
            % Add labels and title
            xlabel('X')
            ylabel('Y')
            zlabel('Value')
            title('2D Grid Surface')
            
            % Adjust the view and aspect ratio
            view(45, 30)
            axis tight
        end




        function addBox(obj, x, y, z, length, width, height)

            % Set pixels of a box object as occupied, given input dimensions
            %
            % x, y, z - coordinates of the box's bottom-left corner
            % length, width, height - dimensions of the box
            
            % Convert the box dimensions from meters to grid indices
            x_start = round(x * obj.resolution) + 1;
            x_end = round((x + length) * obj.resolution);
            y_start = round(y * obj.resolution) + 1;
            y_end = round((y + width) * obj.resolution);
            z_start = round(z * obj.resolution) + 1;
            z_end = round((z + height) * obj.resolution);
            
            % Set the corresponding grid elements as occupied (1)
            obj.grid(y_start:y_end, x_start:x_end, z_start:z_end) = 1;

        end

    end
end