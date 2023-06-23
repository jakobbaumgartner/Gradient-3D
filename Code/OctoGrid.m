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

        function showGridITV(obj)

            % Display the occupancy grid (Image Toolbox Viewer)
            % Uses Image Processing Toolbox
            volshow(obj.grid);
                        sum(sum(sum(obj.grid)))

      
        end

        function addBox(obj, x, y, z, length, width, height)
            % Set pixels of a box object as occupied, given input dimensions
            
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
            obj.grid(x_start:x_end, y_start:y_end, z_start:z_end) = 1;

            sum(sum(sum(obj.grid)))
        end

    end
end