function space = create_3d_grid(space_resolution, space_height, space_width, space_length)
% Creates a 3D matrix of voxels with given dimensions
% Args:
%   space_resolution (int): resolution of the space in centimeters
%   space_height (int): height of the space in centimeters
%   space_width (int): width of the space in centimeters
%   space_length (int): length of the space in centimeters
% Returns:
%   space (matrix): 3D matrix of voxels

% calculate the number of voxels in each dimension
num_voxels_x = space_length / space_resolution;
num_voxels_y = space_width / space_resolution;
num_voxels_z = space_height / space_resolution;

% create a 3D matrix of zeros with the calculated dimensions
space = zeros(num_voxels_x, num_voxels_y, num_voxels_z);

end
