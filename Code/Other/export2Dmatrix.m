% create a 50x50 matrix with random ones and zeros to represent occupied and empty cells
matrix = rot90(grid_distance(:,:,2))

% plot the matrix with occupied cells in red and empty cells in blue
imagesc(matrix);
colormap(flipud(gray)); % use gray colormap with flipped order
axis equal; % set equal scales for x and y axes
axis off; % turn off axis labels and ticks
hold on;

% % plot a colorbar to indicate the meaning of the colors
% c = colorbar;
% c.Label.String = 'Occupancy';
% c.Ticks = [0 1];
% c.TickLabels = {'Empty', 'Occupied'};

filename = "C:\Users\Jakob\Documents\GitHub\Gradient-3D\Code\Other\2Ddistances_lab_1.csv";
writematrix(matrix, filename); % Write the matrix to the file