function voxel(i,d,c,alpha)
% VOXEL function to draw a 3-D voxel in a 3-D plot
%
% USAGE:
% voxel(start,size,color,alpha);
%
% This function will draw a voxel at 'start' of size 'size' of color 'color'
% and transparency alpha (1 for opaque, 0 for transparent).
%
% start is a three element vector [x,y,z]
% size is a three element vector [dx,dy,dz]
% color is a character string to specify color (type 'help plot' to see list of valid colors)
% alpha is a value between 0 and 1 representing transparency (optional, defaults to 1)
%
% EXAMPLES:
% voxel([2 3 4],[1 2 3],'r',0.7);
% axis([0 10 0 10 0 10]);
%
% Author: Suresh Joel, April 15, 2003
% Updated by: Suresh Joel, Feb 25, 2004
% Added comments: Chat GPT-3 30.3.2023 

% Check the number of input arguments
switch(nargin)
    case 0
        % If no arguments are provided, print an error message and return
        disp('Too few arguments for voxel');
        return;
    case 1
        % If only the start position is provided, use default values for size and color
        d = [1 1 1]; %default length of side of voxel is 1
        c = 'b';   %default color of voxel is blue
    case 2
        % If size is also provided, use default color
        c = 'b';
    case 3
        % If color is also provided, use default value for alpha
        alpha = 1;
    case 4
        % If all four arguments are provided, use them as is
    otherwise
        % If more than four arguments are provided, print an error message and return
        disp('Too many arguements for voxel');
%         return;
end

% Construct the vertices of the voxel
x=[i(1)+[0 0 0 0 d(1) d(1) d(1) d(1)]; ...
        i(2)+[0 0 d(2) d(2) 0 0 d(2) d(2)]; ...
        i(3)+[0 d(3) 0 d(3) 0 d(3) 0 d(3)]]';

% Sort the vertices in the correct order for the patch function
for n = 1:3
    if n == 3
        x = sortrows(x,[n,1]);
    else
        x = sortrows(x,[n n+1]);
    end
    
    % Swap vertices 3 and 4 to ensure correct ordering of polygon vertices
    temp = x(3,:);
    x(3,:) = x(4,:);
    x(4,:) = temp;
    
    % Draw a face of the voxel using the patch function
    h = patch(x(1:4,1),x(1:4,2),x(1:4,3),c);
    set(h,'FaceAlpha',alpha);
    
    % Swap vertices 7 and 8 to ensure correct ordering of polygon vertices
    temp = x(7,:);
    x(7,:) = x(8,:);
    x(8,:) = temp;
    
    % Draw the other face of the voxel using the patch function
    h = patch(x(5:8,1),x(5:8,2),x(5:8,3),c);
    set(h,'FaceAlpha',alpha);
end
