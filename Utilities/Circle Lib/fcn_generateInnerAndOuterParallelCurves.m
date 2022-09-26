function [path_inner,path_outer,radius_of_curvature,unit_normal_vec,concavity,overlap]=...
    fcn_generateInnerAndOuterParallelCurves(path,lateral_offset,flag_plot,flag_concavity,...
    window_length)
% % parallel_curve: Calculates the inner and outer parallel curves to the given x, y pairs.
% %
% % Syntax:
% % [path_inner,path_outer,radius_of_curvature,unit_normal_vec,concavity,overlap]=...
% % fcn_generateInnerAndOuterParallelCurves(path,lateral_offset,flag_plot,flag_concavity,...
% % window_length)
% %
% % **********************************************************************
% %
% % Description:
% %
% % Calculates the inner and outer parallel curves to the given x, y
% % coordinate pairs.  By default the inner parallel is toward the center 
% % of curvature while the outer parallel is away from the center of 
% % curvature. Use flag_concavity = 0 to make the parallels stay on opposite
% % sides of the curve. Input the x and y coordinate pairs, distance between 
% % the curve and the parallel, and whether to plot the curves.
% %
% % Program is currently limited to rectangular coordinates.
% % Attempts to make sure the parellels are always inner or outer.
% % The inner parallel is toward the center of curvature
% % while the outer parallel is away from the center of curvature.
% % If the radius of curvature become infinite and the center of curvature 
% % changes sides then the parallels will switch sides.  If the parallels 
% % should stay on the same sides then set flag_concavity=0 to keep the 
% % parallels on the sides.  
% % 
% % Implements "axis equal" so that the curves appear with equal
% % scaling.  If this is a problem, type "axis normal" and the scaling goes
% % back to the default.  This will have to be done for every plot or feel
% % free to modify the program.
% % **********************************************************************
% % Input Variables:
% %
% % x;              % (meters) position column-vector in the x-direction
% % d;              % (meters) distance from curve to the parallel curve
% %             	% default is d=1;
% % flag_plot=1:    1 makes a plot of the curve and parallels
% %                 otherwise only plots are generated.
% %                 default is flag_plot=1;
% % flag_concavity: if we use the concavity when calculate the parallel
% %                 curve, default 1
% % window_length:  window length of filter which used to smooth the
% %                 curvature of the curve, do not set this input if you do 
% %                 not want to use filter.
% % **********************************************************************
% % Output Variables:
% %
% % path_inner:             [m] x, y positions of the inner parallel curve
% % path_outer:             [m] x, y positions of the outer parallel curve
% % radius_of_curvature:    [m] radius of curvature
% % unit_normal_vec:        [m] unit normal vector at each position
% % concavity:              (-1) concave down
% %                         (+1) concave up
% % overlap:    (boolean) indicates whether the distance between the
% %             parallel curves is greater than the radius of curvature.
% %             if overlap is 1 then there may be cusps and the parallels
% %             may be overlapping one another.
% % **********************************************************************
% % References:
% % Gray, A. "Parallel Curves." §5.7 in Modern Differential Geometry of
% %           Curves and Surfaces with Mathematica, 2nd ed. Boca Raton,
% %           FL: CRC Press, pp. 115-117, 1997.
% % Lawrence, J. D. A Catalog of Special Plane Curves. New York: Dover,
% %           pp.42-43, 1972.
% % Yates, R. C. "Parallel Curves." A Handbook on Curves and Their
% %               Properties. Ann Arbor, MI: J. W. Edwards, pp. 155-159,
% %               1952.
% % http://en.wikipedia.org/wiki/Parallel_curve
% % http://xahlee.info/SpecialPlaneCurves_dir/Parallel_dir/parallel.html
% % **********************************************************************
% %
% % parallel_curve was created by Edward L. Zechmann, modified by Liming,
% % Satya Prasad
% %
% % modified      2 July        2010    Updated Comments
% % modified     25 September   2010    Added option to avoid following
% %                                     the change in concavity when
% %                                     radius of curvture is infinite.
% %                                     Fixed bug with indicating the 
% %                                     overlap.   
% % modified      2 July        2020    Add filter and add Comments
% % modified     19 November    2021    Replaced the 'vecnorm' with pure 
% %                                     calculation. Modified the inputs
% %                                     from two Nx1 vectors to one 
% %                                     Nx2 matrix. Modified the outputs 
% %                                     from four Nx1 vectors to two Nx2 
% %                                     matrices. Modified input check 
% %                                     according to the above changes. 
% %                                     Replaced 'gradient' with 'diff'. 
% %                                     Added comments.
% % **********************************************************************
%% Check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _       
%  |_   _|                 | |      
%    | |  _ __  _ __  _   _| |_ ___ 
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |                  
%              |_| 
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin < 1 || isempty(path) || ~isnumeric(path)
    path_x=1:100;
    path_y=path_x.^2;
    path  = [path_x', path_y'];
end

if nargin < 2 || isempty(lateral_offset) || ~isnumeric(lateral_offset)
    lateral_offset=1;
end

if nargin < 3 || isempty(flag_plot) || ~isnumeric(flag_plot)
    flag_plot=1;
end

if nargin < 4 || isempty(flag_concavity) || ~isnumeric(flag_concavity)
    flag_concavity=1;
end

if nargin < 5 || isempty(window_length) || ~isnumeric(window_length)
    window_length=0;
end

%% Offset input curve
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Make sure that path_x and path_y are column vectors.
path_x = path(:,1);
path_y = path(:,2);

% Estimate first derivative
dx = diff(path_x);
dx(0==dx) = eps; % Replace zeros to avoid division by zero
dy = diff(path_y);
first_derivative = dy./dx;
first_derivative = [first_derivative; first_derivative(end)];

% Estimate second derivative
second_derivative = diff(first_derivative)./dx;
second_derivative(0==second_derivative) = eps; % Replace zeros to avoid division by zero
second_derivative = [second_derivative; second_derivative(end)];

% Slope is dy/dx and the vector form is [dx, dy]
normal_vec = [dy, -dx]; % Calculate the normal vector (slope x k_hat)
normal_vec = [normal_vec; [dy(end), -dx(end)]];

% Normalize the normal vectors
norm_normal_vec = sqrt(sum(normal_vec.^2,2)); % Norm of normal vector
unit_normal_vec = normal_vec./norm_normal_vec;

radius_of_curvature = ((1+first_derivative.^2).^1.5)./abs(second_derivative); % Radius of curvature

if 0<window_length
    % Smooth the result using Gaussian moving average filter
    radius_of_curvature = smoothdata(radius_of_curvature,'gaussian',window_length);
end

% Determine overlap points for inner normal curve
overlap=radius_of_curvature < lateral_offset; % Might not be visible in a road

% Determine concavity
concavity=sign(second_derivative);

% Inner and outer words are very subjective
if flag_concavity
    % For inner normal curve
    path_inner = path-unit_normal_vec.*concavity*lateral_offset;
    
    % For outer normal curve
    path_outer = path+unit_normal_vec.*concavity*lateral_offset;
else
    % For inner normal curve
    path_inner = path-unit_normal_vec*lateral_offset;
    
    % For outer normal curve
    path_outer = path+unit_normal_vec*lateral_offset;
end

%% Make a simple plot of the curve and the parallels
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _                 
%  |  __ \     | |                
%  | |  | | ___| |__  _   _  __ _ 
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/ 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if 1 == flag_plot
    figure;
    plot(path_x,path_y,'r','Linewidth',1.2)
    hold on
    plot(path_inner(:,1),path_inner(:,2),'b','Linewidth',1.2)
    plot(path_outer(:,1),path_outer(:,2),'g','Linewidth',1.2)
    grid on
    legend({'Curve', 'Inner Parallel', 'Outer Parallel'},'Location','best')
    % The axis scaling can be modified.
    % axis equal makes the plots more realistic for geometric constructions. 
    % if this is a problem, change to 'axis normal'.
    axis equal
%     axis normal
    xlabel('X-Coordinate'); ylabel('Y-Coordinate');
end
