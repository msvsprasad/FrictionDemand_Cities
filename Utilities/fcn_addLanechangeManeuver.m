%%%%%%%%%%%%%%%%%%%% Function fcn_addLanechangeManeuver %%%%%%%%%%%%%%%%%%%%
% Purpose:
%   fcn_addLanechangeManeuver trasnforms input path into a lane change
%   maneuver.
% 
% Format:
%   output_path = fcn_addLanechangeManeuver(input_path,lane_width,direction)
% 
% INPUTS:
%   input_path: Nx2 matrix.
%   lane_width: Width of a lane [m].
%   direction:  Direction of lane change.
%               '1' for right lane change
%               '-1' for left lane change
% 
% OUTPUTS:
%   output_path: Nx2 matrix.
% 
% Author:  Satya Prasad
% Created: 2022/04/09
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output_path = fcn_addLanechangeManeuver(input_path,lane_width,direction)
flag_doDebug = 0; % DEBUG flag. Set it to one to compare input and output trajectories

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
% Are there right number of inputs?
if 3~=nargin
    error('fcn_addLanechangeManeuver: Incorrect number of input arguments.')
end

%% Add offset to input path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Convert path to a traversal
input_traversal = fcn_Path_convertPathToTraversalStructure(input_path);

%% Fill in the array of offset distances
decimation_vector = (input_traversal.Station-input_traversal.Station(1))/...
                    (input_traversal.Station(end)-input_traversal.Station(1));
decimation_vector(1<decimation_vector)=1;
offset_from_reference = direction*(lane_width/2)*(cos(pi*decimation_vector)+1);

%% Find projection from reference orthogonally
% Set the projection type (see help in function below for details)
flag_rounding_type = 4; % This averages the projection vectors along segments

% Find the unit normal vectors at each of the station points
[unit_normal_vector_start,unit_normal_vector_end] = ...
    fcn_Path_findOrthogonalTraversalVectorsAtStations(...
    input_traversal.Station,input_traversal,flag_rounding_type);
unit_normal_vectors = unit_normal_vector_end-unit_normal_vector_start;

%% Offset the input path to generate ouput path
output_path = unit_normal_vector_start + ...
              unit_normal_vectors.*offset_from_reference;

%% Plot the results (for debugging)?
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
if flag_doDebug
    figure(12345)
    clf
    plot(input_path(:,1),input_path(:,2),'b.','Markersize',10)
    hold on
    plot(output_path(:,1),output_path(:,2),'g.','Markersize',10)
    grid on
    title('Vehicle Path')
    xlabel('X-Coordinate(m)')
    ylabel('Y-Coordinate(m)')
    legend('Input Path','Output Path')
end

end