%%%%%%%%%%%%%%%%%%%%% fcn_smoothLanechangeOfTrajectory %%%%%%%%%%%%%%%%%%%%
% Purpose:
%   fcn_smoothLanechangeOfTrajectory smooths lane change maneuver in the
%   input trajectory.
% 
% Format:
%   output_trajectory = fcn_smoothLanechangeOfTrajectory(input_trajectory,...
%                       lane_width,fieldsTrajectory)
% 
% INPUTS:
%   input_trajectory: Nx15 matrix.
%   lane_width: Width of a lane [m].
%   fieldsTrajectory: field names of vehicle trajectory before being used 
%   in the simulation
% 
% OUTPUTS:
%   output_trajectory: Nx15 matrix.
% 
% Author:  Satya Prasad
% Created: 2022/03/08
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output_trajectory = fcn_smoothLanechangeOfTrajectory(input_trajectory,...
                             lane_width,false_lanechange)
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
    error('fcn_smoothLanechangeOfTrajectory: Incorrect number of input arguments.')
end

%% Smooth input trajectory by replacing lane change maneuver with sine wave
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[~,unique_indices,ic] = unique(input_trajectory.station_total); % Indices of unique station coordinates
input_trajectory      = input_trajectory(unique_indices,:); % Trajectory with unique station coordinates

% Indices where lane change happens
temp_var = (1:size(input_trajectory,1))';
indices_of_lanechange = 1+temp_var([0~=diff(input_trajectory.lane_number); false]);
if ~isnan(false_lanechange)
    indices_of_lanechange = indices_of_lanechange(indices_of_lanechange<false_lanechange);
end

% Indices where there is shift from 'no lane change' to 'lane change' and viceversa
indices_of_directionChange = 1+temp_var([0~=diff(input_trajectory.direction); false]);

% Change a step lane change to sine/cosine curve
for i = 1:numel(indices_of_lanechange)
    start_index = indices_of_lanechange(i); % Index at which lane change begins
    
    % Index at which lane change ends
    target_index = find(indices_of_directionChange==start_index)+1;
    if ~isempty(target_index)
        if target_index <= numel(indices_of_directionChange)
            end_index = indices_of_directionChange(target_index)-1;
            if 9>end_index-start_index
                end_index = start_index+9;
            end
        else
            end_index = size(input_trajectory,1);
        end
    else
        end_index = start_index+9;
    end
    if i==numel(indices_of_lanechange)
        end_index = min(end_index,size(input_trajectory,1));
    else
        end_index = min(end_index,indices_of_lanechange(i+1)-1);
    end
    
    if 2<=end_index-start_index
        % Change sudden jump to a smooth sine/cosine wave
        input_path  = input_trajectory{start_index:end_index,...
                                       {'positionfront_x','positionfront_y'}};
        direction   = input_trajectory.direction(start_index);
        if 0==direction
            direction = input_trajectory.lane_number(start_index-1)-...
                        input_trajectory.lane_number(start_index);
        end
        output_path = fcn_addLanechangeManeuver(input_path,lane_width,direction);
        % Update the trajectory
        input_trajectory.positionfront_x(start_index:end_index) = output_path(:,1);
        input_trajectory.positionfront_y(start_index:end_index) = output_path(:,2);
    end
end % NOTE: Ends for loop 'indices_of_lanechange'

output_trajectory = input_trajectory(ic,:);
end