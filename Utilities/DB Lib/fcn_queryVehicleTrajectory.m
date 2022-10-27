%%%%%%%%%%%%%%%%%%% Function fcn_queryVehicleTrajectory %%%%%%%%%%%%%%%%%%%
% Purpose:
%   fcn_queryVehicleTrajectory queries a vehicle's trajectory uniquely 
%   identified by 'vehicle_id' in trip 'trip_id'.
% 
% Format:
%   vehicle_trajectory = fcn_queryVehicleTrajectory(vehicle_id,trip_id,dbInput)
% 
% INPUTS:
%   vehicle_id: ID of the vehicle. A positive integer.
%   trip_id: Id of a trip. A positive integer.
%   dbInput: It's a structure containing name of the database and tables.
% 
% OUTPUTS:
%   queryVehicleTrajectory: Contains all the attributes defined by 
%   'trajectory_attributes' sorted in the order of aimsun_time. 
%   It's a Nx17 table.
% 
% Author:  Satya Prasad
% Created: 2022-03-28
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function vehicle_trajectory = fcn_queryVehicleTrajectory(vehicle_id,trip_id,dbInput)
trajectory_attributes = ['trips_id, vehicle_id, '...
                         'section_id, junction_id, lane_number, direction, '...
                         'position_front_x, position_front_y, position_front_z, '...
                         'station_total, current_speed, '...
                         'aimsun_time, system_entrance_time']; % attributes in the vehicle trajectory
% deleted current_pos because I don't collect it
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
    error('fcn_queryVehicleTrajectory: Incorrect number of input arguments.')
end

% Check the size and validity of vehicle_id
if ~isnumeric(vehicle_id) || 1~=numel(vehicle_id) || any(0>=vehicle_id) || ...
        any(vehicle_id~=round(vehicle_id))
    % display an error message if 'vehicle_id' is not a positive integer
    error('vehicle_id must be a POSITIVE INTEGER')
end

% Check the size and validity of trip_id
if ~isnumeric(trip_id) || 1~=numel(trip_id) || any(0>=trip_id) || ...
        any(trip_id~=round(trip_id))
    % display an error message if 'trip_id' is not a positive integer
    error('trip_id must be a POSITIVE INTEGER')
end

%% Query vehicle trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% connect to the database
DB = Database(dbInput.db_name,dbInput.ip_address,dbInput.port,...
              dbInput.username,dbInput.password);

% SQL statement to query vehicle trajectory
traj_query = ['SELECT ' trajectory_attributes...
             ' FROM ' dbInput.traffic_table...
             ' WHERE vehicle_id = ' num2str(vehicle_id)...
               ' AND trips_id = ' num2str(trip_id)...
                   ' ORDER BY aimsun_time'];

% query trajectory data from the DB
vehicle_trajectory = fetch(DB.db_connection, traj_query);

% Disconnect from the database
DB.disconnect();
end