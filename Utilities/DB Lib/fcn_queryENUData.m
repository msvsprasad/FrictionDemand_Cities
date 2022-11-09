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
function enu_data = fcn_queryENUData(dbInput)
% enu_attributes = ['id, name, date_added, '...
%                   'latitude, longitude, altitude, geography, '...
%                   'epsg_code, latitude_std, longitude_std, altitude_std, '...
%                   'timestamp']; % attributes in the enu reference table

enu_attributes = ['id, name, date_added, '...
    'latitude, longitude, altitude, geography, '...
    'epsg_code, latitude_std, longitude_std, altitude_std, '...
    'timestamp']; % attributes in the enu reference table
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
if 1~=nargin
    error('fcn_queryVehicleTrajectory: Incorrect number of input arguments.')
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

% SQL statement to query enu data
enu_query = ['SELECT ' enu_attributes...
             ' FROM ' dbInput.traffic_table...
                   ' ORDER BY date_added'];

% query trajectory data from the DB
enu_data = fetch(DB.db_connection, enu_query);

% Disconnect from the database
DB.disconnect();
end