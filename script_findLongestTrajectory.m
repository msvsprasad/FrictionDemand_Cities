%%%%%%%%%%%%%%%%%%%%%% script_findLongestTrajectory.m %%%%%%%%%%%%%%%%%%%%%
%% Purpose:
%   The purpose of this script is to find ID of vehicle that travelled the
%   longest distance and also the ID of the vehicle that travelled for
%   longest duration.
% 
% Author:  Satya Prasad
% Created: 2022/03/30
%% Prepare the workspace
clear all %#ok<CLALL>
close all
clc

%% Add path to dependencies
addpath('./Utilities/DB Lib'); % all the functions and wrapper class

%% Define inputs and parameters
dbInput.ip_address = '130.203.223.234'; % Ip address of server host
dbInput.port       = '5432'; % port number
dbInput.username   = 'brennan'; % user name for the server
dbInput.password   = 'ivsg@Reber320'; % password

dbInput.db_name       = 'roi_db'; % database name
dbInput.traffic_table = 'road_traffic_raw'; % table containing traffic simulation data
dbInput.trip_id       = 13; % traffic simulation id

longer_duration_vehicle = -1; prev_veh_duration = 0;
longer_station_vehicle  = -1; prev_veh_station  = 0;

%% Query for valid Section ID and Vehicle ID combinations
SectionId_VehID    = fcn_findValidSectionandVehicleId(dbInput.trip_id,dbInput);
list_of_vehicleIds = unique(SectionId_VehID(:,2));

for index_vehicle = 1:numel(list_of_vehicleIds)
    %% Query for vehicle trajectory
    raw_trajectory = fcn_queryVehicleTrajectory(list_of_vehicleIds(index_vehicle),...
                                                dbInput.trip_id,dbInput);
    
    %% Estimate station and duration of the trajectory
    vehicle_path = raw_trajectory{:,{'positionfront_x','positionfront_y'}};
    cg_station   = cumsum([0; sqrt(sum(diff(vehicle_path).^2,2))]);
    cg_duration  = raw_trajectory{end,'aimsun_time'}-raw_trajectory{1,'aimsun_time'};
    
    if cg_station(end)>prev_veh_station
        longer_station_vehicle = raw_trajectory{1,'vehicle_id'};
        prev_veh_station = cg_station;
    end % NOTE: END IF statement
    if cg_duration>prev_veh_duration
        longer_duration_vehicle = raw_trajectory{1,'vehicle_id'};
        prev_veh_duration = cg_duration;
    end % NOTE: END IF statement
end % NOTE: END FOR loop
