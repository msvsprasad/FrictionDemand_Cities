%%%%%%%%%%%%%%%%%%%%%% script_saveVehicleTrajectory.m %%%%%%%%%%%%%%%%%%%%%
%% Purpose:
%   The purpose of this script is to query and save trajectory as a '.mat'
%   file.
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

vehicle_id = 295; % longest duration: 295; longest distance: 5007;

%% Query for vehicle trajectory
raw_trajectory = fcn_queryVehicleTrajectory(vehicle_id,dbInput.trip_id,...
                                            dbInput);

%% Save the trajectory
save(['raw_trajectory_V' num2str(vehicle_id) '_T' num2str(dbInput.trip_id) '.mat'],...
     'raw_trajectory');
