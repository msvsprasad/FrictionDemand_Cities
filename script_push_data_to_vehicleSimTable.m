%%%%%%%%%%%%%%%%%% script_push_data_to_vehicleSimTable.m %%%%%%%%%%%%%%%%%%
%% Purpose:
%   The purpose of this script is to add data to the table 
%   'vehicle_simulation_trips'.
%
% Author:  Satya Prasad (szm888@psu.edu)
% Created: 2022/04/09

%% Prepare workspace
clear all %#ok<CLALL>
clc

%% Data to push to the table 'vehicle_simulation_trips'
gps_utc_time = 18; % [seconds]
dt = datetime(datestr(now),'TimeZone','America/New_York');
matsim_unix_time = posixtime(dt); % [seconds]
matsim_gps_time  = matsim_unix_time+gps_utc_time; % [seconds]

id          = 702;
name        = "State College Road Network";
time_stamp  = matsim_gps_time;
description = "Estimating parameters from traffic trajectories with half sine modification.";
date_added  = dt;
input_table = table(id,name,time_stamp,description,date_added);

%% ------------------------ CONNECT TO DATABASE ------------------------ %%
% Database parameters
databasename = 'nsf_roadtraffic_friction_v2'; % database name
username = 'brennan'; % user name for the server
password = 'ivsg@Reber320'; % password
driver = 'org.postgresql.Driver';   % JDBC Driver
url    = ['jdbc:postgresql://130.203.223.234:5432/',databasename]; % This defines the IP address and port of the computer hosting the data (MOST important)

% connect to databse
conn = database(databasename,username,password,driver,url);

% check the connection status and 
% try to reconnect if the connection is not successful
while 0~=size(conn.Message)
    fprintf('Trying to connect to the DB \n');
    conn = database(databasename,username,password,driver,url);
end
fprintf('Connected to the DB \n');

%% ----------------------- PUSH DATA TO DATABASE ----------------------- %%
tablename = 'vehicle_simulation_trips';  % relation name
% insert data into the table
sqlwrite(conn, tablename, input_table);

%% ----------------------- DISCONNECT TO DATABASE ---------------------- %%
close(conn);
