%% Purpose:
%   The purpose of the script is to plot friction demand data.
% 
% Author:  Satya Prasad (szm888@psu.edu)
% Created: 2022/04/09

%% Prepare the workspace
clear all %#ok<CLALL>
close all
clc

%% Add path to dependencies
addpath('./Utilities/DB Lib');

%% Define inputs and other parameters
dbInput.ip_address = '130.203.223.234'; % Ip address of server host
dbInput.port       = '5432'; % port number
dbInput.username   = 'brennan'; % user name for the server
dbInput.password   = 'ivsg@Reber320'; % password
dbInput.db_name    = 'nsf_roadtraffic_friction_v2'; % name of the database

dbInput.sim_traffic_table = 'compare_vehicle_sims'; % name of the table containing simulated traffic data

vehicle_sim_trip_id = 102;
number_of_colormaps = 101;
colormap_friction_demand = jet(number_of_colormaps);

%% Query friction demand data
% Connect to the database
DB = Database(dbInput.db_name,dbInput.ip_address,dbInput.port,...
              dbInput.username,dbInput.password);

% SQL statement to query friction demand and position data
sql_query = ['SELECT cg_latitude, cg_longitude, friction_true_fl'...
            ' FROM ' dbInput.sim_traffic_table...
            ' WHERE vehicle_sim_trip_id = ' num2str(vehicle_sim_trip_id)];
friction_demand = fetch(DB.db_connection,sql_query);

% Disconnect from the database
DB.disconnect();

%% Plot the friction demand data
% Find minimum and maximum friction demand
max_friction_demand = max(0.9*friction_demand.friction_true_fl);
if 1<max_friction_demand
    max_friction_demand = 1;
end
min_friction_demand = min(0.9*friction_demand.friction_true_fl);

% Define colour vector based on friction demand
colormap_indices = round(100*(friction_demand.friction_true_fl-...
                              min_friction_demand)/(max_friction_demand-...
                                                    min_friction_demand))+1;
colormap_indices(colormap_indices>number_of_colormaps) = number_of_colormaps;

% Define axis limits
min_lat = min(friction_demand.cg_latitude);
max_lat = max(friction_demand.cg_latitude);
buffer_spacing_lat = 0.05*(max_lat-min_lat);
min_lon = min(friction_demand.cg_longitude);
max_lon = max(friction_demand.cg_longitude);
buffer_spacing_lon = 0.05*(max_lon-min_lon);

hFigAnimation = figure(12345); % Opens up the figure with a number
set(hFigAnimation,'Name','Friction Demand','Position',[200, 240, 746, 420]); % Puts a name on the figure and fix its size
geoscatter(friction_demand.cg_latitude,friction_demand.cg_longitude,...
           10,colormap_friction_demand(colormap_indices,:),'Marker','.');
geobasemap satellite; % set the background
geolimits([min_lat-buffer_spacing_lat max_lat+buffer_spacing_lat],...
          [min_lon-buffer_spacing_lon max_lon+buffer_spacing_lon]); % set the lat-lon limits
colormap(jet(number_of_colormaps));
colorbar;
caxis([min_friction_demand, max_friction_demand]);
