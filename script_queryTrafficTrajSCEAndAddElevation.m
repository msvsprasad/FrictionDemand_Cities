%% Prepare the workspace
clear all %#ok<CLALL>
close all
clc

%% Add path to dependencies
addpath('./Datafiles'); % all .mat files
addpath('./Utilities')
addpath('./Utilities/DB Lib'); % all the functions and wrapper class
addpath('./Utilities/VD Lib');
addpath('./Utilities/VD Lib/Dualtrack');
addpath('./Utilities/Path Lib');
addpath('./Utilities/Circle Lib');
addpath('./Utilities/UTM Lib');

% Global variables for the vehicle model
global flag_update global_acceleration

%% Query and store ENU reference data
enu_attributes = ['id, name, date_added, '...
                  'latitude, longitude, altitude, geography, '...
                  'epsg_code, latitude_std, longitude_std, altitude_std, '...
                  'timestamp']; % attributes in the enu reference table

% connect to the database
db_name = 'nsf_roadtraffic_friction_v2';
db_ip_address = '130.203.223.234'; % IP address of server host
db_port = '5432';
db_username = 'brennan';
db_password = 'ivsg@Reber320';
traffic_table = 'enu_reference';
DB = Database(db_name,db_ip_address,db_port,...
              db_username,db_password);

% SQL statement to query vehicle trajectory
enu_query = ['SELECT ' enu_attributes...
             ' FROM ' traffic_table...
                   ' ORDER BY date_added'];

% query trajectory data from the DB
enu_reference_table = fetch(DB.db_connection, enu_query);

% Disconnect from the database
DB.disconnect();

%% Set coordinates of local origin for converting LLA to ENU
lat0 = enu_reference_table{1,{'latitude'}};
lon0 = enu_reference_table{1,{'longitude'}};
h0 = enu_reference_table{1,{'altitude'}};
wgs84 = wgs84Ellipsoid;

%% Define inputs and parameters
% define database for vehicle trajectories
dbInput.ip_address = '130.203.223.234'; % Ip address of server host
dbInput.port       = '5432'; % port number
dbInput.username   = 'brennan'; % user name for the server
dbInput.password   = 'ivsg@Reber320'; % password

dbInput.db_name       = 'roi_db'; % database name
dbInput.traffic_table = 'road_traffic_raw_extend_2'; % table containing traffic simulation data
dbInput.trip_id       = 15; % traffic simulation id

% flag triggers
flag.dbQuery  = true; % set to 'true' to query from the database
flag.doDebug  = true; % set to 'true' to print trajectory information to command window
flag.plot     = true; % set to 'true' to plot
flag.dbInsert = false; % set to 'true' to insert data to database

enu_reference_id = 3;
simulink_trip_id = 503;

deltaT           = 0.01; % Vehicle simulation step-size
aimsun_step_size = 0.1; % Microscopic simulation step-size

%% Define vehicle and controller properties
% Define a MATLAB structure that specifies the physical values for a vehicle.
% For convenience, we ask that you call this stucture 'vehicle'.
vehicle.m   = 1600; % mass (kg)
vehicle.Izz = 2500; % mass moment of inertia (kg m^2)
vehicle.Iw  = 1.2; % mass moment of inertia of a wheel (kg m^2)
vehicle.Re  = 0.32; % effective radius of a wheel (m)
vehicle.a   = 1.3; % length from front axle to CG (m)
vehicle.L   = 2.6; % wheelbase (m)
vehicle.b   = 1.3; % length from rear axle to CG (m)
vehicle.d   = 1.5; % track width (m)
vehicle.h_cg = 0.42; % height of the cg (m)
vehicle.Ca  = [95000; 95000; 110000; 110000]; % wheel cornering stiffnesses
vehicle.Cx  = [65000; 65000; 65000; 65000]; % longitudinal stiffnesses

vehicle.contact_patch_length = 0.15; % [meters]
vehicle.friction_ratio       = 1; % [No units]

controller.look_ahead_distance = 20; % look-ahead distance [meters]
controller.steering_Pgain      = 0.1; % P gain for steering control
controller.velocity_Pgain      = 200; % P gain for steering control

% Parameters and initial conditions for simulink and matlab model
road_properties.grade = 0; road_properties.bank_angle = 0; % road properties
friction_coefficient  = 0.9*ones(4,1);

%% Define load transfer conditions
vdParam.longitudinalTransfer = 0;
if vdParam.longitudinalTransfer
    vdParam.lateralTransfer = 1;
    type_of_transfer = 'both';
else
    vdParam.lateralTransfer = 0;
    type_of_transfer = 'default';
end

%% Get the current UTC and GPS time
gps_utc_time = 18; % [seconds] Difference between GPS and UTC time
matsim_unix_time = posixtime(datetime('now','TimeZone','America/New_York')); % [seconds] UTC Time
matsim_gps_time  = matsim_unix_time+gps_utc_time; % [seconds] GPS Time

%% Load Geotiff elevation data
load('A.mat')
load('R.mat')
% Get altitude data from USGS website
% [A,R] = readgeoraster('USGS_13_n41w078_20220429.tif'); % MATLAB R2020b>
% [A,R] = geotiffread('USGS_13_n41w078_20220429.tif'); % MATLAB R2019b

%% Convert lat and lon coordinates to UTM
% interpolate lat and lon limits
LatLim = (39.9994:(1/(3*3600)):41.0006-(1/(3*3600)));
LonLim = (-78.0006+(1/(3*3600)):(1/(3*3600)):-76.9994);

% convert vectors to a grid
[lat_grid,long_grid] = meshgrid(flip(LatLim),LonLim);

% create the elevation map
elevation_map = [lat_grid(:) long_grid(:) A(:)];

% convert lat and long to UTM
[X,Y] = ll2utm(elevation_map(:,1),elevation_map(:,2),18);

%% Query for valid Section ID and Vehicle ID combinations
if flag.dbQuery
    SectionId_VehID    = fcn_findValidSectionandVehicleId(dbInput.trip_id,dbInput);
    list_of_vehicleIds = unique(SectionId_VehID(:,2));
else
    list_of_vehicleIds = [5007; 295];
end % NOTE: END IF statement 'flag.dbQuery'

%% Query for vehicle trajectory
%for index_vehicle = 1:numel(list_of_vehicleIds)
index_vehicle = 1;
    if flag.dbQuery
        raw_trajectory = fcn_queryVehicleTrajectory(list_of_vehicleIds(index_vehicle),...
            dbInput.trip_id,dbInput);
    else
        load(['raw_trajectory_V' num2str(list_of_vehicleIds(index_vehicle)) '_T' num2str(dbInput.trip_id) '.mat']);
    end % NOTE: END IF statement 'flag.dbQuery'
    
    %% Create the elevation map
    % Find the nearest neighbors
    Idx = knnsearch([X,Y],[raw_trajectory{:,{'position_front_x','position_front_y'}}],"K",2);
    
    % find elevation by interpolation
    path_vector = [X(Idx(:,2))-X(Idx(:,1)), Y(Idx(:,2))-Y(Idx(:,1))];
    path_segment_length  = sum(path_vector.^2,2).^0.5;
    point_vector = [raw_trajectory{:,{'position_front_x'}}-X(Idx(:,1)), raw_trajectory{:,{'position_front_y'}}-Y(Idx(:,1))];
    projection_distance  = dot(path_vector,point_vector)./path_segment_length; % Do dot product
    percent_along_length = projection_distance./path_segment_length;
    
    % Calculate the outputs
    alt = elevation_map(Idx(:,1),3) + (elevation_map(Idx(:,2),3) - elevation_map(Idx(:,1),3)).*percent_along_length;
    
    %% Convert lla to enu
    % lat and lon is as querried from the database
    % height is alt
    [xEast, yNorth, zUp] = geodetic2enu(raw_trajectory{:,{'latitude_front'}},...
        raw_trajectory{:,{'longitude_front'}},...
        alt, lat0, lon0, h0, wgs84);
%end