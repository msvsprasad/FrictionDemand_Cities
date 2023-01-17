%% Define inputs and parameters
% Database (DB) parameters that will NOT change
dbInput.ip_address = '130.203.223.234'; % Ip address of server host
dbInput.port       = '5432'; % port number
dbInput.username   = 'brennan'; % user name for the server
dbInput.password   = 'ivsg@Reber320'; % password
dbInput.db_name    = 'roi_db'; % database name
dbInput.trip_id    = 16; % traffic simulation id

% DB parameters that will change
dbInput.traffic_table = 'enu_reference_roi_db';

%% Query and store ENU reference data


% query ENU data
disp('Query for ENU reference data')
enu_reference_table = fcn_queryENUData(dbInput);

% Set coordinates of local origin for converting LLA to ENU
lat0 = enu_reference_table{1,{'latitude'}};
lon0 = enu_reference_table{1,{'longitude'}};
h0 = enu_reference_table{1,{'altitude'}};
wgs84 = wgs84Ellipsoid;

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

% convert lat and lon to UTM
[X,Y] = ll2utm(elevation_map(:,1),elevation_map(:,2),18);

%% Get the road centerline section data from AIMSUN shape file
% Load necessary mat files into the workspace
load('sections_shape.mat') % shape file used in the AIMSUN simulation
% load('turning_shape.mat')

% Calculate centerline position and orientation (pose) section ID, X, Y, yaw
road_centerline_section = fcn_calculateRoadCenterlineSection(sections_shape);
% road_centerline_junction = fcn_calculateRoadCenterlineTurning(turning_shape);

%% Add elevation to the road centerline
% set the minimun and maximum X and Y values to limit the nearest
% neighbors search to just the specific trajectory
position_front_x_min_RCL = min(road_centerline_section(:,2));
position_front_x_max_RCL = max(road_centerline_section(:,2));
position_front_y_max_RCL = max(road_centerline_section(:,3));
position_front_y_min_RCL = min(road_centerline_section(:,3));

elevation_map_range_RCL = (X>=position_front_x_min_RCL & ...
    X<=position_front_x_max_RCL & ...
    Y>=position_front_y_min_RCL & ...
    Y<=position_front_y_max_RCL);
Xnew = X(X>=position_front_x_min_RCL & X<=position_front_x_max_RCL & ...
    Y>=position_front_y_min_RCL & Y <=position_front_y_max_RCL);
Ynew = Y(X>=position_front_x_min_RCL & X<=position_front_x_max_RCL & ...
    Y>=position_front_y_min_RCL & Y <=position_front_y_max_RCL);

% create a new elevation map with the specific trajecotry range
elevation_map_new_RCL = elevation_map(elevation_map_range_RCL,:);

% Find the nearest neighbors
Idx = knnsearch([Xnew,Ynew],[road_centerline_section(:,2) road_centerline_section(:,3)],"K",2);
%%
% interpolate the altitude (average it)
if length(Idx)>=1 && length(Xnew)>1 && length(Ynew)>1
    path_vector = [Xnew(Idx(:,2))-Xnew(Idx(:,1)),...
        Ynew(Idx(:,2))-Ynew(Idx(:,1))];
    path_segment_length  = sum(path_vector.^2,2).^0.5;
    point_vector = [road_centerline_section(:,2)-Xnew(Idx(:,1)),...
        road_centerline_section(:,3)-Ynew(Idx(:,1))];
    projection_distance  = (path_vector(:,1).*point_vector(:,1)+...
        path_vector(:,2).*point_vector(:,2))...
        ./path_segment_length; % Do dot product
    percent_along_length = projection_distance./path_segment_length;

    % Calculate the outputs
    alt = elevation_map(Idx(:,1),3) + (elevation_map(Idx(:,2),3) - elevation_map(Idx(:,1),3)).*percent_along_length;
    
    %% Convert UTM to LL
    [lat_RCL, lon_RCL] = utm2ll(road_centerline_section(:,2),road_centerline_section(:,3),18);
    
    %% Convert lla to enu
    % lat and lon is as querried from the database
    % height is alt
    [cg_east_RCL, cg_north_RCL, cg_up_RCL] = geodetic2enu(lat_RCL, lon_RCL,...
        alt, lat0, lon0, h0, wgs84);
    road_centerline_section(:,2) = cg_east_RCL;
    road_centerline_section(:,3) = cg_north_RCL;
end
