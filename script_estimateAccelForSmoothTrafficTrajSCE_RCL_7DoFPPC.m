%%%%%%%%%% script_estimateAccelForSmoothTrafficTrajSCE_7DoFPPC.m %%%%%%%%%%
%% Purpose:
%   The pupose of this script is to estimate friction demand by letting a
%   7DOF vehicle model track the trajectory simulated by microscopic
%   traffic simulations in Aimsun.
%
% Author: Satya Prasad
% Created: 2022/04/09
% Updated: 2022/09/26

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

%% Add necessary .mat files to the workspace
load('A.mat') % geotiff file used to add elevation to coordinates
load('sections_shape.mat') % shape file used to find section IDs of traffic simulation
% load('turning_shape.mat') % shape file used to find junction IDs of traffic simulation
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

% flag triggers
flag.dbQuery  = true; % set to 'true' to query from the database
flag.dbQuerySectionID_VehID = false;
flag.doDebug  = false; % set to 'true' to print trajectory information to command window
flag.plot     = true; % set to 'true' to plot
flag.dbInsert = false; % set to 'true' to insert data to database

enu_reference_id = 2;
% change trip ID to something else so you can query new data
simulink_trip_id = 504;

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

%% Query and store ENU reference data
% set traffic table name
dbInput.traffic_table = 'enu_reference_roi_db';

% query ENU data
disp('Query for ENU reference data')
enu_reference_table = fcn_queryENUData(dbInput);

% Set coordinates of local origin for converting LLA to ENU
lat0 = enu_reference_table{1,{'latitude'}};
lon0 = enu_reference_table{1,{'longitude'}};
h0 = enu_reference_table{1,{'altitude'}};
wgs84 = wgs84Ellipsoid;

%% Create an elevation map and convert lat and lon coordinates to UTM
% the elevation map will be used to add elevation to the road centerline
% and the trajectories
[X,Y,elevation_map] = fcn_createElevationMapAndConvertLL2UTM(A);

%% Get the road centerline  data from AIMSUN shape file
% centerline data for road SECTIONS
% [sectionID number_of_lanes X Y]
road_centerline_section = fcn_calculateRoadCenterlineSection(sections_shape);

% centerline data for road JUNCTIONS
% road_centerline_junction = fcn_calculateRoadCenterlineTurning(turning_shape);

%% Convert road centerline position from UTM to ENU
% [section ID, # of lanes, cg_east, cg_north, cg_up]
road_centerline_section_ENU = fcn_addElevationAndConvertUTM2ENU_RCL...
    (road_centerline_section,X,Y,lat0,lon0,h0,wgs84,elevation_map);
%% Query for valid Section ID and Vehicle ID combinations
% set traffic table to collect vehicle trajectory data
dbInput.traffic_table = 'road_traffic_raw_extend_2'; % table containing traffic simulation data

if flag.dbQuerySectionID_VehID
    disp('Query for section and vehicle ID')
    SectionId_VehID    = fcn_findValidSectionandVehicleId(dbInput.trip_id,dbInput);
    list_of_vehicleIds = unique(SectionId_VehID(:,2));
    list_of_sectionIds = unique(SectionId_VehID(:,1));
else
    %     list_of_vehicleIds = [5007; 295];
    load('SectionId_VehID.mat')
    list_of_vehicleIds = unique(SectionId_VehID(:,2));
    list_of_sectionIds = unique(SectionId_VehID(:,1));
end % NOTE: END IF statement 'flag.dbQuery'

%% Query for section ID vehicle trajectory
for index_sectionID = 454%1:numel(list_of_sectionIds)
    if list_of_sectionIds(index_sectionID) > 0
        disp(index_sectionID)
        if flag.dbQuery
            disp('Query for section ID vehicle trajectory')
            raw_trajectory = fcn_querySectionIdVehicleTrajectory(list_of_sectionIds(index_sectionID),...
                dbInput.trip_id,dbInput);
        else
            load(['raw_trajectory_V' num2str(list_of_vehicleIds(index_sectionID)) '_T' num2str(dbInput.trip_id) '.mat']);
        end % NOTE: END IF statement 'flag.dbQuery'

        if flag.doDebug
            %        Plot trajectory
            fcn_VD_plotTrajectory(raw_trajectory{:,{'position_front_x',...
                'position_front_y'}},12345);
            temp_raw_trajectory = raw_trajectory;
        end % NOTE: END IF statement 'flag.doDebug'


        %% Add elevation to the State College road-network and convert to ENU
        [cg_east_VT, cg_north_VT, cg_up_VT] = fcn_addElevationAndConvertUTM2ENU_VT...
            (raw_trajectory,X,Y,elevation_map,lat0,lon0,h0,wgs84);

        if isempty(cg_east_VT) == 0
            % change UTM coordinates to new ENU coordinates in the raw
            %   trajectory
            raw_trajectory{:,{'position_front_x'}} = cg_east_VT;
            raw_trajectory{:,{'position_front_y'}} = cg_north_VT;

            %% Calculate lane centerline (LCL) from RCL for the specific section ID
            % Find the index to find the path coordinates for this section
            % ID
            index_RCL_path = road_centerline_section_ENU(:,1) == list_of_sectionIds(index_sectionID);

            % calculate station
            % RCL path for 1 section ID (index_sectionID)
            RCL_path = road_centerline_section_ENU(index_RCL_path,:);
            RCL_diff_station = sqrt(sum(diff(RCL_path).^2,2));
            if 0 == RCL_diff_station(1)
                RCL_station = cumsum([0; RCL_diff_station(2:end)]);
                RCL_diff_station    = [RCL_diff_station(2:end); RCL_diff_station(end)];
                raw_trajectory  = raw_trajectory(2:end,:);
                RCL_path    = [road_centerline_section_ENU(:,3),...
                    road_centerline_section_ENU(:,4)];
            else
                RCL_station = cumsum([0; RCL_diff_station]);
                RCL_diff_station    = [RCL_diff_station; RCL_diff_station(end)];  %#ok<AGROW>
            end

            % create a reference traversal structure
            % the reference traversal in the RCL
            reference_traversal.X = RCL_path(:,3); % x coordinate of RCL path
            reference_traversal.Y = RCL_path(:,4); % y coordinate of RCL path
            reference_traversal.Z = RCL_path(:,5); % z coordinate of RCL path
            reference_traversal.Station = RCL_station; % station that was just calculated

            % Determine how many LCL need to be calculated based on the
            % number of lanes in that section
            number_of_lanes_in_sectionID = unique(RCL_path(:,2));

            % Calculate the lane centerlines: [X, Y, Yaw, Station, lane #]
            [LCL_lane1, LCL_lane2, LCL_lane3, LCL_lane4] = ...
                fcn_calculateLaneCenterline(reference_traversal,number_of_lanes_in_sectionID);

            % Calculate the nearest neighbors for the yaw of each LCL, only
            % if there is a lane centerline to calculate
            if isempty(LCL_lane1) == 0
                LCL_lane1_yaw_interp = fcn_calculateLCLyawNearestNeighbors...
                    (LCL_lane1(:,1),LCL_lane1(:,2),vehicle_path_XYLN,LCL_lane1(:,3));
            end
            if isempty(LCL_lane2) == 0
                LCL_lane2_yaw_interp = fcn_calculateLCLyawNearestNeighbors...
                    (LCL_lane2(:,1),LCL_lane2(:,2),vehicle_path_XYLN,LCL_lane2(:,3));
            end
            if isempty(LCL_lane3) == 0
                LCL_lane3_yaw_interp = fcn_calculateLCLyawNearestNeighbors...
                    (LCL_lane3(:,1),LCL_lane3(:,2),vehicle_path_XYLN,LCL_lane3(:,3));
            end
            if isempty(LCL_lane4) == 0
                LCL_lane4_yaw_interp = fcn_calculateLCLyawNearestNeighbors...
                    (LCL_lane4(:,1),LCL_lane4(:,2),vehicle_path_XYLN,LCL_lane4(:,3));
            end
            %% Check if raw trajectory data points fall on the LCL
            % find unique vehicle IDs that traveled this section ID
            if flag.doDebug
                vehicleIds_on_sectionId = unique(raw_trajectory{:,'vehicle_id'});
                for index_vehicle = 1:length(vehicleIds_on_sectionId)
                    index_raw_path = raw_trajectory{:,'vehicle_id'} == vehicleIds_on_sectionId(index_vehicle);
                    raw_path = raw_trajectory{index_raw_path,{'position_front_x','position_front_y','lane_number'}};

                    figure(2)
                    hold on
                    plot(raw_path(:,1),raw_path(:,2),'g.')
                    hold on
                    axis equal
                end
            end
            %% Run vehicle simulation
            % calculate how many vehicles drive on this section ID and loop
            % through those vehicles
            vehicleIds_on_sectionId = unique(raw_trajectory{:,'vehicle_id'});
            for index_vehicle = 1:length(vehicleIds_on_sectionId)
                %% calculate the trajectory for the index_vehicle
                index_raw_path = raw_trajectory{:,'vehicle_id'} == vehicleIds_on_sectionId(index_vehicle);
                vehicle_path_XY = raw_trajectory{index_raw_path,...
                    {'position_front_x','position_front_y'}};
                vehicle_path_XYLN = raw_trajectory{index_raw_path,...
                    {'position_front_x','position_front_y','lane_number'}};

                %% Calculate station coordinates
                diff_station = sqrt(sum(diff(vehicle_path_XY).^2,2));
                if 0 == diff_station(1)
                    vehicle_station = cumsum([0; diff_station(2:end)]);
                    diff_station    = [diff_station(2:end); diff_station(end)];
                    raw_trajectory  = raw_trajectory(2:end,:);
                    vehicle_path_XYLN    = raw_trajectory{index_raw_path,...
                        {'position_front_x','position_front_y','lane_number'}};
                else
                    vehicle_station = cumsum([0; diff_station]);
                    diff_station    = [diff_station; diff_station(end)];  %#ok<AGROW>
                end

                %% Create the yaw variable for the index_vehicle
                % check how many lane numbers the vehicle uses
                lanes_driven_on = unique(vehicle_path_XYLN(:,3));
                
                % Find at what indeces the vehicle drove on each lane and
                % create the yaw variable for the vehicle
                start_point = 1; % starting index for the while loop
                lane_range_index = []; % store range of indeces that a vehicle was on a lane

                LCL_vehicle_yaw = []; % initialize yaw variable
                index_lane_number = start_point; % while loop index

                while (index_lane_number <= length(vehicle_path_XYLN))
                    if index_lane_number == start_point
                        lane_number_last = vehicle_path_XYLN(index_lane_number,3);
                        lane_range_index = [lane_range_index; index_lane_number];
                        index_lane_number = index_lane_number + 1;
                        % store index to pull out yaw values
                    elseif index_lane_number > start_point
                        % check current lane number value
                        lane_number = vehicle_path_XYLN(index_lane_number,3);
                        if lane_number == lane_number_last
                            lane_number_last = lane_number;
                            lane_range_index = [lane_range_index; index_lane_number];
                            index_lane_number = index_lane_number + 1;
                        else
                            % store the index you are at
                            start_point = index_lane_number;
                            % check what the last lane # was to
                            % decide which LCL yaw to grab
                            if lane_number_last == 1
                                for i = lane_range_index(1):lane_range_index(length(lane_range_index))
                                    LCL_vehicle_yaw = [LCL_vehicle_yaw; LCL_lane1_yaw_interp(i)];
                                end
                                lane_range_index = [];
                            end
                            if lane_number_last == 2
                                for i = lane_range_index(1):lane_range_index(length(lane_range_index))
                                    LCL_vehicle_yaw = [LCL_vehicle_yaw; LCL_lane2_yaw_interp(i)];
                                end
                                lane_range_index = [];
                            end
                            if lane_number_last == 3
                                for i = lane_range_index(1):lane_range_index(length(lane_range_index))
                                    LCL_vehicle_yaw = [LCL_vehicle_yaw; LCL_lane3_yaw_interp(i)];
                                end
                                lane_range_index = [];
                            end
                            if lane_number_last == 4
                                for i = lane_range_index(1):lane_range_index(length(lane_range_index))
                                    LCL_vehicle_yaw = [LCL_vehicle_yaw; LCL_lane4_yaw_interp(i)];
                                end
                                lane_range_index = [];
                            end
                        end

                    end
                end % NOTE: END WHILE loop to calculate trajectory yaw

                %% Calcuate the start stop trajectory (if any)
                % Indices where the vehicle stopped
                temp_var         = (1:size(vehicle_path_XY,1))';
                % Indices at which the vehicle is at rest
                indices_to_rest  = temp_var(diff_station==0);
                % Indices at which the vehicle begins to stop
                % so indices to start becomes 1, indeces to stop becomes last
                % index (size of trajectory)
                if length(indices_to_rest) < 1
                    indices_to_stop = length(vehicle_path_XY(:,1));
                    indices_to_start = 1;
                else
                    indices_to_stop  = indices_to_rest([true; 1~=diff(indices_to_rest)]);
                    % Indices at which the vehicle begins to move
                    indices_to_start = [1; indices_to_rest([1~=diff(indices_to_rest); false])+1];
                    % Total number of times a vehicle is stoping
                end
                number_of_stops  = numel(indices_to_stop);
                min_trip_size = 1;
                %             %% Process a vehicle trajectory between a start and stop
                %             for index_stop = 1:number_of_stops
                %
                %                 start_stop_trajectory = raw_trajectory(indices_to_start(index_stop):...
                %                     indices_to_stop(index_stop),:);
                %
                %                 if size(start_stop_trajectory,1) > min_trip_size

                % Initial conditions
                global_acceleration = zeros(7,1); % global indicates that it's a global variable
                input_states = [start_stop_trajectory.current_speed(1); 0; 0; ...
                    start_stop_trajectory.current_speed(1)*ones(4,1)/vehicle.Re; ...
                    start_stop_trajectory.position_front_x(1); ...
                    start_stop_trajectory.position_front_y(1); ...
                    vehicle_yaw(indices_to_start(index_stop))];
                U = input_states(1);

                % Reference traversal for the vehicle to track
                reference_traversal.X   = start_stop_trajectory.position_front_x;
                reference_traversal.Y   = start_stop_trajectory.position_front_y;
                reference_traversal.Yaw = vehicle_yaw(indices_to_start(index_stop):...
                    indices_to_stop(index_stop)); % smooth yaw here, change variable name to variable
                % calculated above
                reference_traversal.Station  = vehicle_station(indices_to_start(index_stop):...
                    indices_to_stop(index_stop));
                reference_traversal.Velocity = start_stop_trajectory.current_speed;

                % Time parameters
                % Note: TotalTime is the time taken in Aimsun, could add a slack to this
                TotalTime = raw_trajectory.aimsun_time(indices_to_stop(index_stop))-...
                    raw_trajectory.aimsun_time(indices_to_start(index_stop));
                % Duration where vehicle is at rest
                if index_stop ~= index_stop
                    duration_of_rest = raw_trajectory.aimsun_time(indices_to_stop(index_stop))-...
                        raw_trajectory.aimsun_time(indices_to_start(index_stop+1));
                else
                    duration_of_rest = 0;
                end

                % Define variable to store vehicle information
                matlab_time   = NaN(floor(TotalTime/deltaT)+1,1);
                matlab_States = NaN(floor(TotalTime/deltaT)+1,9);
                matlab_pose   = NaN(floor(TotalTime/deltaT)+1,3);
                counter = 1;
                for t = 0:deltaT:TotalTime
                    matlab_time(counter)       = t;
                    matlab_States(counter,1:7) = input_states(1:7)';
                    matlab_pose(counter,:)     = input_states(8:10)';

                    %% Controller: Steering + Velocity
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Note: Controller need to be tuned, particularly for the
                    % velocity
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % add if statement with total time to set target_U = 0
                    pose = matlab_pose(counter,:)';
                    [target_lookAhead_pose,target_U] = ...
                        fcn_VD_snapLookAheadPoseOnToTraversal(pose,reference_traversal,controller);
                    steering_angle = fcn_VD_lookAheadLatController(pose,target_lookAhead_pose,...
                        controller);
                    if 0<=U
                        wheel_torque = fcn_VD_velocityController(U,target_U,controller);
                    else
                        wheel_troque = zeros(4,1);
                    end

                    %% 7-DoF Vehicle Model
                    flag_update = true; % set it to to true before every call to RK4 method
                    if 0.5<=U
                        [~,y] = fcn_VD_RungeKutta(@(t,y) fcn_VD_dt7dofModelForController(t,y,...
                            steering_angle,wheel_torque,...
                            vehicle,road_properties,friction_coefficient,type_of_transfer),...
                            input_states,t,deltaT);
                        U = y(1); V = y(2); r = y(3); omega = y(4:7);
                        input_states = y; clear y;
                    elseif 0<=U
                        kinematic_input_states = [input_states(1); input_states(8:10)];
                        [~,y] = fcn_VD_RungeKutta(@(t,y) fcn_VD_dtKinematicModelForController(t,y,...
                            steering_angle,wheel_torque,...
                            vehicle,road_properties,type_of_transfer),...
                            kinematic_input_states,t,deltaT);
                        U = y(1); V = 0; omega = (U/vehicle.Re)*ones(4,1);
                        r = fcn_VD_kinematicYawRate(U,steering_angle,vehicle);
                        input_states = [U; V; r; omega; y(2:4)]; clear y;
                    end

                    matlab_States(counter,8:9) = global_acceleration(1:2)';
                    counter = counter+1;
                end % NOTE: END FOR loop for vehicle controller and model

                % Longitudinal and lateral acceleration
                lon_accel = matlab_States(:,8); lat_accel = matlab_States(:,9);
                % Longitudinal, lateral, and yaw velocity
                lon_vel   = matlab_States(:,1); lat_vel = matlab_States(:,2); yaw_rate = matlab_States(:,3);
                % Pose of the vehicle
                pose      = matlab_pose;
                % add snap funciton from Satya to find Up coord of ENU
                cg_station = cumsum([0; sqrt(sum(diff(pose(:,[1,2])).^2,2))]);
                % Friction demand
                friction_utilization = ((vehicle.b/vehicle.L)*((vehicle.m*long_vel^2)/R))...
                    /(vehicle.m*9.81*(vehicle.b/vehicle.L)+...
                    vehicle.m*lon_accel*vehicle.h_cg/vehicle.L);
                friction_demand = sqrt(lon_accel.^2 + lat_accel.v^2)/9.81;

                %% Plot the results
                if flag.plot
                    %                     fcn_VD_plotStationLongitudinalAcceleration(cg_station,lon_accel,01); % Plot longitudinal acceleration
                    %                     fcn_VD_plotStationLateralAcceleration(cg_station,lat_accel,02); % Plot lateral acceleration
                    %                     fcn_VD_plotStationLongitudinalVelocity(cg_station,lon_vel,03); % Plot longitudinal velocity
                    %                     fcn_VD_plotStationLateralVelocity(cg_station,lat_vel,04); % Plot lateral velocity
                    %                     fcn_VD_plotStationYawRate(cg_station,yaw_rate,05); % Plot yaw rate
                    fcn_VD_plotTrajectory(pose(:,[1,2]),06); % Plot output trajectory
                    %                     fcn_VD_plotStationYaw(cg_station,pose(:,3),07); % Plot yaw

                    %                     fcn_VD_plotStationFrictionDemand(cg_station,friction_demand,08); % Plot force ratio
                end % NOTE: END IF statement 'flag.plot'

                %% Insert data into database
                if flag.dbInsert
                    % store data to struct
                    output_data_length = length(cg_station);

                    % reference for LLA to ENU transformation and viceversa
                    friction_measurement.enu_reference_id = ...
                        enu_reference_id*ones(output_data_length,1);

                    % traffic and vehicle dynamic simulation information
                    friction_measurement.traffic_sim_trip_id = ...
                        dbInput.trip_id*ones(output_data_length,1);
                    friction_measurement.vehicle_sim_trip_id = ...
                        simulink_trip_id*ones(output_data_length,1);

                    % road segment and vehicle information
                    %             friction_measurement.road_segment_id = raw_trajectory.section_id;
                    friction_measurement.vehicle_id = ...
                        list_of_vehicleIds(index_sectionID)*ones(output_data_length,1);


                    % = ...
                    %   utm2ll(pose(:,1),pose(:,2),18
                    %  w/ wgs84 = wgs84Ellipsoid; as spheroid
                    %              = raw_trajectory.altitude;

                    % CG Pose
                    %                 friction_measurement.cg_east = cg_east;
                    %                 friction_measurement.cg_north = cg_north;
                    %                 friction_measurement.cg_up = cg_up;

                    friction_measurement.cg_east  = pose(:,1);
                    friction_measurement.cg_north = pose(:,2);
                    %               friction_measurement.cg_up    = raw_trajectory.position_front_z; MATLAB variable that gets created
                    [cg_new,ia,~] = unique([cg_east,cg_north],'rows');
                    cg_east_new = cg_new(:,1);
                    cg_north_new = cg_new(:,2);
                    cg_up_new = cg_up(ia);
                    Idx = knnsearch([cg_east_new,cg_north_new],[pose(:,1),pose(:,2)],"K",2);

                    % interpolate the altitude (average it)
                    path_vector = [cg_east_new(Idx(:,2))-cg_east_new(Idx(:,1)),...
                        cg_north_new(Idx(:,2))-cg_north_new(Idx(:,1))];
                    path_segment_length  = sum(path_vector.^2,2).^0.5;
                    point_vector = [pose(:,1)-cg_east_new(Idx(:,1)),...
                        pose(:,2)-cg_north_new(Idx(:,1))];
                    projection_distance  = (path_vector(:,1).*point_vector(:,1)+...
                        path_vector(:,2).*point_vector(:,2))...
                        ./path_segment_length; % Do dot product
                    percent_along_length = projection_distance./path_segment_length;
                    friction_measurement.cg_up = cg_up_new(Idx(:,1)) + (cg_up_new(Idx(:,2)) - cg_up_new(Idx(:,1))).*percent_along_length;

                    %                 friction_measurement.cg_up = interp2(cg_east,cg_north,cg_up,pose(:,1),pose(:,2),'extrapval',3);
                    friction_measurement.yaw      = pose(:,3);
                    friction_measurement.cg_station = cg_station;
                    % vehicle cg information in LLA
                    [friction_measurement.cg_latitude,...
                        friction_measurement.cg_longitude,...
                        friction_measurement.cg_altitude] = enu2geodetic(friction_measurement.cg_east,...
                        friction_measurement.cg_north,...
                        friction_measurement.cg_up,lat0,lon0,h0,wgs84);
                    % CG Velocity
                    friction_measurement.longitudinal_velocity = lon_vel;
                    friction_measurement.lateral_velocity      = lat_vel;
                    friction_measurement.yaw_rate              = yaw_rate;

                    % CG Acceleration
                    friction_measurement.longitudinal_acceleration = lon_accel;
                    friction_measurement.lateral_acceleration      = lat_accel;

                    % Force Demand
                    friction_measurement.friction_true_fl = friction_demand;

                    % Time information
                    friction_measurement.simulation_time = raw_trajectory.aimsun_time((indices_to_start(index_stop)+1))+matlab_time; % Aimsun Simulation time
                    friction_measurement.sim_wall_time   = matsim_gps_time+friction_measurement.simulation_time;
                    time_zone = datetime(matsim_unix_time*ones(output_data_length,1),...
                        'ConvertFrom', 'posixtime','TimeZone','America/New_York','Format','yyyy-MM-dd HH:mm:ss');
                    friction_measurement.timestamp = convertTo(time_zone,'excel'); % Convert to cell array of character vectors

                    % convert the struct format to table format
                    friction_measurement_table = struct2table(friction_measurement);

                    % push data to database
                    % fcn_pushDataToIVSGdb(friction_measurement_table);
                end % NOTE: END IF statement 'flag.dbInsert'
            end % NOTE: END FOR loop of vehicles on sectionID
        end %NOTE: END IF statement to check that ENU for VT was calculated
    end % NOTE: END IF statement for section ID > 0 (only evaluate sections)
end % NOTE: END FOR loop for evaluating section ID trajectories
% end  NOTE: END IF statement for min_trip_size
% end FOR loop 'number_of_stops'
