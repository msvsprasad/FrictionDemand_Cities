%%%%%%%%%%%%%% script_estimateAccelForSmoothTrafficTrajSCE.m %%%%%%%%%%%%%%
%% Purpose:
%   The pupose of this script is to estimate lateral acceleration from
%   smoothed vehicle trajectory. Smoothing means replacing step lane change
%   with a half sine wave.
%
% Author: Satya Prasad
% Created: 2022/04/09

%% Prepare the workspace
clear all %#ok<CLALL>
% close all
clc

%% Add path to dependencies
addpath('./Datafiles'); % all .mat files
addpath('./Utilities'); % all the functions and wrapper class
addpath('./Utilities/DB Lib');
addpath('./Utilities/VD Lib');
addpath('./Utilities/Path Lib');
addpath('./Utilities/Circle Lib');
addpath('./Utilities/UTM Lib');

%% Define inputs and parameters
dbInput.ip_address = '130.203.223.234'; % Ip address of server host
dbInput.port       = '5432'; % port number
dbInput.username   = 'brennan'; % user name for the server
dbInput.password   = 'ivsg@Reber320'; % password

dbInput.db_name       = 'roi_db'; % database name
dbInput.traffic_table = 'road_traffic_raw'; % table containing traffic simulation data
dbInput.trip_id       = 13; % traffic simulation id

% flag triggers
flag.dbQuery  = true; % set to 'true' to query from the database
flag.doDebug  = false; % set to 'true' to print trajectory information to command window
flag.plot     = false; % set to 'true' to plot
flag.dbInsert = true; % set to 'true' to insert data to database

g = 9.81; % acceleration due to gravity [m/s^2]
dummy_enu_reference_id = 3;
dummy_traffic_id = 1;
utm_zone         = 18;
simulink_trip_id = 702;

%% Get the current UTC and GPS time
gps_utc_time = 18; % [seconds] Difference between GPS and UTC time
dt = datetime('now','TimeZone','America/New_York');
matsim_unix_time = posixtime(dt); % [seconds] UTC Time
matsim_gps_time  = matsim_unix_time+gps_utc_time; % [seconds] GPS Time

%% Query for valid Section ID and Vehicle ID combinations
if flag.dbQuery
    SectionId_VehID    = fcn_findValidSectionandVehicleId(dbInput.trip_id,dbInput);
    list_of_vehicleIds = unique(SectionId_VehID(:,2));
else
    list_of_vehicleIds = [5007; 295; 7511; 7308];
end % NOTE: END IF statement 'flag.dbQuery'

for index_vehicle = 1:numel(list_of_vehicleIds)
    %% Query for vehicle trajectory
    if flag.dbQuery
        raw_trajectory = fcn_queryVehicleTrajectory(list_of_vehicleIds(index_vehicle),...
                                                    dbInput.trip_id,dbInput);
    else
        load(['raw_trajectory_V' num2str(list_of_vehicleIds(index_vehicle)) '_T' num2str(dbInput.trip_id) '.mat']);
    end % NOTE: END IF statement 'flag.dbQuery'
    
    if flag.doDebug
        % Plot trajectory
        fcn_VD_plotTrajectory(raw_trajectory{:,{'positionfront_x',...
                                                'positionfront_y'}},12345);
        temp_raw_trajectory = raw_trajectory;
    end % NOTE: END IF statement 'flag.doDebug'
    
    %% Update step lanechange with half sine wave
    lane_width = 3.0; % [meter]
    indices_of_trajectory  = (1:size(raw_trajectory,1))';
    start_index_of_feature = indices_of_trajectory([true; 0~=diff(raw_trajectory.section_id)]);
    end_index_of_feature   = indices_of_trajectory([0~=diff(raw_trajectory.section_id); true]);
    number_of_features     = numel(start_index_of_feature);
    for index_feature = 1:number_of_features
        if -1~=raw_trajectory.section_id(start_index_of_feature(index_feature))
            if index_feature~=number_of_features
                if -1~=raw_trajectory.section_id(start_index_of_feature(index_feature+1))
                    current_feature_trajectory = ...
                        raw_trajectory(start_index_of_feature(index_feature):...
                                       end_index_of_feature(index_feature+1),:);
                    if 2<=size(current_feature_trajectory,1)
                        diff_station = sqrt(sum(diff([current_feature_trajectory.positionfront_x, ...
                                                      current_feature_trajectory.positionfront_y]).^2,2));
                        current_feature_trajectory.station_total = cumsum([0; diff_station]);
                        false_lanechange  = start_index_of_feature(index_feature+1)-...
                                            start_index_of_feature(index_feature)+1;
                        output_trajectory = fcn_smoothLanechangeOfTrajectory(...
                                            current_feature_trajectory,lane_width,false_lanechange);
                        raw_trajectory.positionfront_x(start_index_of_feature(index_feature):...
                                                       end_index_of_feature(index_feature+1)) = ...
                                                       output_trajectory.positionfront_x;
                        raw_trajectory.positionfront_y(start_index_of_feature(index_feature):...
                                                       end_index_of_feature(index_feature+1)) = ...
                                                       output_trajectory.positionfront_y;
                    end
                else
                    current_feature_trajectory = ...
                        raw_trajectory(start_index_of_feature(index_feature):...
                                       end_index_of_feature(index_feature),:);
                    if 2<=size(current_feature_trajectory,1)
                        diff_station = sqrt(sum(diff([current_feature_trajectory.positionfront_x, ...
                                                      current_feature_trajectory.positionfront_y]).^2,2));
                        current_feature_trajectory.station_total = cumsum([0; diff_station]);
                        output_trajectory = fcn_smoothLanechangeOfTrajectory(...
                                            current_feature_trajectory,lane_width,NaN);
                        raw_trajectory.positionfront_x(start_index_of_feature(index_feature):...
                                                       end_index_of_feature(index_feature)) = ...
                                                       output_trajectory.positionfront_x;
                        raw_trajectory.positionfront_y(start_index_of_feature(index_feature):...
                                                       end_index_of_feature(index_feature)) = ...
                                                       output_trajectory.positionfront_y;
                    end
                end % NOTE: END IF statement
            else
                current_feature_trajectory = ...
                    raw_trajectory(start_index_of_feature(index_feature):...
                                   end_index_of_feature(index_feature),:);
                if 2<=size(current_feature_trajectory,1)
                    diff_station = sqrt(sum(diff([current_feature_trajectory.positionfront_x, ...
                                                  current_feature_trajectory.positionfront_y]).^2,2));
                    current_feature_trajectory.station_total = cumsum([0; diff_station]);
                    output_trajectory = fcn_smoothLanechangeOfTrajectory(...
                                        current_feature_trajectory,lane_width,NaN);
                    raw_trajectory.positionfront_x(start_index_of_feature(index_feature):...
                                                   end_index_of_feature(index_feature)) = ...
                                                   output_trajectory.positionfront_x;
                    raw_trajectory.positionfront_y(start_index_of_feature(index_feature):...
                                                   end_index_of_feature(index_feature)) = ...
                                                   output_trajectory.positionfront_y;
                end
            end % NOTE: END IF statement
        end % NOTE: END IF statement
    end % NOTE: END FOR loop 'index_feature'
    
    if flag.doDebug
        figure(12346)
        clf
        plot(temp_raw_trajectory.positionfront_x,temp_raw_trajectory.positionfront_y,...
             'b.','Markersize',2)
        hold on
        grid on
        xlabel('East $[m]$','Interpreter','Latex','Fontsize',13)
        ylabel('North $[m]$','Interpreter','Latex','Fontsize',13)
        set(gca,'Fontsize',13)
        axis equal
        for index_feature = 1:number_of_features
            plot(raw_trajectory.positionfront_x(start_index_of_feature(index_feature):...
                                                end_index_of_feature(index_feature)),...
                 raw_trajectory.positionfront_y(start_index_of_feature(index_feature):...
                                                end_index_of_feature(index_feature)),...
                                                'go','Markersize',2)
        end % NOTE: END FOR loop 'index_feature'
    end % NOTE: END IF statement 'flag.doDebug'
    
    %% Estimate yaw, yaw rate, longitudinal acceleration, lateral acceleration
    vehicle_path = raw_trajectory{:,{'positionfront_x','positionfront_y'}};
    diff_station = sqrt(sum(diff(vehicle_path).^2,2));
    if 0 == diff_station(1)
        raw_trajectory = raw_trajectory(2:end,:);
        vehicle_path   = raw_trajectory{:,{'positionfront_x','positionfront_y'}};
        diff_station = sqrt(sum(diff(vehicle_path).^2,2));
    end
    cg_station   = cumsum([0; diff_station]);
    diff_station = [diff_station; diff_station(end)];  %#ok<AGROW>
    
    [~,ia,ic]    = unique(cg_station);
    vehicle_path = vehicle_path(ia,:);
    vehicle_yaw  = fcn_Path_calcYawFromPathSegments(vehicle_path);
    vehicle_yaw  = [vehicle_yaw; vehicle_yaw(end)]; %#ok<AGROW>
    vehicle_yaw  = vehicle_yaw(ic);
    % Pose
    pose = [raw_trajectory.positionfront_x,raw_trajectory.positionfront_y,...
            vehicle_yaw];
    % Velocities
    lon_vel   = raw_trajectory.current_speed;
    lon_vel(0==diff_station) = 0;
    lat_vel   = zeros(size(cg_station));
    yaw_rate  = diff(pose(:,3))./diff(raw_trajectory.aimsun_time);
    yaw_rate  = [yaw_rate; yaw_rate(end)]; %#ok<AGROW>
    % Acceleration
    lon_accel = diff(lon_vel)./diff(raw_trajectory.aimsun_time);
    lon_accel = [lon_accel; lon_accel(end)]; %#ok<AGROW>
    [~,~,radius_of_curvature,~,concavity,~] = ...
        fcn_generateInnerAndOuterParallelCurves(vehicle_path,1,0,0);
    radius_of_curvature = radius_of_curvature(ic);
    concavity           = concavity(ic);
    lat_accel = (lon_vel.^2)./(radius_of_curvature.*concavity); % lateral acceleration
    % Friction demand
    friction_demand = sqrt(lon_accel.^2 + lat_accel.^2)/g;
    
    %% Plot the results
    if flag.plot
        fcn_VD_plotStationLongitudinalAcceleration(cg_station,lon_accel,01); % Plot longitudinal acceleration
        fcn_VD_plotStationLateralAcceleration(cg_station,lat_accel,02); % Plot lateral acceleration
        
        fcn_VD_plotStationLongitudinalVelocity(cg_station,lon_vel,03); % Plot longitudinal velocity
        fcn_VD_plotStationLateralVelocity(cg_station,lat_vel,04); % Plot lateral velocity
        fcn_VD_plotStationYawRate(cg_station,yaw_rate,05); % Plot yaw rate
        
        fcn_VD_plotTrajectory(pose(:,[1,2]),06); % Plot output trajectory
        fcn_VD_plotStationYaw(cg_station,pose(:,3),07); % Plot yaw
        
        fcn_VD_plotStationFrictionDemand(cg_station,friction_demand,08); % Plot force ratio
    end % NOTE: END IF statement 'flag.plot'
    
    %% Insert data into database
    if flag.dbInsert
        % store data to struct
        output_data_length = length(cg_station);
        
        % reference for LLA to ENU transformation and viceversa
        friction_measurement.enu_reference_id = ...
            dummy_enu_reference_id*ones(output_data_length,1);
        
        % traffic and vehicle dynamic simulation information
        friction_measurement.traffic_sim_trip_id = ...
            dummy_traffic_id*ones(output_data_length,1);
        friction_measurement.vehicle_sim_trip_id = ...
            simulink_trip_id*ones(output_data_length,1);
        
        % road segment and vehicle information
        friction_measurement.road_segment_id = raw_trajectory.section_id;
        friction_measurement.vehicle_id = ...
            list_of_vehicleIds(index_vehicle)*ones(output_data_length,1);
        
        % vehicle cg information in LLA
        [friction_measurement.cg_latitude,friction_measurement.cg_longitude] = ...
            utm2ll(pose(:,1),pose(:,2),utm_zone);
        friction_measurement.cg_altitude = raw_trajectory.altitude;
        
        % CG Pose
        friction_measurement.cg_east  = pose(:,1);
        friction_measurement.cg_north = pose(:,2);
        friction_measurement.cg_up    = raw_trajectory.positionfront_z;
        friction_measurement.yaw      = pose(:,3);
        friction_measurement.cg_station = raw_trajectory.station_total;
        
        % CG Velocity
        friction_measurement.longitudinal_velocity = lon_vel;
        friction_measurement.lateral_velocity      = lat_vel;
        friction_measurement.yaw_rate              = yaw_rate;
        
        % CG Acceleration
        friction_measurement.longitudinal_acceleration = lon_accel;
        friction_measurement.lateral_acceleration      = lat_accel;
        
        % Force Demand
        friction_measurement.friction_demand = friction_demand;
        friction_measurement.direction = raw_trajectory.direction;
        
        % Time information
        friction_measurement.simulation_time = raw_trajectory.aimsun_time; % Aimsun Simulation time
        friction_measurement.sim_wall_time   = matsim_gps_time+...
                                               raw_trajectory.aimsun_time;
        time_zone = datetime(matsim_unix_time*ones(output_data_length,1),...
            'ConvertFrom', 'posixtime','TimeZone','America/New_York','Format','yyyy-MM-dd HH:mm:ss');
        friction_measurement.timestamp = cellstr(time_zone); %Convert to cell array of character vectors
        
        % convert the struct format to table format
        friction_measurement_table = struct2table(friction_measurement);
        
        % push data to database
        fcn_pushDataToIVSGdb(friction_measurement_table);
    end % NOTE: END IF statement 'flag.dbInsert'
end
