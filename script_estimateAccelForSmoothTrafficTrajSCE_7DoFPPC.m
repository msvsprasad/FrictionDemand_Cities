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
addpath('./Utilities/VD Lib\Dualtrack');
addpath('./Utilities/Path Lib');
addpath('./Utilities/Circle Lib');
addpath('./Utilities/UTM Lib');

% Global variables for the vehicle model
global flag_update global_acceleration

%% Define inputs and parameters
dbInput.ip_address = '130.203.223.234'; % Ip address of server host
dbInput.port       = '5432'; % port number
dbInput.username   = 'brennan'; % user name for the server
dbInput.password   = 'ivsg@Reber320'; % password

dbInput.db_name       = 'roi_db'; % database name
dbInput.traffic_table = 'road_traffic_raw'; % table containing traffic simulation data
dbInput.trip_id       = 13; % traffic simulation id

% flag triggers
flag.dbQuery  = false; % set to 'true' to query from the database
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

%% Query for valid Section ID and Vehicle ID combinations
if flag.dbQuery
    SectionId_VehID    = fcn_findValidSectionandVehicleId(dbInput.trip_id,dbInput);
    list_of_vehicleIds = unique(SectionId_VehID(:,2));
else
    list_of_vehicleIds = [5007; 295];
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
    
    %% Run the simulation
    % Vehicle path in EN/XY coordinates
    vehicle_path = raw_trajectory{:,{'positionfront_x','positionfront_y'}};
    diff_station = sqrt(sum(diff(vehicle_path).^2,2));
    if 0 == diff_station(1)
        vehicle_station = cumsum([0; diff_station(2:end)]);
        diff_station    = [diff_station(2:end); diff_station(end)];
        raw_trajectory  = raw_trajectory(2:end,:);
        vehicle_path    = raw_trajectory{:,{'positionfront_x','positionfront_y'}};
    else
        vehicle_station = cumsum([0; diff_station]);
        diff_station    = [diff_station; diff_station(end)];  %#ok<AGROW>
    end
    % Orientation of the vehicle
    [~,ia,ic]    = unique(vehicle_station);
    temp_vehicle_path = vehicle_path(ia,:);
    vehicle_yaw  = fcn_Path_calcYawFromPathSegments(temp_vehicle_path);
    vehicle_yaw  = [vehicle_yaw; vehicle_yaw(end)]; %#ok<AGROW>
    vehicle_yaw  = vehicle_yaw(ic);
    clear ia ic temp_vehicle_path
    
    % Indices where the vehicle stopped
    temp_var         = (1:size(raw_trajectory,1))';
    % Indices at which the vehicle is at rest
    indices_to_rest  = temp_var(diff_station==0);
    % Indices at which the vehicle begins to stop
    indices_to_stop  = indices_to_rest([true; 1~=diff(indices_to_rest)]);
    % Indices at which the vehicle begins to move
    indices_to_start = [1; indices_to_rest([1~=diff(indices_to_rest); false])+1];
    % Total number of times a vehicle is stoping
    number_of_stops  = numel(indices_to_stop);
    
    % Process a vehicle trajectory between a start and stop
    for index_stop = 1:number_of_stops
        temp_trajectory = raw_trajectory(indices_to_start(index_stop):...
                                         indices_to_stop(index_stop),:);
        
        % Initial conditions
        global_acceleration = zeros(7,1); % global indicates that it's a global variable
        input_states = [temp_trajectory.current_speed(1); 0; 0; ...
                        temp_trajectory.current_speed(1)*ones(4,1)/vehicle.Re; ...
                        temp_trajectory.positionfront_x(1); ...
                        temp_trajectory.positionfront_y(1); ...
                        vehicle_yaw(indices_to_start(index_stop))];
        U = input_states(1);
        
        % Reference traversal for the vehicle to track
        reference_traversal.X   = temp_trajectory.positionfront_x;
        reference_traversal.Y   = temp_trajectory.positionfront_y;
        reference_traversal.Yaw = vehicle_yaw(indices_to_start(index_stop):...
                                              indices_to_stop(index_stop));
        reference_traversal.Station  = vehicle_station(indices_to_start(index_stop):...
                                                       indices_to_stop(index_stop));
        reference_traversal.Velocity = temp_trajectory.current_speed;
        
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
            pose = matlab_pose(counter,:)';
            [target_lookAhead_pose,target_U] = ...
                fcn_VD_snapLookAheadPoseOnToTraversal(pose,reference_traversal,controller);
            steering_angle = fcn_VD_lookAheadLatController(pose,target_lookAhead_pose,...
                             controller);
            wheel_torque   = fcn_VD_velocityController(U,target_U,controller);
            
            %% 7-DoF Vehicle Model
            flag_update = true; % set it to to true before every call to RK4 method
            if 1<=U
                [~,y] = fcn_VD_RungeKutta(@(t,y) fcn_VD_dt7dofModelForController(t,y,...
                    steering_angle,wheel_torque,...
                    vehicle,road_properties,friction_coefficient,type_of_transfer),...
                    input_states,t,deltaT);
                U = y(1); V = y(2); r = y(3); omega = y(4:7);
                input_states = y; clear y;
            elseif 0.2<=U
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
        end
        
        % Longitudinal and lateral acceleration
        lon_accel = matlab_States(:,8); lat_accel = matlab_States(:,9);
        % Longitudinal, lateral, and yaw velocity
        lon_vel   = matlab_States(:,1); lat_vel = matlab_States(:,2); yaw_rate = matlab_States(:,3);
        % Pose of the vehicle
        pose      = matlab_pose;
        cg_station = cumsum([0; sqrt(sum(diff(pose(:,[1,2])).^2,2))]);
        % Friction demand
        friction_demand = sqrt(lon_accel.^2 + lat_accel.^2)/9.81;
        
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
                enu_reference_id*ones(output_data_length,1);
            
            % traffic and vehicle dynamic simulation information
            friction_measurement.traffic_sim_trip_id = ...
                1*ones(output_data_length,1);
            friction_measurement.vehicle_sim_trip_id = ...
                simulink_trip_id*ones(output_data_length,1);
            
            % road segment and vehicle information
%             friction_measurement.road_segment_id = raw_trajectory.section_id;
            friction_measurement.vehicle_id = ...
                list_of_vehicleIds(index_vehicle)*ones(output_data_length,1);
            
            % vehicle cg information in LLA
            [friction_measurement.cg_latitude,friction_measurement.cg_longitude] = ...
                utm2ll(pose(:,1),pose(:,2),18);
%             friction_measurement.cg_altitude = raw_trajectory.altitude;
            
            % CG Pose
            friction_measurement.cg_east  = pose(:,1);
            friction_measurement.cg_north = pose(:,2);
%             friction_measurement.cg_up    = raw_trajectory.positionfront_z;
            friction_measurement.yaw      = pose(:,3);
            friction_measurement.cg_station = cg_station;
            
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
    end % NOTE: END FOR loop 'number_of_stops'
end
