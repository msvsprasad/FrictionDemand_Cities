%% have mat file of processed data from this script
clear
close all
clc
%% Add path dependencies
addpath('./Datafiles'); % all .mat files
addpath('./Utilities')
addpath('./Utilities/DB Lib'); % all the functions and wrapper class
addpath('./Utilities/VD Lib');
addpath('./Utilities/VD Lib/Dualtrack');
addpath('./Utilities/Path Lib');
addpath('./Utilities/Circle Lib');
addpath('./Utilities/UTM Lib');

global flag_update global_acceleration

%% Load necessary mat files into the workspace
load('sections_shape.mat') % shape file used in the AIMSUN simulation
load('raw_trajectory_SCE_1.mat')

%% Flag triggers
flag_plot = false;

%% Define vehicle properties
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

%% Calculate centerline position and orientation (pose)
% section ID, X, Y, yaw
centerline_pose = fcn_calculateCenterlineYaw(sections_shape);

%% Create path variables for the vehicle trajectory and the centerline path
vehicle_path = raw_trajectory{:,{'position_front_x',...
    'position_front_y'}};

% find list of unique 'section_id' from raw trajectory
list_of_sectionIDs_traj_1 = unique(raw_trajectory{:,{'section_id'}});

processed_vehicle_yaw_interp = [];
% loop over those sections
for i = 1:length(list_of_sectionIDs_traj_1)
    close all
    if list_of_sectionIDs_traj_1(i) > 0
        % find subset of the raw trajectory where section id == i
        temp_raw_trajectory = raw_trajectory((raw_trajectory{:,{'section_id'}} == list_of_sectionIDs_traj_1(i)),:);
        vehicle_path_temp = temp_raw_trajectory{:,{'position_front_x',...
            'position_front_y'}};
        % find subset of processsed data where section id == i
        centerline_path_temp = centerline_pose((centerline_pose(:,1) == list_of_sectionIDs_traj_1(i)),:);

        X_veh_traj = vehicle_path_temp(:,1);
        Y_veh_traj = vehicle_path_temp(:,2);
        X_centerline_path = centerline_path_temp(:,2);
        Y_centerline_path = centerline_path_temp(:,3);
        yaw_centerline = centerline_path_temp(:,4);

        Idx = knnsearch([X_centerline_path,Y_centerline_path],...
            [X_veh_traj,Y_veh_traj],"K",2);

        path_vector = [X_centerline_path(Idx(:,2))-X_centerline_path(Idx(:,1)),...
            Y_centerline_path(Idx(:,2))-Y_centerline_path(Idx(:,1))];
        path_segment_length  = sum(path_vector.^2,2).^0.5;
        point_vector = [X_veh_traj-X_centerline_path(Idx(:,1)),...
            Y_veh_traj-Y_centerline_path(Idx(:,1))];
        projection_distance  = (path_vector(:,1).*point_vector(:,1)+...
            path_vector(:,2).*point_vector(:,2))...
            ./path_segment_length; % Do dot product
        percent_along_length = projection_distance./path_segment_length;

        vehicle_yaw_interp = yaw_centerline(Idx(:,1))+ ...
            (yaw_centerline(Idx(:,2)) - ...
            yaw_centerline(Idx(:,1))).*percent_along_length;

        processed_vehicle_yaw_interp = [processed_vehicle_yaw_interp; ...
            list_of_sectionIDs_traj_1(i)*ones(numel(vehicle_yaw_interp),1),...
            vehicle_yaw_interp];

        if flag_plot
            figure(1)
            %             plot(vehicle_yaw_interp,'b.')
            %             hold on
            %             plot(yaw_centerline,'ro')
            %             hold on
            plot(X_veh_traj,vehicle_yaw_interp,'b.')
            hold on
            plot(X_centerline_path,yaw_centerline,'ro')

            %             figure(2)
            %             plot(X_veh_traj,Y_veh_traj,'b.')
            %             hold on
            %             plot(X_centerline_path,Y_centerline_path,'ro')
        end % NOTE: end flag plot if statement
    end % NOTE: end if statement for section ID > 0
end % NOTE: end for loop for unique section IDs

% %%
% diff_station = sqrt(sum(diff(vehicle_path).^2,2));
% if 0 == diff_station(1)
%     vehicle_station = cumsum([0; diff_station(2:end)]);
%     diff_station    = [diff_station(2:end); diff_station(end)];
%     raw_trajectory  = raw_trajectory(2:end,:);
%     vehicle_path    = raw_trajectory{:,{'position_front_x','position_front_y'}};
% else
%     vehicle_station = cumsum([0; diff_station]);
%     diff_station    = [diff_station; diff_station(end)];  %#ok<AGROW>
% end
% % Indices where the vehicle stopped
% temp_var         = (1:size(raw_trajectory,1))';
% % Indices at which the vehicle is at rest
% indices_to_rest  = temp_var(diff_station==0);
% % Indices at which the vehicle begins to stop
% % so indices to start becomes 1, indeces to stop becomes last
% % index (size of trajectory)
% if length(indices_to_rest) < 1
%     indices_to_stop = length(raw_trajectory{:,{'position_front_x'}});
%     indices_to_start = 1;
% else
%     indices_to_stop  = indices_to_rest([true; 1~=diff(indices_to_rest)]);
%     % Indices at which the vehicle begins to move
%     indices_to_start = [1; indices_to_rest([1~=diff(indices_to_rest); false])+1];
%     % Total number of times a vehicle is stoping
% end
% number_of_stops  = numel(indices_to_stop);
% min_trip_size = 1;
% %%
% for index_stop = 1:number_of_stops
%     start_stop_trajectory = raw_trajectory(indices_to_start(index_stop):...
%         indices_to_stop(index_stop),:);
% 
%     if size(start_stop_trajectory,1) > min_trip_size
% 
%         % Initial conditions
%         global_acceleration = zeros(7,1); % global indicates that it's a global variable
%         input_states = [start_stop_trajectory.current_speed(1); 0; 0; ...
%             start_stop_trajectory.current_speed(1)*ones(4,1)/vehicle.Re; ...
%             start_stop_trajectory.position_front_x(1); ...
%             start_stop_trajectory.position_front_y(1); ...
%             vehicle_yaw(indices_to_start(index_stop))];
%         U = input_states(1);
% 
%         % Reference traversal for the vehicle to track
%         reference_traversal.X   = start_stop_trajectory.position_front_x;
%         reference_traversal.Y   = start_stop_trajectory.position_front_y;
%         reference_traversal.Yaw = vehicle_yaw(indices_to_start(index_stop):...
%             indices_to_stop(index_stop)); % smooth yaw here, change variable name to variable
%         % calculated above
%         reference_traversal.Station  = vehicle_station(indices_to_start(index_stop):...
%             indices_to_stop(index_stop));
%         reference_traversal.Velocity = start_stop_trajectory.current_speed;
%     end
% end
