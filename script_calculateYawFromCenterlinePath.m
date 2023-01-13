%% Prepare the workspace
clear all %#ok<CLALL>
close all
clc

%% Add path dependencies
addpath('./Datafiles'); % all .mat files
addpath('./Utilities')
addpath('./Utilities/Path Lib');
addpath('./Utilities/Circle Lib');
addpath('./Utilities/UTM Lib');

%% Load necessary mat files into the workspace
load('sections_shape.mat') % shape file used in the AIMSUN simulation
load('turning_shape.mat')
load('raw_trajectory_SCE_1.mat')

vehicle.Re  = 0.32;
%% Flag triggers
flag_plot = false;

%% Calculate centerline position and orientation (pose)
% section ID, X, Y, yaw
centerline_pose = fcn_calculateCenterlineYaw(sections_shape);
centerline_turn_pose = fcn_calculateTurningYaw(turning_shape);

%% Create path variables for the vehicle trajectory and the centerline path

% find list of unique 'section_id' from raw trajectory
list_of_sectionIDs_traj_1 = unique(raw_trajectory{:,{'section_id'}});
list_of_junctionIDs_traj_1 = unique(raw_trajectory{:,{'junction_id'}});

processed_vehicle_yaw_interp = [];
processed_vehicle_station = [];
% List of section IDs on which the vehicle traveled
% There could be duplicates as the vehicle might travel on the same section
% more than once
sequence_of_traversed_sections = ...
    raw_trajectory{([true; 0~=diff(raw_trajectory{:,{'section_id'}})]),{'section_id'}};
% There is an assumption that two junctions are never one after another
sequence_of_traversed_junctions = ...
    raw_trajectory{([true; 0~=diff(raw_trajectory{:,{'section_id'}})]),{'junction_id'}};
temp_var = (1:size(raw_trajectory,1))';
index_list_start_of_section  = temp_var([true; 0~=diff(raw_trajectory{:,{'section_id'}})]);
index_list_start_of_junction = temp_var([true; 0~=diff(raw_trajectory{:,{'section_id'}})]);
clear temp_var;
%sequence_of_traversed_sections = sequence_of_traversed_sections(-1~=sequence_of_traversed_sections);
% loop over those sections
for i = 1:numel(sequence_of_traversed_sections)
    if 0 < sequence_of_traversed_sections(i)
        %for i = 1:length(list_of_sectionIDs_traj_1)
        %if list_of_sectionIDs_traj_1(i) > 0
        % find subset of the raw trajectory where section id == i
        index_start_of_section = index_list_start_of_section(i);
        if i~=numel(sequence_of_traversed_sections)
            index_end_of_section = index_list_start_of_section(i+1)-1;
        else
            index_end_of_section = size(raw_trajectory,1);
        end
        temp_raw_trajectory = raw_trajectory(index_start_of_section:index_end_of_section,:);
        %temp_raw_trajectory = ...
        %raw_trajectory((raw_trajectory{:,{'section_id'}} == sequence_of_traversed_sections(i)),:);
        vehicle_path_temp = temp_raw_trajectory{:,{'position_front_x','position_front_y'}};

        % find subset of processsed data where section id == i
        centerline_path_temp = centerline_pose((centerline_pose(:,1) == sequence_of_traversed_sections(i)),:);
        vehicle_yaw_interp = NaN(size(vehicle_path_temp,1),1);
        reference_traversal.X   = centerline_path_temp(:,2);
        reference_traversal.Y   = centerline_path_temp(:,3);
        reference_traversal.Yaw = centerline_path_temp(:,4);
        reference_traversal.Station = cumsum([0; sqrt(sum(diff([reference_traversal.X,reference_traversal.Y]).^2,2))]);
        vector_first_path_point_index = NaN(size(vehicle_path_temp,1),1);
        for j = 1:size(vehicle_path_temp,1)
            [~,~,vehicle_yaw_interp(j),first_path_point_index,second_path_point_index,percent_along_length] = ...
                fcn_Path_snapPointOntoNearestTraversalNew(vehicle_path_temp(j,:), reference_traversal);
            vector_first_path_point_index(i) = first_path_point_index;
        end
        %         processed_vehicle_station = [processed_vehicle_station; ...
        %             list_of_sectionIDs_traj_1(i)*ones(numel(reference_traversal.Station),1),...
        %             reference_traversal.Station];
        %         processed_vehicle_yaw_interp = [processed_vehicle_yaw_interp; ...
        %             list_of_sectionIDs_traj_1(i)*ones(numel(vehicle_yaw_interp),1),...
        %             vehicle_yaw_interp];

        vector_first_path_point_index = first_path_point_index;
        if any(diff(vector_first_path_point_index)<0)
            processed_vehicle_yaw_interp = [processed_vehicle_yaw_interp; ...
                sequence_of_traversed_sections(i)*ones(numel(vehicle_yaw_interp),1),...
                vehicle_yaw_interp+pi];
        else
            processed_vehicle_yaw_interp = [processed_vehicle_yaw_interp; ...
                sequence_of_traversed_sections(i)*ones(numel(vehicle_yaw_interp),1),...
                vehicle_yaw_interp];
        end

        if flag_plot
            figure(01)
            clf
            hold on
            grid on
            plot(cumsum([0; sqrt(sum((diff(vehicle_path_temp)).^2,2))]),vehicle_yaw_interp,'b.')
            plot(cumsum([0; sqrt(sum((diff(centerline_path_temp(:,[2,3]))).^2,2))]),centerline_path_temp(:,4),'ro')

            figure(02)
            clf
            hold on
            grid on
            plot(vehicle_path_temp(:,1),vehicle_path_temp(:,2),'b.')
            plot(centerline_path_temp(:,2),centerline_path_temp(:,3),'r.')
            axis equal
        end % NOTE: end flag plot if statement

    elseif sequence_of_traversed_sections(i) == -1
        % find subset of the raw trajectory where section id == i
        current_junction_id = sequence_of_traversed_junctions(i);
        %for k = 1:length(list_of_junctionIDs_traj_1)
        %if list_of_junctionIDs_traj_1(k) > 0
        index_start_of_junction = index_list_start_of_junction(i);
        if i~=numel(sequence_of_traversed_junctions)
            index_end_of_junction = index_list_start_of_junction(i+1)-1;
        else
            index_end_of_junction = size(raw_trajectory,1);
        end
        temp_raw_trajectory = raw_trajectory(index_start_of_junction:index_end_of_junction,:);
        %temp_raw_trajectory = ...
            %raw_trajectory((raw_trajectory{:,{'junction_id'}} == list_of_junctionIDs_traj_1(current_junction_id)),:);
        vehicle_path_temp = temp_raw_trajectory{:,{'position_front_x','position_front_y'}};

        % find subset of processsed data where section id == i
        centerline_turn_path_temp = centerline_turn_pose((centerline_turn_pose(:,1) == current_junction_id),:);
        if isempty(centerline_turn_path_temp) == 0
            vehicle_yaw_interp = NaN(size(vehicle_path_temp,1),1);

            reference_traversal.X   = centerline_turn_path_temp(:,2);
            reference_traversal.Y   = centerline_turn_path_temp(:,3);
            reference_traversal.Yaw = centerline_turn_path_temp(:,4);
            reference_traversal.Station = cumsum([0; sqrt(sum(diff([reference_traversal.X,reference_traversal.Y]).^2,2))]);
            vector_first_path_point_index = NaN(size(vehicle_path_temp,1),1);
            for j = 1:size(vehicle_path_temp,1)
                [~,~,vehicle_yaw_interp(j),first_path_point_index,second_path_point_index,percent_along_length] = ...
                    fcn_Path_snapPointOntoNearestTraversalNew(vehicle_path_temp(j,:), reference_traversal);
                vector_first_path_point_index(i) = first_path_point_index;
            end
            processed_vehicle_station = [processed_vehicle_station; ...
                current_junction_id*ones(numel(reference_traversal.Station),1),...
                reference_traversal.Station];

            %                     processed_vehicle_yaw_interp = [processed_vehicle_yaw_interp; ...
            %                         list_of_junctionIDs_traj_1(i)*ones(numel(vehicle_yaw_interp),1),...
            %                         vehicle_yaw_interp];


            if any(diff(vector_first_path_point_index)<0)
                processed_vehicle_yaw_interp = [processed_vehicle_yaw_interp; ...
                    current_junction_id*ones(numel(vehicle_yaw_interp),1),...
                    vehicle_yaw_interp+pi];
            else
                processed_vehicle_yaw_interp = [processed_vehicle_yaw_interp; ...
                    current_junction_id*ones(numel(vehicle_yaw_interp),1),...
                    vehicle_yaw_interp];
            end

            if flag_plot
                figure(01)

                grid on
                plot(cumsum([0; sqrt(sum((diff(vehicle_path_temp)).^2,2))]),vehicle_yaw_interp,'b.')
                plot(cumsum([0; sqrt(sum((diff(centerline_turn_path_temp(:,[2,3]))).^2,2))]),centerline_turn_path_temp(:,4),'ro')
                hold on

                figure(02)
                grid on
                plot(vehicle_path_temp(:,1),vehicle_path_temp(:,2),'b.')
                plot(centerline_turn_path_temp(:,2),centerline_turn_path_temp(:,3),'r.')
                hold on
                axis equal
            end % NOTE: end if statement for section ID > 0
        end
        %end
        %end
    end % NOTE: end for loop for unique section IDs
end

%%
vehicle_yaw_centerline = fcn_calculateVehicleYawFromCenterlineData(raw_trajectory,centerline_pose,centerline_turn_pose);
%%
vehicle_path = raw_trajectory{:,{'position_front_x','position_front_y'}};
diff_station = sqrt(sum(diff(vehicle_path).^2,2));
if 0 == diff_station(1)
    vehicle_station = cumsum([0; diff_station(2:end)]);
    diff_station    = [diff_station(2:end); diff_station(end)];
    raw_trajectory  = raw_trajectory(2:end,:);
    vehicle_path    = raw_trajectory{:,{'position_front_x','position_front_y'}};
else
    vehicle_station = cumsum([0; diff_station]);
    diff_station    = [diff_station; diff_station(end)];  %#ok<AGROW>
end
% Indices where the vehicle stopped
temp_var         = (1:size(raw_trajectory,1))';
% Indices at which the vehicle is at rest
indices_to_rest  = temp_var(diff_station==0);
% Indices at which the vehicle begins to stop
% so indices to start becomes 1, indeces to stop becomes last
% index (size of trajectory)
if length(indices_to_rest) < 1
    indices_to_stop = length(raw_trajectory{:,{'position_front_x'}});
    indices_to_start = 1;
else
    indices_to_stop  = indices_to_rest([true; 1~=diff(indices_to_rest)]);
    % Indices at which the vehicle begins to move
    indices_to_start = [1; indices_to_rest([1~=diff(indices_to_rest); false])+1];
    % Total number of times a vehicle is stoping
end
number_of_stops  = numel(indices_to_stop);
min_trip_size = 1;
%%
for index_stop = 1:number_of_stops

    start_stop_trajectory = raw_trajectory(indices_to_start(index_stop):...
        indices_to_stop(index_stop),:);

    if size(start_stop_trajectory,1) > min_trip_size

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

        %         figure(03)
        %         plot(reference_traversal.Station,reference_traversal.Y)
        %         hold on
        %         figure(04)
        %         plot(reference_traversal.Station,reference_traversal.Y)
        figure(05)
        plot(reference_traversal.Station,reference_traversal.Yaw)
        hold on
    end
end
