function vehicle_yaw_centerline = fcn_calculateVehicleYawFromCenterlineData(raw_trajectory,centerline_section_pose,centerline_junction_pose)
%% Create path variables for the vehicle trajectory and the centerline path
% vehicle_path = raw_trajectory{:,{'position_front_x','position_front_y'}};

% find list of unique 'section_id' from raw trajectory
% list_of_sectionIDs_traj_1 = unique(raw_trajectory{:,{'section_id'}});
% list_of_junctionIDs_traj_1 = unique(raw_trajectory{:,{'junction_id'}});

vehicle_yaw_centerline = [];
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
        centerline_path_temp = centerline_section_pose((centerline_section_pose(:,1) == sequence_of_traversed_sections(i)),:);
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

%         vector_first_path_point_index = first_path_point_index;
        if any(diff(vector_first_path_point_index)<0)
            vehicle_yaw_centerline = [vehicle_yaw_centerline; ...
                sequence_of_traversed_sections(i)*ones(numel(vehicle_yaw_interp),1),...
                vehicle_yaw_interp+pi];
        else
            vehicle_yaw_centerline = [vehicle_yaw_centerline; ...
                sequence_of_traversed_sections(i)*ones(numel(vehicle_yaw_interp),1),...
                vehicle_yaw_interp];
        end

    elseif sequence_of_traversed_sections(i) == -1
        % find subset of the raw trajectory where section id == i
        current_junction_id = sequence_of_traversed_junctions(i);
        index_start_of_junction = index_list_start_of_junction(i);
        if i~=numel(sequence_of_traversed_junctions)
            index_end_of_junction = index_list_start_of_junction(i+1)-1;
        else
            index_end_of_junction = size(raw_trajectory,1);
        end
        temp_raw_trajectory = raw_trajectory(index_start_of_junction:index_end_of_junction,:);
        vehicle_path_temp = temp_raw_trajectory{:,{'position_front_x','position_front_y'}};

        % find subset of processsed data where section id == i
        centerline_turn_path_temp = centerline_junction_pose((centerline_junction_pose(:,1) == current_junction_id),:);
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

            if any(diff(vector_first_path_point_index)<0)
                vehicle_yaw_centerline = [vehicle_yaw_centerline; ...
                    current_junction_id*ones(numel(vehicle_yaw_interp),1),...
                    vehicle_yaw_interp+pi];
            else
                vehicle_yaw_centerline = [vehicle_yaw_centerline; ...
                    current_junction_id*ones(numel(vehicle_yaw_interp),1),...
                    vehicle_yaw_interp];
            end
        end
    end % NOTE: end for loop for unique section IDs
end
vehicle_yaw_centerline = vehicle_yaw_centerline;
end
