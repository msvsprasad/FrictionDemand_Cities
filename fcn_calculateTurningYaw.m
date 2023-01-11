function processed_centerline_pose_turning = fcn_calculateTurningYaw(turning_shape)
%% Extract ID values from shape file struct
id_turning_shp = (extractfield(turning_shape,'id_node'))';

%% get the centerline data for network
% initialize variable to store processed position orientation (pose) data
processed_centerline_pose_turning = [];
list_of_sectionIds_yaw = id_turning_shp;
for i = 1:length(list_of_sectionIds_yaw)
    % create the centerline path for each section ID
    centerline_path = [turning_shape(i).X', turning_shape(i).Y'];

    % clear NaN values from the data
    centerline_path = centerline_path(~isnan(centerline_path(:,1)),:);

    % calculate yaw based off of the centerline path
    
    centerline_yaw = fcn_Path_calcYawFromPathSegments(centerline_path);
%     centerline_yaw = atan2(diff(sections_shape(i).Y'),diff(sections_shape(i).X'));
%     centerline_yaw = centerline_yaw(~isnan(centerline_yaw(:,1)),:);
    centerline_yaw = [centerline_yaw; centerline_yaw(end)];

    % processed centerline data: section ID, path, yaw
    processed_centerline_pose_turning = [processed_centerline_pose_turning; ...
        list_of_sectionIds_yaw(i)*ones(numel(centerline_yaw),1),...
        centerline_path, centerline_yaw];
end
end
