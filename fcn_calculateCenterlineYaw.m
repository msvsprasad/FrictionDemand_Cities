%%%%%%%%%% fcn_calculateCenterlineYaw.m %%%%%%%%%%
%% Purpose:
%   The purpose of this function is to calculate the yaw of the vehicle
%   using the centerline data from the shape file used for the AIMSUN
%   simulation
%
% Author: Juliette Mitrovich
% Created: 2022/12/21
% Updated: 2022/12/21

%%
function processed_centerline_pose = fcn_calculateCenterlineYaw(sections_shape)
%% Extract ID values from shape file struct
id_shp = (extractfield(sections_shape,'id'))';

%% get the centerline data for network
% initialize variable to store processed position orientation (pose) data
processed_centerline_pose = [];
list_of_sectionIds_yaw = id_shp;
for i = 1:length(list_of_sectionIds_yaw)
    % create the centerline path for each section ID
    centerline_path = [sections_shape(i).X', sections_shape(i).Y'];

    % clear NaN values from the data
    centerline_path = centerline_path(~isnan(centerline_path(:,1)),:);

    % calculate yaw based off of the centerline path
    centerline_yaw = fcn_Path_calcYawFromPathSegments(centerline_path);
    centerline_yaw = [centerline_yaw; centerline_yaw(end)];

    % processed centerline data: section ID, path, yaw
    processed_centerline_pose = [processed_centerline_pose; ...
        list_of_sectionIds_yaw(i)*ones(numel(centerline_yaw),1),...
        centerline_path, centerline_yaw];
end
end
