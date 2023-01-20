%%%%%%%%%%%%%% Function fcn_calculateRoadCenterlineSection %%%%%%%%%%%%%%
% Purpose:
%   fcn_calculateRoadCenterlineSection calculates the centerline of the
%   entire State College road network using the section.shp shape file of
%   the OSM road network. Each coordinate is associated with a section ID
% 
% Format:
%   road_centerline_section = fcn_calculateRoadCenterlineSection...
%       (sections_shape)
% 
% INPUTS:
%   sections_shape: sections.shp shape file from the OSM State College road
%       network
% 
% OUTPUTS:
%   road_centerline_section: consists of...
%       associated road section ID
%       UTM x coordinate
%       UTM y coordinate
% 
% Author: Juliette Mitrovich
% Created: 2022/12/21
% Updated: 2023/01/17

%%
function road_centerline_section = fcn_calculateRoadCenterlineSection(sections_shape)
%% Extract ID values from shape file struct
SectionID_numberOfLanes = [(extractfield(sections_shape,'id'))',...
    (extractfield(sections_shape,'nb_lanes'))'];

% id_sections_shp = (extractfield(sections_shape,'id'))';
% nb_lanes_shp = (extractfield(sections_shape,'nb_lanes'))';

%% get the centerline data for network
% initialize variable to store processed position orientation (pose) data
road_centerline_section = [];
list_of_sectionIds = SectionID_numberOfLanes(:,1);
list_number_of_lanes = SectionID_numberOfLanes(:,2);
for i = 1:length(list_of_sectionIds)
    % create the centerline path for each section ID
    centerline_path = [sections_shape(i).X', sections_shape(i).Y'];

    % clear NaN values from the data
    centerline_path = centerline_path(~isnan(centerline_path(:,1)),:);

    % processed centerline data: section ID, path, yaw
    road_centerline_section = [road_centerline_section; ...
        list_of_sectionIds(i)*ones(numel(centerline_path(:,1)),1),...
        list_number_of_lanes(i)*ones(numel(centerline_path(:,1)),1),...
        centerline_path];
    
    % calculate yaw based off of the centerline path
    
%     centerline_yaw = fcn_Path_calcYawFromPathSegments(centerline_path);
% %     centerline_yaw = atan2(diff(sections_shape(i).Y'),diff(sections_shape(i).X'));
% %     centerline_yaw = centerline_yaw(~isnan(centerline_yaw(:,1)),:);
%     centerline_yaw = [centerline_yaw; centerline_yaw(end)];
% 
%     % processed centerline data: section ID, path, yaw
%     road_centerline_section = [road_centerline_section; ...
%         list_of_sectionIds_yaw(i)*ones(numel(centerline_yaw),1),...
%         centerline_path, centerline_yaw];
end
end
