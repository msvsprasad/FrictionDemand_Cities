%%%%%%%%%% script_calculateCenterlineYaw.m %%%%%%%%%%
%% Purpose:
%   The purpose of this script is to calculate the yaw of the vehicle
%   using the centerline data from the shape file used for the AIMSUN
%   simulation
%
% Author: Juliette Mitrovich
% Created: 2022/12/16
% Updated: 2022/12/21

%%
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
%% Add necessary path files to the workspace
load('sections_shape.mat') % shape file used in the AIMSUN simulation

%% Define flag triggers
flag_plot = false;

%% Extract X and ID values from shape file struct
% X is only used to calculate the size of the variable to hold the
% processed data
% X_shp = (extractfield(sections_shape,'X'))'; 
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
    
    if flag_plot
        % plot data to check if yaw is smooth
        figure(01)
        clf
        plot(centerline_path(:,1),centerline_path(:,2),'b')
        hold on
        grid on
        axis equal

        figure(02)
        clf
        plot(cumsum([0; sqrt(sum(diff(centerline_path).^2,2))]),centerline_yaw)
    end

    % processed centerline data: section ID, path, yaw
    processed_centerline_pose = [processed_centerline_pose; ...
        list_of_sectionIds_yaw(i)*ones(numel(centerline_yaw),1),...
        centerline_path, centerline_yaw];
end
%% 
% associate the section IDs with each X and Y field
% (don't actually need to do this because you only need the unique IDs but
% still good to know how to do)
% id_shp = ones(numel(X_shp),1);
% count_pos = 0;
% count = 0;
% for i = 1:length(sections_shape)
%     c = sections_shape(i,:).X;
%     for j = 1+count_pos:length(c)+count
%         id_shp(j) = id_shp_raw(i);
%         count = count+1;
%     end
%     count_pos = count;
% end