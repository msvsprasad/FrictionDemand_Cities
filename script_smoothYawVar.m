%% Smoothing Yaw data test
%% Add path dependencies
addpath('./Datafiles'); % all .mat files
addpath('./Utilities')
addpath('./Utilities/DB Lib'); % all the functions and wrapper class
addpath('./Utilities/VD Lib');
addpath('./Utilities/VD Lib/Dualtrack');
addpath('./Utilities/Path Lib');
addpath('./Utilities/Circle Lib');
addpath('./Utilities/UTM Lib');
addpath('./Friction Demand Code MAT files');
%% Add necessary path files to the workspace
load('raw_veh_traj_1.mat')
load('sections_shape.mat')
load('SectionId_vehID.mat')

%% Extract X Y and ID values from shape file struct
X_shp = (extractfield(sections_shape,'X'))';
Y_shp = (extractfield(sections_shape,'Y'))';
id_shp_raw = (extractfield(sections_shape,'id'))';

% associate the section IDs with each X and Y field
% (don't actually need to do this because you only need the unique IDs but
% still good to know how to do)
id_shp = ones(numel(X_shp),1);

count_pos = 0;
count = 0;
for i = 1:length(sections_shape)
    c = sections_shape(i,:).X;
    for j = 1+count_pos:length(c)+count
        id_shp(j) = id_shp_raw(i);
        count = count+1;
    end
    count_pos = count;
end

%% get the centerline data for network
% add yaw and smooth (possibly)
% for loop to loop through each section (just the vehicle sections
% from section.shp want X, Y and ID


% run as separate file
% smooth X and Y: use filter or curve fitting
% calculate yaw using fcn_Path_calcYawFromPathSegments with
% smoothed X and Y as input

list_of_sectionIds_yaw = unique(id_shp);
for i = 1:length(list_of_sectionIds_yaw)
    vehicle_path = raw_trajectory{:,{'position_front_x','position_front_y'}};
    % if -1 is there, don't run the code
    % add if statement to only run code for > 0
    if list_of_sectionIds_yaw > 0
        yaw_temp_trajectory = ; % subset of raw_trajectory
        
        vehicle_yaw  = fcn_Path_calcYawFromPathSegments(yaw_temp_trajectory);
        
        Idx = knnsearch([X_yaw_smooth,Y_yaw_smooth],...
            [raw_trajectory{:,{'position_front_x'}},...
            raw_trajectory{:,{'position_front_y'}}],"K",2);
        % cg_east_new is X smooth
        % cg_north_new is Y smooth
        % cg_up is yaw smooth
        % pose(:,1) is position_front_x
        % pose(:,2) is position_front_y
        
        path_vector = [X_yaw_smooth(Idx(:,2))-X_yaw_smooth(Idx(:,1)),...
            Y_yaw_smooth(Idx(:,2))-Y_yaw_smooth(Idx(:,1))];
        path_segment_length  = sum(path_vector.^2,2).^0.5;
        point_vector = [raw_trajectory{:,{'position_front_x'}}-X_yaw_smooth(Idx(:,1)),...
            raw_trajectory{:,{'position_front_y'}}-Y_yaw_smooth(Idx(:,1))];
        projection_distance  = (path_vector(:,1).*point_vector(:,1)+...
            path_vector(:,2).*point_vector(:,2))...
            ./path_segment_length; % Do dot product
        percent_along_length = projection_distance./path_segment_length;
        
        vehicle_yaw_interp = vehicle_yaw(Idx(:,1)) + ...
            (vehicle_yaw(Idx(:,2)) - ...
            vehicle_yaw(Idx(:,1))).*percent_along_length;
    end
end