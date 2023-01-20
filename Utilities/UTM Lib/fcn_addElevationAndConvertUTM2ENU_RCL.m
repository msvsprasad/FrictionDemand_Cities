%%%%%%%%%%%%%% Function fcn_addElevationAndConvertUTM2ENU_RCL %%%%%%%%%%%%%%
% Purpose:
%   fcn_addElevationAndConvertUTM2ENU_RCL uses UTM coordinates and the
%   output of fcn_createElevationMapAndConvertLL2UTM to calculate elevation
%   (altitude) for the road centerline (RCL). Then the UTM coordinates of
%   the RCL are converted to LLA. Finally, LLA is converted to ENU
% 
% Format:
%   road_centerline_section_ENU = fcn_addElevationAndConvertUTM2ENU_RCL...
%       (road_centerline_section,X,Y,lat0,lon0,h0,wgs84,elevation_map)
% 
% INPUTS:
%   road_centerline_section: road centerline section data calculated from
%       fcn_calculateRoadCenterlineSection in UTM coordinates
%   lat0: latitude reference point to convert lla to enu
%   lon0: longitude reference point to convert lla to enu
%   h0: altitude reference point to convert lla to enu
%   wgs84: a referenceEllipsoid object for the World Geodetic System of 
%       1984
% 
% OUTPUTS:
%   road_centerline_section_ENU: consists of...
%       road section ID
%       cg_east_RCL: east coordinate of the RCL
%       cg_north_RCL: east coordinate of the RCL
%       cg_up_RCL: east coordinate of the RCL
% 
% Author:  Juliette Mitrovich
% Created: 2023-01-17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function road_centerline_section_ENU = fcn_addElevationAndConvertUTM2ENU_RCL(road_centerline_section,X,Y,lat0,lon0,h0,wgs84,elevation_map)
%% Allocate each column of road_centerline_section to individual varaibles
sectionID = road_centerline_section(:,1);
number_of_lanes = road_centerline_section(:,2);
x_RCL = road_centerline_section(:,3);
y_RCL = road_centerline_section(:,4);

% set the minimun and maximum X and Y values to limit the nearest
% neighbors search to just the specific trajectory
position_front_x_min_RCL = min(x_RCL);
position_front_x_max_RCL = max(x_RCL);
position_front_y_max_RCL = max(y_RCL);
position_front_y_min_RCL = min(y_RCL);
% 
% elevation_map_range_RCL = (X>=position_front_x_min_RCL & ...
%     X<=position_front_x_max_RCL & ...
%     Y>=position_front_y_min_RCL & ...
%     Y<=position_front_y_max_RCL);
Xnew = X(X>=position_front_x_min_RCL & X<=position_front_x_max_RCL & ...
    Y>=position_front_y_min_RCL & Y <=position_front_y_max_RCL);
Ynew = Y(X>=position_front_x_min_RCL & X<=position_front_x_max_RCL & ...
    Y>=position_front_y_min_RCL & Y <=position_front_y_max_RCL);

% create a new elevation map with the specific trajecotry range
% elevation_map_new_RCL = elevation_map(elevation_map_range_RCL,:);

% Find the nearest neighbors
Idx = knnsearch([Xnew,Ynew],[x_RCL y_RCL],"K",2);
%%
% interpolate the altitude (average it)
if length(Idx)>=1 && length(Xnew)>1 && length(Ynew)>1
    path_vector = [Xnew(Idx(:,2))-Xnew(Idx(:,1)),...
        Ynew(Idx(:,2))-Ynew(Idx(:,1))];
    path_segment_length  = sum(path_vector.^2,2).^0.5;
    point_vector = [x_RCL-Xnew(Idx(:,1)),...
        y_RCL-Ynew(Idx(:,1))];
    projection_distance  = (path_vector(:,1).*point_vector(:,1)+...
        path_vector(:,2).*point_vector(:,2))...
        ./path_segment_length; % Do dot product
    percent_along_length = projection_distance./path_segment_length;

    % Calculate the outputs
    alt = elevation_map(Idx(:,1),3) + (elevation_map(Idx(:,2),3) - elevation_map(Idx(:,1),3)).*percent_along_length;
    
    %% Convert UTM to LL
    [lat_RCL, lon_RCL] = utm2ll(x_RCL,y_RCL,18);
    
    %% Convert lla to enu
    % lat and lon is as querried from the database
    % height is alt
    [cg_east_RCL, cg_north_RCL, cg_up_RCL] = geodetic2enu(lat_RCL, lon_RCL,...
        alt, lat0, lon0, h0, wgs84);
%     road_centerline_section(:,3) = cg_east_RCL;
%     road_centerline_section(:,4) = cg_north_RCL;
end
road_centerline_section_ENU = [sectionID number_of_lanes cg_east_RCL cg_north_RCL cg_up_RCL];
end