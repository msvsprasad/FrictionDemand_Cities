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
%% Add elevation to the road centerline
% set the minimun and maximum X and Y values to limit the nearest
% neighbors search to just the specific trajectory
position_front_x_min_RCL = min(road_centerline_section(:,2));
position_front_x_max_RCL = max(road_centerline_section(:,2));
position_front_y_max_RCL = max(road_centerline_section(:,3));
position_front_y_min_RCL = min(road_centerline_section(:,3));
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
Idx = knnsearch([Xnew,Ynew],[road_centerline_section(:,2) road_centerline_section(:,3)],"K",2);
%%
% interpolate the altitude (average it)
if length(Idx)>=1 && length(Xnew)>1 && length(Ynew)>1
    path_vector = [Xnew(Idx(:,2))-Xnew(Idx(:,1)),...
        Ynew(Idx(:,2))-Ynew(Idx(:,1))];
    path_segment_length  = sum(path_vector.^2,2).^0.5;
    point_vector = [road_centerline_section(:,2)-Xnew(Idx(:,1)),...
        road_centerline_section(:,3)-Ynew(Idx(:,1))];
    projection_distance  = (path_vector(:,1).*point_vector(:,1)+...
        path_vector(:,2).*point_vector(:,2))...
        ./path_segment_length; % Do dot product
    percent_along_length = projection_distance./path_segment_length;

    % Calculate the outputs
    alt = elevation_map(Idx(:,1),3) + (elevation_map(Idx(:,2),3) - elevation_map(Idx(:,1),3)).*percent_along_length;
    
    %% Convert UTM to LL
    [lat_RCL, lon_RCL] = utm2ll(road_centerline_section(:,2),road_centerline_section(:,3),18);
    
    %% Convert lla to enu
    % lat and lon is as querried from the database
    % height is alt
    [cg_east_RCL, cg_north_RCL, cg_up_RCL] = geodetic2enu(lat_RCL, lon_RCL,...
        alt, lat0, lon0, h0, wgs84);
    road_centerline_section(:,2) = cg_east_RCL;
    road_centerline_section(:,3) = cg_north_RCL;
end
road_centerline_section_ENU = [road_centerline_section(:,1) cg_east_RCL cg_north_RCL cg_up_RCL];
end