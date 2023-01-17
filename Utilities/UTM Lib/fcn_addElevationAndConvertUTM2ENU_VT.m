%%%%%%%%%%%%%% Function fcn_addElevationAndConvertUTM2ENU_VT %%%%%%%%%%%%%%
% Purpose:
%   fcn_addElevationAndConvertUTM2ENU_VT uses UTM coordinates and the
%   output of fcn... to calculate elevation (altitude) for the raw vehicle 
%   trajectory (VT). Then the calculated altitude, LLA reference points, 
%   and latitude and longitude values from raw_trajectory are used to
%   convert LLA to ENU.
% 
% Format:
%   [cg_east_VT, cg_north_VT, cg_up_VT] = 
%   fcn_addElevationAndConvertUTM2ENU_VT...
%       (raw_trajectory,X,Y,elevation_map,lat0,lon0,h0,wgs84)
% 
% INPUTS:
%   raw trajectory: specifically, position_front_x (UTM x),
%   position_front_y (UTM y), latitude_front, longitude_front
%   lat0: latitude reference point to convert lla to enu
%   lon0: longitude reference point to convert lla to enu
%   h0: altitude reference point to convert lla to enu
%   wgs84: a referenceEllipsoid object for the World Geodetic System of 1984
% 
% OUTPUTS:
%   cg_east_VT: east coordinate of the vehicle trajectory
%   cg_north_VT: east coordinate of the vehicle trajectory
%   cg_up_VT: east coordinate of the vehicle trajectory
% 
% Author:  Juliette Mitrovich
% Created: 2023-01-17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [cg_east_VT, cg_north_VT, cg_up_VT] = fcn_addElevationAndConvertUTM2ENU_VT(raw_trajectory,X,Y,elevation_map,lat0,lon0,h0,wgs84)
%% Add elevation to the State College road-network
% set the minimun and maximum X and Y values to limit the nearest
% neighbors search to just the specific trajectory
position_front_x_min = min(raw_trajectory{:,{'position_front_x'}});
position_front_x_max = max(raw_trajectory{:,{'position_front_x'}});
position_front_y_max = max(raw_trajectory{:,{'position_front_y'}});
position_front_y_min = min(raw_trajectory{:,{'position_front_y'}});

Xnew = X(X>=position_front_x_min & X<=position_front_x_max & ...
    Y>=position_front_y_min & Y <=position_front_y_max);
Ynew = Y(X>=position_front_x_min & X<=position_front_x_max & ...
    Y>=position_front_y_min & Y <=position_front_y_max);

% Find the nearest neighbors
Idx = knnsearch([Xnew,Ynew],[raw_trajectory{:,{'position_front_x','position_front_y'}}],"K",2);

% Interpolate the altitude
if length(Idx)>=1 && length(Xnew)>1 && length(Ynew)>1
    path_vector = [Xnew(Idx(:,2))-Xnew(Idx(:,1)),...
        Ynew(Idx(:,2))-Ynew(Idx(:,1))];
    path_segment_length  = sum(path_vector.^2,2).^0.5;
    point_vector = [raw_trajectory{:,{'position_front_x'}}-Xnew(Idx(:,1)),...
        raw_trajectory{:,{'position_front_y'}}-Ynew(Idx(:,1))];
    projection_distance  = (path_vector(:,1).*point_vector(:,1)+...
        path_vector(:,2).*point_vector(:,2))...
        ./path_segment_length; % Do dot product
    percent_along_length = projection_distance./path_segment_length;

    % Calculate the outputs
    alt = elevation_map(Idx(:,1),3) + (elevation_map(Idx(:,2),3) - elevation_map(Idx(:,1),3)).*percent_along_length;
    %% Convert lla to enu
    [cg_east_VT, cg_north_VT, cg_up_VT] = geodetic2enu(raw_trajectory{:,{'latitude_front'}},...
        raw_trajectory{:,{'longitude_front'}}, alt, lat0, lon0, h0, wgs84);
end
end