[A,R] = readgeoraster('USGS_13_n41w078_20220429.tif');
LatLim = (39.9994:(1/(3*3600)):41.0006-(1/(3*3600)));
LongLim = (-78.0006+(1/(3*3600)):(1/(3*3600)):-76.9994);

% convert vectors to a grid
[lat_grid,long_grid] = meshgrid(flip(LatLim),LongLim);

elevation_map = [lat_grid(:) long_grid(:) A(:)];

% convert lat and long to UTM

[X,Y] = ll2utm(elevation_map(:,1),elevation_map(:,2),18);

Idx = knnsearch([X,Y],[1000, 1000],"K",2);

% find elevation by interpolation
path_vector = [X(Idx(:,2))-X(Idx(:,1)), Y(Idx(:,2))-Y(Idx(:,1))];
%     path_vector  = path(second_path_point_index,:)-path(first_path_point_index,:);
    path_segment_length  = sum(path_vector.^2,2).^0.5;
    point_vector = [veh_traj(:,1)-X(Idx(:,1)), veh_traj(:,2)-Y(Idx(:,1))];
%     point_vector = point-path(first_path_point_index,:);
    projection_distance  = dot(path_vector,point_vector)/path_segment_length; % Do dot product
    percent_along_length = projection_distance./path_segment_length;
    
    % Calculate the outputs
%    elevation_path_point = path(first_path_point_index,:) + path_vector*percent_along_length;
alt = elevation_map(Idx(:,1),3) + (elevation_map(Idx(:,2),3) - elevation_map(Idx(:,1),3))*percent;
% alt 1: elevation_map(first_path_point_index,3)
% geodetic2enu