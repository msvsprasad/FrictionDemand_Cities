function yaw_interp = fcn_calculateLCLyawNearestNeighbors(X,Y,vehicle_path,LCL_lane_yaw)
    Idx = knnsearch([X,Y],[vehicle_path(:,1),vehicle_path(:,2)],"K",2);

    % Interpolate the altitude
    if length(Idx)>=1 && length(X)>1 && length(Y)>1
        path_vector = [X(Idx(:,2))-X(Idx(:,1)),...
            Y(Idx(:,2))-Y(Idx(:,1))];
        path_segment_length  = sum(path_vector.^2,2).^0.5;
        point_vector = [vehicle_path(:,1)-X(Idx(:,1)),...
            vehicle_path(:,2)-Y(Idx(:,1))];
        projection_distance  = (path_vector(:,1).*point_vector(:,1)+...
            path_vector(:,2).*point_vector(:,2))...
            ./path_segment_length; % Do dot product
        percent_along_length = projection_distance./path_segment_length;

        % Calculate the outputs
        yaw_interp = LCL_lane_yaw(Idx(:,1),1) + (LCL_lane_yaw(Idx(:,2),1) - ...
            LCL_lane_yaw(Idx(:,1),1)).*percent_along_length;
    end
end
