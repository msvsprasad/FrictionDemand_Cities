%%%%%%%%%%%%%%%%%%% Function fcn_queryVehicleTrajectory %%%%%%%%%%%%%%%%%%%
% Purpose:
%   Query enu_reference table from nsf_roadtraffic_friction_v2 database
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

enu_attributes = ['id, name, date_added, '...
                  'latitude, longitude, altitude, geography, '...
                  'epsg_code, latitude_std, longitude_std, altitude_std, '...
                  'timestamp']; % attributes in the enu reference table

%% Query vehicle trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _       
%  |  \/  |     (_)      
%  | \  / | __ _ _ _ __  
%  | |\/| |/ _` | | '_ \ 
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% connect to the database
db_name = 'nsf_roadtraffic_friction_v2';
db_ip_address = '130.203.223.234'; % IP address of server host
db_port = '5432';
db_username = 'brennan';
db_password = 'ivsg@Reber320';
traffic_table = 'enu_reference';
DB = Database(db_name,db_ip_address,db_port,...
              db_username,db_password);

% SQL statement to query vehicle trajectory
enu_query = ['SELECT ' enu_attributes...
             ' FROM ' traffic_table...
                   ' ORDER BY date_added'];

% query trajectory data from the DB
enu_reference_table = fetch(DB.db_connection, enu_query);

% Disconnect from the database
DB.disconnect();