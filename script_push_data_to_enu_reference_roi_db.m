%% ------------------------ CONNECT TO DATABASE ------------------------ %%
% Database parameters
databasename = 'roi_db'; % database name
username = 'brennan'; % user name for the server
password = 'ivsg@Reber320'; % password
driver = 'org.postgresql.Driver';   % JDBC Driver
url    = ['jdbc:postgresql://130.203.223.234:5432/',databasename]; % This defines the IP address and port of the computer hosting the data (MOST important)

% connect to databse
conn = database(databasename,username,password,driver,url);

% check the connection status and 
% try to reconnect if the connection is not successful
while size(conn.Message) ~= 0
    fprintf('Trying to connect to the DB \n');
    conn = database(databasename,username,password,driver,url);
end
fprintf('Connected to the DB \n');

%% ----------------------- PUSH DATA TO DATABASE ----------------------- %%
tablename = 'enu_reference_roi_db';  % relation name
% insert data into the table
sqlwrite(conn, tablename, vehicle_trajectory);

%% ----------------------- DISCONNECT TO DATABASE ---------------------- %%
close(conn);
