%% Circular Fit model
% fcn_CircularFitModel creates a 2D circular fit based on inputted X and Y
% datasets. The number of fits created will be user defined using the input
% N_fits. Additionally, there are two different fit styles, one which uses
% solely circular fits, and another which uses a curing process to get rid
% of circular fits with large radii --> these large radii circles are then
% properly defined as linear fits.
%
%User Created Functions Used:
% circfit - This function creates a best fit circle model that is used
% throughout this function
% can be accesssed - > https://www.mathworks.com/matlabcentral/fileexchange/56412-circfit
% Citation - Andrew Horchler (2020). circfit (https://github.com/horchler/circfit), GitHub. Retrieved August 17, 2020.
%
% fcn_LinearFitModel - Created by myself. Can be accessed from github.
%
%Less known built-in Functions Used:
% dbscan - https://www.mathworks.com/help/stats/dbscan.html
%
%Format:
% [Circular_fits,Linear_fits] = fcn_CircularFitModel(Xdata,Ydata,N_fits,Fit_type)
%
%Inputs:
%       Xdata: A Nx1 dataset where N is the size of the dataset and size
%       must be atleast 3. This represents the X-coordinate of the inputted
%       dataset.
%
%       Ydata: Same as above except for the Y-coordinate of the data.
%
%       N_fits: This input details how many individual fits the user wants to be
%       created in order to create the overall fit. In general, more fits will lead to
%       a more accurate fit at the expense of time spent calculating. This
%       value ultimately decides what 'N' equals where N = N_fits + 1. This
%       input does not guarantee that N_fits are used becauset this
%       function cures concentric fits as well as large radii circular fits
%       so this ultimately leads to outputting fewerer cumulative fits.
%
%
%       The computation for N_fits uses floor, this will be explained in
%       the following 3 examples (note 4 lines means 5 data points in X,Y,Z):
%
%       ex 1 - dataset with 99 points broken up into 4 lines -> 5 data points
%       with length 19,19,19,19,23 (19 is determined by floor(99/5))
%       ex 2 - dataset with 100 points broken up into 4 lines -> 5 data points
%       with length 20,20,20,20,20
%       ex 3 - dataset with 101 points broken up into 4 lines -> 5 data points
%       with length 20,20,20,20,21
%
%       Fit_type: Fit type has two options, 'Standard' and 'Composite' which are
%       to inputted as strings. 'Standard' fit refers to a purely circular
%       fit approach. Meanwhile, 'Composite' fit refers to an approach that
%       includes curing poor circular fits with linear fits.
%
%       --Note--
%       based on the above example, you can see that this function should
%       not be used if you want to break up a 100 point dataset
%       into ~60 lines. This is because it will result in most of the
%       datapoints containing length 1, and the final datapoint taking care
%       of the remainder (in this case around length 40) -- this would lead to poor results in most cases.
%
%
%       --IMPORTANT--
%       This function also includes 4 pseudo-inputs: epsilon and minpts for
%       dbscan, Radius_offset, and Radius_Threshold. These are explained in
%       more detail below, and these settings can be adjusted at the user's
%       discretion.
%       Location:
%       epsilon: Line
%       minpts: Line
%       Radius_offset: Line
%       Radius_Threshold: Line
%
%Outputs:
%       Circular_fits: A Nx3 dataset which the the x-coord and y-coord for
%       the centerpoitn of the circular fit in addition to the radius of
%       the circular fit. This output is formatted :
%       [xcenter,ycenter,Radius]. the specific values for
%       xcenter,ycenter,and Radius can be one of three things: a real
%       number, NaN, and 1. A real number output is the general output and
%       this means that the fit is good to go. A NaN output means that this
%       circular fit was part of a concentric fit and was cured - this
%       means that the fit after a NaN output will encompass the data for
%       both of the concentric fits as just one circular fit. Lastly, an
%       output of 1 means that this section is better defined as a linear
%       fit based on the previously defined 'Radius_Threshold'. This
%       sections data was then replaced with a 1 and a circular fit will
%       can be found in the next output's space.
%
%       Linear_fits: A Nx2 matrix detailing x-coords and y-coords of linear
%       fits. This has the format [x-coord;y-coord]. This also output has 2 possible entries, a real number and
%       NaN. In this case, NaN just signifies a break in linear fits. For
%       example, if we have a matrix Linear_fits that looks like
%       [1 2 3 NaN 4 5;6 7 8 NaN 9 10] This would be significant of two
%       linear fits, the  first from (1,6) to (2,7) ending at (3,8). Then,
%       the NaN signifies a break in the linear fits so a new fit starts at
%       (4,9) to (5,10).
%
%       --Note--
%       The output Linear_fits will be empty if the 'standard' Fit_type is
%       chosen because this fit type only uses circular fits.
%
%       --Note--
%       Using this approach to calculating error combined with how the
%       function approaches remainders leads to most of the error taking
%       place within the range where the remainder is addressed. Because of
%       this, two values are printed which represents max error excluding
%       the remainder, and max error overall.
%
%Example:
%Xdata = (1:100)';
%Ydata = (sin(.1*x))';
%N_fits = 5
%Fit_type = 'Composite'
%
%[Circular_fits,Linear_fits] = fcn_CircularFitModel(Xdata,Ydata,N_fits,Fit_type)
%
%For this particular example, the Linear_fits output is empty because for
%the selected 'Radius_Threshold', no circular fits were identified to be
%better defined by a line segment. You can also see that for the
%Circular_fits output we have a NaN entry in the 3rd column. This means
%that the 3rd and 4th fits were initially identified as concentric fits...
%then, these concentric fits were redone for just one more properly defined
%circular fits -> this data is found in column 4.
%
%Things to work on:
%- include examples for other features than just the general use
%-clean up certain processes... there are surely cleaner ways to do some of
%these things
%-incorporate IVSG eqs for connecting circular and linear fits together to
%get a continuous fit --> linear fits currently do not match up perfectly
%so must think of a way to remedy this.
%
%This function was written on 8/16/20 by Daniel Fescenmyer
%
%Revision History:
%-None to data
%% Body of Function
function [Circular_fits,Linear_fits] = fcn_CircularFitModel(Xdata,Ydata,N_fits,Fit_type)

epsilon = 1000; % input for dbscan -> subject to change by user
minpts = 1; % input for dbscan -> subject to change by user
Radius_offset = 50; % Radius offset refers to the tolerance between neighboring circle's radii.
% if two circles differ by less than the 'Radius_offset', they could be
% tagged as concentric circles.
Radius_Threshold = 5000; % Radius_Threshold is the breakpoint at which large radius circles
% will start to be redefined as linear fits. This value is defined in
% meters so an input of 5000 would mean that circles with radius larger
% than 5000 will be converted to linear fits.
map = {'Linear'}; % this is a map that is used to incorporate string data into a numeric array used below
% here we start by grouping the data, we are checking if the data can be
% grouped without remainder according to N_fits, if not, the remainder will
% be calculated and dealt with individually.
Zdata = zeros(numel(Xdata),1); % This is needed for fcn_LinearFitModel, we use zeros because this is for 2D applications
if rem(numel(Xdata),N_fits) == 0
    X = Xdata(1:numel(Xdata));
    Y = Ydata(1:numel(Xdata));
    
    ReshapedX = reshape(X,[],N_fits);
    ReshapedY = reshape(Y,[],N_fits);
    
    xc = zeros(N_fits,1); % pre-allocating our 3 output matrices
    yc = xc;
    Radius = xc;
    for i  = 1:N_fits
        [xc(i),yc(i),Radius(i)] = circfit(ReshapedX(:,i),ReshapedY(:,i));
    end
    Length_of_Data = numel(Xdata);
    Length_Used = floor(Length_of_Data/N_fits);
    Circular_fits = [xc,yc,Radius];
else
    % accounting for the case with a remainder
    Length_of_Data = numel(Xdata);
    Length_Used = floor(Length_of_Data/N_fits); % this line here shows how the data will be grouped
    % if a remainder is present. for example, if we have 100 points
    % divided in 3 groups, they will be divided as 33,33, and 34. the
    % remainder will always be > the length of the other datasets.
    Remainder = Length_of_Data - (Length_Used*(N_fits-1));
    Length_for_loop = Length_of_Data - Remainder;
    
    X = Xdata(1:Length_for_loop);
    Y = Ydata(1:Length_for_loop);
    
    ReshapedX = reshape(X,[],N_fits-1);
    ReshapedY = reshape(Y,[],N_fits-1);
    
    xc = zeros(N_fits,1);
    yc = xc;
    Radius = xc;
    
    RemX = Xdata(Length_for_loop+1:end);
    RemY = Ydata(Length_for_loop+1:end);
    
    for i  = 1:N_fits-1
        [xc(i),yc(i),Radius(i)] = circfit(ReshapedX(:,i),ReshapedY(:,i));
    end
    [Remxc,Remyc,RemRadius] = circfit(RemX(:),RemY(:));
    
    xc(end) = Remxc; % adding the remainder on at the end for each of our outputs
    yc(end) = Remyc;
    Radius(end) = RemRadius;
    Circular_fits = [xc,yc,Radius];
end


% at this point, we have computed the circular fits for the case where
% there is a remainder and the case where the N_fits nicely results in no
% remainder.
%
% From here, we will now check that all these fits make sense and there are
% no concentric fits. If concentric fits are found, we will convert the 2
% fits that are concentric into just one fit.
idx = [];
Cluster_data = [];
counter = 0;
% this section is testing for concentric circles. This is done by
% using the built-in clustering algo, dbscan. We are then checking
% pairs to see if a cluster is identified by dbcscan. If a cluster is
% identified, then we are also checking to see if they have similar
% radii, according to the previously defined 'Radius_offest'. If they
% are concentric fits, then they are dealt with below.
for i = 1:N_fits-1
    Cluster_data = [xc(i:i+1),yc(i:i+1)];
    idx = dbscan(Cluster_data,epsilon,minpts);
    counter = counter +1; % counter used to make sure remainder is properly accounted for below
    if idx(1) == idx(2) && abs(Radius(i)-Radius(i+1)) <= Radius_offset
        % if the two circle centers form a cluster, we must also check the radius to see
        % if they could be concentric fits.
        % the value NaN is significant of a concentric fit. concentric
        % fits involve two or more circles and in this case, NaN is
        % assigned to the first circle and then the fit is redone for
        % the second cirlce to encompass the data from the two
        % concentric fits.
        xc(i) = NaN;
        yc(i) = NaN;
        Radius(i) = NaN;
        if counter == numel(xc)-1 % this is the case to deal with the last value since
            % the process for dealing with the last value is different
            % from the rest.
            Startvalue = (numel(Xdata)-Remainder-Length_Used+1);
            GroupedX = Xdata(Startvalue:end);
            GroupedY = Ydata(Startvalue:end);
            [xc(i+1),yc(i+1),Radius(i+1)] = circfit(GroupedX,GroupedY);
        else % this is the general process.
            x = i-1;
            Startvalue = x*Length_Used+1;
            GroupedX = Xdata(Startvalue:Startvalue + Length_Used);
            GroupedY = Ydata(Startvalue:Startvalue + Length_Used);
            [xc(i+1),yc(i+1),Radius(i+1)] = circfit(GroupedX,GroupedY);
        end
    end
end
% At this point, the data has been clustered and checked for concentric
% fits. These concentric fits have also been taken care of. If the user
% defined the 'Standard' mode for Fit_type, the final
% output is ready. If the user defined the 'Composite' mode for Fit_type,
% we must also check for circles with large radii that should be converted
% into linear fits.
if strcmp(Fit_type,'Composite') % there are two options: composite and standard
    Linear_fits = zeros(N_fits +1,2); %pre-allocate
    [FinalVector,~] = fcn_LinearFitModel(Xdata,Ydata,Zdata,numel(xc));
    % the above line pre-defines all of our circlular fits. We then
    % will draw from the required fits.
    if Radius(1) > Radius_Threshold % we are dealing with just the first value so that we can use (i-1) later on.
        xc(1) = find(strcmp(map,'Linear')); % this goes back to the map which we are using to
        yc(1) = find(strcmp(map,'Linear')); % incorpate string data into a numerical array.
        Radius(1) = find(strcmp(map,'Linear')); % this will result in a value of 1 which means 'Linear'.
        Linear_fits(1:2,:) = [FinalVector(1,1),FinalVector(1,2); FinalVector(2,1),FinalVector(2,2)];
        Linear_fits(3) = NaN(1,2);
        % you will see that for the linear fits, NaN is used to breakup
        % linear fits. This is for ease of use for the user.
    end
    for i = 2:numel(xc) % process for all values other than 1
        if Radius(i) > Radius_Threshold
            xc(i) = find(strcmp(map,'Linear'));
            yc(i) = find(strcmp(map,'Linear'));
            Radius(i) = find(strcmp(map,'Linear'));
            if isnan(Radius(i-1)) == 1
                % the above line means that if the previous input
                % is NaN, we must use two lines because NaN is
                % significant of a circlular fit for concentric
                % circles which contains the data for 2 circlular
                % fits... thus we use 2 linear fits.
                Linear_fits(i:i+2,:) = [FinalVector(i,1),FinalVector(i,2);FinalVector(i+1,1),FinalVector(i+1,2);FinalVector(i+2,1),FinalVector(i+2,2)];
                Linear_fits(i+3,:) = NaN(1,2);
            elseif Radius(i-1) == 1
                % the above line means that if the previous value
                % also is a linear fit, we only need the end point
                % since the first linear fit's endpoint is the
                % startpoint of the next linear fit.
                Linear_fits(i+1,:) = [FinalVector(i+1,1),FinalVector(i+1,2)];
                Linear_fits(i+2,:) = NaN(1,2);
            else % this addresses the general case where the previous value is just some real number
                if isnan(Linear_fits(i)) == 1
                    % this above case addresses if there was a NaN
                    % input from a preceding case. Our approach then
                    % is to just extend where this data is inputted
                    % since we are not concerned with correlating
                    % which fit takes place where, but rather the
                    % existence of fits.
                    % --Note--
                    % with some cleverness, the user can determine
                    % where each fit is slotting in.
                    Linear_fits(i+1:i+2,:) = [FinalVector(i+1,1),FinalVector(i+1,2);FinalVector(i+2,1),FinalVector(i+2,2)];
                    Linear_fits(i+3,:) = NaN(1,2);
                else
                    Linear_fits(i:i+1,:) = [FinalVector(i,1),FinalVector(i,2);FinalVector(i+1,1),FinalVector(i+1,2)];
                    Linear_fits(i+2,:) = NaN(1,2);
                end
            end
        end
    end
    % here we are eliminating zeros which are left over from the
    % preallocation. This will leave us with just our linear fits
    % and our NaN breaks in between fits
    Linear_fits_X = Linear_fits(Linear_fits(:,1)~=0,1);
    Linear_fits_Y = Linear_fits(Linear_fits(:,1)~=0,2);
    Linear_fits = [Linear_fits_X,Linear_fits_Y];
    Circular_fits = [xc,yc,Radius]; % redefining in order to take into account changes to xc,yc,Radius
    
elseif strcmp(Fit_type,'Standard')
    Circular_fits = [xc,yc,Radius];
    Linear_fits = [];
    % The standard case is far more simple since we do not need to take
    % into account possible large radii circles that should be
    % considered lines.
end
if any(isnan(Circular_fits)) == 1
    disp('One or more circular fits were found to be concentric and fitted appropriately.This will reduce the overall number of fits.')
else
    disp('No circular fits were found to be concentric.')
end
if any(Circular_fits == 1)
    disp('One or more circular fits were converted to linear fits due to radius constraints')
else
end
end













