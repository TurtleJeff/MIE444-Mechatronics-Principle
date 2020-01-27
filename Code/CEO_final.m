%CEO_final
%% map generation
%initalization of the world
dim1 = 8; dim2 = 4;   
locationindex = reshape(1:dim1*dim2,dim1,dim2)'; %1:512
n = numel(locationindex);

Blocks = [2, 3; 3, 2; 4, 3; 5, 1; 5, 3; 7, 1; 7, 3; 7, 4;];
for xx = 1:size(Blocks,1)
	x = Blocks(xx,1); y = Blocks(xx,2);
	M(1+(y-1)*1:(y-1)*1+1, 1+(x-1)*1:(x-1)*1+1) = 1;
end
M=[0     0     0     0     1     0     1    0;
   0     0     1     0     0     0     0    0;
   0     1     0     1     1     0     1    0;
   0     0     0     0     0     0     1    0];
M = [ones(dim2,1) M ones(dim2,1)];
M = [ones(1, dim1+2); M; ones(1, dim1+2)];  %1 represents block 6*10

%generate ultrasonic world  block is represented by numbers>5
ultra =  [2     1     5     2     9     3     9    3;
          1     2     8     2     5     0     5    1;
          5     8     3     9     9     5     9    5;
          2     5     1     5     5     2     10   3];

%create mask for blocks  0:block 16*32
M = abs(M-1);
M = M(2:end-1, 2:end-1);
%figure; imagesc((bw+1).*M); colormap(gray);

%initialize probability
p = ones(dim2,dim1)*(1/n); 
% navigation arrays (maps)
% South(down), 270, South(up), 90, East(left), 180, West(right) 0 
% 999 is destination, 1 is wall, 11 is loading zone
go_to_loadingzone = [11, 11, 999, 180, 1, 270, 1, 270, 11, 11, 1, 90, 180, 180, 180, 180, 999, 1, 270, 1, 1, 90, 1, 90, 90, 180, 180, 180, 180, 180, 1, 90];

go_to_destination_one = [11, 11, 0, 270, 1, 270, 1, 270, 11, 11, 1, 0, 0, 270, 180, 180, 270, 1, 999, 1, 1, 270, 1, 90, 0, 0, 90, 180, 180, 180, 1, 90];

go_to_destination_two = [11, 11, 0, 270, 1, 999, 1, 270, 11, 11, 1, 0, 0, 90, 180, 180, 270, 1, 270, 1, 1, 90, 1, 90, 0, 0, 0, 0, 0, 90, 1, 90];
               
go_to_destination_three = [11, 11, 0, 270, 1, 270, 1, 999, 11, 11, 1, 0, 0, 0, 0, 90, 270, 1, 270, 1, 1, 90, 1, 90, 0, 0, 0, 0, 0, 90, 1, 90];

go_to_destination_four = [11, 11, 0, 270, 1, 270, 1, 270, 11, 11, 1, 0, 0, 0, 0, 270, 270, 1, 270, 1, 1, 90, 1, 270, 0, 0, 0, 0, 0, 90, 1, 999];

%% Connect to the Manager
s = serial('COM8'); %tara term to check the com #
s.Baudrate = 9600;
fopen(s)
pause(3);

fwrite(s,'u');
while s.BytesAvailable() == 0
end
fscanf(s)
%% clear
while(s.BytesAvailable()) %clear the inputs
    fscanf(s)
end
%% get_the_compass_reading, define the south (prior to start)
fwrite(s,'C');
while s.BytesAvailable() == 0
end
compass_value = fscanf(s);
compass_value = str2num(compass_value);
south_value = compass_value;
%% get_the_compass_reading, calculate the heading
fwrite(s,'C');
while s.BytesAvailable() == 0
end
compass_value = fscanf(s);
compass_value = str2num(compass_value);

relative_value = compass_value - south_value;

if relative_value > 180
    relative_value = relative_value - 360;
elseif relative_value < -180
    relative_value = relative_value + 360;
end
  
if relative_value > -45 && relative_value < 45
    heading = 270;
elseif relative_value > 45 && relative_value < 135 
    heading = 0;
elseif relative_value > -135 && relative_value < -45
    heading = 180;
else
    heading = 90;
end
fwrite(s, 'x');
pause(3);
fwrite(s, 'x');
pause(3);
fwrite(s, 'X');
pause(1);
%%localization 
%figure;
%localizatoin loop

fwrite(s,'s'); %read the ultrasonic data
while s.BytesAvailable() == 0
end
    r_front = fscanf(s);
    r_front = str2num(r_front);
    r_left_front = fscanf(s);
    r_left_front = str2num(r_left_front);
    r_left_back = fscanf(s);
    r_left_back = str2num(r_left_back);
    r_right_front = fscanf(s);
    r_right_front = str2num(r_right_front);
    r_right_back = fscanf(s);
    r_right_back = str2num(r_right_back);
    r_back = fscanf(s);
    r_back = str2num(r_back);

type = status(r_front, r_right_front, r_right_back, r_left_front, r_left_back, r_back);
localization_status = 0;

p = sense_u(ultra, M, p, type); %update the probability, ultra is the status map
imagesc(p);
pause(1);
%title(['step: 0']);

while(localization_status==0)
    
    if r_front > 20
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        direction = 'w';
    elseif r_right_front > 20
        fwrite(s, 'r'); %turn right();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        direction = 'd';
    elseif r_left_front > 20
        fwrite(s, 'l'); %turn left();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        direction = 'a';
        
    else
        fwrite(s, 'l'); %turn left();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s, 'l'); %turn left();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        direction = 's';
    end
        
    
    %sensor update
    fwrite(s,'s'); %read the ultrasonic data
    while s.BytesAvailable() == 0
    end
    r_front = fscanf(s);
    r_front = str2num(r_front);
    r_left_front = fscanf(s);
    r_left_front = str2num(r_left_front);
    r_left_back = fscanf(s);
    r_left_back = str2num(r_left_back);
    r_right_front = fscanf(s);
    r_right_front = str2num(r_right_front);
    r_right_back = fscanf(s);
    r_right_back = str2num(r_right_back);
    r_back = fscanf(s);
    r_back = str2num(r_back);

    type = status(r_front, r_right_front, r_right_back, r_left_front, r_left_back, r_back);
    

    %movement update
    [p, heading] = move(p, M, heading, direction);
    p = sense_u(ultra, M, p, type);
    imagesc(p);
    pause(1);
    title(max(max(p)));
    
    for row=1:4
        for col=1:8
            if p(row, col)>0.5
                localization_status=1;
                %fwrite(b,'y');
            end
        end
    end
end
% fclose(b);
% clear b;
update = "localization completed"
%%navigation loop (after localization)

selected_map = go_to_loadingzone; %choose the map we need for navigation

navigation_status = 0;

max_probability = max(max(p));
[position_x, position_y] = find(p == max_probability); %find the coordination from the localization; 

while navigation_status == 0
    go_direction = selected_map(locationindex(position_x, position_y));
    
    if heading == go_direction
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        heading = go_direction;
        if heading == 0
            position_y = position_y + 1;
        end        
        if heading == 90
            position_x = position_x - 1;
        end
        if heading == 180
            position_y = position_y - 1;
        end
        if heading == 270
            position_x = position_x + 1;
        end                    

    elseif heading - go_direction == 90 || heading - go_direction == -270
        fwrite(s, 'r'); %turn right();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        heading = go_direction;
        if heading == 0
            position_y = position_y + 1;
        end        
        if heading == 90
            position_x = position_x - 1;
        end
        if heading == 180
            position_y = position_y - 1;
        end
        if heading == 270
            position_x = position_x + 1;
        end         
    
    elseif heading - go_direction == -90 || heading - go_direction == 270
        fwrite(s, 'l'); %turn left();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        heading = go_direction;
        if heading == 0
            position_y = position_y + 1;
        end        
        if heading == 90
            position_x = position_x - 1;
        end
        if heading == 180
            position_y = position_y - 1;
        end
        if heading == 270
            position_x = position_x + 1;
        end
        
    else
        fwrite(s, 'l'); %turn left();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s, 'l'); %turn left();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        heading = go_direction;
        if heading == 0
            position_y = position_y + 1;
        end        
        if heading == 90
            position_x = position_x - 1;
        end
        if heading == 180
            position_y = position_y - 1;
        end
        if heading == 270
            position_x = position_x + 1;
        end
    end
    
    if selected_map(locationindex(position_x, position_y)) == 999 %check if the navigation is completed.
        navigation_status = 1;
    end        
end
update = "loading zone arrived"
%%Loading zone, pick up the block.
fwrite(s,'v');
pause(10);
fwrite(s, 'x');
pause(2);
fwrite(s, 'X');
fwrite(s,'L');
while s.BytesAvailable() == 0
end
fscanf(s)
update = "Loading zone activity completed"
pause(10); %just in case the block is not picked up.

%%go to the destination
while(s.BytesAvailable()) %clear the inputs
    fscanf(s)
end

pause(2);
%%ultrasonic readings
fwrite(s,'s'); %read the ultrasonic data
while s.BytesAvailable() == 0
end
    r_front = fscanf(s);
    r_front = str2num(r_front);
    while s.BytesAvailable() == 0
end
    r_left_front = fscanf(s);
    r_left_front = str2num(r_left_front);
    while s.BytesAvailable() == 0
end
    r_left_back = fscanf(s);
    r_left_back = str2num(r_left_back);
    while s.BytesAvailable() == 0
end
    r_right_front = fscanf(s);
    r_right_front = str2num(r_right_front);
    while s.BytesAvailable() == 0
end
    r_right_back = fscanf(s);
    r_right_back = str2num(r_right_back);
    while s.BytesAvailable() == 0
end
    r_back = fscanf(s);
    r_back = str2num(r_back);
    
if r_left_back + r_left_front > 50
    heading = 270;
    position_x = 4;
    position_y = 1;
else 
    heading = 0;
    position_x = 1;
    position_y = 4;
end
    

selected_map = go_to_destination_four; %choose the map we need for navigation

navigation_status = 0;

while navigation_status == 0
go_direction = selected_map(locationindex(position_x, position_y));
    
    if heading == go_direction
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        heading = go_direction;
        if heading == 0
            position_y = position_y + 1;
        end        
        if heading == 90
            position_x = position_x - 1;
        end
        if heading == 180
            position_y = position_y - 1;
        end
        if heading == 270
            position_x = position_x + 1;
        end                    

    elseif heading - go_direction == 90 || heading - go_direction == -270
        fwrite(s, 'r'); %turn right();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        heading = go_direction;
        if heading == 0
            position_y = position_y + 1;
        end        
        if heading == 90
            position_x = position_x - 1;
        end
        if heading == 180
            position_y = position_y - 1;
        end
        if heading == 270
            position_x = position_x + 1;
        end         
    
    elseif heading - go_direction == -90 || heading - go_direction == 270
        fwrite(s, 'l'); %turn left();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        heading = go_direction;
        if heading == 0
            position_y = position_y + 1;
        end        
        if heading == 90
            position_x = position_x - 1;
        end
        if heading == 180
            position_y = position_y - 1;
        end
        if heading == 270
            position_x = position_x + 1;
        end
        
    else
        fwrite(s, 'l'); %turn left();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s, 'l'); %turn left();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        fwrite(s,'f'); %move forward();
        while s.BytesAvailable() == 0
        end
        fscanf(s)
        
        pause(1);
        fwrite(s, 'x');
        pause(2);
        
        heading = go_direction;
        if heading == 0
            position_y = position_y + 1;
        end        
        if heading == 90
            position_x = position_x - 1;
        end
        if heading == 180
            position_y = position_y - 1;
        end
        if heading == 270
            position_x = position_x + 1;
        end
    end
    
    if selected_map(locationindex(position_x, position_y)) == 999 %check if the navigation is completed.
        navigation_status = 1;
    end        
end
update = "destination arrived"
%%the end, drop off the blovck 't'
fwrite(s,'v');
pause(5);
fwrite(s, 't');