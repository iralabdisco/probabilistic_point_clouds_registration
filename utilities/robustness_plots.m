[FileName,PathName] = uigetfile('*.txt');
data = csvread(fullfile(PathName, FileName),2,0);

angles = data(:,1);

r1 = normalized(data(:,2));
r2 = normalized(data(:,3));
r3 = normalized(data(:,4));
r4 = normalized(data(:,5));
r5 = normalized(data(:,6));

p1 = plot(angles, r1, '.');
hold on;
p2 = plot(angles, r2, '.');
hold on;
p3 = plot(angles, r3, '.');
hold on; 
p4 = plot(angles, r4, '.');
hold on
p5 = plot(angles, r5, '.');

legend([p1,p2,p3,p4,p5],{'2' , '3' ,'4' , '5', '6'});


function n = normalized(vector)
max_v = max(vector);
n = vector/max_v;
end