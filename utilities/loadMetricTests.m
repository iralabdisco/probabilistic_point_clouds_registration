[FileName,PathName] = uigetfile('*.txt');
data = csvread(fullfile(PathName, FileName),2,0);

num_data = 31;
%x = [0:3:360];
%y = [0:3:360];

x = [0:1:30];
y = [0:1:30];

sse = vec2mat(data(:,5),num_data);
robust_sse = vec2mat(data(:,6),num_data);
robust_sse_avg = vec2mat(data(:,7),num_data);

median = vec2mat(data(:,4),num_data);
robust_median = vec2mat(data(:,5),num_data);

max_v = max(sse(:));
sse = sse/max_v;

max_v = max(robust_sse(:));
robust_sse = robust_sse/max_v;

max_v = max(robust_sse_avg(:));
robust_sse_avg = robust_sse_avg/max_v;

max_v = max(median(:));
median = median/max_v;

max_v = max(robust_median(:));
robust_median = robust_median/max_v;

color1 = rand(num_data);
color2 = rand(num_data);
color3 = rand(num_data);
color1(:) = 255;
color2(:) = 500;
color3(:) = 100;

colormap(jet);

f1 = figure('units','normalized','outerposition',[0 0 1 1]);
s1 = surf(x,y,sse,color1);
hold on;
s2 = surf(x,y,robust_sse,color2);
hold on;
s3 = surf(x,y,robust_sse_avg, color3);
legend([s1,s2,s3],{'SumSquaredErrors','Robust SSE','Normalized Robust SSE'});

folders = regexp(PathName,'/','split');
name = folders{end-1};
saveas(f1,strcat(name,'_multiple_sse.png'));
view([0,1]);
saveas(f1,strcat(name,'_multiple_sse_side.png'));

f2 = figure('units','normalized','outerposition',[0 0 1 1]);

s4 = surf(x,y,sse,color1);
hold on;
s5 = surf(x,y,robust_median,color2);
hold on;
s6 = surf(x,y,median, color3);
legend([s4,s5,s6],{'SumSquaredErrors','Robust Median','Median'});

saveas(f2,strcat(name,'_multiple.png'));
view([0,1]);
saveas(f2,strcat(name,'_multiple_side.png'));

printMinIndex(median);
printMinIndex(robust_median);
printMinIndex(robust_sse);
printMinIndex(robust_sse_avg);

function [] = printMinIndex(matrix)
[v,i] = min(matrix(:));
[r,c] = ind2sub(size(matrix),i);
disp([inputname(1),' ', num2str(r), ' ',num2str(c) ]);
end