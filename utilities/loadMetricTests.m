[FileName,PathName] = uigetfile('*.txt');
data = csvread(fullfile(PathName, FileName),2,0);

sse = vec2mat(data(:,5),121);
robust_sse = vec2mat(data(:,6),121);
robust_sse_avg = vec2mat(data(:,7),121);

median = vec2mat(data(:,4),121);
average = vec2mat(data(:,5),121);

max_v = max(sse(:));
sse = sse/max_v;

max_v = max(robust_sse(:));
robust_sse = robust_sse/max_v;

max_v = max(robust_sse_avg(:));
robust_sse_avg = robust_sse_avg/max_v;

max_v = max(median(:));
median = median/max_v;

max_v = max(average(:));
average = average/max_v;

color1 = rand(121);
color2 = rand(121);
color3 = rand(121);
color1(:) = 255;
color2(:) = 500;
color3(:) = 100;

x = [0:3:360];
y = [0:3:360];

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
s5 = surf(x,y,average,color2);
hold on;
s6 = surf(x,y,median, color3);
legend([s4,s5,s6],{'SumSquaredErrors','Average','Median'});

saveas(f2,strcat(name,'_multiple.png'));
view([0,1]);
saveas(f2,strcat(name,'_multiple_side.png'));
