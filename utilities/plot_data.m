data = csvread('debug_file.txt',1,0);

subplot(3,2,1)
plot(data(:,1),data(:,2))
ylabel('N success steps')

subplot(3,2,2)
yyaxis left
plot(data(:,1),data(:,3))
ylabel('Initial cost')
yyaxis right
plot(data(:,1),data(:,4))
ylabel('Final cost')

subplot(3,2,3)
drop = data(:,3) - data(:,4);
plot(data(:,1),drop)
ylabel('Cost drop')

subplot(3,2,4)
plot(data(:,1),data(:,5))
hold on;
plot(data(:,1),data(:,6))
hold on;
plot(data(:,1),data(:,7))
ylabel('Translation')

subplot(3,2,5)
plot(data(:,1),data(:,8))
hold on;
plot(data(:,1),data(:,9))
hold on;
plot(data(:,1),data(:,10))
hold on;
ylabel('Rotation')

subplot(3,2,6)
yyaxis left;
plot(data(:,1),data(:,11))
ylabel('MSE previous')
yyaxis right;
plot(data(:,1),data(:,12))
ylabel('MSE gtruth')