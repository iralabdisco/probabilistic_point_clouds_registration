data = csvread('debug_file.txt',1,0);

subplot(3,2,1)
plot(m(:,1),m(:,2))
ylabel('N success steps')

subplot(3,2,2)
yyaxis left
plot(m(:,1),m(:,3))
ylabel('Initial cost')
yyaxis right
plot(m(:,1),m(:,4))
ylabel('Final cost')

subplot(3,2,3)
drop = m(:,3) - m(:,4);
plot(m(:,1),drop)
ylabel('Cost drop')

subplot(3,2,4)
plot(m(:,1),m(:,5))
hold on;
plot(m(:,1),m(:,6))
hold on;
plot(m(:,1),m(:,7))
ylabel('Translation')

subplot(3,2,5)
plot(m(:,1),m(:,8))
hold on;
plot(m(:,1),m(:,9))
hold on;
plot(m(:,1),m(:,10))
hold on;
ylabel('Rotation')

subplot(3,2,6)
yyaxis left;
plot(m(:,1),m(:,11))
ylabel('MSE previous')
yyaxis right;
plot(m(:,1),m(:,12))
ylabel('MSE gtruth')