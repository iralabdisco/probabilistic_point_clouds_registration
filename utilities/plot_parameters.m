icp = csvread('filt_test_icp.txt');
x = icp(:,1);
icp = icp(:,2);

gicp = csvread('filt_test_gicp.txt');
gicp = gicp(:,2);

prob = csvread('filt_test_prob.txt');

p1 = plot(x, icp, 'r.','MarkerSize',12);
hold on;
p2 = plot(x, gicp, 'm.','MarkerSize',12);
hold on;
p3 = plot(x, prob, 'b.','MarkerSize',12);

legend([p1,p2,p3],{'ICP' , 'GICP' ,'Probabilistic'});