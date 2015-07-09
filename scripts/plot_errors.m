clear

weights = csvread('weights.txt');
sparse_points = csvread('sparse_points.txt');
dense_points = csvread('dense_points.txt');

dim = 3;
mean = [0 0 0];
covariance = zeros(dim);
residuals = zeros(size(sparse_points,1)*size(dense_points,1),dim);
k = 1;
%weights_vector = zeros(size(sparse_points,1)*size(dense_points,1),1);
for i = 1:size(sparse_points,1)
    for j = 1:size(dense_points,1)
        residual = sparse_points(i,:) - dense_points(j,:);
        residuals(k,:) = residual;
        %weights_vector(k,:) = weights(i,j);
        k = k + 1;
        mean = mean + weights(i,j)*residual;
        covariance = covariance + weights(i,j)* (residual*residual');
    end
end
weights_sum = sum(sum(weights));
mean = mean./weights_sum;
covariance = covariance ./ weights_sum;
chol_factor = chol(covariance, 'lower');

errors = zeros(size(sparse_points,1)*size(dense_points,1),1); %preallocate for speed
k = 1;
for i = 1:size(sparse_points,1)
    row_sum = 0;
    error = 0;
    for j = 1:size(dense_points,1)
        residual = sparse_points(i,:) - dense_points(j,:);
        residual = chol_factor \ (residual - mean)';
        error = error + weights(i,j)*norm(residual,2);
        row_sum = row_sum + weights(i,j);
    end
    if row_sum ~= 0
        errors(k) = error/row_sum;
        k = k+1;
    end
end
last_element = find(errors,1,'last');
errors = errors(1:last_element); %removes trailing zeros
histogram(errors./dim);