M = csvread('map.txt');
P = csvread('robotMovement_2.csv');
%P = P(1:50, :);
M = [M;P];
x = [M(:,1) M(:,3)];
y = [M(:,2) M(:,4)];
plot(x',y');