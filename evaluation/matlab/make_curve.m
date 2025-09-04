% Script for obtaining the required results for IROS'18
base_dir = '/home/noah/tfm/src/btc_descriptor/evaluation/';
test_dir = '2_05_128/';
kitti_dir = 'KITTI05/';

% Configuring subpaths
addpath('AcademicFigures/');

% Obtaining CityCenter results
curr_dir = strcat(base_dir, test_dir, kitti_dir);
filename = 'out1.txt';
loops_filename = strcat(curr_dir, filename);
out1 = load(loops_filename);

filename = 'out2.txt';
loops_filename2 = strcat(curr_dir, filename);
out2 = load(loops_filename2);

filename = 'out3.txt';
loops_filename3 = strcat(curr_dir, filename);
out3 = load(loops_filename3);

filename = 'out4.txt';
loops_filename4 = strcat(curr_dir, filename);
out4 = load(loops_filename4);

filename = 'out5.txt';
loops_filename5 = strcat(curr_dir, filename);
out5 = load(loops_filename5);

% Calculate distances for out1 and append to the matrix
dist1 = [1.0];
for i=2:10000
    point = [out1(i, 2) , out1(i, 3)];
    dist = norm(point - [1,1]);
    dist1 = [dist1 ; dist];
end

best = [out1(2:end, :) , dist1];
average = out1;

% Calculate distances for out2 and append to the matrix
dist2 = [1.0];
for i=2:10000
    point = [out2(i, 2), out2(i, 3)];
    dist = norm(point - [1, 1]);
    if dist < best(i, 4)
        best(i, 2) = out2(i, 2);
        best(i, 3) = out2(i, 3);
        best(i, 4) = dist;
    end
    average(i, 2) = average(i, 2) + out2(i, 2);
    average(i, 3) = average(i, 3) + out2(i, 3);
end


% Calculate distances for out3 and append to the matrix
dist3 = [1.0];
for i=2:10000
    point = [out3(i, 2), out3(i, 3)];
    dist = norm(point - [1, 1]);
    if dist < best(i, 4)
        best(i, 2) = out3(i, 2);
        best(i, 3) = out3(i, 3);
        best(i, 4) = dist;
    end
    average(i, 2) = average(i, 2) + out3(i, 2);
    average(i, 3) = average(i, 3) + out3(i, 3);
end

% Calculate distances for out4 and append to the matrix
dist4 = [1.0];
for i=2:10000
    point = [out4(i, 2), out4(i, 3)];
    dist = norm(point - [1, 1]);
    if dist < best(i, 4)
        best(i, 2) = out4(i, 2);
        best(i, 3) = out4(i, 3);
        best(i, 4) = dist;
    end
    average(i, 2) = average(i, 2) + out4(i, 2);
    average(i, 3) = average(i, 3) + out4(i, 3);
end

% Calculate distances for out5 and append to the matrix
dist5 = [1.0];
for i=2:10000
    point = [out5(i, 2), out5(i, 3)];
    dist = norm(point - [1, 1]);
    if dist < best(i, 4)
        best(i, 2) = out5(i, 2);
        best(i, 3) = out5(i, 3);
        best(i, 4) = dist;
    end
    average(i, 2) = average(i, 2) + out5(i, 2);
    average(i, 3) = average(i, 3) + out5(i, 3);
end

[R, I] = sort(best(:,3));
P = best(:,2);    
for i=1:numel(I)
    P(i) = best(I(i),2);
end

average(:, 2) = average(:, 2) / 5;
average(:, 3) = average(:, 3) / 5;

% P/R curves
afigure;
hold on;
% plot(best(2:end,3),best(2:end,2));
plot(out1(2:end,3),out1(2:end,2), '--d', 'MarkerIndices', 10000);
plot(out2(2:end,3),out2(2:end,2), '--^', 'MarkerIndices', 10000);
plot(out3(2:end,3),out3(2:end,2), '--*', 'MarkerIndices', 10000);
plot(out4(2:end,3),out4(2:end,2), '--s', 'MarkerIndices', 10000);
plot(out5(2:end,3),out5(2:end,2), '--o', 'MarkerIndices', 10000);

plot(average(2:end,3),average(2:end,2), '-');

xlabel('Recall');
ylabel('Precision');
xlim([0, 1.02]);
ylim([0, 1.02]);
% legend('CC', 'NC', 'L6I', 'L6O', 'K00', 'K05', 'K06', 'Location', 'SouthWest');
legend('ou1', 'out2', 'out3', 'out4', 'out5', 'Average');
hold off;
print('-depsc', strcat(curr_dir, 'Mod_PR_curves'));