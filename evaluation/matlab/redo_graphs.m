% Script for obtaining the required results for IROS'18
base_dir = '/home/noah/tfm/src/btc_descriptor/evaluation/';
test_dir = 'side_lengths';
kitti_dir = 'KITTI05';

% Configuring subpaths
addpath('AcademicFigures/');

% Obtaining CityCenter results
curr_dir = strcat(base_dir, test_dir,'/', kitti_dir, '/');
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

% Obtaining CityCenter results
curr_dir = strcat(base_dir, test_dir,'/', kitti_dir, '/');
filename = 'mod_out1.txt';
loops_filename = strcat(curr_dir, filename);
mod_out1 = load(loops_filename);

filename = 'mod_out2.txt';
loops_filename2 = strcat(curr_dir, filename);
mod_out2 = load(loops_filename2);

filename = 'mod_out3.txt';
loops_filename3 = strcat(curr_dir, filename);
mod_out3 = load(loops_filename3);

filename = 'mod_out4.txt';
loops_filename4 = strcat(curr_dir, filename);
mod_out4 = load(loops_filename4);

filename = 'mod_out5.txt';
loops_filename5 = strcat(curr_dir, filename);
mod_out5 = load(loops_filename5);

% P/R curves
afigure;
hold on;
plot(mod_out1(:,3), mod_out1(:,2), '-d', 'MarkerIndices', length(mod_out1(:,1)));
plot(mod_out2(:,3), mod_out2(:,2), '--^', 'MarkerIndices', length(mod_out2(:,1)));
plot(mod_out3(:,3), mod_out3(:,2), '-*', 'MarkerIndices', length(mod_out3(:,1)));
plot(mod_out4(:,3), mod_out4(:,2), '--s', 'MarkerIndices', length(mod_out4(:,1)));
plot(mod_out5(:,3), mod_out5(:,2), '-o', 'MarkerIndices', length(mod_out5(:,1)));

xlabel('Recall');
ylabel('Precision');
xlim([0, 1.02]);
ylim([0, 1.02]);
% legend('CC', 'NC', 'L6I', 'L6O', 'K00', 'K05', 'K06', 'Location', 'SouthWest');
legend('Exec 1', 'Exec 2', 'Exec 3', 'Exec 4', 'Exec 5');
hold off;
print('-depsc', strcat(curr_dir, 'modPR_curves'));

% P/R curves
afigure;
hold on;
plot(out1(2:end,3), out1(2:end,2), '-d', 'MarkerIndices', length(out1(:,1))-1);
plot(out2(2:end,3), out2(2:end,2), '--^', 'MarkerIndices', length(out2(:,1))-1);
plot(out3(2:end,3), out3(2:end,2), '-*', 'MarkerIndices', length(out3(:,1))-1);
plot(out4(2:end,3), out4(2:end,2), '--s', 'MarkerIndices', length(out4(:,1))-1);
plot(out5(2:end,3), out5(2:end,2), '-o', 'MarkerIndices', length(out5(:,1))-1);

xlabel('Recall');
ylabel('Precision');
xlim([0, 1.02]);
ylim([0, 1.02]);
% legend('CC', 'NC', 'L6I', 'L6O', 'K00', 'K05', 'K06', 'Location', 'SouthWest');
legend('Exec 1', 'Exec 2', 'Exec 3', 'Exec 4', 'Exec 5');
hold off;
print('-depsc', strcat(curr_dir, 'PR_curves'));