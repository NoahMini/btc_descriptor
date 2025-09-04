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


interval = 0.05;
joint = [0 1 1 1];
for i=1:(1/interval)-1
    minval = interval*i-interval/2;
    maxval = interval*i+interval/2;
    filteredResults = []; % Initialize filtered results for each loop
    for n=1:length(out1)
        if out1(n, 3) >= minval && out1(n, 3) < maxval
            % Store the filtered results in a new matrix
            filteredResults = [filteredResults; out1(n, 2)];
        end
    end
    for n=1:length(out2)
        if out2(n, 3) >= minval && out2(n, 3) < maxval
            % Store the filtered results in a new matrix
            filteredResults = [filteredResults; out2(n, 2)];
        end
    end
    for n=1:length(out3)
        if out3(n, 3) >= minval && out3(n, 3) < maxval
            % Store the filtered results in a new matrix
            filteredResults = [filteredResults; out3(n, 2)];
        end
    end
    for n=1:length(out4)
        if out4(n, 3) >= minval && out4(n, 3) < maxval
            % Store the filtered results in a new matrix
            filteredResults = [filteredResults; out4(n, 2)];
        end
    end
    for n=1:length(out5)
        if out5(n, 3) >= minval && out5(n, 3) < maxval
            % Store the filtered results in a new matrix
            filteredResults = [filteredResults; out5(n, 2)];
        end
    end
    pre = [min(filteredResults) max(filteredResults) mean(filteredResults)]
    if isnan(pre)
        break
    end

    pre = [interval*i, pre];
    joint = [joint; pre];
end

% Filter values
for i=2:length(joint)
    joint(i,2) = min(joint(i-1,2),joint(i,2));
    joint(i,3) = min(joint(i-1,3),joint(i,3));
    joint(i,4) = min(joint(i-1,4),joint(i,4));
end

afigure;
hold on;
% xaxis = [0; 0.05; 0.1; 0.15; 0.2; 0.25; 0.3; 0.35; 0.4; 0.45; 0.5; 0.55; 0.6];
xaxis = joint(:,1);
plot(xaxis, joint(:,2));
plot(xaxis, joint(:,3));
plot(xaxis, joint(:,4));

% x2 = [xaxis; flipud(xaxis)];
% inBetween = [joint(:,2); flipud(joint(:,3))];
% fill(x2, inBetween, 'g');
% x=0:20; y=x.^.5; ci1=0.8*y; ci2=1.2*y;
figure(1); clf; hold on
fill([xaxis; flipud(xaxis)], [joint(:,2); flipud(joint(:,3))], [0.1 0.8 0.8], EdgeColor='none')
plot(xaxis,joint(:,4), 'k', 'LineWidth', 2)
plot(xaxis,[joint(:,2),joint(:,3)], 'k--', 'LineWidth', 2)


xlabel('Recall');
ylabel('Precision');
xlim([0, 1.02]);
ylim([0, 1.02]);
legend('', 'Average');
hold off;
print('-depsc', strcat(curr_dir, test_dir,'_', kitti_dir,'_joint_curves_005'));

% % P/R curves
% afigure;
% hold on;
% plot(out1(:,3), out1(:,2), '-d', 'MarkerIndices', length(out1(:,2)));
% plot(out2(:,3), out2(:,2), '--^', 'MarkerIndices', length(out2(:,2)));
% plot(out3(:,3), out3(:,2), '-*', 'MarkerIndices', length(out3(:,2)));
% plot(out4(:,3), out4(:,2), '--s', 'MarkerIndices', length(out4(:,2)));
% plot(out5(:,3), out5(:,2), '-o', 'MarkerIndices', length(out5(:,2)));
% 
% xlabel('Recall');
% ylabel('Precision');
% xlim([0, 1.02]);
% ylim([0, 1.02]);
% % legend('CC', 'NC', 'L6I', 'L6O', 'K00', 'K05', 'K06', 'Location', 'SouthWest');
% legend('Exec 1', 'Exec 2', 'Exec 3', 'Exec 4', 'Exec 5');
% hold off;
% print('-depsc', strcat(curr_dir, 'mod_PR_curves'));

au_max = sum(joint(:,3))*0.05;

au_av = sum(joint(:,4))*0.05;