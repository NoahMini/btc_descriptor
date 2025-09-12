% Script for obtaining the required results for IROS'18

base_dir = '/home/noah/tfm/src/btc_descriptor/evaluation/';
test_dir = '1_05_128/';
kitti0_dir = 'KITTI00/';
kitti5_dir = 'KITTI05/';
gt_neigh = 40;
compensate = false;

% Configuring subpaths
addpath('AcademicFigures/');

% Obtaining CityCenter results
curr_dir = strcat(base_dir, test_dir, kitti0_dir);
filename = 'loops1.txt';
loops_filename = strcat(curr_dir, filename);
loops_file1 = load(loops_filename);

filename = 'loops2.txt';
loops_filename = strcat(curr_dir, filename);
loops_file2 = load(loops_filename);

filename = 'loops3.txt';
loops_filename = strcat(curr_dir, filename);
loops_file3 = load(loops_filename);

filename = 'loops4.txt';
loops_filename = strcat(curr_dir, filename);
loops_file4 = load(loops_filename);

filename = 'loops5.txt';
loops_filename = strcat(curr_dir, filename);
loops_file5 = load(loops_filename);

filename = 'out1.txt';
loops_filename = strcat(curr_dir, filename);
out1 = load(loops_filename);
t_out1 = [];
for i=1:size(out1, 1)
    if out1(i,2)==1
        t_out1(i)=out1(i,3);
    end
end

filename = 'out2.txt';
loops_filename2 = strcat(curr_dir, filename);
out2 = load(loops_filename2);
t_out2 = [];
for i = 1:size(out2, 1)
    if out2(i, 2) == 1
        t_out2(i) = out2(i, 3);
    end
end

filename = 'out3.txt';
loops_filename3 = strcat(curr_dir, filename);
out3 = load(loops_filename3);
t_out3 = [];
for i = 1:size(out3, 1)
    if out3(i, 2) == 1
        t_out3(i) = out3(i, 3);
    end
end

filename = 'out4.txt';
loops_filename4 = strcat(curr_dir, filename);
out4 = load(loops_filename4);
t_out4 = [];
for i = 1:size(out4, 1)
    if out4(i, 2) == 1
        t_out4(i) = out4(i, 3);
    end
end

filename = 'out5.txt';
loops_filename5 = strcat(curr_dir, filename);
out5 = load(loops_filename5);
t_out5 = [];
for i = 1:size(out5, 1)
    if out5(i, 2) == 1
        t_out5(i) = out5(i, 3);
    end
end

% Process the loaded loop files for analysis
all_loops = [loops_file1; loops_file2; loops_file3; loops_file4; loops_file5];
% Further processing can be added here

%av_time0 = round(mean([mean(loops_file1(:,5)),mean(loops_file2(:,5)),mean(loops_file3(:,5)),mean(loops_file4(:,5)),mean(loops_file5(:,5))]))

av_max_vocab0 = round(mean([max(loops_file1(:,6)),max(loops_file2(:,6)),max(loops_file3(:,6)),max(loops_file4(:,6)),max(loops_file5(:,6))]))

%max_av_time0 = round(max([mean(loops_file1(:,5)),mean(loops_file2(:,5)),mean(loops_file3(:,5)),mean(loops_file4(:,5)),mean(loops_file5(:,5))]))

max_max_vocab0 = round(max([max(loops_file1(:,6)),max(loops_file2(:,6)),max(loops_file3(:,6)),max(loops_file4(:,6)),max(loops_file5(:,6))]))

%max_r0 = max([max(out1(:,3)),max(out2(:,3)),max(out3(:,3)),max(out4(:,3)),max(out5(:,3))])

%av_max_r0 = mean([max(out1(:,3)),max(out2(:,3)),max(out3(:,3)),max(out4(:,3)),max(out5(:,3))])

%av_max_t_r0 = mean([max(t_out1),max(t_out2),max(t_out3),max(t_out4),max(t_out5)])

%max_t_r0 = max([max(t_out1),max(t_out2),max(t_out3),max(t_out4),max(t_out5)])

% Obtaining CityCenter results
curr_dir = strcat(base_dir, test_dir, kitti5_dir);
filename = 'loops1.txt';
loops_filename = strcat(curr_dir, filename);
loops_file1 = load(loops_filename);

filename = 'loops2.txt';
loops_filename = strcat(curr_dir, filename);
loops_file2 = load(loops_filename);

filename = 'loops3.txt';
loops_filename = strcat(curr_dir, filename);
loops_file3 = load(loops_filename);

filename = 'loops4.txt';
loops_filename = strcat(curr_dir, filename);
loops_file4 = load(loops_filename);

filename = 'loops5.txt';
loops_filename = strcat(curr_dir, filename);
loops_file5 = load(loops_filename);

filename = 'out1.txt';
loops_filename = strcat(curr_dir, filename);
out1 = load(loops_filename);
t_ou1 = [];
for i=1:size(out1, 1)
    if out1(i,2)==1
        t_out1(i)=out1(i,3);
    end
end

filename = 'out2.txt';
loops_filename2 = strcat(curr_dir, filename);
out2 = load(loops_filename2);
t_out2 = [];
for i = 1:size(out2, 1)
    if out2(i, 2) == 1
        t_out2(i) = out2(i, 3);
    end
end

filename = 'out3.txt';
loops_filename3 = strcat(curr_dir, filename);
out3 = load(loops_filename3);
t_out3 = [];
for i = 1:size(out3, 1)
    if out3(i, 2) == 1
        t_out3(i) = out3(i, 3);
    end
end

filename = 'out4.txt';
loops_filename4 = strcat(curr_dir, filename);
out4 = load(loops_filename4);
t_out4 = [];
for i = 1:size(out4, 1)
    if out4(i, 2) == 1
        t_out4(i) = out4(i, 3);
    end
end

filename = 'out5.txt';
loops_filename5 = strcat(curr_dir, filename);
out5 = load(loops_filename5);
t_out5 = [];
for i = 1:size(out5, 1)
    if out5(i, 2) == 1
        t_out5(i) = out5(i, 3);
    end
end

% Process the loaded loop files for analysis
all_loops = [loops_file1; loops_file2; loops_file3; loops_file4; loops_file5];
% Further processing can be added here

%av_time5 = round(mean([mean(loops_file1(:,5)),mean(loops_file2(:,5)),mean(loops_file3(:,5)),mean(loops_file4(:,5)),mean(loops_file5(:,5))]))

av_max_vocab5 = round(mean([max(loops_file1(:,6)),max(loops_file2(:,6)),max(loops_file3(:,6)),max(loops_file4(:,6)),max(loops_file5(:,6))]))

%max_av_time5 = round(max([mean(loops_file1(:,5)),mean(loops_file2(:,5)),mean(loops_file3(:,5)),mean(loops_file4(:,5)),mean(loops_file5(:,5))]))

max_max_vocab5 = round(max([max(loops_file1(:,6)),max(loops_file2(:,6)),max(loops_file3(:,6)),max(loops_file4(:,6)),max(loops_file5(:,6))]))

%max_r5 = max([max(out1(:,3)),max(out2(:,3)),max(out3(:,3)),max(out4(:,3)),max(out5(:,3))])

%av_max_r5 = mean([max(out1(:,3)),max(out2(:,3)),max(out3(:,3)),max(out4(:,3)),max(out5(:,3))])

%av_max_t_r5 = mean([max(t_out1),max(t_out2),max(t_out3),max(t_out4),max(t_out5)])

%max_t_r5 = max([max(t_out1),max(t_out2),max(t_out3),max(t_out4),max(t_out5)])

