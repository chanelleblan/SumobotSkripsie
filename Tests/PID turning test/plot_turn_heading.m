runNumber = 7;

files = dir(fullfile(fileparts(mfilename('fullpath')), 'PIDturn_Run*.csv'));
names = {files.name};
idx = find(startsWith(names, sprintf('PIDturn_Run%02d_', runNumber)), 1);
data = readtable(fullfile(files(idx).folder, names{idx}));

figure
plot(data.time_s, data.desired_heading_deg, 'LineWidth', 1.5)
hold on
plot(data.time_s, data.actual_heading_deg, 'r', 'LineWidth', 1)
hold off
grid on
xlabel('time (s)')
ylabel('heading (deg)')
legend('Expected heading', 'Actual heading', 'Location', 'southeast')
title(sprintf('Run %d', runNumber))
