%% pivot_response_comparison.m
% Overlays the measured 90 deg pivot response of the robot on the Simulink
% simulation of the heading controller (Section 6.4 simulation).
%   - simulation : dashed blue line
%   - actual run : solid red line
%
% Each logged run must be a CSV with a time column and a heading column.
% Header names are matched loosely (see readRun below): the time column must
% contain "time" or be called "t" (seconds, or ms if the name contains "ms"),
% and the heading column must contain "actual", "heading" or "yaw".
%
% The Simulink model is only simulated in memory and is never saved, so the
% model file is not changed.

clear; close all; clc;

%% ---------------- User settings ----------------------------------------
scriptDir = fileparts(mfilename('fullpath'));

modelFile = fullfile(scriptDir, '..', 'Simulink stuff', 'SumoBot_PID_N20.slx');

% One entry per figure. Kp, Kd and uMax are applied to the simulation so
% that it matches the gains and duty limit used in that run.
runs = struct( ...
    'csv',   {fullfile(scriptDir, 'run66_200.csv'), fullfile(scriptDir, 'run67_250.csv')}, ...
    'title', {'90^\circ pivot response at 200 DC', '90^\circ pivot response at 250 DC'}, ...
    'Kp',    {13, 13}, ...
    'Kd',    {0.3, 0.3}, ...
    'uMax',  {200, 250}, ...
    'png',   {'pivot_response_200DC.png', 'pivot_response_250DC.png'});

stepDeg    = 90;      % commanded heading step [deg]
tEnd       = 1.5;     % x-axis limit [s]
showSetpt  = false;   % thin dotted line at the 90 deg setpoint
exportPng  = true;    % save each figure next to this script

%% ---------------- Run the simulation for each set of gains -------------
[~, mdl] = fileparts(modelFile);
load_system(modelFile);
cleanup = onCleanup(@() discardModel(mdl));         % discard, never save

for k = 1:numel(runs)
    r = runs(k);

    % The model keeps Kp and Kd as literal gains: set them for this run.
    set_param([mdl '/Kp'],  'Gain', num2str(r.Kp));
    set_param([mdl '/-Kd '], 'Gain', num2str(-r.Kd));

    in = Simulink.SimulationInput(mdl);
    in = in.setModelParameter('StopTime', num2str(tEnd));
    in = in.setVariable('theta_ref', stepDeg, 'Workspace', mdl);
    in = in.setVariable('u_max',     r.uMax,  'Workspace', mdl);
    out = sim(in);

    ys = out.get('yaw_deg');
    if isa(ys, 'timeseries')
        tSim = ys.Time;  hSim = ys.Data;
    else
        tSim = out.tout; hSim = ys;
    end

    %% ---------------- Load the measured run ----------------------------
    [tAct, hAct] = readRun(r.csv);

    %% ---------------- Plot --------------------------------------------
    fig = figure('Color', 'w', 'Position', [100 100 760 480]);
    ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

    if showSetpt
        yline(ax, stepDeg, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1, ...
            'HandleVisibility', 'off');
    end
    plot(ax, tSim, hSim, '--', 'Color', 'b', 'LineWidth', 1.8, ...
        'DisplayName', 'Simulated');
    plot(ax, tAct, hAct, '-',  'Color', 'r', 'LineWidth', 1.8, ...
        'DisplayName', 'Actual');

    xlabel(ax, 'Time [s]');
    ylabel(ax, 'Heading [degrees]');
    title(ax, r.title);
    legend(ax, 'Location', 'southeast');
    xlim(ax, [0 tEnd]);
    ylim(ax, [0 max([hAct(:); hSim(:); stepDeg]) * 1.1]);
    set(ax, 'FontSize', 11);

    if exportPng
        exportgraphics(fig, fullfile(scriptDir, r.png), 'Resolution', 300);
    end
end

%% ---------------- Local functions --------------------------------------
function discardModel(mdl)
% Close the model without saving the gain changes made for each run.
    if bdIsLoaded(mdl)
        set_param(mdl, 'Dirty', 'off');
        bdclose(mdl);
    end
end

function [t, h] = readRun(csvFile)
% Read time and measured heading from a CSV log.
    T = readtable(csvFile, 'VariableNamingRule', 'preserve');
    names = lower(T.Properties.VariableNames);

    it = find(contains(names, 'time') | strcmp(names, 't'), 1);
    ih = find(contains(names, 'actual'), 1);
    if isempty(ih), ih = find(contains(names, 'heading'), 1); end
    if isempty(ih), ih = find(contains(names, 'yaw'), 1); end
    if isempty(it) || isempty(ih)
        error('readRun:columns', ...
            'Could not find time/heading columns in %s (found: %s).', ...
            csvFile, strjoin(T.Properties.VariableNames, ', '));
    end

    t = T{:, it};
    h = T{:, ih};
    if contains(names{it}, 'ms'), t = t / 1000; end
    t = t - t(1);                                   % start at 0 s
end
