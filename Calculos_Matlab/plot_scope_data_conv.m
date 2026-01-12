function plot_scope_data_conv(fileName, labelInfo, axisLimits, fileOutputName)
% PLOT_SCOPE_DATA Imports CSV data and creates a dual-axis plot.
%
%   Usage:
%       plot_scope_data(fileName, labelInfo, axisLimits)
%
%   Inputs:
%       fileName   - String. Name of the CSV file (e.g., 'scope_38.csv').
%       labelInfo  - Cell array or String array of length 4.
%                    Format: {'X_Axis_Name', 'Y1_Axis_Name', 'Y2_Axis_Name', 'Title'}
%       axisLimits - Numerical array of length 6.
%                    Format: [x_min, x_max, y1_min, y1_max, y2_min, y2_max]
%                    Use NaN for any limit to set it to
%                    'auto'
%       fileOutputName   - String. Name of output figure.
%
%   Example:
%       labels = {'Time', 'Input Voltage', 'Output Voltage', 'Test Results'};
%       limits = [NaN NaN -10 10 0 30]; % Auto X-axis, fixed Y-axes
%       plot_scope_data('scope_38.csv', labels, limits);

    % ---------------------------------------------------------
    % 1. Load Data & Units
    % ---------------------------------------------------------
    
    % Read numeric data, skipping 2 header rows
    data = readmatrix(fileName, 'NumHeaderLines', 2);
    
    time = data(:, 1);
    signal1 = data(:, 2);
    signal2 = data(:, 3);

    % ---------------------------------------------------------
    % 2. Parse Inputs
    % ---------------------------------------------------------
    
    % Map User Labels
    xName  = labelInfo{1};
    y1Name = labelInfo{2};
    y2Name = labelInfo{3};
    plotTitle = labelInfo{4};
    plotSubTitle = labelInfo{5};


    % ---------------------------------------------------------
    % 3. Generate Plot
    % ---------------------------------------------------------
    figure('Name', plotTitle, 'Color', 'w');
    
    % --- Primary Y-Axis (Left) ---
    yyaxis left
    plot(time, signal1, 'LineStyle', '-', 'LineWidth', 1.5);
    ylabel(y1Name);
    grid on;
    
    % Apply Y1 Limits (indices 3 and 4)
    if ~isnan(axisLimits(3)) && ~isnan(axisLimits(4))
        ylim([axisLimits(3) axisLimits(4)]);
    elseif ~isnan(axisLimits(3))
        ylim([axisLimits(3) max(signal1)]); % Partial limit
    elseif ~isnan(axisLimits(4))
        ylim([min(signal1) axisLimits(4)]);
    end

    % --- Secondary Y-Axis (Right) ---
    yyaxis right
    plot(time, signal2, 'LineStyle', '-', 'LineWidth', 1.5);
    ylabel(y2Name);
    
    % Apply Y2 Limits (indices 5 and 6)
    if ~isnan(axisLimits(5)) && ~isnan(axisLimits(6))
        ylim([axisLimits(5) axisLimits(6)]);
    end

    % --- Common X-Axis and Formatting ---
    xlabel(xName);
    title(plotTitle,'FontSize',14);
    subtitle(plotSubTitle,'FontSize',8);

    % Apply X Limits (indices 1 and 2)
    if ~isnan(axisLimits(1)) && ~isnan(axisLimits(2))
        xlim([axisLimits(1) axisLimits(2)]);
    end

    % Legend
    legend({y1Name, y2Name}, 'Location', 'best');

    % Save plot
    exportgraphics(gcf,fileOutputName);
end