clc; clear;
close all;

exp_fig = 1;
%% interface settings
region_areas      = 0;
fig_occupancy     = 0;
fig_origin_polygon= 0;
    add_distance =  0;
fig_VBD_solution  = 0;
    add_path      = 0;
fig_camefrom      = 0;
    add_pivots    = 0;
fig_sol_comp      = 1;
fig_camefrom_vbm  = 0;
%% Parser - parse settings
% Open the file for reading
fid = fopen('config/settings.config', 'r');

% Define the format string for textscan
formatSpec = '%s';

% Read the settings using textscan
settingsCell = textscan(fid, formatSpec, 'Delimiter', '=', 'CommentStyle', '#');

% Close the file
fclose(fid);

% Convert the cell array to a struct
settings = struct();
for i = 1:2:length(settingsCell{1})
    key = settingsCell{1}(i);
    value = settingsCell{1}(i+1);
    % Check if the value is a number
    if ~isnan(str2double(value{1}))
        % Convert the value to a number
        value = str2double(value{1});
    end
    % Save the key-value pair in the settings struct
    if strcmp(key{1}, 'imagePath') || strcmp(key{1}, 'initialFrontline')
        % Don't convert imagePath and initialFrontline to numbers
        settings.(key{1}) = value{1};
    else
        settings.(key{1}) = value;
    end
    if strcmp(key{1}, 'initialFrontline')
        settings.(key{1}) = eval(strrep(strrep(value{1}, '{', '['), '}', ']'));
    end
end

IF_length = length(settings.initialFrontline);
nb_sources = IF_length/2;
sp_x = settings.initialFrontline(1:2:IF_length-1)+1;
sp_y = settings.initialFrontline(2:2:IF_length)+1; 
ep_o = [settings.target_x, settings.target_y]+1;

%% create output dir for exp fig
outputDir = "expfig/";
if exp_fig
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end
end
%% copy saved results
assert(settings.saveResults, 'The matlab interface requires the results to be saved');
if settings.saveOccupancyField
    filename_OccupancyField = "output/occupancyField.txt";
    T_OccupancyField = readtable(filename_OccupancyField,'Delimiter',' ');
    OccupancyField = T_OccupancyField.Variables;
end
if settings.savegScore
    filename_VBD = "output/VBD_gScore.txt";
    T_VBD = readtable(filename_VBD,'Delimiter',' ');
    VBD = T_VBD.Variables;
    VBD(isinf(VBD)) = 0;
end
if settings.saveCameFrom
    filename_VBD_cameFrom = "output/VBD_cameFrom.txt";
    T_VBD_cameFrom = readtable(filename_VBD_cameFrom,'Delimiter',' ');
    VBD_cameFrom = T_VBD_cameFrom.Variables;
end
if settings.savePivots
    filename_VBD_pivots = "output/VBD_pivots.txt";
    T_VBD_pivots = readtable(filename_VBD_pivots, 'Delimiter',' ');
    VBD_pivots = T_VBD_pivots.Variables;
    VBD_pivots = VBD_pivots + 1;
end
if settings.savegScore && settings.vbm
    filename_VBM = "output/VBM_gScore.txt";
    T_VBM = readtable(filename_VBM,'Delimiter',' ');
    VBM = T_VBM.Variables;
    VBM(isinf(VBM)) = 0;
end

%% Occupancy grid

if fig_occupancy
    assert(settings.saveOccupancyField, 'Construction of occupancy grid requires saveOccupancyField ')
    [nx, ny] = size(OccupancyField);

figure(1)
    set(gcf, 'Name', 'Visibility field (occupancy grid)')
    clf
    mesh(OccupancyField,'FaceLighting','phong','FaceColor','interp',...
        'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
    colormap(flipud(gray))
    view(0,90)
    axis equal
    axis([1 ny 1 nx])
    hold on

    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    if exp_fig
        export_fig(fullfile(outputDir, 'occupancy_grid'), '-r250', '-transparent', '-png', '-painters');
    end
end
if region_areas
    assert(settings.saveOccupancyField, 'Save the occupancy field for calculations of region areas')
    assert(settings.saveCameFrom, 'Save the camefrom field for calculation of region areas')
    [nx, ny] = size(OccupancyField);
    null_idx = ny*(nx+1);
    size = nx*ny;
    occ_count = 0;
    base_count = 0;
    for i = 1:nx
        for j = 1:ny
            occ_count = occ_count + OccupancyField(i,j);
            if VBD_cameFrom(i,j) == null_idx
                base_count = base_count + 1;
            end
        end
    end
    occupancy_percentage = occ_count/size*100
    origin_percentage = base_count/size*100
end

%% VBD_origin_solution

if fig_origin_polygon
    assert(settings.savegScore, 'Save the gScore to visualize the origin polygon');
    assert(settings.saveCameFrom, 'Save the camefrom map for visualizing the origin polygon');
    assert(settings.saveOccupancyField, 'Save the occupancy field to visualize the orign polygon');
    [nx,ny] = size(VBD);
    sp_idx = (settings.initialFrontline(1)) + ny*(nx-1-settings.initialFrontline(2));
    origin = zeros(nx,ny);
    unvisited = ones(nx,ny);
    for i = 1:nx
        for j =1:ny
            if VBD_cameFrom(nx+1-i,j) == sp_idx
                origin(i,j)= VBD(i,j);
                unvisited(i,j) = 0;
            else
                origin(i,j) = 0;
            end
        end
    end
figure(2)
    clf
    hold on

    if add_distance
        imagesc(origin)
        colormap(jet)
        
        minimum = min(min(origin));
        maximum = max(max(origin));
    
        clim([minimum maximum])
        contour(origin+1, linspace(minimum+0.0001, maximum, 30), 'LineWidth', 2 ,'EdgeColor', 'k');

        % Add a colorbar
        cb = colorbar;
        cb.FontWeight = 'bold';
        cb.FontSize = 14;
        cb.Position = [0.89 0.02 0.02 0.96];
    end
    axis equal
    axis([1 ny 1 nx])
    plot(settings.initialFrontline(1)+1, settings.initialFrontline(2)+1,'o',...
    'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
    'MarkerSize', 15, 'LineWidth', 2)
    % Create a black mask for the nodes that did not belong to the start
    mask = repmat(unvisited, [1, 1, 3]);
    mask(:, :, 1) = mask(:, :, 1) * 0;
    mask(:, :, 2) = mask(:, :, 2) * 0;
    mask(:, :, 3) = mask(:, :, 3) * 0;
    % Overlay the black mask on top of the grid
    h = image(mask);
    set(h, 'AlphaData', unvisited);

    % Create a gray mask for the obstacle positions
    mask = repmat(OccupancyField, [1, 1, 3]);
    mask(:, :, 1) = mask(:, :, 1) * 0.5;
    mask(:, :, 2) = mask(:, :, 2) * 0.5;
    mask(:, :, 3) = mask(:, :, 3) * 0.5;
    % Overlay the gray mask on top of the grid
    h = image(mask);
    set(h, 'AlphaData', OccupancyField);

    if add_pivots
        assert(settings.savePivots, 'save pivots to add pivot to origin');
        for i = 1:size(VBD_pivots, 1)
            pt_ = VBD_pivots(i,:);
            if origin(pt_(2), pt_(1)) ~= 0
            plot3(pt_(1), pt_(2), 20,'o',...
                'MarkerFaceColor','white', 'MarkerEdgeColor','black',...
                'MarkerSize', 8, 'LineWidth', 2)
            end
        end
    end
    grid off
    box on
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca, 'ztick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca, 'YDir','normal')

    if exp_fig
        export_fig(fullfile(outputDir, 'base_visibility'), '-r250', '-transparent', '-png', '-painters');
    end
end

%% OD-VBM Solution

if fig_VBD_solution
    assert(settings.savegScore, 'Save the gScore to visualize the VBD solution')
    [nx, ny] = size(VBD);
figure(3);
    set(gcf, 'Name', 'Visibility-based solver')
    clf
    hold on
    imagesc(VBD)
    colormap(jet)
    
    minimum = min(min(VBD));
    maximum = max(max(VBD));

    clim([minimum maximum])
    axis equal
    axis([1 ny 1 nx])
    contour(VBD+1, linspace(minimum, maximum, 40), 'LineWidth', 2,...
    'EdgeColor', 'k');

    %plot start location
    for i = 1:2:IF_length
        piv = [settings.initialFrontline(i), settings.initialFrontline(i+1)] + 1;
        plot(piv(1), piv(2),'o',...
            'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
            'MarkerSize', 15, 'LineWidth', 2)
    end
    % Create a gray mask for the obstacle positions
    mask = repmat(OccupancyField, [1, 1, 3]);
    mask(:, :, 1) = mask(:, :, 1) * 0.5;
    mask(:, :, 2) = mask(:, :, 2) * 0.5;
    mask(:, :, 3) = mask(:, :, 3) * 0.5;
    % Overlay the black mask on top of the grid
    h = image(mask);
    set(h, 'AlphaData', OccupancyField);

    % Enable if you want to plot a path read from the C matrix
    % pt need not be ep_o, can be any other feasible point in
    % the grid
    if add_path
        assert(settings.saveCameFrom,'Save the cameFrom to show the path on the VBD solution')
        if true 
            pt = ep_o;
            plot(ep_o(1), ep_o(2),'o',...
                  'MarkerFaceColor','magenta', 'MarkerEdgeColor','black',...
                  'MarkerSize', 15, 'LineWidth', 2)
            if (ep_o(1) <= ny && ep_o(2) <= nx) 
                points = [pt];
                atInitialFrontline = false;
                while ~atInitialFrontline
                    x_matches = find(sp_x==pt(1));
                    if ~isempty(x_matches)
                        for i = 1:length(x_matches)
                            if sp_y(x_matches(i)) == pt(2)
                                atInitialFrontline = true;
                                break;
                            end
                        end
                    end
                    if ~atInitialFrontline 
                        pt_idx = VBD_cameFrom(nx-points(end,2)+1, points(end,1));
                        pt = [rem(pt_idx,ny)+1, nx-floor(pt_idx/ny)];
                        points(end+1, :) = pt;
                        
                        line([points(end,1),points(end-1,1)],...
                            [points(end,2),points(end-1,2)], ...
                            'LineWidth', 4, "color", "magenta")
%                             [VBD(points(end,2), points(end,1))+10 ...
%                              VBD(points(end-1,2), points(end-1,1))+10],...
                    end
                end
            end
        end
    end
    
    % Add a colorbar
    cb = colorbar;
    cb.FontWeight = 'bold';
    cb.FontSize = 14;
    cb.Position = [0.89 0.02 0.02 0.96];

    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca, 'ztick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca, 'YDir','normal')

    if exp_fig
        export_fig(fullfile(outputDir, 'OD_VBM_sol'), '-r250', '-transparent', '-png', '-painters');
    end
end

%% Visualize CameFrom Regions

if fig_camefrom 
    assert(settings.saveCameFrom, 'construct of camefrom regions requires saveOccupancyField')
    assert(settings.saveOccupancyField, 'construction of camefrom regions requires saveOccupancyField')
    assert(settings.savePivots, 'construction of camefrom regions requires savePivots')
    [nx, ny] = size(VBD_cameFrom);
figure(4)
    hold on
    numColors = size(VBD_pivots, 1)+nb_sources;
    % Define the range of values for each color
    colorRanges = linspace(1, numColors+1, numColors+1);
   
    % create the color_nb dictionary
    pivot_color_nb = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
    null_idx = ny*(nx+1);
    pivot_color_nb(null_idx) = 1;
    for i = 1:nb_sources
        sp_idx = (sp_x(i)-1) + ny*(nx-sp_y(i));
        pivot_color_nb(sp_idx) = i;
    end 
    for i = 1:size(VBD_pivots, 1)
        pivot_idx = (VBD_pivots(i,1)-1) + ny*(nx-VBD_pivots(i,2));
        pivot_color_nb(pivot_idx) = i+nb_sources;
    end
    % Create the colormap
    colorIndices = randperm(numColors);
    colors = jet(numColors);
    colors = colors(colorIndices, :);
    
    %plot start locations
    for i = 1:2:IF_length
        piv = [settings.initialFrontline(i), settings.initialFrontline(i+1)] + 1;
        plot(piv(1), piv(2),'o',...
            'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
            'MarkerSize', 15, 'LineWidth', 2)
    end

    % Display the grid with obstacles
    Cmatrix = flip(VBD_cameFrom);
    for i = 1:size(Cmatrix, 1)
        for j = 1:size(Cmatrix, 2)
            Cmatrix(i, j) = pivot_color_nb(Cmatrix(i, j));
        end
    end
    imagesc(Cmatrix);
    colormap(colors);
    
    % Create a gray mask for the obstacle positions
    mask = repmat(OccupancyField, [1, 1, 3]);
    mask(:, :, 1) = mask(:, :, 1) * 0.5;
    mask(:, :, 2) = mask(:, :, 2) * 0.5;
    mask(:, :, 3) = mask(:, :, 3) * 0.5;
    % Overlay the black mask on top of the grid
    h = image(mask);
    set(h, 'AlphaData', OccupancyField);
    
    %color pivots
    if (add_pivots)
        assert(settings.savePivots, 'Save the pivot list to add them on the cameFrom figure')
        for i = 1:size(VBD_pivots, 1)
            pt_ = VBD_pivots(i,:);
            plot3(pt_(1), pt_(2), numColors,'o',...
                'MarkerFaceColor','white', 'MarkerEdgeColor','black',...
                'MarkerSize', 8, 'LineWidth', 2)
        end
    end

    grid on
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca, 'ztick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca, 'YDir','normal')

    axis equal
    axis([1 ny 1 nx])

    if exp_fig
        export_fig(fullfile(outputDir, 'OD_VBM_camefrom'), '-r250', '-transparent', '-png', '-painters');
    end
end

%% accuracy

if fig_sol_comp
    assert(settings.vbm, 'save the VBM result for comparison')
    assert(settings.savegScore, 'Save the gScores for the VBM comparison')
    vb_diff = (VBD-VBM);
    squared_diff = (vb_diff).^2;
    RMSE = sqrt(mean(mean(squared_diff)))
    [nx, ny] = size(vb_diff);
figure(5);
    rel_err = vb_diff./VBM;
    rel_err(isnan(rel_err)) = 0;
    clf
    hold on;
    set(gcf, 'Name', 'visibility-base methods comperaison')
    imagesc(rel_err)
    colormap(jet)
    
    minimum = min(min(rel_err));
    maximum = max(max(rel_err));
    clim([minimum maximum])

    %plot start locations
    for i = 1:2:IF_length
        piv = [settings.initialFrontline(i), settings.initialFrontline(i+1)] + 1;
        plot(piv(1), piv(2),'o',...
            'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
            'MarkerSize', 15, 'LineWidth', 2)
    end
    % Create a gray mask for the obstacle positions
    mask = repmat(OccupancyField, [1, 1, 3]);
    mask(:, :, 1) = mask(:, :, 1) * 0.5;
    mask(:, :, 2) = mask(:, :, 2) * 0.5;
    mask(:, :, 3) = mask(:, :, 3) * 0.5;
    % Overlay the gray mask on top of the grid
    h = image(mask);
    set(h, 'AlphaData', OccupancyField);
    
    axis equal
    axis([1 ny 1 nx])

    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca, 'ztick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca, 'YDir','normal')

    cb = colorbar;
    cb.FontWeight = 'bold';
    cb.FontSize = 14;
    cb.Position = [0.87 0.02 0.02 0.96];
    ticks = linspace(cb.Limits(1), cb.Limits(2), 5);
    cb.Ticks = ticks;
    cb.TickLabels = arrayfun(@(x) sprintf('%.2f%%', x * 100), ticks, 'UniformOutput', false);
    currentPosition = get(gca, 'Position');
    newPosition = currentPosition;
    newPosition(1) = newPosition(1) - 0.02;  % Shift left by 0.05 units
    set(gca, 'Position', newPosition);


    if exp_fig
        export_fig(fullfile(outputDir, 'VBM_comp'), '-r250', '-transparent', '-png', '-painters');
    end
end
%% VBM camefrom
if fig_camefrom_vbm
    assert(settings.vbm, 'save the vbm results for camefrom vbm')
    assert(settings.saveCameFrom, 'construct of camefrom regions requires saveOccupancyField')
    assert(settings.saveOccupancyField, 'construction of camefrom regions requires saveOccupancyField')
    assert(settings.savePivots, 'construction of camefrom regions requires savePivots')
    if settings.saveCameFrom
        filename_VBM_cameFrom = "output/VBM_cameFrom.txt";
        T_VBM_cameFrom = readtable(filename_VBM_cameFrom,'Delimiter',' ');
        VBM_cameFrom = T_VBM_cameFrom.Variables;
    end
    if settings.savePivots
        filename_VBM_pivots = "output/VBM_pivots.txt";
        T_VBM_pivots = readtable(filename_VBM_pivots, 'Delimiter',' ');
        VBM_pivots = T_VBM_pivots.Variables;
        VBM_pivots = VBM_pivots + 1;
    end
        [nx, ny] = size(VBM_cameFrom);
figure(6)
    hold on
    gridSize = size(OccupancyField, 1);
    numColors = size(VBM_pivots, 1);
    % Define the range of values for each color
    colorRanges = linspace(1, numColors+1, numColors+1);
   
    % Create the colormap
    colorIndices = randperm(numColors);
    colors = jet(numColors);
    colors = colors(colorIndices, :);

    % Display the grid with obstacles
    imagesc(VBM_cameFrom);
    colormap(colors);
    % Create a gray mask for the obstacle positions
    mask = repmat(OccupancyField, [1, 1, 3]);
    mask(:, :, 1) = mask(:, :, 1) * 0.5;
    mask(:, :, 2) = mask(:, :, 2) * 0.5;
    mask(:, :, 3) = mask(:, :, 3) * 0.5;
    % Overlay the black mask on top of the grid
    h = image(mask);
    set(h, 'AlphaData', OccupancyField);
    
    %color pivots
    if (add_pivots)
        assert(settings.savePivots, 'Save the pivot list to add them on the cameFrom figure')
        for i = 1:size(VBM_pivots, 1)
            pt_ = VBM_pivots(i,:);
            plot3(pt_(1), pt_(2), numColors,'o',...
                'MarkerFaceColor','white', 'MarkerEdgeColor','black',...
                'MarkerSize', 8, 'LineWidth', 2)
        end
    end

    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca, 'ztick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca, 'YDir','normal')

    axis equal
    axis([1 ny 1 nx])

    if exp_fig
        export_fig(fullfile(outputDir, 'VBM_camefrom'), '-r250', '-transparent', '-png', '-painters');
    end
end