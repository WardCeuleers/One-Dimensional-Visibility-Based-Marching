clc; clear;
close all;

%% interface settings
fig_occupancy    = 0;
fig_VBD_solution = 0;
    add_path     = 0;
fig_camefrom     = 0;
    add_pivots   = 0;
fig_comparison   = 1;
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
    if strcmp(key{1}, 'imagePath')
        % Don't convert imagePath and initialFrontline to numbers
        settings.(key{1}) = value{1};
    else
        settings.(key{1}) = value;
    end
end

sp_o = [settings.pos_x, settings.pos_y]+1;
ep_o = [settings.target_x, settings.target_y]+1;

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
end
%% VBD Solution
if fig_VBD_solution
    assert(settings.savegScore, 'Save the gScore to visualize the VBD solution')
    [nx, ny] = size(VBD);
figure(2);
    set(gcf, 'Name', 'Visibility-based solver')
    clf
    imagesc(VBD)
    colormap(jet)
    
    minimum = min(min(VBD));
    maximum = max(max(VBD));
    
    piv = [settings.pos_x, settings.pos_y] + 1;
    VBD(piv(1), piv(2)) = 0;

    clim([minimum maximum])
    axis equal
    axis([1 ny 1 nx])
    hold on
    contour(VBD+1, linspace(minimum, maximum, 40), 'LineWidth', 2,...
            'EdgeColor', 'k');
    
    %plot start location
    piv = [settings.pos_x, settings.pos_y] + 1;
    plot(piv(1), piv(2),'o',...
        'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
        'MarkerSize', 15, 'LineWidth', 2)

    % Enable if you want to plot a path read from the C matrix
    % pt need not be ep_o, can be any other feasible point in
    % the grid
    if add_path
        assert(settings.saveCameFrom,'Save the cameFrom to show the path on the VBD solution')
        if true 
            pt = ep_o;
            plot(ep_o(1), ep_o(2),'o',...
                  'MarkerFaceColor','red', 'MarkerEdgeColor','black',...
                  'MarkerSize', 15, 'LineWidth', 2)
            if (ep_o(1) <= ny && ep_o(2) <= nx) 
                points = [pt];
                while true
                    if pt(1) == sp_o(1) && pt(2) == sp_o(2)
                        break
                    end
                    pt_idx = VBD_cameFrom(nx-points(end,2)+1, points(end,1));
                    pt = [rem(pt_idx,ny)+1, nx-floor(pt_idx/ny)];
                    points(end+1, :) = pt;
                    
                    line([points(end,1),points(end-1,1)],...
                        [points(end,2),points(end-1,2)], ...
                        [VBD(points(end,2), points(end,1))+10 ...
                         VBD(points(end-1,2), points(end-1,1))+10],...
                        'LineWidth', 4, "color", "magenta")
                end
            end
        end
    end
    % Create a gray mask for the obstacle positions
    mask = repmat(OccupancyField, [1, 1, 3]);
    mask(:, :, 1) = mask(:, :, 1) * 0.5;
    mask(:, :, 2) = mask(:, :, 2) * 0.5;
    mask(:, :, 3) = mask(:, :, 3) * 0.5;
    % Overlay the black mask on top of the grid
    h = image(mask);
    set(h, 'AlphaData', OccupancyField);

    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca, 'ztick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca, 'YDir','normal')
end
%% Visualize CameFrom Regions
if fig_camefrom 
    assert(settings.saveCameFrom, 'construct of camefrom regions requires saveOccupancyField')
    assert(settings.saveOccupancyField, 'construction of camefrom regions requires saveOccupancyField')
    assert(settings.savePivots, 'construction of camefrom regions requires savePivots')
    [nx, ny] = size(VBD_cameFrom);
figure(3)
    hold on
    gridSize = size(OccupancyField, 1);
    numColors = size(VBD_pivots, 1)+1;
    % Define the range of values for each color
    colorRanges = linspace(1, numColors+1, numColors+1);
   
    % create the color_nb dictionary
    null_idx = ny*(nx+1);
    pivot_color_nb = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
    sp_o_idx = (sp_o(1)-1) + ny*(nx-sp_o(2));
    pivot_color_nb(sp_o_idx) = 1;
    pivot_color_nb(null_idx) = 1;
    for i = 1:size(VBD_pivots, 1)
        pivot_idx = (VBD_pivots(i,1)-1) + ny*(nx-VBD_pivots(i,2));
        pivot_color_nb(pivot_idx) = i+1;
    end
    % Create the colormap
    colorIndices = randperm(numColors);
    colors = jet(numColors);
    colors = colors(colorIndices, :);

    % Display the grid with obstacles
    Cmatrix = flip(VBD_cameFrom);
    for i = 1:size(Cmatrix, 1)
        for j = 1:size(Cmatrix, 2)
            Cmatrix(i, j) = pivot_color_nb(Cmatrix(i, j));
        end
    end
    imagesc(Cmatrix);
    % mesh(VBD_cameFrom,'FaceLighting','phong','FaceColor','interp',...
    % 'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
    %view(0,90)
    % set(gca, 'YDir', 'reverse');
    % mesh(VBD_cameFrom);
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

    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca, 'ztick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca, 'YDir','normal')

    axis equal
    axis([1 ny 1 nx])
end
%% average accuragy
if fig_comparison
    assert(settings.vbm, 'save the VBM result for comparison')
    assert(settings.savegScore, 'Save the gScores for the VBM comparison')
    vb_diff = VBD-VBM;
    [nx, ny] = size(vb_diff);
figure(4);
    clf
    hold on;
    set(gcf, 'Name', 'visibility-base methods comperaison')
    imagesc(vb_diff)
    colormap(jet)
    
    minimum = min(min(vb_diff));
    maximum = max(max(vb_diff));
    clim([minimum maximum])

    piv = [settings.pos_x, settings.pos_y] + 1;
    %plot start location
    plot(piv(1), piv(2),'o',...
        'MarkerFaceColor','green', 'MarkerEdgeColor','black',...
        'MarkerSize', 15, 'LineWidth', 2)
    % Create a gray mask for the obstacle positions
    mask = repmat(OccupancyField, [1, 1, 3]);
    mask(:, :, 1) = mask(:, :, 1) * 0.5;
    mask(:, :, 2) = mask(:, :, 2) * 0.5;
    mask(:, :, 3) = mask(:, :, 3) * 0.5;
    % Overlay the gray mask on top of the grid
    h = image(mask);
    set(h, 'AlphaData', OccupancyField);
    colorbar
    axis equal
    axis([1 ny 1 nx])
    grid off
    set(gca, 'xtick', [-1e6 1e6]);
    set(gca, 'ytick', [-1e6 1e6]);
    set(gca, 'ztick', [-1e6 1e6]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca, 'YDir','normal')
end
 %% MSFM

% if settings.msfm && settings.saveResults && settings.saveOccupancyField
%     addpath(genpath("fast_marching"))
%     % Load a maze image
%     I1 = OccupancyField;
%     [nx, ny] = size(I1);
%     % Convert the image to a speed map
%     SpeedImage = I1;
%     % I1; % imcomplement(I1)*2;
%     % Set the source to end of the maze
%     SourcePoint = [sp_o(2); sp_o(1)];
%     % SourcePoint = [582; 618];
%     % Calculate the distance map (distance to source)
%     tic
%     DistanceMap = msfm2d(SpeedImage, SourcePoint, true, true); 
%     toc
%     
%     max_lim = 1000;
%     DistanceMap(DistanceMap>max_lim) = 0;
%     
%     % Show the distance map
%     % figure, imshow(DistanceMap,[0 3400])
%     figure(10)
%     clf
%     set(gcf, 'Name', 'MSFM solution')
%     mesh(DistanceMap,'FaceLighting','phong','FaceColor','interp',...
%         'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
%     colormap(jet)
%     % caxis([0 800])
%     view(0,90)
%     axis equal
%     axis([1 ny 1 nx])
%     hold on
%     
%     DistanceMap(isinf(DistanceMap)) = 0;
%     minimum = min(min(DistanceMap));
%     maximum = max_lim; % 800; % max(max(DistanceMap));
%     contour3(DistanceMap, linspace(minimum, maximum, 60), 'LineWidth', 2,...
%             'EdgeColor', 'k');
%     
%     grid off
%     set(gca, 'xtick', [-1e6 1e6]);
%     set(gca, 'ytick', [-1e6 1e6]);
%     set(gca,'LooseInset',get(gca,'TightInset'));
% end