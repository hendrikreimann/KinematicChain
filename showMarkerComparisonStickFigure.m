function stickFigure = showMarkerComparisonStickFigure(plant, jointAngleTrajectories, markerTrajectories, sceneBound)
    plant_visualization = plant.copy();
    
    %% create the stick figure
    if nargin < 4
        sceneBound = 1*[-0.1; 1.1; 0; 1; 0; 2];
    end
    stickFigure = KinematicTreeStickFigure(plant_visualization, sceneBound);
%     stickFigure.showStickFigure = false;
%     stickFigure.showMarkers = false;
    stickFigure.showLinkMassEllipsoids = true;
    stickFigure.update();
    view(30, 30)
    
    first_data_point = find(~isnan(jointAngleTrajectories(:, 1)), 1);
    last_data_point = find(~isnan(jointAngleTrajectories(:, 1)), 1, 'last');

    number_of_time_steps = size(jointAngleTrajectories, 1);
	number_of_markers = plant_visualization.getNumberOfMarkers;
    
    %% create the control figure
    control_figure = figure('Position', [100, 650, 500, 100]);    
    current_time_step_slider = uicontrol(control_figure, 'Style', 'Slider',...
                                                   'Callback', @timeStepSliderChanged,...
                                                   'Position', [10, 10, 450, 10]);
    set( ...
         current_time_step_slider, ...
         'Min', 1, ...
         'Max', number_of_time_steps, ...
         'Value', first_data_point, ...
         'sliderStep', [1*(number_of_time_steps-1)^(-1) number_of_time_steps^(-1)*10] ...
         );
    current_time_step_label = uicontrol(control_figure, 'Style', 'text', 'Position', [460, 5, 40, 15]);
    
    % video stuff
                          uicontrol(control_figure, 'Style', 'text', 'Position', [10, 70, 40, 18], 'string', 'Start:');
    video_start_box     = uicontrol(control_figure, 'Style', 'edit', 'Position', [50, 70, 40, 20], 'string', num2str(first_data_point));
                          uicontrol(control_figure, 'Style', 'text', 'Position', [10, 50, 40, 18], 'string', 'End:');
    video_end_box       = uicontrol(control_figure, 'Style', 'edit', 'Position', [50, 50, 40, 20], 'string', num2str(last_data_point));
                          uicontrol(control_figure, 'Style', 'text', 'Position', [10, 30, 40, 18], 'string', 'Interval:');
    video_interval_box  = uicontrol(control_figure, 'Style', 'edit', 'Position', [50, 30, 40, 20], 'string', '5');
    record_video_checkbox = uicontrol(control_figure, 'Style', 'checkbox', 'Position', [92, 70, 18, 18]);
    uicontrol(control_figure, 'Style', 'text', 'Position', [112, 70, 40, 18], 'string', 'record');
    uicontrol(control_figure, 'Style', 'pushbutton', 'Position', [90, 30, 80, 40], 'string', 'Play Video', 'Callback', @playVideo);
                                               
    uicontrol(control_figure, 'Style', 'pushbutton', 'Position', [420, 30, 80, 40], 'string', 'Quit', 'Callback', @quit);
        
    %% add plots for recorded markers
    recorded_marker_plots = zeros(1, number_of_markers);
    if ~isempty(markerTrajectories)
        for i_marker = 1 : number_of_markers
            recorded_marker_plots(i_marker) = plot3 ...
              ( ...
                stickFigure.sceneAxes, ...
                markerTrajectories(1, 3*(i_marker-1)+1), ...
                markerTrajectories(1, 3*(i_marker-1)+2), ...
                markerTrajectories(1, 3*(i_marker-1)+3), ...
                'color', stickFigure.getMarkerColor(plant.markerExportMap(1, i_marker), plant.markerExportMap(2, i_marker)), ...
                'markersize', 10, ...
                'linewidth', 2, ...
                'Marker', 'o' ...
              );
        end
    end



    showTimeStep(first_data_point);

%% set the time step for display
function showTimeStep(index, hObject)
    
    if nargin < 2
        hObject = 0;
    end
    
    theta = jointAngleTrajectories(index, :)';
    plant_visualization.jointAngles = theta;
    plant_visualization.updateConfiguration;
    stickFigure.update();
    
    % update label
    set(current_time_step_label, 'String', num2str(index));
    
    % update slider
    if hObject ~= current_time_step_slider
        set(current_time_step_slider, 'Value', index)
    end
    
    
    if ~isempty(markerTrajectories)
        for i_marker = 1 : number_of_markers
            set ...
              ( ...
                recorded_marker_plots(i_marker), ...
                'xdata', markerTrajectories(index, 3*(i_marker-1)+1), ...
                'ydata', markerTrajectories(index, 3*(i_marker-1)+2), ...
                'zdata', markerTrajectories(index, 3*(i_marker-1)+3) ...
              );
        end
    end
    drawnow;
end




%% deal with time step slider change
function timeStepSliderChanged(hObject, eventdata) %#ok<INUSD>
    % get data
    current_time_step = floor(get(current_time_step_slider(1), 'Value'));
    
    % update figure
    showTimeStep(current_time_step, hObject);
end

%% play video
function playVideo(hObject, eventdata) %#ok<INUSD>
    % get data
    start_time_step = str2double(get(video_start_box, 'string'));
    end_time_step = str2double(get(video_end_box, 'string'));
    interval = str2double(get(video_interval_box, 'string'));
    frames = start_time_step : interval : end_time_step;
    number_of_frames = length(frames);
    record_video = get(record_video_checkbox, 'value');
    
    % create recording container
    if record_video
        video_figure = stickFigure.sceneFigure;
        window_size = get(video_figure, 'Position');
        window_size(1:2) = [0 0];
        movie_container = moviein(number_of_frames, video_figure, window_size);
        set(video_figure, 'NextPlot', 'replacechildren');
    end
    
    % play movie
    for i_frame = 1 : number_of_frames
        showTimeStep(frames(i_frame));
        if record_video
            movie_container(:, i_frame) = getframe(video_figure, window_size); 
        end
    end
    
    % save
    if record_video
        mpgwrite(movie_container, jet, 'movie.mpg');
    end
    
    % TODO: display the time 
end

%% quit
function quit(hObject, eventdata) %#ok<INUSD>
    close all
end

end