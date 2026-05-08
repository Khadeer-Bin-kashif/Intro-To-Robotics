function take_labeled_screenshot()
    %% 1. Hardware Initialization & Acquisition
    disp('Initializing Camera...');
    pipe = realsense.pipeline();
    profile = pipe.start();
    
    dev = profile.get_device();  
    depth_sensor = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();
    depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    depth_intrinsics = depth_stream.get_intrinsics();
    
    % Wait for auto-exposure to settle
    disp('Capturing frame...');
    for i = 1:30
        fs = pipe.wait_for_frames();
    end
    
    % Align depth to color frame so pixels match up perfectly
    align_to_color = realsense.align(realsense.stream.color);
    fs = align_to_color.process(fs);
    pipe.stop();
    disp('Camera stopped. Processing image...');

    %% 2. Data Extraction
    depth = fs.get_depth_frame();
    color = fs.get_color_frame();
    
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data',[depth.get_width(), depth.get_height()]),[2 1]);
    
    color_data = color.get_data();
    I = permute(reshape(color_data',[3, color.get_width(), color.get_height()]),[3 2 1]);

    %% 3. 2D HSV Color Segmentation (BLUE)
    hsvI = rgb2hsv(I);
    H = hsvI(:,:,1); 
    S = hsvI(:,:,2); 
    V = hsvI(:,:,3);
    hueMask = (H > 0.53) & (H < 0.75);
    level = graythresh(S); 
    mask = hueMask & (S > level) & (V > 0.15);
    
    mask = bwareaopen(mask, 100); 
    mask = imfill(mask, 'holes');

    %% 4. 3D Reconstruction & Masking
    intrinsics = cameraIntrinsics([depth_intrinsics.fx, depth_intrinsics.fy], ...
                                  [depth_intrinsics.ppx, depth_intrinsics.ppy], size(depth_frame));
    ptCloud = pcfromdepth(depth_frame, 1/depth_scaling, intrinsics, 'ColorImage', I);
    roi_indices = find(mask);
    
    % Setup the figure for the screenshot
    figure('Name', 'Camera Coordinate Screenshot', 'Position', [100, 100, 800, 600]);
    imshow(I);
    hold on;
    title('Raw Camera Coordinates (X, Y, Z)', 'FontSize', 14);
    
    %% --- NEW: Draw Camera Coordinate Axes in the Top-Left Corner ---
    % Define origin for the axes (pixels) and line length
    origin_x = 50; 
    origin_y = 50; 
    ax_len = 60;
    
    % X-Axis: Right (Red)
    quiver(origin_x, origin_y, ax_len, 0, 'Color', 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
    text(origin_x + ax_len + 10, origin_y, 'X', 'Color', 'r', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Y-Axis: Down (Green)
    quiver(origin_x, origin_y, 0, ax_len, 'Color', 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
    text(origin_x, origin_y + ax_len + 20, 'Y', 'Color', 'g', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Z-Axis: Forward/Into Screen (Blue) - Drawn diagonally to simulate 3D depth
    z_dx = -ax_len * 0.6;
    z_dy = ax_len * 0.6;
    quiver(origin_x, origin_y, z_dx, z_dy, 'Color', 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
    text(origin_x + z_dx - 20, origin_y + z_dy + 15, 'Z (in)', 'Color', 'b', 'FontSize', 14, 'FontWeight', 'bold');
    % ----------------------------------------------------------------
    
    if isempty(roi_indices)
        disp('No Blue objects detected in the screenshot.');
        text(origin_x, origin_y + ax_len + 60, 'No blue objects detected.', 'Color', 'red', 'FontSize', 16, 'BackgroundColor', 'black');
        hold off;
        return;
    end
    
    cubeCloud = select(ptCloud, roi_indices);

    %% 5. Find object and Label the 2D Image
    [labels, numClusters] = pcsegdist(cubeCloud, 0.03); 
    
    for k = 1:numClusters
        clusterIndices = find(labels == k);
        oneCube = select(cubeCloud, clusterIndices);
        
        % Calculate 3D Position in the RAW Camera Frame
        pos_cam = mean(oneCube.Location, 1);
        
        % Map the 3D Camera Coordinate back to a 2D pixel for plotting the text
        u_proj = (pos_cam(1) / pos_cam(3)) * depth_intrinsics.fx + depth_intrinsics.ppx;
        v_proj = (pos_cam(2) / pos_cam(3)) * depth_intrinsics.fy + depth_intrinsics.ppy;
        
        % Draw a red star at the center of the object
        plot(u_proj, v_proj, 'r*', 'MarkerSize', 12, 'LineWidth', 2);
        
        % Create the label text showing what the camera thinks the XYZ is
        label_text = sprintf('Cam X: %.3f m\nCam Y: %.3f m\nCam Z: %.3f m', ...
                             pos_cam(1), pos_cam(2), pos_cam(3));
        
        % Draw the text box slightly offset from the star
        text(u_proj + 20, v_proj, label_text, ...
            'Color', 'white', ...
            'FontSize', 11, ...
            'FontWeight', 'bold', ...
            'BackgroundColor', [0 0 0 0.7], ... % Semi-transparent black background
            'EdgeColor', 'yellow', ...
            'Margin', 3);
            
        fprintf('Labeled Object %d at Camera XYZ: [%.3f, %.3f, %.3f]\n', k, pos_cam(1), pos_cam(2), pos_cam(3));
    end
    
    hold off;
    disp('Screenshot complete!');
end