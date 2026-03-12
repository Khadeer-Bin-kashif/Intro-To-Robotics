function perception_pipeline_3d_regionprops()
    %% 1. Hardware Initialization & Acquisition
    pipe = realsense.pipeline();
    profile = pipe.start();
    
    dev = profile.get_device();  
    depth_sensor = dev.first('depth_sensor');
    depth_scaling = depth_sensor.get_depth_scale();
    depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    depth_intrinsics = depth_stream.get_intrinsics();
    
    for i = 1:30
        fs = pipe.wait_for_frames();
    end
    
    align_to_color = realsense.align(realsense.stream.color);
    fs = align_to_color.process(fs);
    pipe.stop();

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
    
    % --- NEW: Get 2D Region Properties (Orientation) ---
    % We get the Centroid and Orientation for every blob in the 2D mask
    stats = regionprops(mask, 'Centroid', 'Orientation');

    %% 4. VISUALIZATION PART 1: 2D Results
    figure('Name', '2D Processing Stages');
    subplot(1,3,1); imshow(I); title('Original');
    subplot(1,3,2); imshow(mask); title('Binary Mask');
    subplot(1,3,3); imshow(bsxfun(@times, I, cast(mask, 'like', I))); title('Segmented Blue');

    %% 5. 3D Reconstruction & Masking
    intrinsics = cameraIntrinsics([depth_intrinsics.fx, depth_intrinsics.fy], ...
                                  [depth_intrinsics.ppx, depth_intrinsics.ppy], size(depth_frame));
    ptCloud = pcfromdepth(depth_frame, 1/depth_scaling, intrinsics, 'ColorImage', I);

    roi_indices = find(mask);
    if isempty(roi_indices)
        disp('No Blue objects detected.');
        return;
    end
    
    cubeCloud = select(ptCloud, roi_indices);

    %% 6. Clustering and Pose Estimation (with Orientation)
    [labels, numClusters] = pcsegdist(cubeCloud, 0.03); 
    
    figure('Name', '3D Perception Output');
    pcshow(cubeCloud, 'VerticalAxisDir', 'Down', 'MarkerSize', 40);
    title(sprintf('Final 3D Point Cloud (Found %d Clusters)', numClusters));
    hold on;
    
    for k = 1:numClusters
        clusterIndices = find(labels == k);
        oneCube = select(cubeCloud, clusterIndices);
        
        % 1. Calculate 3D Position (Centroid)
        pos = mean(oneCube.Location, 1); % [X, Y, Z] in meters
        
        % 2. Find corresponding 2D Orientation
        u_proj = (pos(1) / pos(3)) * depth_intrinsics.fx + depth_intrinsics.ppx;
        v_proj = (pos(2) / pos(3)) * depth_intrinsics.fy + depth_intrinsics.ppy;
        
        % Find the closest 2D blob to this projected pixel
        minDist = inf;
        bestIdx = -1;
        for j = 1:length(stats)
            centroid2D = stats(j).Centroid;
            dist = sqrt((centroid2D(1)-u_proj)^2 + (centroid2D(2)-v_proj)^2);
            if dist < minDist
                minDist = dist;
                bestIdx = j;
            end
        end
        
        orientationAngle = 0;
        if bestIdx ~= -1
            orientationAngle = stats(bestIdx).Orientation;
        end

        % 3. Visualization
        plot3(pos(1), pos(2), pos(3), 'r*', 'MarkerSize', 20, 'LineWidth', 2);
        
        rad = deg2rad(orientationAngle);
        lineLength = 0.05; % 5cm line
        dx = lineLength * cos(rad);
        dy = lineLength * sin(rad);
        plot3([pos(1)-dx, pos(1)+dx], [pos(2)-dy, pos(2)+dy], [pos(3), pos(3)], 'g-', 'LineWidth', 3);

        % Update Label with Angle
        text(pos(1), pos(2), pos(3)-0.05, sprintf('Cube %d\nPos: [%.2f, %.2f, %.2f]\nAngle: %.1f^{\\circ}', ...
            k, pos(1), pos(2), pos(3), orientationAngle), ...
            'Color', 'y', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', 'k');
        
        fprintf('Blue Cube %d Pose: X=%.3fm, Y=%.3fm, Z=%.3fm, Angle=%.2f deg\n', k, pos(1), pos(2), pos(3), orientationAngle);
    end
    
    view(0, -90); 
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    grid on;
    hold off;
end