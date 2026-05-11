% function perception_pipeline_3d_regionprops()
%     %% 1. Hardware Initialization & Acquisition
%     pipe = realsense.pipeline();
%     profile = pipe.start();
% 
%     dev = profile.get_device();  
%     depth_sensor = dev.first('depth_sensor');
%     depth_scaling = depth_sensor.get_depth_scale();
%     depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
%     depth_intrinsics = depth_stream.get_intrinsics();
% 
%     for i = 1:30
%         fs = pipe.wait_for_frames();
%     end
% 
%     align_to_color = realsense.align(realsense.stream.color);
%     fs = align_to_color.process(fs);
%     pipe.stop();
% 
%     %% 2. Data Extraction
%     depth = fs.get_depth_frame();
%     color = fs.get_color_frame();
% 
%     depth_data = double(depth.get_data());
%     depth_frame = permute(reshape(depth_data',[depth.get_width(), depth.get_height()]),[2 1]);
% 
%     color_data = color.get_data();
%     I = permute(reshape(color_data',[3, color.get_width(), color.get_height()]),[3 2 1]);
% 
%     %% 3. 2D HSV Color Segmentation (BLUE)
%     hsvI = rgb2hsv(I);
%     H = hsvI(:,:,1); 
%     S = hsvI(:,:,2); 
%     V = hsvI(:,:,3);
% 
%     hueMask = (H > 0.53) & (H < 0.75);
%     level = graythresh(S); 
%     mask = hueMask & (S > level) & (V > 0.15);
% 
%     mask = bwareaopen(mask, 100); 
%     mask = imfill(mask, 'holes');
% 
%     % --- NEW: Get 2D Region Properties (Orientation) ---
%     % We get the Centroid and Orientation for every blob in the 2D mask
%     stats = regionprops(mask, 'Centroid', 'Orientation');
% 
%     %% 4. VISUALIZATION PART 1: 2D Results
%     figure('Name', '2D Processing Stages');
%     subplot(1,3,1); imshow(I); title('Original');
%     subplot(1,3,2); imshow(mask); title('Binary Mask');
%     subplot(1,3,3); imshow(bsxfun(@times, I, cast(mask, 'like', I))); title('Segmented Blue');
% 
%     %% 5. 3D Reconstruction & Masking
%     intrinsics = cameraIntrinsics([depth_intrinsics.fx, depth_intrinsics.fy], ...
%                                   [depth_intrinsics.ppx, depth_intrinsics.ppy], size(depth_frame));
%     ptCloud = pcfromdepth(depth_frame, 1/depth_scaling, intrinsics, 'ColorImage', I);
% 
%     roi_indices = find(mask);
%     if isempty(roi_indices)
%         disp('No Blue objects detected.');
%         return;
%     end
% 
%     cubeCloud = select(ptCloud, roi_indices);
% 
%     %% 6. Clustering and Pose Estimation (with Orientation)
%     [labels, numClusters] = pcsegdist(cubeCloud, 0.03); 
% 
%     figure('Name', '3D Perception Output');
%     pcshow(cubeCloud, 'VerticalAxisDir', 'Down', 'MarkerSize', 40);
%     title(sprintf('Final 3D Point Cloud (Found %d Clusters)', numClusters));
%     hold on;
%     persistent T_cam_to_world
%     for k = 1:numClusters
%         clusterIndices = find(labels == k);
%         oneCube = select(cubeCloud, clusterIndices);
% 
%        % 1. Calculate 3D Position (Centroid) in Camera Frame
%         % 1. Calculate 3D Position (Centroid) in Camera Frame
%         % pos_cam is already a 1x3 ROW vector
%         pos_cam = mean(oneCube.Location, 1)
% 
%         % Load the matrix we generated using the 4 motor corners
%         if isempty(T_cam_to_world)
%             load('camera_transform.mat', 'T_cam_to_world');
%         end
% 
%         % --- MATH FIX: MATLAB Post-Multiplication ---
%         % Make the camera position a 1x4 homogeneous ROW vector
%         pos_cam_homogeneous = [pos_cam, 1]
% 
%         % --- THE MATH FIX: Column Vectors and Pre-Multiplication ---
%         R = T_cam_to_world(1:3, 1:3);
% 
%         % 2. Extract the Translation Vector (top-right 3x1)
%         d = T_cam_to_world(1:3, 4);
% 
%         % 3. Calculate the Inverse components
%         R_inv = R';                 % Transpose of R
%         d_inv = -R' * d;            % -R^T * d
% 
%         % 4. Reconstruct the Inverse Matrix (World from Camera)
%         T_inv = [R_inv, d_inv; 
%                  0, 0, 0, 1];
% 
%         % 5. Convert Camera Position to a 4x1 COLUMN Vector
%         pos_cam_homogeneous = [pos_cam(1); pos_cam(2); pos_cam(3); 1];
% 
%         % 6. PRE-MULTIPLY to get World coordinates (Matrix * Column)
%         pos_world_homogeneous = T_inv * pos_cam_homogeneous;
% 
%         % 7. Extract the final X, Y, Z and transpose back to 1x3 row for plotting
%         pos_world = pos_world_homogeneous(1:3)'; 
% 
% 
%         % 2. Find corresponding 2D Orientation
%         u_proj = (pos_cam(1) / pos_cam(3)) * depth_intrinsics.fx + depth_intrinsics.ppx;
%         v_proj = (pos_cam(2) / pos_cam(3)) * depth_intrinsics.fy + depth_intrinsics.ppy;
% 
%         minDist = inf;
%         bestIdx = -1;
%         for j = 1:length(stats)
%             centroid2D = stats(j).Centroid;
%             dist = sqrt((centroid2D(1)-u_proj)^2 + (centroid2D(2)-v_proj)^2);
%             if dist < minDist
%                 minDist = dist;
%                 bestIdx = j;
%             end
%         end
% 
%         orientationAngle = 0;
%         if bestIdx ~= -1
%             orientationAngle = stats(bestIdx).Orientation;
%         end
% 
%         % --- VISUAL FIX: Plotting on the Camera Cloud ---
%         % We use 'pos_cam' for plotting so the star actually touches the cube!
%         plot3(pos_cam(1), pos_cam(2), pos_cam(3), 'r*', 'MarkerSize', 20, 'LineWidth', 2);
% 
%         rad = deg2rad(orientationAngle);
%         lineLength = 0.05; % 5cm line
%         dx = lineLength * cos(rad);
%         dy = lineLength * sin(rad);
%         plot3([pos_cam(1)-dx, pos_cam(1)+dx], [pos_cam(2)-dy, pos_cam(2)+dy], [pos_cam(3), pos_cam(3)], 'g-', 'LineWidth', 3);
% 
%         % Update Label to display the WORLD coordinates, even though it's plotted in Cam space
%         text(pos_cam(1), pos_cam(2), pos_cam(3)-0.05, sprintf('Cube %d\nWorld: [%.3f, %.3f, %.3f]\nAngle: %.1f^{\\circ}', ...
%             k, pos_world(1), pos_world(2), pos_world(3), orientationAngle), ...
%             'Color', 'y', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', 'k');
% 
%         % Print the final World coordinates to the command window
%         fprintf('Blue Cube %d WORLD Pose: X=%.3fm, Y=%.3fm, Z=%.3fm, Angle=%.2f deg\n', k, pos_world(1), pos_world(2), pos_world(3), orientationAngle);
%     end
% 
%     view(0, -90); 
%     xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
%     grid on;
%     hold off;
% end














function perception_pipeline_3d_regionprops()
    % PERCEPTION_PIPELINE_3D_REGIONPROPS 
    % Detects a blue cube using RealSense camera and calculates its 3D centroid
    % using HSV thresholding, geometric filtering, and point cloud clustering.

    %% 1. Hardware Initialization & Configuration
    pipe = realsense.pipeline();
    cfg = realsense.config();

    % Enable streams (rgb8 for native MATLAB color processing)
    cfg.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30);
    cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);

    % Initialize pointcloud object
    pc = realsense.pointcloud();

    % Create align object to ensure depth perfectly matches color pixels
    align_to_color = realsense.align(realsense.stream.color);

    % Start pipeline
    profile = pipe.start(cfg);

    disp('Warming up camera...');
    i = 0;
    while i < 5
        % Wait for frames
        fs = pipe.wait_for_frames();
        
        % Align frames so depth and color pixels correspond 1:1
        fs = align_to_color.process(fs);
        
        % Get depth and color frames
        depth_frame = fs.get_depth_frame();
        color_frame = fs.get_color_frame();
        
        i = i + 1;
        
        % Map pointcloud to color and calculate points
        pc.map_to(color_frame);
        points = pc.calculate(depth_frame);
        
        % Get vertices natively
        vtx = points.get_vertices();
        
        fprintf('Processed frame %d/5\n', i);
    end

    % Stop streaming
    pipe.stop();
    disp('Camera stopped. Processing final frame for Blue Objects...');

    %% 2. Extract RGB Image
    % Pull raw data from the RealSense frame and reshape it into a standard MATLAB Image matrix
    color_data = color_frame.get_data();
    I = permute(reshape(color_data', [3, 640, 480]), [3 2 1]);

    %% 3. 2D HSV Color Segmentation (BLUE)
    hsvI = rgb2hsv(I);
    H = hsvI(:,:,1); 
    S = hsvI(:,:,2); 
    V = hsvI(:,:,3);

    % BLUE COLOR THRESHOLDS
    % Slightly widened Hue, but much stricter Saturation and Value (Brightness)
    hueMask = (H > 0.52) & (H < 0.65); 
    % Ensure Saturation is at least 40% (graythresh sometimes dips too low on dull backgrounds)
    level = max(graythresh(S), 0.40); 
    % RAISED VALUE THRESHOLD: 0.25 rejects dark shadows and black plastic
    mask = hueMask & (S > level) & (V > 0.25); 

    % GREEN COLOR THRESHOLDS

    % Clean up the mask (remove noise, fill holes)
    mask = bwareaopen(mask, 100); 
    mask = imfill(mask, 'holes');

    %% 4. Geometric Filtering (The "Robust" Fix)
    % Analyze the remaining white blobs for their physical properties
    stats = regionprops(mask, 'Area', 'PixelIdxList', 'Solidity');

    % Create a blank mask to hold our geometrically filtered results
    robust_mask = false(size(mask));

    for j = 1:length(stats)
        % Filter Conditions:
        % - Area > 300: Must be larger than background noise specks
        % - Area < 15000: Must be smaller than massive objects (like a robot base)
        % - Solidity > 0.80: Must be a solid shape (cubes are highly solid)
        if stats(j).Area > 300 && stats(j).Area < 15000 && stats(j).Solidity > 0.80
            % If it passes the geometry test, keep it!
            robust_mask(stats(j).PixelIdxList) = true;
        end
    end

    % Overwrite the old mask with our new robust mask
    mask = robust_mask;

    % Show the 2D mask to verify it found ONLY the cube
    figure('Name', 'Blue Detection Mask');
    subplot(1,2,1); imshow(I); title('RGB Image');
    subplot(1,2,2); imshow(mask); title('Robust Blue Mask');

    %% 5. Filter the Point Cloud
    % The 'vtx' matrix is 307200x3 (which is 640 * 480). 
    % It stores points row-by-row. We need to flatten our 2D mask to match this order.
    mask_1d = reshape(mask', [], 1);

    % 1. Filter out points where the sensor read 0 depth
    valid_depth = (vtx(:,3) > 0); 

    % 2. Combine depth filter with our Blue mask filter
    blue_indices = valid_depth & mask_1d;

    % 3. Extract only the vertices that are Blue
    blue_vtx = vtx(blue_indices, :);

    if isempty(blue_vtx)
        disp('No blue objects found in the 3D space!');
        return;
    end
    
    disp('Clustering Point Cloud to remove noise...');
    
    % Create a MATLAB pointCloud object using the blue vertices
    ptCloud = pointCloud(blue_vtx);
    
    % --- THE FIX: Cluster the points to remove floating noise ---
    % Group points that are within 0.8cm (0.008m) of each other
    [labels, numClusters] = pcsegdist(ptCloud, 0.008);
    
    if numClusters == 0
        disp('Could not find a distinct physical object.');
        return;
    end
    
    % Find the largest dense cluster (assuming the biggest blue thing is our cube)
    clusterSizes = zeros(numClusters, 1);
    for c = 1:numClusters
        clusterSizes(c) = sum(labels == c);
    end
    [~, biggestClusterIdx] = max(clusterSizes);
    
    % Extract ONLY the points belonging to the biggest cluster (the actual cube)
    cubeIndices = find(labels == biggestClusterIdx);
    cubeCloud = select(ptCloud, cubeIndices);
    
    %% 6. Calculate Centroid and Visualize
    % Calculate the centroid of ONLY the clustered cube
    pos_cam = mean(cubeCloud.Location, 1);
    
    % Print the coordinates to the command window
    fprintf('\n--- TARGET DETECTED (Clustered) ---\n');
    fprintf('Camera X: %.3f m\n', pos_cam(1));
    fprintf('Camera Y: %.3f m\n', pos_cam(2));
    fprintf('Camera Z: %.3f m\n', pos_cam(3));
    fprintf('-----------------------------------\n\n');
    
    % Plot the cleaned-up point cloud
    figure('Name', 'Segmented & Clustered 3D Point Cloud', 'Position', [150, 150, 800, 600]);
    pcshow(cubeCloud, 'MarkerSize', 40);
    hold on;
    
    % Plot the centroid and a label on the 3D graph
    plot3(pos_cam(1), pos_cam(2), pos_cam(3), 'r*', 'MarkerSize', 20, 'LineWidth', 2);
    text(pos_cam(1), pos_cam(2), pos_cam(3) - 0.02, ...
         sprintf(' Center: [%.3f, %.3f, %.3f]', pos_cam(1), pos_cam(2), pos_cam(3)), ...
         'Color', 'white', 'FontSize', 12, 'FontWeight', 'bold');
    
    title('Filtered & Clustered Point Cloud (Cube Only)');
    xlabel('X (meters)');
    ylabel('Y (meters)');
    zlabel('Z (meters)');
    
    % RealSense standard coordinates: Y is down, Z is forward
    set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
    view(0, -90); % Top-down isometric view
    hold off;
end