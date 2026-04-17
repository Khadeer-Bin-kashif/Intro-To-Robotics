function isColliding = checkSelfCollision(q_init, q_final)
    pincher = rigidBodyTree('DataFormat', 'row');

    % Link lengths from pincherFK.m (converted to meters for standard ROS/MATLAB compatibility)
    L1 = 0.1218;  
    L2 = 0.10238; 
    L3 = 0.10238; 
    L4 = 0.07691; 
    
    % Define a standard radius to encapsulate the physical servos and brackets
    r = 0.025; % 2.5 cm radius should comfortably enclose the AX-12A servos
    
    %% 1. Base Link (Link 1)
    body1 = rigidBody('link1');
    jnt1 = rigidBodyJoint('jnt1', 'revolute');
    setFixedTransform(jnt1, trvec2tform([0 0 0])); % Adjust based on your DH
    jnt1.JointAxis = [0 0 1];
    body1.Joint = jnt1;
    
    % Add collision: Cylinder enclosing the base to the shoulder joint
    % The transform centers the cylinder along the link's length
    colTform1 = trvec2tform([0 0 L1/2]); 
    addCollision(body1, 'cylinder', [r L1], colTform1);
    addBody(pincher, body1, 'base');
    
    %% 2. Shoulder to Elbow (Link 2)
    body2 = rigidBody('link2');
    jnt2 = rigidBodyJoint('jnt2', 'revolute');
    setFixedTransform(jnt2, trvec2tform([0 0 L1])); % Adjust based on your DH
    jnt2.JointAxis = [0 1 0];
    body2.Joint = jnt2;
    
    % Add collision: Cylinder enclosing the upper arm
    colTform2 = trvec2tform([0 0 L2/2]); 
    addCollision(body2, 'cylinder', [r L2], colTform2);
    addBody(pincher, body2, 'link1');
    
    %% 3. Elbow to Wrist (Link 3)
    body3 = rigidBody('link3');
    jnt3 = rigidBodyJoint('jnt3', 'revolute');
    setFixedTransform(jnt3, trvec2tform([0 0 L2])); % Adjust based on your DH
    jnt3.JointAxis = [0 1 0];
    body3.Joint = jnt3;
    
    % Add collision: Cylinder enclosing the forearm
    colTform3 = trvec2tform([0 0 L3/2]);
    addCollision(body3, 'cylinder', [r L3], colTform3);
    addBody(pincher, body3, 'link2');
    
    %% 4. Wrist to Gripper (Link 4)
    body4 = rigidBody('link4');
    jnt4 = rigidBodyJoint('jnt4', 'revolute');
    setFixedTransform(jnt4, trvec2tform([0 0 L3])); % Adjust based on your DH
    jnt4.JointAxis = [0 1 0];
    body4.Joint = jnt4;
    
    % Add collision: Box enclosing the wrist and gripper mechanism
    % Box dimensions: [length, width, height]
    colTform4 = trvec2tform([0 0 L4/2]);
    addCollision(body4, 'box', [0.05 0.05 L4], colTform4);
    addBody(pincher, body4, 'link3');
    
    
    ground = collisionBox(1, 1, 0.05); 
    ground.Pose = trvec2tform([0 0 -0.025]); 
    
    % Package it into a cell array as expected by MATLAB
    world = {ground};
    max_step_size = 0.05; % ~2.8 degrees maximum per step
    max_diff = max(abs(q_final - q_init));
    numSteps = max(ceil(max_diff / max_step_size), 2); 

    % Pre-allocate the trajectory matrix
    numJoints = length(q_init);
    q_traj = zeros(numSteps, numJoints);

    % 2. Interpolate the Path
    for i = 1:numJoints
        q_traj(:, i) = linspace(q_init(i), q_final(i), numSteps);
    end

    % 3. Evaluate Collisions
    isColliding = false; 

    for i = 1:numSteps
        config = q_traj(i, :);
        
        inCollision = checkCollision(pincher, config, world,'SkippedSelfCollisions', 'parent');
        
        % 4. Early Exit on Collision Detection
        if inCollision
            isColliding = true;
            fprintf('Collision detected at step %d of %d!\n', i, numSteps);
            return; 
        end
    end
end