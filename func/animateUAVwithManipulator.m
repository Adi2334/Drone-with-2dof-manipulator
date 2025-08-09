function animateUAVwithManipulator(time, states, joint_angles,dt)
    % Inputs:
    % time: Time vector [1 x N]
    % states: Matrix of UAV states over time [N x 6], where each row is [x, y, z, psi, theta, phi]
    % link_lengths: [L1, L2] for the manipulator links
    % joint_angles: Matrix of joint angles over time [N x 2], each row is [q1, q2]

    % Create a new figure for animation
    % figure;
    hold on;
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    title('UAV with 2-DOF Manipulator Animation');

    % Set axis limits (adjust according to your trajectory)
    border = 1;
    xlim([min(states(:,1))-border max(states(:,1))+border]); ylim([min(states(:,2))-border max(states(:,2))+border]); zlim([min(states(:,3))-border max(states(:,3))+border]);

    % Unpack link lengths
    L1 = 0.25;
    L2 = 0.25;

    % Initialize plot objects for the UAV and manipulator
    droneXY = plot3(0, 0, 0, 'k-', 'LineWidth', 4);   % Drone line in XY plane
    droneXZ = plot3(0, 0, 0, 'g-', 'LineWidth', 4);   % Drone line in XZ plane
    link1 = plot3(0, 0, 0, 'r-', 'LineWidth', 3);     % First manipulator link
    link2 = plot3(0, 0, 0, 'b-', 'LineWidth', 3);     % Second manipulator link
    link3 = plot3(0, 0, 0, 'g-', 'LineWidth', 3);     % Second manipulator link
        
    % v = VideoWriter('PDFB_manipulator_2.mp4', 'MPEG-4'); % Create video file (MP4 format)
    % v.FrameRate = 60; % Set frame rate (adjust as needed)
    % open(v)
    % Animate the UAV and manipulator over time
    for i = 1:length(time)-1
        % Get current state and joint angles
        state = states(i, :);         % [x, y, z, psi, theta, phi]
        q = joint_angles(i, :);       % [q1, q2]

        % Unpack state variables
        x = state(1);
        y = state(2);
        z = state(3);
        psi = state(4);
        theta = state(5);
        phi = state(6);

        % Compute rotation matrix from UAV orientation
        R = eul2rotm([psi, theta, phi]);
        % R = R';
        % Compute endpoints for the drone's cross lines
        arm_length = 0.5;  % Length of each arm of the drone
        xy_end1 = [arm_length; 0; 0];    % End of XY line
        xy_end2 = [-arm_length; 0; 0];
        xz_end1 = [0; arm_length; 0];    % End of XZ line
        xz_end2 = [0; -arm_length; 0];

        % Rotate and translate the endpoints
        xy_end1 = R * xy_end1 + [x; y; z];
        xy_end2 = R * xy_end2 + [x; y; z];
        xz_end1 = R * xz_end1 + [x; y; z];
        xz_end2 = R * xz_end2 + [x; y; z];

        % Compute the base of the manipulator (at the drone center)
        manipBase = [x; y; z];
        Link3 = manipBase - [0;0;0];
        % Compute manipulator joint positions
        joint1 = Link3 + R * [L1 * cos(q(1)); 0; L1 * sin(q(1))] ;
        joint2 = joint1 + R * [L2 * cos(q(1) + q(2)); 0; L2 * sin(q(1) + q(2))];

        % Update drone XY and XZ lines
        set(droneXY, 'XData', [xy_end1(1), xy_end2(1)], ...
                     'YData', [xy_end1(2), xy_end2(2)], ...
                     'ZData', [xy_end1(3), xy_end2(3)]);

        set(droneXZ, 'XData', [xz_end1(1), xz_end2(1)], ...
                     'YData', [xz_end1(2), xz_end2(2)], ...
                     'ZData', [xz_end1(3), xz_end2(3)]);
    
        % Update the first manipulator link
        set(link3, 'XData', [manipBase(1), Link3(1)], ...
                   'YData', [manipBase(2), Link3(2)], ...
                   'ZData', [manipBase(3), Link3(3)]);
        % Update the first manipulator link
        set(link1, 'XData', [Link3(1), joint1(1)], ...
                   'YData', [Link3(2), joint1(2)], ...
                   'ZData', [Link3(3), joint1(3)]);

        % Update the second manipulator link
        set(link2, 'XData', [joint1(1), joint2(1)], ...
                   'YData', [joint1(2), joint2(2)], ...
                   'ZData', [joint1(3), joint2(3)]);

        % Pause to create animation effect
        pause(dt);  % Adjust for smoother animation
        % frame = getframe(gcf); % Capture the current figure as a frame
        % writeVideo(v, frame); % Write the frame to the video file
    end
    % close(v);
    hold off;
end

% Helper function to convert Euler angles (Z-Y-X) to rotation matrix
function R = eul2rotm(eul)
    psi = eul(3); theta = eul(2); phi = eul(1);
    Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
    Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
    RctoI = [0 1 0;
    1 0 0;
    0 0 -1];
    R = Rz * Ry * Rx ;
end
