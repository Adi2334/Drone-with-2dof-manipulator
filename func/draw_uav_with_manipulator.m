function draw_uav_with_manipulator(states, joint_angles)
    % states = [x, y, z, psi, theta, phi]
    % link_lengths: [L1, L2] for the 2-DOF manipulator
    % joint_angles: [q1, q2] representing joint angles of the 2-DOF manipulator
    
    % Unpack state variables
    x = states(1);
    y = states(2);
    z = states(3);
    psi = states(4);  % Yaw
    theta = states(5); % Pitch
    phi = states(6);  % Roll
    link_lengths =[0.1,0.1];
    % Unpack link lengths and joint angles
    L1 = link_lengths(1);
    L2 = link_lengths(2);
    q1 = joint_angles(1);
    q2 = joint_angles(2);

    % Create a new figure
    figure;
    hold on;
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    title('3D UAV with 2-DOF Manipulator');

    % Transformation matrix for UAV orientation (Z-Y-X Euler angles)
    R = eul2rotm([psi, theta, phi]);

    % Draw the UAV body (a cuboid)
    uavSize = [0.2, 0.2, 0.01]; % Length, width, height of the UAV body
    drawCuboid([x, y, z], R, uavSize);

    % Compute the base of the manipulator (bottom of UAV)
    manipBase = [x; y; z] - R * [0; 0; uavSize(3) / 2];

    % Draw the 2-DOF manipulator in the XZ-plane (rotated by q1 and q2)
    joint1 = manipBase + R * [L1 * cos(q1); 0; L1 * sin(q1)];
    joint2 = joint1 + R * [L2 * cos(q1 + q2); 0; L2 * sin(q1 + q2)];

    % Plot the manipulator links
    plot3([manipBase(1), joint1(1)], [manipBase(2), joint1(2)], ...
          [manipBase(3), joint1(3)], 'r-', 'LineWidth', 3);
    plot3([joint1(1), joint2(1)], [joint1(2), joint2(2)], ...
          [joint1(3), joint2(3)], 'b-', 'LineWidth', 3);

    % Plot joints
    plot3(manipBase(1), manipBase(2), manipBase(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    plot3(joint1(1), joint1(2), joint1(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    plot3(joint2(1), joint2(2), joint2(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

    hold off;
end

% Helper function to draw a cuboid representing the UAV body
function drawCuboid(center, R, size)
    % Compute the 8 corners of the cuboid
    l = size(1) / 2; w = size(2) / 2; h = size(3) / 2;
    corners = [ -l, -w, -h;
                 l, -w, -h;
                 l,  w, -h;
                -l,  w, -h;
                -l, -w,  h;
                 l, -w,  h;
                 l,  w,  h;
                -l,  w,  h]';

    % Rotate and translate the corners
    corners = R * corners + center';

    % Define faces of the cuboid
    faces = [1, 2, 3, 4;
             5, 6, 7, 8;
             1, 2, 6, 5;
             2, 3, 7, 6;
             3, 4, 8, 7;
             4, 1, 5, 8];

    % Draw the cuboid
    patch('Vertices', corners', 'Faces', faces, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.5);
end

% Helper function to convert Euler angles (Z-Y-X) to a rotation matrix
function R = eul2rotm(eul)
    psi = eul(1); theta = eul(2); phi = eul(3);
    Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
    Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
    R = Rz * Ry * Rx;
end
