% Open the bag file
 % Replace with your actual bag file name
bag = rosbag('bag_1');

% Initialize variables
num_joints = 7; % Number of joints
joint_data = cell(1, num_joints); % Cell array to store joint data

% Loop through all joints and read messages
for joint_idx = 1:num_joints
    topic_name = sprintf('iiwa/iiwa_joint_%d_effort_controller/command', joint_idx);
    joint = select(bag, 'Topic', topic_name);
    
    % Read messages
    msgStructs = readMessages(joint, 'DataFormat', 'struct');
    
    % Convert to double and store in the array
    joint_data{joint_idx} = cellfun(@(m) double(m.data), msgStructs);
end

% Now joint_data{1} contains data for joint 1, joint_data{2} for joint 2, and so on
% If you want to concatenate them into a matrix:
joint_matrix = cell2mat(cellfun(@(j) j(:), joint_data, 'UniformOutput', false));


% Time array based on the number of samples (assumes uniform sampling)
% Replace with actual time data if available in the bag file
num_samples = length(joint_data{1});
time = linspace(0, num_samples-1, num_samples); 

% Create a figure and plot all joints
figure;
hold on;
for joint_idx = 1:num_joints
    plot(time, joint_data{joint_idx}, 'DisplayName', sprintf('Joint %d', joint_idx));
end
hold off;

% Add labels, legend, and title
xlabel('Time (samples)'); % Update to actual time if available
ylabel('Torque (Nm)');
title('Joint Torque Commands');
legend show;
grid on;
