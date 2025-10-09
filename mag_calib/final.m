%% Inspired by https://github.com/jremington/ICM_20948-AHRS/blob/main/calibrate3.py

clear; clc; close all;

%% Load Data
fprintf('Loading magnetometer data...\n');
% data = readmatrix("data2.csv");
data = readmatrix("one.csv");

% Extract X, Y, Z columns
X_raw = data(:, 1);
Y_raw = data(:, 2);
Z_raw = data(:, 3);

fprintf('Loaded %d data points\n', length(X_raw));

% Basic statistics
fprintf('Data ranges:\n');
fprintf('  X: %.1f to %.1f\n', min(X_raw), max(X_raw));
fprintf('  Y: %.1f to %.1f\n', min(Y_raw), max(Y_raw));
fprintf('  Z: %.1f to %.1f\n', min(Z_raw), max(Z_raw));

%% Plot Raw Data
figure(1);
scatter3(X_raw, Y_raw, Z_raw, 10, 'b', 'filled');
xlabel('X (raw)'); ylabel('Y (raw)'); zlabel('Z (raw)');
title('Raw Magnetometer Data');
grid on; axis equal;
view(45, 30);

%% Li & Griffiths Ellipsoid-Specific Fitting
fprintf('\nApplying Li & Griffiths ellipsoid-specific fitting...\n');

% Arbitrary norm of magnetic field vectors (same as Python implementation)
% F_norm = 1000;

F_norm = mean(sqrt(X_raw.^2 + Y_raw.^2 + Z_raw.^2));

% Prepare data matrix (samples as columns, like Python)
s = [X_raw'; Y_raw'; Z_raw'];  % 3 x N matrix
n = length(X_raw);

% D matrix (design matrix with quadratic terms)
D = [s(1,:).^2; s(2,:).^2; s(3,:).^2;
     2.*s(2,:).*s(3,:); 2.*s(1,:).*s(3,:); 2.*s(1,:).*s(2,:);
     2.*s(1,:); 2.*s(2,:); 2.*s(3,:); ones(1, size(s,2))];

% S matrix and sub-matrices (eq. 11 in paper)
S = D * D';
S_11 = S(1:6, 1:6);
S_12 = S(1:6, 7:10);
S_21 = S(7:10, 1:6);
S_22 = S(7:10, 7:10);

% C matrix (Eq. 8, k=4) - constraint for ellipsoid
C = [-1,  1,  1,  0,  0,  0;
      1, -1,  1,  0,  0,  0;
      1,  1, -1,  0,  0,  0;
      0,  0,  0, -4,  0,  0;
      0,  0,  0,  0, -4,  0;
      0,  0,  0,  0,  0, -4];

% E matrix (eq. 15, solution)
E = inv(C) * (S_11 - S_12 * (S_22 \ S_21));

% Eigenvalue decomposition
[E_v, E_w] = eig(E);
[~, max_idx] = max(diag(E_w));
v_1 = E_v(:, max_idx);

% Ensure positive orientation
if v_1(1) < 0
    v_1 = -v_1;
end

% v_2 (eq. 13, solution)
v_2 = -(S_22 \ S_21) * v_1;

% Extract ellipsoid parameters
% Note: parameters h and f swapped as per correction mentioned in Python
M = [v_1(1), v_1(6), v_1(5);
     v_1(6), v_1(2), v_1(4);
     v_1(5), v_1(4), v_1(3)];
n_vec = [v_2(1); v_2(2); v_2(3)];
d = v_2(4);

% Calculate calibration parameters
M_inv = inv(M);
b = -M_inv * n_vec;  % Hard iron bias (relative to centered data)

% Soft iron transformation matrix
sqrt_term = real(F_norm / sqrt(n_vec' * M_inv * n_vec - d));
A_inv = sqrt_term * real(sqrtm(M));

%% Apply Calibration
fprintf('Applying calibration...\n');

X_cal = zeros(n, 1);
Y_cal = zeros(n, 1);
Z_cal = zeros(n, 1);

for i = 1:n
    % Subtract hard iron offset (from original centered data)
    xm_off = X_raw(i) - b(1);
    ym_off = Y_raw(i) - b(2);
    zm_off = Z_raw(i) - b(3);
    
    % Apply soft iron correction
    X_cal(i) = xm_off * A_inv(1,1) + ym_off * A_inv(1,2) + zm_off * A_inv(1,3);
    Y_cal(i) = xm_off * A_inv(2,1) + ym_off * A_inv(2,2) + zm_off * A_inv(2,3);
    Z_cal(i) = xm_off * A_inv(3,1) + ym_off * A_inv(3,2) + zm_off * A_inv(3,3);
end

% Final offsets (add back to original offset)
final_offset_x = b(1);
final_offset_y = b(2);
final_offset_z = b(3);

%% Validation and Results
fprintf('\n=== CALIBRATION RESULTS ===\n');
fprintf('Method: Li & Griffiths Ellipsoid-Specific Fitting\n');

% Calculate magnitudes
mag_raw = sqrt(X_raw.^2 + Y_raw.^2 + Z_raw.^2);
mag_cal = sqrt(X_cal.^2 + Y_cal.^2 + Z_cal.^2);

% Statistics
fprintf('\nRaw Data Statistics:\n');
fprintf('  Mean magnitude: %.2f\n', mean(mag_raw));
fprintf('  Std deviation:  %.2f\n', std(mag_raw));
fprintf('  Min magnitude:  %.2f\n', min(mag_raw));
fprintf('  Max magnitude:  %.2f\n', max(mag_raw));
fprintf('  Variation:      %.1f%%\n', 100 * std(mag_raw) / mean(mag_raw));

fprintf('\nCalibrated Data Statistics:\n');
fprintf('  Mean magnitude: %.2f\n', mean(mag_cal));
fprintf('  Std deviation:  %.2f\n', std(mag_cal));
fprintf('  Min magnitude:  %.2f\n', min(mag_cal));
fprintf('  Max magnitude:  %.2f\n', max(mag_cal));
fprintf('  Variation:      %.1f%%\n', 100 * std(mag_cal) / mean(mag_cal));

fprintf('\nHard Iron Offsets:\n');
fprintf('  X offset: %8.2f\n', final_offset_x);
fprintf('  Y offset: %8.2f\n', final_offset_y);
fprintf('  Z offset: %8.2f\n', final_offset_z);

fprintf('\nSoft Iron Correction Matrix:\n');
fprintf('  [%8.4f %8.4f %8.4f]\n', A_inv(1,:));
fprintf('  [%8.4f %8.4f %8.4f]\n', A_inv(2,:));
fprintf('  [%8.4f %8.4f %8.4f]\n', A_inv(3,:));

%% Plotting Results
% Plot comparison
figure(2);
subplot(2,2,1);
scatter3(X_raw, Y_raw, Z_raw, 10, 'r', 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Raw Data');
grid on; axis equal;

subplot(2,2,2);
scatter3(X_cal, Y_cal, Z_cal, 10, 'g', 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Calibrated Data');
grid on; axis equal;

subplot(2,2,3);
plot(mag_raw, 'r-');
xlabel('Sample'); ylabel('Magnitude');
title('Raw Magnitude');
grid on;
ylim([min(mag_raw)*0.9, max(mag_raw)*1.1]);

subplot(2,2,4);
plot(mag_cal, 'g-');
xlabel('Sample'); ylabel('Magnitude');
title('Calibrated Magnitude');
grid on;
ylim([min(mag_cal)*0.9, max(mag_cal)*1.1]);

% Magnitude histogram comparison
figure(3);
subplot(1,2,1);
histogram(mag_raw, 50, 'FaceColor', 'r', 'FaceAlpha', 0.7);
xlabel('Magnitude'); ylabel('Count');
title('Raw Magnitude Distribution');
grid on;

subplot(1,2,2);
histogram(mag_cal, 50, 'FaceColor', 'g', 'FaceAlpha', 0.7);
xlabel('Magnitude'); ylabel('Count');
title('Calibrated Magnitude Distribution');
grid on;

fprintf('\n=== CALIBRATION C CODE ===\n');
fprintf('// Hard iron offsets\n');
fprintf('float offset_x = %.2f;\n', final_offset_x);
fprintf('float offset_y = %.2f;\n', final_offset_y);
fprintf('float offset_z = %.2f;\n', final_offset_z);
fprintf('\n// Soft iron correction matrix\n');
fprintf('float calib_matrix[3][3] = {\n');
fprintf('  {%8.4f, %8.4f, %8.4f},\n', A_inv(1,:));
fprintf('  {%8.4f, %8.4f, %8.4f},\n', A_inv(2,:));
fprintf('  {%8.4f, %8.4f, %8.4f}\n', A_inv(3,:));
fprintf('};\n\n');

fprintf('// Apply calibration function:\n');
fprintf('void calibrateMagnetometer(float raw_x, float raw_y, float raw_z,\n');
fprintf('                          float* cal_x, float* cal_y, float* cal_z) {\n');
fprintf('  // Remove hard iron offsets\n');
fprintf('  float centered_x = raw_x - offset_x;\n');
fprintf('  float centered_y = raw_y - offset_y;\n');
fprintf('  float centered_z = raw_z - offset_z;\n\n');
fprintf('  // Apply soft iron correction\n');
fprintf('  *cal_x = calib_matrix[0][0]*centered_x + calib_matrix[0][1]*centered_y + calib_matrix[0][2]*centered_z;\n');
fprintf('  *cal_y = calib_matrix[1][0]*centered_x + calib_matrix[1][1]*centered_y + calib_matrix[1][2]*centered_z;\n');
fprintf('  *cal_z = calib_matrix[2][0]*centered_x + calib_matrix[2][1]*centered_y + calib_matrix[2][2]*centered_z;\n');
fprintf('}\n');