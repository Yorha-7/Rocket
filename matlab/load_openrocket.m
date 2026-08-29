clear;
clc;
close all;

%% Load OpenRocket CSV
data = readtable("./../data/openrocket.csv", ...
    "VariableNamingRule", "preserve");

%% Display original OpenRocket headers
disp("Available columns:");
disp(data.Properties.VariableNames);

%% Extraction of coloumns from data:
t = data.("# Time (s)");
m = data.("Mass (g)");
C_d = data.("Drag coefficient (​)");
C_n = data.("Normal force coefficient (​)");
T = data.("Thrust (N)");
I_xx = data.("Rotational moment of inertia (kg·m²)");
I_yy = data.("Longitudinal moment of inertia (kg·m²)");
I_zz = data.("Longitudinal moment of inertia (kg·m²)");
C_p = data.("CP location (cm)");
C_g = data.("CG location (cm)");

%% Remove invalid samples
valid = isfinite(t) & isfinite(m) & isfinite(T) & isfinite(C_d) ...
    & isfinite(I_xx) & isfinite(I_yy) & isfinite(I_zz) & isfinite(C_n) ...
    & isfinite(C_p) & isfinite(C_g);

fprintf("Removing %d invalid rows.\n", sum(~valid));

t = t(valid);
m = m(valid);
T = T(valid);
C_d = C_d(valid);
C_n = C_n(valid);
I_xx = I_xx(valid);
I_yy = I_yy(valid);
I_zz = I_zz(valid);
C_p = C_p(valid);
C_g = C_g(valid);

%% Sort by time
[t, idx] = sort(t);

m = m(idx);
T = T(idx);
C_d = C_d(idx);
C_n = C_n(idx);
I_xx = I_xx(idx);
I_yy = I_yy(idx);
I_zz = I_zz(idx);
C_p = C_p(idx);
C_g = C_g(idx);

%% Remove duplicate timestamps
keep = [true; diff(t) ~= 0];

t = t(keep);
m = m(keep);
T = T(keep);
C_d = C_d(keep);
C_n = C_n(keep);
I_xx = I_xx(keep);
I_yy = I_yy(keep);
I_zz = I_zz(keep);
C_p = C_p(keep);
C_g = C_g(keep);

%% Check data
fprintf("Samples: %d\n", length(t));
fprintf("Initial mass: %.4f g\n", m(1));
fprintf("Final mass: %.4f g\n", m(end));
fprintf("Maximum thrust: %.4f N\n", max(T));

%% Prepare Simulink inputs
mass_data = [t m];
thrust_data = [t T];
drag_coefficient = [t,C_d];
normal_coefficient = [t,C_n];
I_xx = [t,I_xx];
I_yy = [t,I_yy];
I_zz = [t,I_zz];
center_of_gravity = [t, C_g];
center_of_pressure = [t, C_p];

%% Save
save("./../data/openrocket_inputs.mat", ...
    "mass_data", ...
    "thrust_data", ...
    "drag_coefficient", ...
    "normal_coefficient", ...
    "I_xx", ...
    "center_of_gravity", ...
    "center_of_pressure");