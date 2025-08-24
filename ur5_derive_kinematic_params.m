% UR5e
alpha_vec = [sym(pi)/2 0 0 sym(pi)/2 -sym(pi)/2 0]; % rad
a_vec = [0 -0.425 -0.3922 0 0 0]; % m
d_vec = [0.1625 0 0 0.1333 0.0997 0.0996]; % m

kin = dh_to_kin(alpha_vec, a_vec, d_vec);

[is_int, is_int_nonconsecutive, is_parallel, is_spherical] =...
    detect_intersecting_parallel_axes(kin);
print_intersecting_parallel_axes(...
    is_int, is_int_nonconsecutive, is_parallel, is_spherical);

disp("P=");disp(vpa(kin.P)) % Display with decimals rather than fractions
disp("H=");disp(kin.H)