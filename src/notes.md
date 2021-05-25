alpha = pi / 12.0
beta = pi / 36.0 

keyswitch_height = 14.4
keyswitch_width = 14.4
mount_width = keyswitch_width + 3
mount_height = keyswitch_height + 3
extra_height = 1.0  # original= 0.5

plate_thickness = 4
sa_profile_key_height = 12.7

cap_top_height = plate_thickness + sa_profile_key_height
cap_top_height = 16.7

row_radius = ((mount_height + extra_height) / 2) / (np.sin(alpha / 2)) + cap_top_height
row_radius = ((17.4 + 1.0) / 2) / (np.sin(pi/12 / 2)) + 16.7
row_radius = 9.95 / 0.13052619222 + 16.7
row_radius = 70.483937695 + 16.7
row_radius = 87.183937695

column_radius = (((mount_width + extra_width) / 2) / (np.sin(beta / 2))) + cap_top_height
column_radius = ((17.4 + 2.5) / 2) / (np.sin( pi / 36.0 / 2))) + 16.7
column_radius = 9.95 / 0.04361938736 + 16.7
column_radius = 228.109576979 + 16.7
column_radius = 244.809576979

column_x_delta = -1 - column_radius * np.sin(beta)
column_x_delta = -1 - 244.809576979 * np.sin(pi / 36.0)
column_x_delta = -1 - 21.3365605133
column_x_delta = -22.3365605133

column_base_angle = beta * (centercol - 2)
column_base_angle = (pi / 36.0) * (3 - 2)
column_base_angle = pi / 36.0
column_base_angle = 5°
