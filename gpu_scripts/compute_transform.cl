__kernel void compute_transform(
    __global const float *marker_data,
    __global const float *wall_markers,
    __global const float *wall_rotations,
    __global float *positions,
    __global float *rotations,
    const int num_markers
) {
    int i = get_global_id(0);
    if (i >= num_markers) return;

    float4 marker = vload4(0, &marker_data[i * 5]);
    int marker_id = (int)marker.x;
    float marker_rel_rotation = marker.y;
    float marker_rel_distance = marker.z;
    float marker_horizontal_angle = marker.w;
    float camera_position = marker_data[i * 5 + 4];

    float2 marker_abs = vload2(0, &wall_markers[marker_id * 2]);
    float marker_abs_rotation = wall_rotations[marker_id / 7];

    float robot_bearing = fmod(marker_abs_rotation - marker_rel_rotation + 2.0f * M_PI_F, 2.0f * M_PI_F);
    float robot_to_marker_bearing = robot_bearing + marker_horizontal_angle;

    float robot_x = marker_abs.x - marker_rel_distance * sin(robot_to_marker_bearing) - camera_position * sin(robot_bearing);
    float robot_y = marker_abs.y - marker_rel_distance * cos(robot_to_marker_bearing) - camera_position * cos(robot_bearing);

    rotations[i] = robot_bearing;
    positions[i * 2] = robot_x;
    positions[i * 2 + 1] = robot_y;
}