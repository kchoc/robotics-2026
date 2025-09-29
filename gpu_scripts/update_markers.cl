#define MAX_DISTANCE 10000.0f
#define HIGH_RISE_THRESHOLD 195
#define HIGH_RISE_WIDTH 324.0f
#define BOX_WIDTH 162.0f

__kernel void update_markers(
        __global const float *marker_data,
        __global float *markers,
        const float robot_rotation,
        const float2 robot_position
    ) {
    int i = get_global_id(0);

    int marker_id = (int)marker_data[i * 6];
    float roll = marker_data[i * 6 + 1];
    float pitch = marker_data[i * 6 + 2];
    float yaw = marker_data[i * 6 + 3];
    float distance = marker_data[i * 6 + 4];
    float horizontal_angle = marker_data[i * 6 + 5];

    if (distance > MAX_DISTANCE)
        return;

    float robot_to_marker_bearing = robot_rotation + horizontal_angle;

    float2 position = robot_position + distance * (float2)(sin(robot_to_marker_bearing), cos(robot_to_marker_bearing));

    float rotation;
    int side = ((int)(roll * 2.0f / M_PI_F + 4.5f)) % 4;
    float rel_rot = (side == 0) ? yaw :
                    (side == 1) ? pitch :
                    (side == 2) ? -yaw : -pitch;

    rotation = fmod(robot_rotation + rel_rot + 3.0f * M_PI_F, 2.0f * M_PI_F);

    float width = (marker_id >= HIGH_RISE_THRESHOLD) ? HIGH_RISE_WIDTH : BOX_WIDTH;

    position -= width * (float2)(sin(rotation), cos(rotation));
    markers[marker_id * 3] = position.x;
    markers[marker_id * 3 + 1] = position.y;
    markers[marker_id * 3 + 2] = rotation;
}