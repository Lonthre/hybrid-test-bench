shader_type spatial;

uniform vec3 bend_origin;
uniform float bend_strength_h : hint_range(-10, 10) = 1.0;
uniform float bend_strength_v : hint_range(-10, 10) = 1.0;

uniform float fixed_start_x = -2.0; // left anchor (local X)
uniform float fixed_end_x = 2.0;    // right anchor (local X)
uniform float bend_edge_blend = 0.1; // fraction of length to ease bend near ends

void vertex() {
    vec3 local_pos = VERTEX;

    // Compute normalized bend amount based on X position
    float t = (local_pos.x - fixed_start_x) / (fixed_end_x - fixed_start_x);
    float blend = smoothstep(0.0, bend_edge_blend, t) * (1.0 - smoothstep(1.0 - bend_edge_blend, 1.0, t));

    // Only bend between anchors
    vec3 bend_local = local_pos - bend_origin;

    float angle_h = bend_local.x * bend_strength_h * 0.01 * blend;
    float angle_v = bend_local.z * bend_strength_v * 0.01 * blend;

    float cos_h = cos(angle_h);
    float sin_h = sin(angle_h);
    float cos_v = cos(angle_v);
    float sin_v = sin(angle_v);

    float x = bend_local.x;
    float y = bend_local.y * cos_h - bend_local.z * sin_h;
    float z = bend_local.y * sin_h + bend_local.z * cos_h;

    float y2 = y * cos_v - z * sin_v;
    float z2 = y * sin_v + z * cos_v;

    VERTEX = bend_origin + vec3(x, y2, z2);
}
