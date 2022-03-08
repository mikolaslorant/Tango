#version 330

layout (lines) in;
layout (triangle_strip, max_vertices = 4) out;

uniform float thickness = 0.01;
uniform vec2 screenSize;

void main()
{
    vec4 p1 = gl_in[0].gl_Position;
    vec4 p2 = gl_in[1].gl_Position;

    vec2 dir = normalize(p2.xy - p1.xy);
    vec2 normal = vec2(dir.y, -dir.x);

    vec2 offset = normal * thickness;
    offset.y = offset.y * screenSize.x / screenSize.y;
    

    vec4 coords[4];
    coords[0] = p1 + vec4(offset, 0, 0);
    coords[1] = p1 - vec4(offset, 0, 0);
    coords[2] = p2 + vec4(offset, 0, 0);
    coords[3] = p2 - vec4(offset, 0, 0);

    for (int i = 0; i < 4; ++i) {
        gl_Position = coords[i];
        EmitVertex();
    }
    EndPrimitive();
}