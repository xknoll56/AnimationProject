#version 330

out vec4 FragColor;
in vec3 Normal;
uniform vec3 color;
uniform vec3 lightDir;

void main()
{
    vec3 lightDir = normalize(lightDir);
    float diffuse = max(dot(Normal, lightDir), 0.35f);
    FragColor = vec4(color*diffuse, 1.0f);
}
