#version 330

out vec4 FragColor;
in vec3 Normal;

void main()
{
    vec3 lightDir = normalize(vec3(-1.0f,-1.0f,-1.0f));
    float diffuse = max(dot(Normal, lightDir), 0.35f);
    vec3 color = vec3(0,0,1.0f);
    FragColor = vec4(color*diffuse, 1.0f);
}
