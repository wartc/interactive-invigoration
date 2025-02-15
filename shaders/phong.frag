#version 330 core

in vec3 fNormal;

out vec4 fragColor;

uniform vec4 color;
uniform vec3 lightDir;
uniform vec3 viewPos;

const float ambientStrength = 0.2;
const float specularStrength = 0.5;
const float shininess = 32.0;

void main() {
    vec3 norm = normalize(fNormal);
    vec3 L = normalize(lightDir);
    
    vec3 ambient = ambientStrength * color.rgb;
    
    float diff = max(dot(norm, L), 0.0);
    vec3 diffuse = diff * color.rgb;

    vec3 viewDir = normalize(viewPos);
    vec3 reflectDir = reflect(-L, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), shininess);
    vec3 specular = specularStrength * spec * vec3(1.0);
    
    vec3 finalColor = ambient + diffuse + specular;
    fragColor = vec4(finalColor, color.a);
}