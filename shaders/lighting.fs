#version 330

// Input vertex attributes (from vertex shader)
in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Output fragment color
out vec4 finalColor;

void main()
{
	float illum = clamp(fragNormal.y, 0.2, 1.0);
	
	finalColor = colDiffuse * illum;
	finalColor.a = 1.0;
}
