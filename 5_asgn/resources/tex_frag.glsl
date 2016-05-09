#version 120

varying vec3 fragPos; // in camera space
varying vec3 fragNor; // in camera space
varying vec2 fragTex;
uniform sampler2D texture0;

void main()
{
	vec3 lightPos = vec3(0.0, 0.0, 0.0); // in camera space
	vec3 tex = texture2D(texture0, fragTex.st).rgb;
	vec3 n = normalize(fragNor);
	vec3 l = normalize(lightPos - fragPos);
	vec3 v = -normalize(fragPos);
	vec3 h = normalize(l + v);
	vec3 colorD = max(dot(l, n), 0.0) * tex;
	vec3 colorS = pow(max(dot(h, n), 0.0), 200) * tex;
	vec3 color = colorD + colorS;
	gl_FragColor = vec4(color, 1.0);
}
