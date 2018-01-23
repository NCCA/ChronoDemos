#version 410 core
in vec3 fragmentNormal;
in vec2 uv;

layout (location =0) out vec4 fragColour;
uniform vec4 colour;
uniform vec3 lightPos;
uniform vec4 lightDiffuse;
uniform float checkSize=10.0;
uniform bool checkOn;

vec4 checker( vec2 uv )
{
  if(checkOn == false)
    return colour;
  else
  {
  float v = floor( checkSize * uv.x ) +floor( checkSize * uv.y );
  if( mod( v, 2.0 ) < 1.0 )
      return vec4( 1, 1, 1, 1 );
  else
     return colour;

  }
}

void main ()
{
  fragColour= vec4(0.);
  vec3 N = normalize(fragmentNormal);
  vec3 L = normalize(lightPos);
  fragColour += checker(uv)*lightDiffuse *dot(L, N);
}
