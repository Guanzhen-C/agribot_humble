// Derived from RVizSplat, Apache-2.0 (renderer/LICENSE).
#version 330 core
uniform float u_alpha_threshold;
in vec4 vColor;
in vec2 vPosition;
out vec4 frag_color;
void main(){
 float alpha=min(.99,exp(-dot(vPosition,vPosition))*vColor.a);
 if(alpha<u_alpha_threshold) discard;
 frag_color=vec4(vColor.rgb*alpha,alpha);
}
