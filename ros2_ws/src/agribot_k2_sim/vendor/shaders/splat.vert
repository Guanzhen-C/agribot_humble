// EWA projection adapted from RVizSplat, Apache-2.0 (renderer/LICENSE).
// Local adaptation: full float32 covariance, portable DC-only source,
// no opacity prefilter, and pixel-space low-pass filtering.
#version 330 core
layout(location=0) in vec2 vertex;
layout(location=8) in float uv0;
uniform samplerBuffer u_splats;
uniform mat4 view_matrix;
uniform mat4 projectionMatrix;
uniform vec4 viewport_size;
out vec4 vColor;
out vec2 vPosition;
void main(){
 int base=int(uv0)*4;
 vec4 t0=texelFetch(u_splats,base);
 vec4 t1=texelFetch(u_splats,base+1);
 vec4 t2=texelFetch(u_splats,base+2);
 vec4 t3=texelFetch(u_splats,base+3);
 vColor=vec4(clamp(vec3(.5)+.28209479177387814*t3.rgb,0.,1.),t0.w);
 vPosition=vertex;
 vec4 camera=view_matrix*vec4(t0.xyz,1.);
 vec4 clip=projectionMatrix*camera;
 if(camera.z>=-.02 || clip.z < -clip.w || abs(clip.x)>1.4*clip.w || abs(clip.y)>1.4*clip.w || t0.w<=0.){
  gl_Position=vec4(0.,0.,2.,1.); return;
 }
 mat3 covariance=mat3(t1.x,t1.y,t1.z,t1.y,t2.x,t2.y,t1.z,t2.y,t2.z);
 mat3 jacobian=mat3(
  -projectionMatrix[0][0]/camera.z,0.,projectionMatrix[0][0]*camera.x/(camera.z*camera.z),
  0.,-projectionMatrix[1][1]/camera.z,projectionMatrix[1][1]*camera.y/(camera.z*camera.z),
  0.,0.,0.);
 mat3 transform=transpose(mat3(view_matrix))*jacobian;
 mat3 projected=transpose(transform)*covariance*transform;
 vec2 pixel=2./viewport_size.xy;
 float d1=projected[0][0]+.3*pixel.x*pixel.x;
 float d2=projected[1][1]+.3*pixel.y*pixel.y;
 float offdiag=projected[0][1];
 float midpoint=.5*(d1+d2);
 float radius=length(vec2(.5*(d1-d2),offdiag));
 float l1=max(midpoint+radius,1e-15),l2=max(midpoint-radius,1e-15);
 vec2 diagonal=vec2(offdiag,l1-d1);
 diagonal=length(diagonal)>1e-12?normalize(diagonal):vec2(1.,0.);
 vec2 a1=sqrt(2.*l1)*diagonal;
 vec2 a2=sqrt(2.*l2)*vec2(diagonal.y,-diagonal.x);
 vec3 center=clip.xyz/clip.w;
 gl_Position=vec4(center.xy+vertex.x*a1+vertex.y*a2,center.z,1.);
}
