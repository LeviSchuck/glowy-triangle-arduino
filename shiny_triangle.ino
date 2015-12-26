#include <Vec.h>

#define USE_HSV

#include <WS2812.h>

#define LED_X 8
#define LED_Y 8

#define LEDCount (LED_X * LED_Y)
#define outputPin 4
#define DELAY_FRAME 4
#define FRAME_RATE 60
#define ANIMATIONS 9


uint16_t framesPerAnim[ANIMATIONS] = {
  FRAME_RATE * 120,
  FRAME_RATE * 10,
  FRAME_RATE * 3,
  FRAME_RATE * 2,
  FRAME_RATE * 5,
  FRAME_RATE * 5,
  FRAME_RATE * 5,
  FRAME_RATE * 20,
  FRAME_RATE * 1
};

float clamp(float x, float minVal = 0, float maxVal = 1)  {
  if(x < minVal) return minVal;
  if(x > maxVal) return maxVal;
  return x;
}

float smoothstep(float edge0, float edge1, float x)
{
    // Scale, bias and saturate x to 0..1 range
    x = clamp((x - edge0)/(edge1 - edge0), 0.0, 1.0); 
    // Evaluate polynomial
    return x*x*(3 - 2*x);
}

float mix(float x, float y, float v) {
  return (1 - v) * x + y * v;
}

float fract(float v) {
  return v - floor(v);
}

float mod(float x, float y) {
  return x - y * floor(x/y);
}


Vec2 iResolution() {
  return Vec2(LED_X, LED_Y);
}



WS2812 LED(LEDCount); 

uint8_t anim;
uint16_t frame;

void setup() {
  // put your setup code here, to run once:
    LED.setOutput(outputPin);
    anim = 0;
    frame = 0;
    randomSeed(analogRead(0));
}

void rainbow(uint16_t frame) {
  cRGB value;
  for(int i = 0; i < LEDCount; i++){
    value.SetHSV((frame + i*5) % 360, 255, sin((float) frame / 30)* 16 + 128);
    LED.set_crgb_at(i,value);
  }
}


Vec3 palettesPal(float t, Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
  // return a + b * cos( 6.28318 * (c * t + d) );
  Vec3 res;
  res.x = a.x + b.x * cos ( 6.28318 * (c.x * t + d.x));
  res.y = a.y + b.y * cos ( 6.28318 * (c.y * t + d.y));
  res.z = a.z + b.z * cos ( 6.28318 * (c.z * t + d.z));
  return res;
}

Vec4 palettes2D(float iGlobalTime, Vec2 fragCoord){
  // palettes : https://www.shadertoy.com/view/ll2GD3
  // vec2 p = fragCoord.xy / iResolution.xy;
  float t = 0.1 * iGlobalTime + fragCoord.x / LED_X;
  Vec3 col;
  /* 
    // compute colors
    vec3                col = pal( p.x, vec3(0.5,0.5,0.5),vec3(0.5,0.5,0.5),vec3(1.0,1.0,1.0),vec3(0.0,0.33,0.67) );
    if( p.y>(1.0/7.0) ) col = pal( p.x, vec3(0.5,0.5,0.5),vec3(0.5,0.5,0.5),vec3(1.0,1.0,1.0),vec3(0.0,0.10,0.20) );
    if( p.y>(2.0/7.0) ) col = pal( p.x, vec3(0.5,0.5,0.5),vec3(0.5,0.5,0.5),vec3(1.0,1.0,1.0),vec3(0.3,0.20,0.20) );
    if( p.y>(3.0/7.0) ) col = pal( p.x, vec3(0.5,0.5,0.5),vec3(0.5,0.5,0.5),vec3(1.0,1.0,0.5),vec3(0.8,0.90,0.30) );
    if( p.y>(4.0/7.0) ) col = pal( p.x, vec3(0.5,0.5,0.5),vec3(0.5,0.5,0.5),vec3(1.0,0.7,0.4),vec3(0.0,0.15,0.20) );
    if( p.y>(5.0/7.0) ) col = pal( p.x, vec3(0.5,0.5,0.5),vec3(0.5,0.5,0.5),vec3(2.0,1.0,0.0),vec3(0.5,0.20,0.25) );
    if( p.y>(6.0/7.0) ) col = pal( p.x, vec3(0.8,0.5,0.4),vec3(0.2,0.4,0.2),vec3(2.0,1.0,1.0),vec3(0.0,0.25,0.25) );

   */
  if(fragCoord.y < 1) {
    col = palettesPal(t,Vec3(0.5,0.5,0.5),Vec3(0.5,0.5,0.5),Vec3(1.0,1.0,1.0),Vec3(0.0,0.33,0.67));
  } else if (fragCoord.y < 2) {
    col = palettesPal(t,Vec3(0.5,0.5,0.5),Vec3(0.5,0.5,0.5),Vec3(1.0,1.0,1.0),Vec3(0.0,0.10,0.20));
  } else if (fragCoord.y < 3) {
    col = palettesPal(t,Vec3(0.5,0.5,0.5),Vec3(0.5,0.5,0.5),Vec3(1.0,1.0,1.0),Vec3(0.3,0.20,0.20));
  } else if (fragCoord.y < 4) {
    col = palettesPal(t,Vec3(0.5,0.5,0.5),Vec3(0.5,0.5,0.5),Vec3(1.0,1.0,0.5),Vec3(0.8,0.90,0.30));
  } else if (fragCoord.y < 5) {
    col = palettesPal(t,Vec3(0.5,0.5,0.5),Vec3(0.5,0.5,0.5),Vec3(1.0,0.7,0.4),Vec3(0.0,0.15,0.20));
  } else if (fragCoord.y < 6) {
    col = palettesPal(t,Vec3(0.5,0.5,0.5),Vec3(0.5,0.5,0.5),Vec3(2.0,1.0,0.0),Vec3(0.5,0.20,0.25));
  } else if (fragCoord.y < 7) {
    col = palettesPal(t,Vec3(0.8,0.5,0.4),Vec3(0.2,0.4,0.2),Vec3(2.0,1.0,1.0),Vec3(0.0,0.25,0.25));
  } else {
    col = palettesPal(t,Vec3(0.5,0.5,0.5),Vec3(1.0,1.0,1.0),Vec3(0.5,0.5,0.5),Vec3(0.0,0.33,0.67));
  }
  

  return Vec4(col,1);
}

Vec4 plasma2D(float iGlobalTime, Vec2 fragCoord){
  // plasma : https://www.shadertoy.com/view/XdX3WN 
  Vec2 iResolution(LED_X,LED_Y);
  
  float pointRadius = 0.16;
  float linkSize = 0.14;
  float noiseStrength = 0.08; // range: 0-1
  
  float minDimension = min(iResolution.x, iResolution.y);
  Vec2 bounds = Vec2(iResolution.x / minDimension, iResolution.y / minDimension);
  Vec2 uv = Vec2(fragCoord.x / minDimension, fragCoord.y / minDimension);
  
  Vec3 pointR = Vec3(0.0, 0.0, 1.0);
  Vec3 pointG = Vec3(0.0, 0.0, 1.0);
  Vec3 pointB = Vec3(0.0, 0.0, 1.0);

  // Make the points orbit round the origin in 3 dimensions.
  // Coefficients are arbitrary to give different behaviours.
  // The Z coordinate should always be >0.0, as it's used directly to
  //  multiply the radius to give the impression of depth.
  pointR.x += 0.32 * sin(1.32 * iGlobalTime);
  pointR.y += 0.3 * sin(1.03 * iGlobalTime);
  pointR.z += 0.4 * sin(1.32 * iGlobalTime);
  
  pointG.x += 0.31 * sin(0.92 * iGlobalTime);
  pointG.y += 0.29 * sin(0.99 * iGlobalTime);
  pointG.z += 0.38 * sin(1.24 * iGlobalTime);
  
  pointB.x += 0.33 * sin(1.245 * iGlobalTime);
  pointB.y += 0.3 * sin(1.41 * iGlobalTime);
  pointB.z += 0.41 * sin(1.11 * iGlobalTime);
  
  // Centre the points in the display
  Vec2 midUV = Vec2(bounds.x * 0.5, bounds.y * 0.5);
  pointR.x += midUV.x;
  pointR.y += midUV.y;
  pointG.x += midUV.x;
  pointG.y += midUV.y;
  pointB.x += midUV.x;
  pointB.y += midUV.y;

  // Calculate the vectors from the current fragment to the coloured points
  Vec2 vecToR = Vec2(pointR.x - uv.x, pointR.y - uv.y);
  Vec2 vecToG = Vec2(pointG.x - uv.x, pointG.y - uv.y);
  Vec2 vecToB = Vec2(pointB.x - uv.x, pointB.y - uv.y);

  Vec2 dirToR = vecToR.norm();
  Vec2 dirToG = vecToG.norm();
  Vec2 dirToB = vecToB.norm();
  
  float distToR = vecToR.length();
  float distToG = vecToG.length();
  float distToB = vecToB.length();
  
  // Calculate the dot product between vectors from the current fragment to each pair
  //  of adjacent coloured points. This helps us determine how close the current fragment
  //  is to a link between points.
  float dotRG = dirToR.dot(dirToG);
  float dotGB = dirToG.dot(dirToB);
  float dotBR = dirToB.dot(dirToR);

  // Start with a bright coloured dot around each point
  Vec3 fragColor;
  fragColor.x = 1 - smoothstep(distToR, 0.0, pointRadius * pointR.z);
  fragColor.y = 1 - smoothstep(distToR, 0.0, pointRadius * pointG.z);
  fragColor.z = 1 - smoothstep(distToR, 0.0, pointRadius * pointB.z);

  // We want to show a coloured link between adjacent points.
  // Determine the strength of each link at the current fragment.
  // This tends towards 1.0 as the vectors to each point tend towards opposite directions.
  float linkStrengthRG = 1.0 - smoothstep(dotRG, -1.01, -1.0 + (linkSize * pointR.z * pointG.z));
  float linkStrengthGB = 1.0 - smoothstep(dotGB, -1.01, -1.0 + (linkSize * pointG.z * pointB.z));
  float linkStrengthBR = 1.0 - smoothstep(dotBR, -1.01, -1.0 + (linkSize * pointB.z * pointR.z));
  
  // If the current fragment is in a link, we need to know how much the
  //  linked points contribute of their colour.
  float sumDistRG = distToR + distToG;
  float sumDistGB = distToG + distToB;
  float sumDistBR = distToB + distToR;
  
  float contribRonRG = 1.0 - (distToR / sumDistRG);
  float contribRonBR = 1.0 - (distToR / sumDistBR);
  
  float contribGonRG = 1.0 - (distToG / sumDistRG);
  float contribGonGB = 1.0 - (distToG / sumDistGB);
  
  float contribBonGB = 1.0 - (distToB / sumDistGB);
  float contribBonBR = 1.0 - (distToB / sumDistBR);
  
  // Additively blend the link colours into the fragment.
  fragColor.x += (linkStrengthRG * contribRonRG) + (linkStrengthBR * contribRonBR);
  fragColor.y += (linkStrengthGB * contribGonGB) + (linkStrengthRG * contribGonRG);
  fragColor.z += (linkStrengthBR * contribBonBR) + (linkStrengthGB * contribBonGB);

  return Vec4(fragColor, 1);
}

float flamesnoise(Vec3 uv, float res){
  const Vec3 s = Vec3(1e0, 1e2, 1e3);
  
  //uv *= res;
  uv = uv * res;
  
  //vec3 uv0 = floor(mod(uv, res))*s;
  Vec3 uv0(floor(mod(uv.x,res)) * s.x, floor(mod(uv.y,res)) * s.y, floor(mod(uv.z,res)) * s.z);
  //vec3 uv1 = floor(mod(uv+vec3(1.), res))*s;
  Vec3 uv1(floor(mod(uv.x+1,res)) * s.x, floor(mod(uv.y,res)) * s.y, floor(mod(uv.z,res)) * s.z);
  
  //vec3 f = fract(uv); f = f*f*(3.0-2.0*f);
  Vec3 f(fract(uv.x), fract(uv.y), fract(uv.z));
  f = f * f * (f * -2.0 + 3.0);

  Vec4 v = Vec4(uv0.x+uv0.y+uv0.z, uv1.x+uv0.y+uv0.z,
              uv0.x+uv1.y+uv0.z, uv1.x+uv1.y+uv0.z);

  //vec4 r = fract(sin(v*1e-1)*1e3);
  Vec4 r(fract(sin(v.x*1e-1)*1e3), fract(sin(v.y*1e-1)*1e3), fract(sin(v.z*1e-1)*1e3), fract(sin(v.w*1e-1)*1e3));
  float r0 = mix(mix(r.x, r.y, f.x), mix(r.z, r.w, f.x), f.y);
  
  //r = fract(sin((v + uv1.z - uv0.z)*1e-1)*1e3);
  r = Vec4( fract(sin((v.x + uv1.z - uv0.z)*1e-1)*1e3),
            fract(sin((v.y + uv1.z - uv0.z)*1e-1)*1e3),
            fract(sin((v.z + uv1.z - uv0.z)*1e-1)*1e3),
            fract(sin((v.w + uv1.z - uv0.z)*1e-1)*1e3));
  float r1 = mix(mix(r.x, r.y, f.x), mix(r.z, r.w, f.x), f.y);
  
  return mix(r0, r1, f.z)*2.0-1.0;
}



Vec4 flame2D(float iGlobalTime, Vec2 fragCoord) {
  //vec2 p = -.5 + fragCoord.xy / iResolution.xy;
  //p.x *= iResolution.x/iResolution.y;
  Vec2 p(-0.6 + ((fragCoord.x + 1) / LED_X) * 1.2, -0.6 + ((fragCoord.y + 1) / LED_Y) * 1.2);
  
  float color = 3.0 - (3.0*(p * 2.0).length());
  
  Vec3 coord = Vec3(atan2(p.x,p.y)/6.2832+0.5, p.length()*0.4, 0.5);
  
  
  for(int i = 1; i <= 7; i++)
  {
    float power = pow(2.0, (float)i);
    color += (1.5 / power) * flamesnoise(coord + Vec3(0.0,-iGlobalTime*0.05, iGlobalTime*0.01), power*16.0);
  }
  
  Vec4 fragColor(color, pow(max(color,0.0),2.0)*0.4, pow(max(color,0.0),3.0)*0.15 , 1.0);

  return fragColor;
}



Vec4 sineBall2D(float iGlobalTime, Vec2 fragCoord) {
  // https://www.shadertoy.com/view/4dt3R2
  Vec2 uv = fragCoord / iResolution();
  Vec2 vc = (fragCoord - iResolution() * 0.5) / iResolution().y * 0.6;
  Vec3 color(uv, 0.5+0.5*sin(iGlobalTime));
  color = color * (0.6 - smoothstep(0.1, 0.8, vc.length()));
  Vec2 pos = uv * 2 - 1.0;
  color = color * abs(sin(pos.x + sin(pos.y + iGlobalTime) * 0.5) * 10);
  return Vec4(color,1);
}

Vec4 sinCosLight2D(float iGlobalTime, Vec2 fragCoord) {
  // sin-cos light https://www.shadertoy.com/view/Xdt3R2
  Vec2 uv = fragCoord / iResolution();
  
  Vec4 color = Vec4(uv,0.5+0.5*sin(iGlobalTime),1.0);
  
  Vec4 color2 = Vec4(1.0 - ((uv.x + uv.y) / 2.0),uv,1.0);
  
  Vec2 pos = uv*2.0-1.0;
  
  color = color * abs(1.0/(sin(pos.y + sin(pos.x + iGlobalTime)*0.7)*sin(iGlobalTime*0.5)*20.0));
  
  color = color + color2 * abs(1./(sin(pos.y + cos(pos.x*.5 + iGlobalTime)*0.8)*10.0));
  return color;
}


Vec4 hypnoRipples2D(float iGlobalTime, Vec2 fragCoord) {
  Vec2 center(0.5,0.5);
  float speed = 0.035;
  float invAr = iResolution().y / iResolution().x;
  Vec2 uv = fragCoord / iResolution();
    
  Vec3 col = Vec3(uv,0.5+0.5*sin(iGlobalTime));
   
  Vec3 texcol;
      
  float x = (center.x-uv.x);
  float y = (center.y-uv.y) * invAr;

  float r = -(x*x + y*y);
  float z = 1.0 + 0.5*sin((r+iGlobalTime*speed)/0.013);
  
  texcol.x = z;
  texcol.y = z;
  texcol.z = z;
  
  return Vec4(col*texcol,1.0);

}

Vec4 hypnoRings(float iGlobalTime, Vec2 fragCoord) {
  // https://www.shadertoy.com/view/Xsl3RX
  const float rings = 1.0;  //exactly the number of complete white rings at any moment.
  const float velocity=4.;  
  const float b = 0.01;   //size of the smoothed border
  Vec2 position = fragCoord/iResolution();
  float dist = (position - Vec2(0.5, 0.5)).length();
  float offset=iGlobalTime*velocity;
  float conv=rings*4.;
  float v=dist*conv-offset;
  float ringr=floor(v);
  float color=smoothstep(-b, b, abs(dist-(ringr+(float)(fract(v)>0.5)+offset)/conv));
  if(mod(ringr,2.)==1.0) color=1.0-color;
  return Vec4(color, color, color, 1.);
}

Vec4 colorCircles(float iGlobalTime, Vec2 fragCoord) {
  // color circles https://www.shadertoy.com/view/llfSzH
  Vec4 f(0,0,0,iGlobalTime);
  Vec3 r(cos (-f.w * 0.85), 1, sin (f.w));
  Vec2 v2 = (fragCoord + fragCoord - iResolution()) / iResolution().y;
  for (float i = .6 ; i > .1 ; i -= .01) {
    f.w = (r.map(fract) - .5).length() - .3;
    if (f.w > .05) {
      r = r + Vec3 (v2, 2) * .06 * f.w;
      f = Vec4(r * i + i, f.w);
    }
  }
  return f;
}

void glslWrap(uint16_t frame, Vec4 (*fun)(float, Vec2), float factor = 1) {
  float iGT = factor * (float) frame / (float) FRAME_RATE;
  cRGB v;
  for(int i = 0; i < LEDCount; i++){
    int x = i % LED_X;
    int y = i / LED_X;
    Vec2 coord(x,y);
    Vec4 color = fun(iGT, coord);
    v.r = clamp(color.x) * 128;
    v.g = clamp(color.y) * 128;
    v.b = clamp(color.z) * 128;
    LED.set_crgb_at(i,v);
  }
}

// dizzy circles https://www.shadertoy.com/view/ldsXWj
// toon cloud https://www.shadertoy.com/view/4t23RR

// old plasma https://www.shadertoy.com/view/MdXGDH
// ether https://www.shadertoy.com/view/MsjSW3


void loop() {
  // put your main code here, to run repeatedly:
  switch(anim) {
    case 0: rainbow(frame); break;
    case 1: glslWrap(frame, palettes2D, 10); break;
    case 2: glslWrap(frame, plasma2D, 10); break;
    case 3: glslWrap(frame, flame2D, 2); break;
    case 4: glslWrap(frame, sineBall2D, 2); break;
    case 5: glslWrap(frame, sinCosLight2D, 1); break;
    case 6: glslWrap(frame, hypnoRipples2D, 1); break;
    case 7: glslWrap(frame, hypnoRings, 1); break;
    case 8: glslWrap(frame, colorCircles, 5); break;
  }
  
  LED.sync();
  delay(DELAY_FRAME);
  if(frame >= framesPerAnim[anim]) {
    frame = 0;
    anim = (anim + 1) % ANIMATIONS;
  }
  frame++;
}
