/* --------------------------------------------------------------------------
 * SimpleOpenNI User3d Test
 * --------------------------------------------------------------------------
 * Processing Wrapper for the OpenNI/Kinect 2 library
 * http://code.google.com/p/simple-openni
 * --------------------------------------------------------------------------
 * prog:  Max Rheiner / Interaction Design / Zhdk / http://iad.zhdk.ch/
 * date:  12/12/2012 (m/d/y)
 * ----------------------------------------------------------------------------
 */
 
import SimpleOpenNI.*;


SimpleOpenNI context;
float        zoomF =0.5f;
float        rotX = radians(180);  // by default rotate the hole scene 180deg around the x-axis, 
                                   // the data from openni comes upside down
float        rotY = radians(0);
boolean      autoCalib=true;

PVector      bodyCenter = new PVector();
PVector      bodyDir = new PVector();
PVector      com = new PVector();                                   
PVector      com2d = new PVector();                                   
color[]      userClr = new color[]{ color(255,0,0),
                                    color(0,255,0),
                                    color(0,0,255),
                                    color(255,255,0),
                                    color(255,0,255),
                                    color(0,255,255)
                                  };
int jointNum = 15;
int pointNum = 800;
PVector[][][] pos = new PVector[6][jointNum][pointNum];
PVector[][][] v = new PVector[6][jointNum][pointNum];
float[][][] err = new float[6][jointNum][pointNum];
float[][][] w = new float[6][jointNum][pointNum];
float[][][] p = new float[6][jointNum][pointNum];
int[] limbOrder = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14};


void setup()
{
  size(1024,768,P3D);  // strange, get drawing error in the cameraFrustum if i use P3D, in opengl there is no problem
  context = new SimpleOpenNI(this);
  if(context.isInit() == false)
  {
     println("Can't init SimpleOpenNI, maybe the camera is not connected!"); 
     exit();
     return;  
  }

  // disable mirror
  context.setMirror(false);

  // enable depthMap generation 
  context.enableDepth();

  // enable skeleton generation for all joints
  context.enableUser();

  stroke(255,255,255);
  smooth();  
  perspective(radians(45),
              float(width)/float(height),
              10,150000);

  initMovePoints();
//  blendMode(ADD);
  background(0,0,0);
 }

void draw()
{
  // update the cam
  context.update();

  background(0,0,0);
  // blendMode(DARKEST);
  // fill(255, 50);
  // rect(0, 0, width, height);
  // blendMode(ADD);
  
  // set the scene pos
  translate(width/2, height/2, 0);
  rotateX(rotX);
  rotateY(rotY);
  scale(zoomF);
  
  int[]   depthMap = context.depthMap();
  int[]   userMap = context.userMap();
  int     steps   = 3;  // to speed up the drawing, draw every third point
  int     index;
  PVector realWorldPoint;
 
  translate(0,0,-1000);  // set the rotation center of the scene 1000 infront of the camera

  // draw the pointcloud
  // beginShape(POINTS);
  // for(int y=0;y < context.depthHeight();y+=steps)
  // {
  //   for(int x=0;x < context.depthWidth();x+=steps)
  //   {
  //     index = x + y * context.depthWidth();
  //     if(depthMap[index] > 0)
  //     { 
  //       // draw the projected point
  //       realWorldPoint = context.depthMapRealWorld()[index];
  //       if(userMap[index] == 0)
  //         stroke(100); 
  //       else
  //         stroke(userClr[ (userMap[index] - 1) % userClr.length ]);        
        
  //       point(realWorldPoint.x,realWorldPoint.y,realWorldPoint.z);
  //     }
  //   } 
  // } 
  // endShape();
  
  // draw the skeleton if it's available
  int[] userList = context.getUsers();
  for(int i=0;i<userList.length;i++)
  {
    if(context.isTrackingSkeleton(userList[i]))
      drawSkeleton(userList[i]);
    
    // draw the center of mass
    // if(context.getCoM(userList[i],com))
    // {
      // stroke(100,255,0);
      // strokeWeight(1);
      // beginShape(LINES);
      //   vertex(com.x - 15,com.y,com.z);
      //   vertex(com.x + 15,com.y,com.z);
        
      //   vertex(com.x,com.y - 15,com.z);
      //   vertex(com.x,com.y + 15,com.z);

      //   vertex(com.x,com.y,com.z - 15);
      //   vertex(com.x,com.y,com.z + 15);
      // endShape();
      
      // fill(0,255,100);
      // text(Integer.toString(userList[i]),com.x,com.y,com.z);
    // }      
  }    
 
  // draw the kinect cam
  // context.drawCamFrustum();
  if (frameCount % 300 == 0) {
    shuffle(limbOrder);
  }
}

// draw the skeleton with the selected joints
void drawSkeleton(int userId)
{
  // to get the 3d joint data
  drawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK, limbOrder[0]);

  drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER, limbOrder[1]);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW, limbOrder[2]);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND, limbOrder[3]);

  drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER, limbOrder[4]);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW, limbOrder[5]);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND, limbOrder[6]);

  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO, limbOrder[7]);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO, limbOrder[8]);

  drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP, limbOrder[9]);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE, limbOrder[10]);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT, limbOrder[11]);

  drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP, limbOrder[12]);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE, limbOrder[13]);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT, limbOrder[14]); 

  // drawPoints(userId, SimpleOpenNI.SKEL_HEAD);
  // drawPoints(userId, SimpleOpenNI.SKEL_NECK);

  // drawPoints(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  // drawPoints(userId, SimpleOpenNI.SKEL_LEFT_ELBOW);
  // drawPoints(userId, SimpleOpenNI.SKEL_LEFT_HAND);

  // drawPoints(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  // drawPoints(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  // drawPoints(userId, SimpleOpenNI.SKEL_RIGHT_HAND);

  // drawPoints(userId, SimpleOpenNI.SKEL_TORSO);

  // drawPoints(userId, SimpleOpenNI.SKEL_LEFT_HIP);
  // drawPoints(userId, SimpleOpenNI.SKEL_LEFT_KNEE);
  // drawPoints(userId, SimpleOpenNI.SKEL_LEFT_FOOT);

  // drawPoints(userId, SimpleOpenNI.SKEL_RIGHT_HIP);
  // drawPoints(userId, SimpleOpenNI.SKEL_RIGHT_KNEE); 
  // drawPoints(userId, SimpleOpenNI.SKEL_RIGHT_FOOT); 

  // draw body direction
  getBodyDirection(userId,bodyCenter,bodyDir);
  
  bodyDir.mult(200);  // 200mm length
  bodyDir.add(bodyCenter);
  
  stroke(255,200,200);
  // line(bodyCenter.x,bodyCenter.y,bodyCenter.z,
       // bodyDir.x ,bodyDir.y,bodyDir.z);

  strokeWeight(1);
 
}

void drawLimb(int userId,int jointType1,int jointType2, int limb)
{
  PVector jointPos1 = new PVector();
  PVector jointPos2 = new PVector();
  float  confidence;

  PVector[] pos_ = pos[userId][limb];
  PVector[] v_ = v[userId][limb];
  float[] err_ = err[userId][limb];
  float[] w_ = w[userId][limb];
  float[] p_ = p[userId][limb];
  
  // draw the joint position
  confidence = context.getJointPositionSkeleton(userId,jointType1,jointPos1);
  confidence = context.getJointPositionSkeleton(userId,jointType2,jointPos2);

  stroke(20,20,255);
  strokeWeight(1);

  for (int i=0; i<pointNum; i++) {
    point(pos_[i].x, pos_[i].y, pos_[i].z);
  }

  for (int i=0; i<pointNum; i++) {
    pos_[i].x = pos_[i].x + v_[i].x;
    pos_[i].y = pos_[i].y + v_[i].y;
    pos_[i].z = pos_[i].z + v_[i].z;
  }

  for (int i=0; i<pointNum; i++) {
    v_[i].x = w_[i] * v_[i].x + ((jointPos1.x + (jointPos2.x - jointPos1.x)/pointNum*i + err_[i]) - pos_[i].x)/p_[i];
    v_[i].y = w_[i] * v_[i].y + ((jointPos1.y + (jointPos2.y - jointPos1.y)/pointNum*i + err_[i]) - pos_[i].y)/p_[i];
    v_[i].z = w_[i] * v_[i].z + ((jointPos1.z + (jointPos2.z - jointPos1.z)/pointNum*i + err_[i]) - pos_[i].z)/p_[i];
  }
}

void drawPoints(int userId,int jointPoint) {
  PVector jointPos = new PVector();
  float  confidence;
  PVector[] pos_ = pos[userId][jointPoint];
  PVector[] v_ = v[userId][jointPoint];
  float[] err_ = err[userId][jointPoint];
  float[] w_ = w[userId][jointPoint];
  float[] p_ = p[userId][jointPoint];

  confidence = context.getJointPositionSkeleton(userId,jointPoint,jointPos);

  stroke(50,50,255,confidence * 200 + 55);
  strokeWeight(10);

  for (int i=0; i<pointNum; i++) {
    point(pos_[i].x, pos_[i].y, pos_[i].z);
  }

  for (int i=0; i<pointNum; i++) {
    pos_[i].x = pos_[i].x + v_[i].x;
    pos_[i].y = pos_[i].y + v_[i].y;
    pos_[i].z = pos_[i].z + v_[i].z;
  }

  for (int i=0; i<pointNum; i++) {
    v_[i].x = w_[i] * v_[i].x + (jointPos.x - pos_[i].x)/p_[i];
    v_[i].y = w_[i] * v_[i].y + (jointPos.y - pos_[i].y)/p_[i];
    v_[i].z = w_[i] * v_[i].z + (jointPos.z - pos_[i].z)/p_[i];
  }
}

// -----------------------------------------------------------------
// SimpleOpenNI user events

void onNewUser(SimpleOpenNI curContext,int userId)
{
  println("onNewUser - userId: " + userId);
  println("\tstart tracking skeleton");
  
  context.startTrackingSkeleton(userId);
}

void onLostUser(SimpleOpenNI curContext,int userId)
{
  println("onLostUser - userId: " + userId);
}

void onVisibleUser(SimpleOpenNI curContext,int userId)
{
  //println("onVisibleUser - userId: " + userId);
}


// -----------------------------------------------------------------
// Keyboard events

void keyPressed()
{
  switch(key)
  {
  case ' ':
    context.setMirror(!context.mirror());
    break;
  }
    
  switch(keyCode)
  {
    case LEFT:
      rotY += 0.01f;
      break;
    case RIGHT:
      // zoom out
      rotY -= 0.01f;
      break;
    case UP:
      if(keyEvent.isShiftDown())
        zoomF += 0.001f;
      else
        rotX += 0.01f;
      break;
    case DOWN:
      if(keyEvent.isShiftDown())
      {
        zoomF -= 0.001f;
        if(zoomF < 0.001)
          zoomF = 0.001;
      }
      else
        rotX -= 0.01f;
      break;
  }
}

void getBodyDirection(int userId,PVector centerPoint,PVector dir)
{
  PVector jointL = new PVector();
  PVector jointH = new PVector();
  PVector jointR = new PVector();
  float  confidence;
  
  // draw the joint position
  confidence = context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_SHOULDER,jointL);
  confidence = context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_HEAD,jointH);
  confidence = context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_SHOULDER,jointR);
  
  // take the neck as the center point
  confidence = context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_NECK,centerPoint);
  
  /*  // manually calc the centerPoint
  PVector shoulderDist = PVector.sub(jointL,jointR);
  centerPoint.set(PVector.mult(shoulderDist,.5));
  centerPoint.add(jointR);
  */
  
  PVector up = PVector.sub(jointH,centerPoint);
  PVector left = PVector.sub(jointR,centerPoint);
    
  dir.set(up.cross(left));
  dir.normalize();
}

void initMovePoints() {
  for (int u=0; u<6; u++) {
    for (int i=0; i<jointNum; i++) {
      for (int j=0; j<pointNum; j++) {
        pos[u][i][j] = new PVector();
        v[u][i][j] = new PVector();
        pos[u][i][j].x = random(0, width);
        pos[u][i][j].y = random(0, height);
        pos[u][i][j].z = random(0, 1000);
        v[u][i][j].x = 0;
        v[u][i][j].y = 0;
        v[u][i][j].z = 0;
        err[u][i][j] = random(-10, 10);
        w[u][i][j] = random(0.5, 0.98);
        p[u][i][j] = random(20, 100);
      }
    }
  }
}

void shuffle(int[] array) {
  for (int i = 0; i < array.length; i++) {
    int dst = floor(random(1) * (i + 1));
    swap(array, i, dst);
  }
}

void swap(int[] array, int i, int j) {
  int tmp = array[i];
  array[i] = array[j];
  array[j] = tmp;
}
