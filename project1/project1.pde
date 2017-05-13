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
import java.util.Arrays;

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
color[]      userClr = new color[] { 
  color(255, 0, 0), 
  color(0, 255, 0), 
  color(0, 0, 255), 
  color(255, 255, 0), 
  color(255, 0, 255), 
  color(0, 255, 255)
};

int version = 1;

int jointNum = 15;
int pointNum = 800;

PVector[][][] pos = new PVector[6][jointNum][pointNum];
PVector[][][] v = new PVector[6][jointNum][pointNum];
float[][][] err = new float[6][jointNum][pointNum];
float[][][] w = new float[6][jointNum][pointNum];
float[][][] p = new float[6][jointNum][pointNum];
float[][][] c = new float[6][jointNum][pointNum];

ArrayList[][] trackParticles = new ArrayList[6][jointNum];
int[][] userColor = new int[6][jointNum];
boolean[] isEmit = new boolean[6];
int[] emissions = new int[6];

int[] userLimbOrder = {
  1, 2, 3, 4, 5, 6
};
int[][] limbOrder = new int[6][jointNum];


void setup()
{
  size(1440, 900, P3D);  // strange, get drawing error in the cameraFrustum if i use P3D, in opengl there is no problem
  context = new SimpleOpenNI(this);
  if (context.isInit() == false)
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

  stroke(255, 255, 255);
  smooth();  
  perspective(radians(45), 
  float(width)/float(height), 
  10, 150000);

  v1InitMovePoints();
  background(0, 0, 0);
}

void draw() {
  switch (version) {
  case 1:
    version1();
    break;
  case 2:
    version2();
    break;
  case 3:
    version3();
    break;
  default :
    version1();
    break;
  }
}


//  .oooooo..o oooooooooooo   .oooooo.   ooooooooooooo ooooo   .oooooo.   ooooo      ooo      .o 
// d8P'    `Y8 `888'     `8  d8P'  `Y8b  8'   888   `8 `888'  d8P'  `Y8b  `888b.     `8'    o888 
// Y88bo.       888         888               888       888  888      888  8 `88b.    8      888 
//  `"Y8888o.   888oooo8    888               888       888  888      888  8   `88b.  8      888 
//      `"Y88b  888    "    888               888       888  888      888  8     `88b.8      888 
// oo     .d8P  888       o `88b    ooo       888       888  `88b    d88'  8       `888      888 
// 8""88888P'  o888ooooood8  `Y8bood8P'      o888o     o888o  `Y8bood8P'  o8o        `8     o888o

void version1()
{
  // update the cam
  context.update();

  background(0, 0, 0);

  // set the scene pos
  translate(width/2, height/2, 0);
  rotateX(rotX);
  rotateY(rotY);
  scale(zoomF);
  scale(-1, 1);

  int[]   depthMap = context.depthMap();
  int[]   userMap = context.userMap();
  int     steps   = 3;  // to speed up the drawing, draw every third point
  int     index;
  PVector realWorldPoint;

  translate(0, 0, -1000);  // set the rotation center of the scene 1000 infront of the camera

  // draw the skeleton if it's available
  int[] userList = context.getUsers();
  for (int i=0; i<userList.length; i++)
  {
    if (context.isTrackingSkeleton(userList[i]))
      v1DrawSkeleton(userList[i]);
  }    

  if (frameCount % 600 == 0) {
    int[] users = Arrays.copyOf(userList, userList.length);
    v1Shuffle(users);
    for (int i=0; i<6; i++) {
      v1Shuffle(limbOrder[i]);
      if (i<users.length) {
        userLimbOrder[i] = users[i];
      } else {
        userLimbOrder[i] = i+1;
      }
    }
  }
}

// draw the skeleton with the selected joints
void v1DrawSkeleton(int userId)
{
  // to get the 3d joint data
  v1DrawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK, limbOrder[userId][0]);

  v1DrawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER, limbOrder[userId][1]);
  v1DrawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW, limbOrder[userId][2]);
  v1DrawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND, limbOrder[userId][3]);

  v1DrawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER, limbOrder[userId][4]);
  v1DrawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW, limbOrder[userId][5]);
  v1DrawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND, limbOrder[userId][6]);

  v1DrawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO, limbOrder[userId][7]);
  v1DrawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO, limbOrder[userId][8]);

  v1DrawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP, limbOrder[userId][9]);
  v1DrawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE, limbOrder[userId][10]);
  v1DrawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT, limbOrder[userId][11]);

  v1DrawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP, limbOrder[userId][12]);
  v1DrawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE, limbOrder[userId][13]);
  v1DrawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT, limbOrder[userId][14]);
}

void v1DrawLimb(int userId, int jointType1, int jointType2, int limb)
{
  PVector jointPos1 = new PVector();
  PVector jointPos2 = new PVector();
  float  confidence;

  PVector[] pos_ = pos[userLimbOrder[userId-1]][limb];
  PVector[] v_ = v[userLimbOrder[userId-1]][limb];
  float[] err_ = err[userLimbOrder[userId-1]][limb];
  float[] w_ = w[userLimbOrder[userId-1]][limb];
  float[] p_ = p[userLimbOrder[userId-1]][limb];
  float[] c_ = c[userLimbOrder[userId-1]][limb];

  // draw the joint position
  confidence = context.getJointPositionSkeleton(userId, jointType1, jointPos1);
  confidence = context.getJointPositionSkeleton(userId, jointType2, jointPos2);

  colorMode(HSB);
  strokeWeight(1);

  for (int i=0; i<pointNum; i++) {
    stroke(c_[i], 255, 255);
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
    c_[i] = map(v_[i].mag(), 0, 30, 180, 0);
  }
}

void v1InitMovePoints() {
  int[] order = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
  };
  for (int u=0; u<6; u++) {
    for (int i=0; i<jointNum; i++) {
      for (int j=0; j<pointNum; j++) {
        pos[u][i][j] = new PVector();
        v[u][i][j] = new PVector();
        pos[u][i][j].x = random(-width/2, width/2);
        pos[u][i][j].y = random(-height/2, height/2);
        pos[u][i][j].z = random(-500, 500);
        v[u][i][j].x = 0;
        v[u][i][j].y = 0;
        v[u][i][j].z = 0;
        err[u][i][j] = random(-10, 10) + random(-10, 10) + random(-10, 10) + random(-10, 10) + random(-10, 10);
        w[u][i][j] = random(0.5, 0.98);
        p[u][i][j] = random(20, 100);
        c[u][i][j] = 180;
      }
    }
    limbOrder[u] = Arrays.copyOf(order, order.length);
  }
}

void v1Shuffle(int[] array) {
  for (int i = 0; i < array.length; i++) {
    int dst = floor(random(1) * (i + 1));
    v1Swap(array, i, dst);
  }
}

void v1Swap(int[] array, int i, int j) {
  int tmp = array[i];
  array[i] = array[j];
  array[j] = tmp;
}


//  .oooooo..o oooooooooooo   .oooooo.   ooooooooooooo ooooo   .oooooo.   ooooo      ooo      .oooo.  
// d8P'    `Y8 `888'     `8  d8P'  `Y8b  8'   888   `8 `888'  d8P'  `Y8b  `888b.     `8'    .dP""Y88b 
// Y88bo.       888         888               888       888  888      888  8 `88b.    8           ]8P'
//  `"Y8888o.   888oooo8    888               888       888  888      888  8   `88b.  8         .d8P' 
//      `"Y88b  888    "    888               888       888  888      888  8     `88b.8       .dP'    
// oo     .d8P  888       o `88b    ooo       888       888  `88b    d88'  8       `888     .oP     .o
// 8""88888P'  o888ooooood8  `Y8bood8P'      o888o     o888o  `Y8bood8P'  o8o        `8     8888888888

void version2()
{
  // update the cam
  context.update();

  background(0, 0, 0);

  // set the scene pos
  translate(width/2, height/2, 0);
  rotateX(rotX);
  rotateY(rotY);
  scale(zoomF);
  scale(-1, 1);

  int[]   depthMap = context.depthMap();
  int[]   userMap = context.userMap();
  int     steps   = 3;  // to speed up the drawing, draw every third point
  int     index;
  PVector realWorldPoint;

  translate(0, 0, -1000);  // set the rotation center of the scene 1000 infront of the camera

  // draw the skeleton if it's available
  int[] userList = context.getUsers();
  for (int i=0; i<userList.length; i++)
  {
    if (context.isTrackingSkeleton(userList[i]))
      v2DrawSkeleton(userList[i]);
  }
}

// draw the skeleton with the selected joints
void v2DrawSkeleton(int userId)
{
  // to get the 3d joint data
  v2DrawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK, limbOrder[userId][0]);

  v2DrawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER, limbOrder[userId][1]);
  v2DrawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW, limbOrder[userId][2]);
  v2DrawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND, limbOrder[userId][3]);

  v2DrawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER, limbOrder[userId][4]);
  v2DrawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW, limbOrder[userId][5]);
  v2DrawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND, limbOrder[userId][6]);

  v2DrawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO, limbOrder[userId][7]);
  v2DrawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO, limbOrder[userId][8]);

  v2DrawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP, limbOrder[userId][9]);
  v2DrawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE, limbOrder[userId][10]);
  v2DrawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT, limbOrder[userId][11]);

  v2DrawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP, limbOrder[userId][12]);
  v2DrawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE, limbOrder[userId][13]);
  v2DrawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT, limbOrder[userId][14]);
}

void v2DrawLimb(int userId, int jointType1, int jointType2, int limb)
{
  PVector jointPos1 = new PVector();
  PVector jointPos2 = new PVector();
  float  confidence;

  PVector[] pos_ = pos[userId][limb];
  PVector[] v_ = v[userId][limb];
  float[] err_ = err[userId][limb];
  float[] w_ = w[userId][limb];
  float[] p_ = p[userId][limb];
  float[] c_ = c[userId][limb];

  // draw the joint position
  confidence = context.getJointPositionSkeleton(userId, jointType1, jointPos1);
  confidence = context.getJointPositionSkeleton(userId, jointType2, jointPos2);

  colorMode(HSB);
  strokeWeight(1);

  for (int i=0; i<pointNum; i++) {
    stroke(255 - c_[i], 255, 255, c_[i]);
    point(pos_[i].x, pos_[i].y, pos_[i].z);
    if (i%5 == 3) {
      line(pos_[i].x, pos_[i].y, pos_[i].z, pos_[i-1].x, pos_[i-1].y, pos_[i-1].z);
    }
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
    c_[i] = map(v_[i].mag(), 0, 30, 0, 255);
  }
}

void v2InitMovePoints() {
  int[] order = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14};
  for (int u=0; u<6; u++) {
    for (int i=0; i<jointNum; i++) {
      for (int j=0; j<pointNum; j++) {
        pos[u][i][j] = new PVector();
        v[u][i][j] = new PVector();
        pos[u][i][j].x = random(-width/2, width/2);
        pos[u][i][j].y = random(-height/2, height/2);
        pos[u][i][j].z = random(-500, 500);
        v[u][i][j].x = 0;
        v[u][i][j].y = 0;
        v[u][i][j].z = 0;
        err[u][i][j] = random(-50, 50);
        w[u][i][j] = random(0.5, 0.98);
        p[u][i][j] = random(20, 100);
        c[u][i][j] = 180;
      }
    }
    limbOrder[u] = Arrays.copyOf(order, order.length);
  }
}


//  .oooooo..o oooooooooooo   .oooooo.   ooooooooooooo ooooo   .oooooo.   ooooo      ooo      .oooo.  
// d8P'    `Y8 `888'     `8  d8P'  `Y8b  8'   888   `8 `888'  d8P'  `Y8b  `888b.     `8'    .dP""Y88b 
// Y88bo.       888         888               888       888  888      888  8 `88b.    8           ]8P'
//  `"Y8888o.   888oooo8    888               888       888  888      888  8   `88b.  8         <88b. 
//      `"Y88b  888    "    888               888       888  888      888  8     `88b.8          `88b.
// oo     .d8P  888       o `88b    ooo       888       888  `88b    d88'  8       `888     o.   .88P 
// 8""88888P'  o888ooooood8  `Y8bood8P'      o888o     o888o  `Y8bood8P'  o8o        `8     `8bd88P'  

void version3()
{
  // update the cam
  context.update();

  background(0, 0, 0);

  // set the scene pos
  translate(width/2, height/2, 0);
  rotateX(rotX);
  rotateY(rotY);
  scale(zoomF);
  scale(-1, 1);

  int[]   depthMap = context.depthMap();
  int[]   userMap = context.userMap();
  int     steps   = 3;  // to speed up the drawing, draw every third point
  int     index;
  PVector realWorldPoint;

  translate(0, 0, -1000);  // set the rotation center of the scene 1000 infront of the camera

  beginShape(POINTS);
  for (int y=0; y < context.depthHeight (); y+=steps)
  {
    for (int x=0; x < context.depthWidth (); x+=steps)
    {
      index = x + y * context.depthWidth();
      if (depthMap[index] > 0)
      { 
        // draw the projected point
        realWorldPoint = context.depthMapRealWorld()[index];
        if (userMap[index] == 0) {
        } else {
          stroke(150);
          vertex(realWorldPoint.x, realWorldPoint.y, realWorldPoint.z);
        }
      }
    }
  } 
  endShape();

  // draw the skeleton if it's available
  int[] userList = context.getUsers();
  for (int i=0; i<userList.length; i++)
  {
    if (context.isTrackingSkeleton(userList[i]))
      if (frameCount % 1 == 0) {
        v3TrackSkeleton(userList[i]);
      }
    v3DrawSkeleton(userList[i]);
  }
  rotY += 0.005f;
  //  rotX += 0.005f;
}

// draw the skeleton with the selected joints
void v3TrackSkeleton(int userId)
{
  // to get the 3d joint data
  v3Track(userId, SimpleOpenNI.SKEL_HEAD);
  v3Track(userId, SimpleOpenNI.SKEL_NECK);

  v3Track(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  v3Track(userId, SimpleOpenNI.SKEL_LEFT_ELBOW);
  v3Track(userId, SimpleOpenNI.SKEL_LEFT_HAND);

  v3Track(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  v3Track(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  v3Track(userId, SimpleOpenNI.SKEL_RIGHT_HAND);

  v3Track(userId, SimpleOpenNI.SKEL_TORSO);

  v3Track(userId, SimpleOpenNI.SKEL_LEFT_HIP);
  v3Track(userId, SimpleOpenNI.SKEL_LEFT_KNEE);
  v3Track(userId, SimpleOpenNI.SKEL_LEFT_FOOT);

  v3Track(userId, SimpleOpenNI.SKEL_RIGHT_HIP);
  v3Track(userId, SimpleOpenNI.SKEL_RIGHT_KNEE);
  v3Track(userId, SimpleOpenNI.SKEL_RIGHT_FOOT);
}

void v3DrawSkeleton(int userId) {
  ArrayList<PVector>[] tracker = trackParticles[userId];
  PVector left = new PVector();
  PVector right = new PVector();
  float  confidence;

  confidence = context.getJointPositionSkeleton(userId, 6, left);
  confidence = context.getJointPositionSkeleton(userId, 7, right);

  noFill();
  colorMode(HSB);
  for (int i=0; i<jointNum; i++) {
    if (i == 6 || i == 7) {
      boolean isSaved = true;
      beginShape(TRIANGLES);
      for (int j=tracker[i].size ()-1; j>3; j--) {
        if (j > 100 && isSaved) {
          if (PVector.dist(tracker[i].get(0), tracker[i].get(j)) < 140) {
            isSaved = true;
          } else {
            isSaved = false;
          }
        }
        int save = isSaved ? 0 : 124;
        stroke(userColor[userId-1][i], save, 255, j);
//        fill(userColor[userId-1][i], save, 255, 1.0*j/pointNum*30);
        vertex(tracker[i].get(j).x, tracker[i].get(j).y, tracker[i].get(j).z);
        vertex(tracker[i].get(j-3).x, tracker[i].get(j-3).y, tracker[i].get(j-3).z);
      }
      endShape();
      if (tracker[i].size()>200 && isSaved) {
        isEmit[userId-1] = true;
      }
      if (isEmit[userId-1]) {
        int emission = emissions[userId-1];
        beginShape(LINES);
        for (int j=tracker[i].size ()-1; j>3; j--) {
          stroke(userColor[userId-1][i], 0, 255, j);
          fill(userColor[userId-1][i], 255, 255, 1.0*j/pointNum*(30+emission));
          vertex(
          tracker[i].get(j).x + random(-1*emission, emission) + random(-1*emission, emission), 
          tracker[i].get(j).y + random(-1*emission, emission) + random(-1*emission, emission), 
          tracker[i].get(j).z + random(-1*emission, emission) + random(-1*emission, emission)
            );
          vertex(
          tracker[i].get(j-3).x + random(-1*emission, emission) + random(-1*emission, emission), 
          tracker[i].get(j-3).y + random(-1*emission, emission) + random(-1*emission, emission), 
          tracker[i].get(j-3).z + random(-1*emission, emission) + random(-1*emission, emission)
            );
        }
        endShape();
        emissions[userId-1] += 1;
      }
      if (emissions[userId-1] > 100) {
        isEmit[userId-1] = false;
        emissions[userId-1] = 1;
        userColor[userId-1][6] = int(random(255));
        userColor[userId-1][7] = int(random(255));
      }
    }
  }
}

void v3Track(int userId, int jointType) {
  PVector jointPos = new PVector();
  PVector trackPos1 = new PVector();
  PVector trackPos2 = new PVector();
  PVector trackPos3 = new PVector();
  float  confidence;
  ArrayList<PVector> tracker = trackParticles[userId][jointType];

  // draw the joint position
  confidence = context.getJointPositionSkeleton(userId, jointType, jointPos);

  if (tracker.size() > pointNum) {
    tracker.remove(0);
    tracker.remove(1);
    tracker.remove(2);
  }

  trackPos1 = new PVector(-1*random(-20, 20), random(-20, 20), random(-20, 20));
  trackPos2 = new PVector(random(-20, 20), -1*random(-20, 20), random(-20, 20));
  trackPos3 = new PVector(random(-20, 20), random(-20, 20), -1*random(-20, 20));
  trackPos1.add(jointPos);
  trackPos2.add(jointPos);
  trackPos3.add(jointPos);
  tracker.add(trackPos1);
  tracker.add(trackPos2);
  tracker.add(trackPos3);
}

void v3InitMovePoints() {
  for (int u=0; u<6; u++) {
    for (int i=0; i<jointNum; i++) {
      for (int j=0; j<pointNum; j++) {
        trackParticles[u][i] = new ArrayList<PVector>();
        userColor[u][i] = int(random(255));
        isEmit[u] = false;
        emissions[u] = 1;
      }
    }
  }
}


//  .oooooo..o oooooooooooo   .oooooo.   ooooooooooooo ooooo   .oooooo.   ooooo      ooo      oooooooo
// d8P'    `Y8 `888'     `8  d8P'  `Y8b  8'   888   `8 `888'  d8P'  `Y8b  `888b.     `8'     dP"""""""
// Y88bo.       888         888               888       888  888      888  8 `88b.    8     d88888b.  
//  `"Y8888o.   888oooo8    888               888       888  888      888  8   `88b.  8         `Y88b 
//      `"Y88b  888    "    888               888       888  888      888  8     `88b.8           ]88 
// oo     .d8P  888       o `88b    ooo       888       888  `88b    d88'  8       `888     o.   .88P 
// 8""88888P'  o888ooooood8  `Y8bood8P'      o888o     o888o  `Y8bood8P'  o8o        `8     `8bd88P'  

// -----------------------------------------------------------------
// SimpleOpenNI user events

void onNewUser(SimpleOpenNI curContext, int userId)
{
  println("onNewUser - userId: " + userId);
  println("\tstart tracking skeleton");

  context.startTrackingSkeleton(userId);
}

void onLostUser(SimpleOpenNI curContext, int userId)
{
  println("onLostUser - userId: " + userId);
}

void onVisibleUser(SimpleOpenNI curContext, int userId)
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
  case 'p':
    version = 1;
    pointNum = 800;
    v1InitMovePoints();
    break;
  case 'l':
    version = 2;
    pointNum = 800;
    v2InitMovePoints();
    break;
  case 'h':
    version = 3;
    pointNum = 400;
    v3InitMovePoints();
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
    if (keyEvent.isShiftDown())
      zoomF += 0.001f;
    else
      rotX += 0.01f;
    break;
  case DOWN:
    if (keyEvent.isShiftDown())
    {
      zoomF -= 0.001f;
      if (zoomF < 0.001)
        zoomF = 0.001;
    } else
      rotX -= 0.01f;
    break;
  }
}

