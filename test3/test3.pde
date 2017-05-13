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
float        zoomF = 0.3f;
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
int pointNum = 400;

ArrayList[][] trackParticles = new ArrayList[6][jointNum];
int[][] userColor = new int[6][jointNum];
boolean[] isEmit = new boolean[6];
int[] emissions = new int[6];

void setup()
{
  size(1400,1000,P3D);  // strange, get drawing error in the cameraFrustum if i use P3D, in opengl there is no problem
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
  background(0,0,0);
  println("SimpleOpenNI.SKEL_HEAD: "+SimpleOpenNI.SKEL_HEAD);
  println("SimpleOpenNI.SKEL_LEFT_HAND: "+SimpleOpenNI.SKEL_LEFT_HAND);
  println("SimpleOpenNI.SKEL_RIGHT_HAND: "+SimpleOpenNI.SKEL_RIGHT_HAND);
  println("SimpleOpenNI.SKEL_LEFT_FOOT: "+SimpleOpenNI.SKEL_LEFT_FOOT);
  println("SimpleOpenNI.SKEL_RIGHT_FOOT: "+SimpleOpenNI.SKEL_RIGHT_FOOT);
  blendMode(ADD);
 }

void draw()
{
  // update the cam
  context.update();

  background(0,0,0);
  
  // set the scene pos
  translate(width/2, height/2, 0);
  rotateX(rotX);
  rotateY(rotY);
  scale(zoomF);
  scale(-1,1);
  
  int[]   depthMap = context.depthMap();
  int[]   userMap = context.userMap();
  int     steps   = 3;  // to speed up the drawing, draw every third point
  int     index;
  PVector realWorldPoint;
 
  translate(0,0,-1000);  // set the rotation center of the scene 1000 infront of the camera
  
  beginShape(POINTS);
  for(int y=0;y < context.depthHeight();y+=steps)
  {
    for(int x=0;x < context.depthWidth();x+=steps)
    {
      index = x + y * context.depthWidth();
      if(depthMap[index] > 0)
      { 
        // draw the projected point
        realWorldPoint = context.depthMapRealWorld()[index];
        if(userMap[index] == 0){
        } else{
          stroke(150);
          vertex(realWorldPoint.x,realWorldPoint.y,realWorldPoint.z);        
        }
      }
    } 
  } 
  endShape();
  
  // draw the skeleton if it's available
  int[] userList = context.getUsers();
  for(int i=0;i<userList.length;i++)
  {
    if(context.isTrackingSkeleton(userList[i]))
      if (frameCount % 1 == 0) {
        trackSkeleton(userList[i]);
      }
      drawSkeleton(userList[i]);
  }
  rotY += 0.005f;
//  rotX += 0.005f;
}

// draw the skeleton with the selected joints
void trackSkeleton(int userId)
{
  // to get the 3d joint data
  track(userId, SimpleOpenNI.SKEL_HEAD);
  track(userId, SimpleOpenNI.SKEL_NECK);

  track(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  track(userId, SimpleOpenNI.SKEL_LEFT_ELBOW);
  track(userId, SimpleOpenNI.SKEL_LEFT_HAND);

  track(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  track(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  track(userId, SimpleOpenNI.SKEL_RIGHT_HAND);

  track(userId, SimpleOpenNI.SKEL_TORSO);

  track(userId, SimpleOpenNI.SKEL_LEFT_HIP);
  track(userId, SimpleOpenNI.SKEL_LEFT_KNEE);
  track(userId, SimpleOpenNI.SKEL_LEFT_FOOT);

  track(userId, SimpleOpenNI.SKEL_RIGHT_HIP);
  track(userId, SimpleOpenNI.SKEL_RIGHT_KNEE);
  track(userId, SimpleOpenNI.SKEL_RIGHT_FOOT);
}

void drawSkeleton(int userId) {
  ArrayList<PVector>[] tracker = trackParticles[userId];
  PVector left = new PVector();
  PVector right = new PVector();
  float  confidence;

  confidence = context.getJointPositionSkeleton(userId,6,left);
  confidence = context.getJointPositionSkeleton(userId,7,right);

  noFill();
  colorMode(HSB);
  for (int i=0; i<jointNum; i++) {
    if (i == 6 || i == 7) {
      boolean isSaved = true;
      beginShape(TRIANGLES);
      for (int j=tracker[i].size()-1; j>3; j--) {
        if (j > 100 && isSaved) {
          if (PVector.dist(tracker[i].get(0), tracker[i].get(j)) < 140) {
            isSaved = true;
          } else {
            isSaved = false;
          }
        }
        int save = isSaved ? 0 : 124;
        stroke(userColor[userId-1][i],save,255,j);
        fill(userColor[userId-1][i],save,255,1.0*j/pointNum*30);
        vertex(tracker[i].get(j).x, tracker[i].get(j).y, tracker[i].get(j).z);
        vertex(tracker[i].get(j-3).x, tracker[i].get(j-3).y, tracker[i].get(j-3).z);
      }
      endShape();
      if (tracker[i].size()>200 && isSaved) {
        isEmit[userId-1] = true;
      }
      if (isEmit[userId-1]) {
        int emission = emissions[userId-1];
        beginShape(TRIANGLES);
        for (int j=tracker[i].size()-1; j>3; j--) {
          stroke(userColor[userId-1][i],255,255,j);
          fill(userColor[userId-1][i],255,255,1.0*j/pointNum*(30+emission));
          vertex(
            tracker[i].get(j).x + random(-1*emission,emission),
            tracker[i].get(j).y + random(-1*emission,emission),
            tracker[i].get(j).z + random(-1*emission,emission)
          );
          vertex(
            tracker[i].get(j-3).x + random(-1*emission,emission),
            tracker[i].get(j-3).y + random(-1*emission,emission),
            tracker[i].get(j-3).z + random(-1*emission,emission)
          );
        }
        endShape();
        emissions[userId-1] += 1;
      }
      if (emissions[userId-1] > 200) {
        isEmit[userId-1] = false;
        emissions[userId-1] = 1;
        userColor[userId-1][6] = int(random(255));
        userColor[userId-1][7] = int(random(255));
      }
    }
  }
}

void track(int userId,int jointType) {
  PVector jointPos = new PVector();
  PVector trackPos1 = new PVector();
  PVector trackPos2 = new PVector();
  PVector trackPos3 = new PVector();
  float  confidence;
  ArrayList<PVector> tracker = trackParticles[userId][jointType];

  // draw the joint position
  confidence = context.getJointPositionSkeleton(userId,jointType,jointPos);

  if (tracker.size() > pointNum) {
    tracker.remove(0);
    tracker.remove(1);
    tracker.remove(2);
  }

  trackPos1 = new PVector(-1*random(-40,40), random(-40,40), random(-40,40));
  trackPos2 = new PVector(random(-40,40), -1*random(-40,40), random(-40,40));
  trackPos3 = new PVector(random(-40,40), random(-40,40), -1*random(-40,40));
  trackPos1.add(jointPos);
  trackPos2.add(jointPos);
  trackPos3.add(jointPos);
  tracker.add(trackPos1);
  tracker.add(trackPos2);
  tracker.add(trackPos3);
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

void initMovePoints() {
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
