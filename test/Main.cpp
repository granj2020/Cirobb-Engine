
/*************************************************************************
* Copyright (c) 2019-2021 Jonathan Peña
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Jonathan Peña makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty. 
**************************************************************************/


#include "..\Cirobb\Scene.h"
#include "..\freeglut\freeglut.h"
#include "Render.h"


int numScene = 1;
const real viewScale = 0.018f;
bool pause = false, InitScene = true;
const real fps = 1.0f / 60.0f;

Scene scene(Vec2(0, -8.0f), 12, 4);

Circle cPrincipal(1.8f); OBB bPrincipal(3.5f, 3.5f);

RigidBody* sPrincipal = new RigidBody(cPrincipal, Vec2(0, -20.0f), 0.0f); 

RigidBody* cLocal = new RigidBody(Circle(3.0f), Vec2(-4.6f, 0.0f), 0.0f);
RigidBody* bLocal = new RigidBody(OBB(7.0f, 5.0f), Vec2(3.6f, 0), -PI / 20.0f);

RigidBody* w1 = new RigidBody(OBB(0.8f, 10), Vec2(4.0f, 0), 0);
RigidBody* w2 = new RigidBody(OBB(0.8f, 10), Vec2(-4.0f, 0), 0);
RigidBody* w3 = new RigidBody(OBB(8.0f, 0.8f), Vec2(0, -5.0f), 0);
RigidBody* w4 = new RigidBody(OBB(8.0f, 0.8f), Vec2(0,  5.0f), 0);



static void DetectionCollision(RigidBody* p)
{
  DrawCircle(cLocal->shape);
  DrawObb(bLocal->shape);
  DrawShape(sPrincipal->shape);

  Manifold m1(p, cLocal);
  Manifold m2(p, bLocal);
  
  if(m1.numContacts == 1) 
  {
    DrawPoint(m1.contacts[0].position, 2, 4);
    Vec2 value = m1.normal * m1.A->shape->radius; 
    DrawLine(m1.A->position + value , m1.A->position + value + m1.normal * m1.contacts[0].penetration);
  }
  
  if(m2.numContacts > 0) 
  {
    for(int i = 0; i < m2.numContacts; i++)
    {
      DrawPoint(m2.contacts[i].position, 2, 4);
      DrawLine(m2.contacts[i].position, m2.contacts[i].position - m2.normal * m2.contacts[i].penetration);
    }
  }
}



static void LowFriction(void)
{
  RigidBody* b1 = new RigidBody(OBB(22.0f, 0.4f), Vec2(0.0f, -4.0f), 0.0f); 
  b1->Static();
  
  OBB w2(10.0, 0.4f);
  
  RigidBody* b2 = new RigidBody(w2, Vec2(-5.0f, 4.0f), -15 * RAD); 
  b2->Static();
  
  RigidBody* b3 = new RigidBody(w2, Vec2(5.0f, 2.0f), 15 * RAD); 
  b3->Static();
  
  RigidBody* b4 = new RigidBody(w2, Vec2(-5.0f, 0), -15 * RAD); 
  b4->Static();
  
  RigidBody* b5 = new RigidBody(Circle(0.4f), Vec2(8.72f, 5.32f), 0.0f); 
  b5->Dynamic(1.0f);
  b5->angularDamping = 0.1f;
  
  RigidBody* b6 = new RigidBody(OBB(0.8f, 0.8f), Vec2(-9.7f, 5.7f), -15 * RAD);
  b6->Dynamic(1.0f);
  b6->friction = 0.01f; 
  
  scene.Add(b1);
  scene.Add(b2);
  scene.Add(b3);
  scene.Add(b4);
  scene.Add(b5);
  scene.Add(b6);	
}



static void Stacking(void)
{
  RigidBody* b1 = new RigidBody(OBB(22.0f, 0.4f), Vec2(0, -4.0), 0.0f); 
  b1->Static();
  scene.Add(b1);
  
  for(int i = 0; i < 10; i++)
  {
    Circle cir(0.4f);  OBB obb(0.8f, 0.8f);
    
    real x = -3.6f;
    real y = -3.4f + i * 0.8f;

    RigidBody* b2 = new RigidBody(obb, Vec2(x, y), 0.0f); 
    b2->Dynamic(1.0f);
    
    RigidBody* b3 = new RigidBody(cir, Vec2(0, y), 0.0f); 
    b3->Dynamic(1.0f);	
    
    RigidBody* b4 = nullptr;
    
    if(i & 1) // (i & 1) == (i % 2 == 0)
    {
      b4 = new RigidBody(obb, Vec2(-x, y), 0.0f); 
      b4->Dynamic(1.0f);	
    }
    else
    {
      b4 = new RigidBody(cir, Vec2(-x, y), 0.0f); 
      b4->Dynamic(1.0f);	
    }
    
    scene.Add(b2);
    scene.Add(b3);
    scene.Add(b4);
  }
}



static void Pyramid(void)
{
  RigidBody* b1 = new RigidBody(OBB(22.0f, 0.4f), Vec2(0, -4.0), 0.0f); 
  b1->Static();
  scene.Add(b1);
  
  int lv = 10; real wh = 0.8f;
  
  for(int i = 0; i < lv; i++)
  {
    for(int j = 0; j < lv - i; j++)
    {
      real x = (j * 2 + i - lv) * wh * 0.5f;
      real y = (i + 0.5f) * wh - 3.8f;
      
      RigidBody* b2 = new RigidBody(OBB(wh, wh), Vec2(x, y) , 0); 
      b2->Dynamic(1.0f);
      scene.Add(b2);
    }
  }
}



static void AngryBirds(void)
{
  RigidBody* b1 = new RigidBody(OBB(22.0f, 0.4f), Vec2(0, -4.0f), 0.0f); 
  b1->Static();
  
  OBB w2(2.0f, 0.6f);
  OBB w3(4.3f, 0.4f);

  RigidBody* b2 = new RigidBody(w2, Vec2(-3.0f, -3.43f), 20 * RAD); 
  b2->Static();
  
  RigidBody* b4 = new RigidBody(w2, Vec2( 3.0f, -3.43f), -20 * RAD); 
  b4->Static();
  
  RigidBody* b3 = new RigidBody(w3, Vec2(0, -3.0f), 0.0f); 
  b3->Static();
  
  scene.Add(b1);
  scene.Add(b2);
  scene.Add(b3);
  scene.Add(b4);
  
  int lv = 5, sn = 3;
  
  for(int i = 0; i < sn; i++) // Column
  {
    for(int j = 0; j < lv; j++) // Levels
    {
      OBB wo(0.16f, 1.0f);

      real x = 1.6f  * i;
      real y = 1.16f * j - 2.3f;

      RigidBody* b5 = new RigidBody(wo, Vec2(x - 2.0f, y), 0.0f); 
      b5->Dynamic(1.0f);
      scene.Add(b5);
      
      RigidBody* b6 = new RigidBody(wo, Vec2(x - 1.16f, y), 0.0f); 
      b6->Dynamic(1.0);
      scene.Add(b6);
      
      RigidBody* b7 = new RigidBody(wo, Vec2(x - 1.58f, y + 0.6f), PI * 0.5f); // 90 degrees 
      b7->Dynamic(1.0f);		
      scene.Add(b7);
    }
  }
}




// this physics engine does not support rotation away from the local center of mass.
// So this is a special scene that was built to roughly rotate 4 OBBs away from the local center of mass.
static void RotaryBox(void)
{
  real angularVelocity = RAD * 0.2f / fps; // Angular Velocity
  
  Mat2 rot(w1->orientation);
  
  w1->angularVelocity = w2->angularVelocity = angularVelocity; 
  w3->angularVelocity = w4->angularVelocity = angularVelocity;
  
  w1->velocity = rot.Rotate(Cross(Vec2(4.0f, 0),-angularVelocity));
  w2->velocity = rot.Rotate(Cross(Vec2(4.0f, 0), angularVelocity));
  w3->velocity = rot.Rotate(Cross(Vec2(0, 5.0f), angularVelocity));
  w4->velocity = rot.Rotate(Cross(Vec2(0, 5.0f),-angularVelocity));
}





static void FreeStyle(void)
{
  RigidBody* b1 = new RigidBody(OBB(22.0f, 0.4f), Vec2(0, -4.0f), 0.0f); 
  b1->Static();
  
  OBB w2(0.8f, 10.0f);
  
  RigidBody* b2 = new RigidBody(w2, Vec2(-12.0f, 0.8f), 10 * RAD);
  b2->Static();
  
  RigidBody* b3 = new RigidBody(w2, Vec2(12.0f, 0.8f), -10 * RAD);
  b3->Static();
  
  scene.Add(b1);
  scene.Add(b2);
  scene.Add(b3);
}


static void (*TestScene[])(void) = {LowFriction, Stacking, Pyramid, AngryBirds, RotaryBox, FreeStyle};


char* SceneStrings[] = 
{
  "Scene 1-7: Collision Detection", 
  "Scene 2-7: Low Friction",
  "Scene 3-7: Stacking Boxes & Circles",
  "Scene 4-7: Pyramid", 
  "Scene 5-7: Angry Bird Style",
  "Scene 6-7: Rotary Box",
  "Scene 7-7: Free Style"
};


char* PositionStrings[] = 
{
  "Position Correction: None",
  "Position Correction: Baumgarte Stabilization", 
  "Position Correction: Non-Linear-Gauss-Seidel"
};


static void Start(void)
{
  if(InitScene)
  {
    InitScene = false;
    
    scene.Clear();
    
    sPrincipal->Dynamic(0.5f);
    
    scene.Add(sPrincipal);
    
    if(numScene == 6) // Special Scene 6
    {
      w1->Static();
      w2->Static();
      w3->Static();
      w4->Static();
      
      scene.Add(w1);
      scene.Add(w2);
      scene.Add(w3);
      scene.Add(w4);
    }
    
    if(numScene > 1) TestScene[numScene - 2]();
  }
}



static void Update(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
  
  DrawTexts(2, 16, SceneStrings[numScene - 1]);
  DrawTexts(2, 44, PositionStrings[Scene::CorrectionType]);
  DrawTexts(2, 72, pause ? "(P)AUSE: TRUE" : "(P)AUSE: FALSE");
  
  if(numScene == 1)
  {
    DetectionCollision(sPrincipal);
  }
  else
  {
    if(!pause) scene.Step(fps);

    if(numScene == 6) RotaryBox();

    for(RigidBody* temp : scene.bodies) DrawShape(temp->shape);
    
    for(auto temp = scene.manifolds.begin(); temp != scene.manifolds.end(); temp++)
    {
      for(int i = 0; i < temp->second.numContacts; i++)
      {
        DrawPoint(temp->second.contacts[i].position,  2, 3);
        DrawPoint(temp->second.contacts[i].warmPoint, 1, 7);
      }
    }
  }
  
  glutSwapBuffers();
}



static void MouseMotion(int x, int y)
{
  real _x = (x - glutGet(GLUT_WINDOW_WIDTH)  * 0.5f) *  viewScale;
  real _y = (y - glutGet(GLUT_WINDOW_HEIGHT) * 0.5f) * -viewScale;

  sPrincipal->position.Set(_x, _y);
  sPrincipal->velocity.SetZero();
  sPrincipal->angularVelocity = 0.0f;
}



static void Keyboard(unsigned char key, int x, int y)
{
  real _x = (x - glutGet(GLUT_WINDOW_WIDTH)  * 0.5f) *  viewScale;
  real _y = (y - glutGet(GLUT_WINDOW_HEIGHT) * 0.5f) * -viewScale;
  
  switch (key)
  {
    case 'q': 
    case 'Q': sPrincipal->orientation += PI / 72;
    break;
    
    case 'e': 
    case 'E': sPrincipal->orientation -= PI / 72;
    break;
    
    case 'b': 
    case 'B':
    {
      OBB temp(Vec2(0.8f, 0.8f));
      RigidBody* b = new RigidBody(temp, Vec2(_x, _y), 0);
      b->Dynamic(1.0f);
      scene.Add(b);
    }
    break;
    
    case 'c':
    case 'C':
    {
      Circle temp(0.4f);
      RigidBody* b = new RigidBody(temp, Vec2(_x, _y), 0.0f);
      b->Dynamic(1.0f);
      scene.Add(b);
    }
    break;
    
    case 'w':
    case 'W':
    {
      if(sPrincipal->shape->type == circle)
      { 
        sPrincipal->shape = &bPrincipal;
      }
      else
      { 
        sPrincipal->shape = &cPrincipal;
      }
      sPrincipal->shape->body = sPrincipal;
      sPrincipal->Dynamic(0.5f);
    }
    break;
    
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    {
      InitScene = true;
      numScene = int(key) - 48; 
      glutPostRedisplay();
    }
    break;
    
    case 'p':
    case 'P':
    pause = !pause;
    break;
    case char(32): // SPACE
    {
      Scene::CorrectionType = Scene::CorrectionType > 1 ? 0 : Scene::CorrectionType + 1; 
    }
    break;
  }
}



static void Reshape(int width, int height)
{
  if(height == 0) height = 1;
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluOrtho2D(0, width * viewScale, 0, height * viewScale);
  glTranslatef(width * viewScale * 0.5f, height * viewScale * 0.5f, 0);
}



int main(int args, char** argv)
{	
  glutInit(&args, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(900, 700);
  glutCreateWindow("Ciroob Engine V1.1.6");
  glutDisplayFunc(Start);
  glutIdleFunc(Update);
  glutReshapeFunc(Reshape);
  glutMotionFunc(MouseMotion);
  glutKeyboardFunc(Keyboard);
  glutMainLoop();
}
