/*************************************************************************
* Copyright (c) 2019-2021 Jonathan Peña
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Jonathan Peña makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty. 
**************************************************************************/

#include "Shape.h"					

/**********************************************************************************************************************
* Although there are phyisics engines that add the masses manually to avoid certain problems such as: Low convergence of the solver
 when a heavy object is on top of a light object. The mass can also be proportional to the geometric Area. For example:

* Circle Area = Radius * PI * PI 

* Rectangle Area = width * height 

These are the two geometric Areas that this physics engine uses to Obtain the relative masses of the shapes.

*There is also the Inertia tensor: The Inertia tensor reflects the mass distribution of a Body or a system of rotating particles,
It can also be said that it's the resistance of the body to the Rotation movement.

inertia Tensor matrix = |X  0  0|
                        |0  Y  0|
                        |0  0  Z|

2D physics engines have 3 Degrees of Freedom 2 of translation [X, Y, 0] and 1 of Rotation [0, 0, Z]. It Rotates on the Z Axis.

inertia Tensor in 2D is a Scalar.

Equation: Inertia = mass * Distance^2

1- Inertia Tensor of a Circle = mass * Radius * Radius / 2.0

2- Inertia Tensor of a Rectangle = mass * (x * x + y * y) / 12.0

***********************************************************************************************************************/
void Circle::CalculateMassInertia(const real& density)
{
  body->m = PI * radius * radius * density;
  body->invm = body->m ? 1.0f / body->m : 0.0f;
  body->I = body->m * radius * radius * 0.5f;
  body->invI = body->I ? 1.0f / body->I : 0.0f;
}



void OBB::CalculateMassInertia(const real& density)
{
  body->m = width.x * width.y * density;
  body->invm = body->m ? 1.0f / body->m : 0.0f;
  body->I = body->m * width.SquareMagnitude() / 12.0f; 
  body->invI = body->I ? 1.0f / body->I : 0.0f;
}



void Circle::DrawShape(void)
{
  const int  countVerts = 40; // Number of Vertices
  const real valueVerts = PI * 2 / countVerts;
  Vec2 verts[countVerts + 1];
  
  for(int i = 0; i <= countVerts; i++) // RADIAN [0, 2PI] --> DEGREE [0, 360]
  {
    verts[i].x = radius * cosf(valueVerts * i); 
    verts[i].y = radius * sinf(valueVerts * i); 
  }
  
  glPushMatrix();
  glTranslatef(body->position.x, body->position.y, 0.0f);	
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_COLOR, GL_ONE_MINUS_SRC_ALPHA);
  glColor4f(0.4f, 0.4f, 0.4f, 0.4f);
  
  glBegin(GL_POLYGON);
  for(int i = 0; i <= countVerts; i++)
  {
    glVertex2f(verts[i].x, verts[i].y);
  }
  glEnd();
  
  glDisable(GL_BLEND);
  
  glColor3f(0.65f, 0.65f, 0.65f);
  
  glBegin(GL_LINE_STRIP);
  for(int i = 0; i <= countVerts; i++)
  {
    glVertex2f(verts[i].x, verts[i].y);
  }
  glEnd();
  
  glBegin(GL_LINES);
  glVertex2f(0, 0);
  glVertex2f(radius * cosf(body->orientation), radius * sinf(body->orientation));
  glEnd();
  
  glColor3f(1.0f, 1.0f, 1.0f);
  
  glPopMatrix();
}



void OBB::DrawShape(void)
{
  glPushMatrix();
  glTranslatef(body->position.x, body->position.y, 0.0f);
  Mat2 Rot(body->orientation);
  Vec2 p1 = Rot * width * 0.5f;
  Vec2 p2 = Rot * Vec2(width.x, -width.y) * 0.5f;	
  
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_COLOR, GL_ONE_MINUS_SRC_ALPHA);
  glColor4f(0.4f, 0.4f, 0.4f, 0.4f);
  glBegin(GL_POLYGON);
  glVertex2f( p1.x, p1.y);
  glVertex2f( p2.x, p2.y);
  glVertex2f(-p1.x,-p1.y);
  glVertex2f(-p2.x,-p2.y);
  glEnd();
  glDisable(GL_BLEND);
  
  glColor3f(0.65f, 0.65f, 0.65f);
  glBegin(GL_LINE_LOOP);
  glVertex2f( p1.x, p1.y);
  glVertex2f( p2.x, p2.y);
  glVertex2f(-p1.x,-p1.y);
  glVertex2f(-p2.x,-p2.y);
  glEnd();
  glColor3f(1.0f, 1.0f, 1.0f);
  glPopMatrix();
}



void DrawPoint(const Vec2& p, const int& cases, const int& size)
{
  switch (cases)
  {
    case 0: glColor3f(1.0f, 0.0f, 0.0f);  break;
    case 1: glColor3f(0.0f, 0.9f, 0.0f);  break;
    case 2: glColor3f(0.0f, 0.5f, 1.0f);  break;
  }
  glPointSize(size);	
  glBegin(GL_POINTS);
  glVertex2f(p.x, p.y);
  glEnd();
  glColor3f(1.0f, 1.0f, 1.0f);
}



void DrawLine(const Vec2& p1, const Vec2& p2)
{
  glLineWidth(1.0);
  glColor3f(1.0f, 0.0, 0.0f);
  glBegin(GL_LINES);
  glVertex2f(p1.x, p1.y);
  glVertex2f(p2.x, p2.y);
  glEnd();
  glColor3f(1.0f, 1.0f, 1.0f);
  glLineWidth(1);
}
