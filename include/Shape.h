/*************************************************************************
* Copyright (c) 2019-2021 Jonathan Peña
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Jonathan Peña makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty. 
**************************************************************************/

#ifndef SHAPE_H
#define SHAPE_H

#include <stdio.h>
#include "RigidBody.h"
#include "glut.h"

enum typeShape {circle, obb};

class Shape
{

public:
  // Circle characteristics
  real radius; 
  // OBB characteristics
  Vec2 width;
  //Type of Shape
  typeShape type;
  // Rigid Body
  RigidBody* body;
  // Virtual Function To Draw Shape.
  virtual void DrawShape(void) = 0;
  // Virtual Function To create Dynamic Shape
  virtual Shape* newShape(void) = 0;
  // Virtual Function to Compute Mass And Inertia 
  virtual void CalculateMassInertia(const real&) = 0;
};

/*********************************** C I R C L E ****************************************/
/*********************************** C I R C L E ****************************************/

class Circle : virtual public Shape
{	

public:

  // Initialized Constructor
  Circle(const real& _radius)
  {
    type = circle; radius = _radius;
  }
  
  // Default Destructor
  ~Circle(void) {;}
  
  Shape* newShape() override {return new Circle(this->radius);}
  
  void DrawShape(void) override;
  
  void CalculateMassInertia(const real&) override;
};


/****************************** O B B ********************************/
/****************************** O B B ********************************/


class OBB : virtual public Shape
{

public:

  // Initialized Constructo with Vector
  OBB(const Vec2& _width)
  {
    type = obb; width = _width;
  }
  
  // Initialized Constructor with scalars
  OBB(const real& x, const real& y)
  {
    type = obb; width.Set(x, y);
  }
  
  // Default Destructor
  ~OBB(void) {;}
  
  Shape* newShape(void) override {return new OBB(this->width);}
  
  void DrawShape(void) override;
  
  void CalculateMassInertia(const real&) override;
};

// Function To Draw A Point(point, Color, Size)
void DrawPoint(const Vec2&, const int&, const int&);

// Function To Draw A Line
void DrawLine(const Vec2&, const Vec2&);

#endif
