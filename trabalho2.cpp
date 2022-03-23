/*
   - divisao dos objetos por quadrante
   - gerenciador que passa quadrante para as tasks
      - testar politicas de escalonamento

   - avaliar performance de cada modelo
*/

//g++ trabalho2.cpp -o t2 -lglut -lGLU -lGL
#include <GL/glut.h>
#include <cmath>
#include <stdlib.h>
#include <vector>
#include <iostream>

#define N 200
#define L 600
#define BOID_SIZE 0.01
#define SPEED 0.0002
#define COHESION 0.01
#define ALIGNMENT 0.01
#define SEPARATION 0.02
#define SIGHT 0.2 
#define SEPARATION_Q 4
#define SMOOTHNESS 0.8

using namespace std;

void triangle(float x, float y, float size, float a){
   glColor4f(1.0, 1.0, 1.0, 0.5);
   glBegin(GL_TRIANGLES);
        glVertex3f(x + size*sin(a), y + size*cos(a), 0.0);
        a += 2.5;
        glVertex3f(x + size*sin(a), y + size*cos(a), 0.0);
        a -= 5;
        glVertex3f(x + size*sin(a), y + size*cos(a), 0.0);
    glEnd();
};

inline float boundedAngle(float a) {
   float temp = fmod(a,2*M_PI);
   if (temp >= 0){
      return temp;
   }
   return temp + 2*M_PI;
}

float my_rand(){
   return ((rand()%2000) - 1000)/1000.0;
}

float angleToPoint(float x1, float y1, float x2, float y2){
   float dy = y2 - y1;
   float dx = x2 - x1;
   float temp = atan(dx/dy);
   if(dy<0)
      temp += M_PI;
   if(temp < 0)
      temp = 2*M_PI + temp;
   return temp;
}

inline float dist(float x0, float y0, float x1, float y1){
   float dx = x0 - x1;
   float dy = y0 - y1;
   return sqrt(dx*dx + dy*dy);
}

class Boid{
   vector<Boid> * boids;
   int id;
   float x;
   float y;

   float v;
   float a;

   float size;

   float avrg_x;
   float avrg_y;
   float avrg_sep_x;
   float avrg_sep_y;
   float avrg_a;

   public: Boid(vector<Boid> * _boids, int _id, float _x, float _y, float _v, float _s){
      boids = _boids;
      id = _id;
      x = _x;
      y = _y;
      v = _v;
      size = _s;
      a = (my_rand() + 1)*M_PI;
   }

   void calculateStats(){
      avrg_x = 0;
      avrg_y = 0;
      avrg_sep_x = 0;
      avrg_sep_y = 0;
      avrg_a = 0;
      int count = 0, count_sep = 0;
      for(int i = 0; i < N; i++){
         float distance = dist(x,y,(*boids)[i].x,(*boids)[i].y);
         if(i != id && distance < SIGHT){
            avrg_x += (*boids)[i].x;
            avrg_y += (*boids)[i].y;
            avrg_a += (*boids)[i].a;
            count ++;
            if(distance < SIGHT/SEPARATION_Q){
               avrg_sep_x += (*boids)[i].x;
               avrg_sep_y += (*boids)[i].y;
               count_sep++;
            }
         }
      }
      if(count != 0){
         avrg_x /= count;
         avrg_y /= count;
         avrg_a /= count;
      }
      else{
         avrg_x = x;
         avrg_y = y;
         avrg_a = a;
      }

      if(count_sep != 0){
            avrg_sep_x /= count_sep;
            avrg_sep_y /= count_sep;
      }
      else{
         avrg_sep_x = x;
         avrg_sep_y = y;
      }
   }

   float adjustedAngle(float a0, float a1, float ammount){
      float da1 = a0 - a1;
      float da2;

      if (da1 > 0)
         da2 = da1 - 2*M_PI;
      else
         da2 = 2*M_PI + da1;

      if(abs(da1) < abs(da2))
         return da1*ammount;
      return da2*ammount;
   }

   float cohesion(float ammount){
      if(avrg_x != x || avrg_y != y)
         return adjustedAngle(angleToPoint(x, y, avrg_x, avrg_y), a, ammount);
      return 0;
   }

   float alignment(float ammount){
      if(avrg_a != a)
         return adjustedAngle(avrg_a, a, ammount);
      return 0;
   }

   float separation(float ammount){
      if(avrg_sep_x != x || avrg_sep_y != y)
         return adjustedAngle(a, angleToPoint(x, y, avrg_sep_x, avrg_sep_y), ammount);
      return 0;
   }

   void limitToScreen(){
      if(abs(x) > 1)
         x = x > 0 ? -1:1;
      if(abs(y) > 1)
         y = y > 0 ? -1:1;
   }

   void applyVelocity(){
      x += v*sin(a);
      y += v*cos(a);

      calculateStats();
      float da = 0;
      da += cohesion(COHESION);
      da += alignment(ALIGNMENT);
      da += separation(SEPARATION);
      da += a;

      a = SMOOTHNESS*a + (1-SMOOTHNESS)*da;

      a = boundedAngle(a);
   }

   void update(){
      limitToScreen();
      applyVelocity();
   }

   void draw(){
      triangle(x, y, size, a);
   }
};

vector<Boid> boids;

void display(void)
{
   glClear(GL_COLOR_BUFFER_BIT);
   glMatrixMode(GL_MODELVIEW);      // To operate on Model-View matrix
   glLoadIdentity();                // Reset the model-view matrix
   for(int i = 0; i < N; i++){
      boids[i].update();
      boids[i].draw();
   }
   
   glutSwapBuffers();
}

int main(int argc, char** argv)
{
   for(int i = 0; i < N; i++){
      boids.push_back(Boid(&boids, i, my_rand(), my_rand(), SPEED, BOID_SIZE));
   } 
   glutInit(&argc, argv);           // Initialize GLUT
   glutInitWindowSize(L, L);        // Set the window's initial width & height - non-square
   glutCreateWindow("Trabalho 2");  // Create window with the given title
   glutInitDisplayMode(GLUT_DOUBLE);
   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glutDisplayFunc(display);        // Register callback handler for window re-paint event
   glutIdleFunc(display);
   glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
   glutMainLoop();                  // Enter the infinite event-processing loop
   return 0;
}