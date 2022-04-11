//g++ trabalho2.cpp -o t2 -lglut -lGLU -lGL -lpthread
#include <GL/glut.h>
#include <cmath>
#include <stdlib.h>
#include <vector>
#include <array>
#include <algorithm>
#include <iostream>

#include <chrono>
#include <bits/stdc++.h>

#include <pthread.h>
#include <semaphore.h>

#define N 1000
#define L 1000
#define BOID_SIZE 0.01
#define SPEED 0.0002
#define COHESION 0.01
#define ALIGNMENT 0.001
#define SEPARATION 0.02
#define BORDER 0.02
#define SIGHT 0.2 
#define SEPARATION_Q 4
#define SMOOTHNESS 0.8

#define BENCHMARK_ITERATIONS 1000

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
   public:
   Boid * boids;
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


   Boid(){}
   Boid(Boid * _boids, int _id, float _x, float _y, float _v, float _s){
      boids = _boids;
      id = _id;
      x = _x;
      y = _y;
      v = _v;
      size = _s;
      a = (my_rand() + 1)*M_PI;
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

   float avoidBorder(float ammount){
      float dx = x, dy = y;
      if(x > 0.9)
         dx = 1;
      if(x < -0.9)
         dx = -1;

      if(y > 0.9)
         dy = 1;
      if(y < -0.9)
         dy = -1;

      if(dx != x || dy != y)
         return adjustedAngle(angleToPoint(dx, dy, x, y), a, ammount);
      return 0;
   }

   void limitToScreen(){
      if(abs(x) > 1)
         x = x > 0 ? 0.99:-0.99;
      if(abs(y) > 1)
         y = y > 0 ? 0.99:-0.99;
   }

   void applyVelocity(){
      x += v*sin(a);
      y += v*cos(a);

      float da = 0;
      da += cohesion(COHESION);
      da += alignment(ALIGNMENT);
      da += separation(SEPARATION);
      da += avoidBorder(BORDER);
      da += a;

      a = SMOOTHNESS*a + (1-SMOOTHNESS)*da;

      a = boundedAngle(a);
   }

   void calculateStats(){
      avrg_x = 0;
      avrg_y = 0;
      avrg_sep_x = 0;
      avrg_sep_y = 0;
      avrg_a = 0;
      int count = 0, count_sep = 0;
      for(int i = 0; i < N; i++){
         float distance = dist(x,y,boids[i].x,boids[i].y);
         if(i != id && distance < SIGHT){
            avrg_x += boids[i].x;
            avrg_y += boids[i].y;
            avrg_a += boids[i].a;
            count ++;
            if(distance < SIGHT/SEPARATION_Q){
               avrg_sep_x += boids[i].x;
               avrg_sep_y += boids[i].y;
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

   void calculateStats(vector<int> neighbors){
      avrg_x = 0;
      avrg_y = 0;
      avrg_sep_x = 0;
      avrg_sep_y = 0;
      avrg_a = 0;
      int count = 0, count_sep = 0;
      for(int j = 0; j < neighbors.size(); j++){
         int i = neighbors[j];
         float bx = boids[i].x, by = boids[i].y;
         float distance = dist(x,y,bx,by);
         if(i != id && distance < SIGHT){
            avrg_x += bx;
            avrg_y += by;
            avrg_a += boids[i].a;
            count ++;
            if(distance < SIGHT/SEPARATION_Q){
               avrg_sep_x += bx;
               avrg_sep_y += by;
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

   void update(){
      limitToScreen();
      applyVelocity();
   }

   void draw(){
      triangle(x, y, size, a);
   }
};

class StaticQuadrants{
   vector<int> quadrants[(int)(2/SIGHT)][(int)(2/SIGHT)];

   float quadrantSize;
   Boid * boids;
   pthread_mutex_t quadrant_locks[(int)(2/SIGHT)][(int)(2/SIGHT)] = {PTHREAD_MUTEX_INITIALIZER};

   public:
   int nQuadrantsSide, nQuadrants;

   StaticQuadrants(Boid* _boids){
      boids = _boids;
      quadrantSize = SIGHT;
      nQuadrantsSide = 2/quadrantSize;
      nQuadrants = nQuadrantsSide*nQuadrantsSide;

      for(int i = 0; i<N; i++){
         int x = (int)floor((boids[i].x + 1)/quadrantSize);
         int y = (int)floor((boids[i].y + 1)/quadrantSize);
         quadrants[x][y].push_back(i);
      }
   }

   void print(){
      for (int i = 0; i < nQuadrantsSide; i++){
         for (int j = 0; j < nQuadrantsSide; j++){
            cout<<i<<','<<j<<": "<<quadrants[i][j].size()<<'\n';
         }
      }
   }

   void updateQuadrant(int x, int y){
      vector<array<int,3>> removed;
      for (int i = 0; i < quadrants[x][y].size(); i++){
         int index = quadrants[x][y][i];
         int x_ = (int)floor((boids[index].x + 1)/quadrantSize);
         int y_ = (int)floor((boids[index].y + 1)/quadrantSize);
         if(x_ != x || y_ != y){
            removed.push_back({index, x_, y_});
         }
      }
      for (int i = removed.size() - 1; i >= 0; i--){
         quadrants[x][y].erase(remove(quadrants[x][y].begin(), quadrants[x][y].end(), removed[i][0]), quadrants[x][y].end());
         quadrants[removed[i][1]][removed[i][2]].push_back(removed[i][0]);
      }
   }

   void updateQuadrantThreaded(int x, int y){
      vector<array<int,3>> removed;
      for (int i = 0; i < quadrants[x][y].size(); i++){
         int index = quadrants[x][y][i];
         int x_ = (int)floor((boids[index].x + 1)/quadrantSize);
         int y_ = (int)floor((boids[index].y + 1)/quadrantSize);
         if(x_ != x || y_ != y){
            removed.push_back({index, x_, y_});
         }
      }
      for (int i = removed.size() - 1; i >= 0; i--){
         pthread_mutex_lock(&(quadrant_locks[x][y]));
         quadrants[x][y].erase(remove(quadrants[x][y].begin(), quadrants[x][y].end(), removed[i][0]), quadrants[x][y].end());
         pthread_mutex_unlock(&(quadrant_locks[x][y])); 

         pthread_mutex_lock(&(quadrant_locks[removed[i][1]][removed[i][2]]));
         quadrants[removed[i][1]][removed[i][2]].push_back(removed[i][0]);
         pthread_mutex_unlock(&(quadrant_locks[removed[i][1]][removed[i][2]])); 
      }
   }

   void calculateStats(int x, int y){
      vector<int> neighbors;

      int sx = x == 0                  ? x : x-1;
      int ex = x == nQuadrantsSide - 1 ? x : x+1;
      int sy = y == 0                  ? y : y-1;
      int ey = y == nQuadrantsSide - 1 ? y : y+1;
      
      for(int i = sx; i <= ex; i++){
         for(int j = sy; j <= ey; j++){
            neighbors.insert(neighbors.end(), quadrants[i][j].begin(), quadrants[i][j].end());
         }
      }
      for (int i = 0; i < quadrants[x][y].size(); i++){
         boids[quadrants[x][y][i]].calculateStats(neighbors);
      }
   }

   void update(int x, int y){
      for (int i = 0; i < quadrants[x][y].size(); i++){
         boids[quadrants[x][y][i]].update();
      }
   }
};

Boid * boids = new Boid[N];
StaticQuadrants s(boids);

pthread_mutex_t main_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t calc_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t up_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t q_lock = PTHREAD_MUTEX_INITIALIZER;
sem_t s_main;
pthread_cond_t c_main = PTHREAD_COND_INITIALIZER;

int N_THREADS = 4;

pthread_t *calcThreads;
pthread_cond_t c_calc = PTHREAD_COND_INITIALIZER;

pthread_t *updateThreads;
pthread_cond_t c_update = PTHREAD_COND_INITIALIZER;

pthread_t *quadrantThreads;
pthread_cond_t c_quadrants = PTHREAD_COND_INITIALIZER;

void * calcTask(void* in){
   int x, y, id, min, max;
   id = *((int*)in);
   min = (int)((((float)id)/N_THREADS)*s.nQuadrants);
   max = (int)(((id + 1.0)/N_THREADS)*s.nQuadrants);
   while(true){
      pthread_mutex_lock(&calc_lock);
         pthread_cond_wait(&c_calc, &calc_lock);
      pthread_mutex_unlock(&calc_lock); 

      for (int i = min; i < max; i++){
         x = i % s.nQuadrantsSide;
         y = i / s.nQuadrantsSide;
         s.calculateStats(x,y);
      }

      if (sem_trywait(&s_main) != 0){
         pthread_mutex_lock(&main_lock);
            pthread_cond_signal(&c_main);
         pthread_mutex_unlock(&main_lock);
      }
   }
   pthread_exit(0);
}

void * updateTask(void* in){
   int x, y, id, min, max;
   id = *((int*)in);
   min = (int)((((float)id)/N_THREADS)*s.nQuadrants);
   max = (int)(((id + 1.0)/N_THREADS)*s.nQuadrants);
   while(true){
      pthread_mutex_lock(&up_lock);
         pthread_cond_wait(&c_update, &up_lock);
      pthread_mutex_unlock(&up_lock); 

      for (int i = min; i < max; i++){
         x = i % s.nQuadrantsSide;
         y = i / s.nQuadrantsSide;
         s.update(x,y);
      }

      if (sem_trywait(&s_main) != 0){
         pthread_mutex_lock(&main_lock);
            pthread_cond_signal(&c_main);
         pthread_mutex_unlock(&main_lock);
      }
   }
   pthread_exit(0);
}

void * quadrantTask(void* in){
   int x, y, id, min, max;
   id = *((int*)in);
   min = (int)((((float)id)/N_THREADS)*s.nQuadrants);
   max = (int)(((id + 1.0)/N_THREADS)*s.nQuadrants);
   while(true){
      pthread_mutex_lock(&q_lock);
         pthread_cond_wait(&c_quadrants, &q_lock);
      pthread_mutex_unlock(&q_lock); 

      for (int i = min; i < max; i++){
         x = i % s.nQuadrantsSide;
         y = i / s.nQuadrantsSide;
         s.updateQuadrantThreaded(x,y);
      }

      if (sem_trywait(&s_main) != 0){
         pthread_mutex_lock(&main_lock);
            pthread_cond_signal(&c_main);
         pthread_mutex_unlock(&main_lock);
      }
   }
   pthread_exit(0);
}

void simpleUpdate(){
   for(int i = 0; i < N; i++){
      boids[i].calculateStats();
   }

   for(int i = 0; i < N; i++){
      boids[i].update();
   }
}

void updateWithQuadrants(){
   for (int i = 0; i < s.nQuadrantsSide; i++){
      for (int j = 0; j < s.nQuadrantsSide; j++){
         s.calculateStats(i,j);
      }
   }

   for (int i = 0; i < s.nQuadrantsSide; i++){
      for (int j = 0; j < s.nQuadrantsSide; j++){
         s.update(i,j);
      }
   }

   
   for (int i = 0; i < s.nQuadrantsSide; i++){
      for (int j = 0; j < s.nQuadrantsSide; j++){
         s.updateQuadrant(i,j);
      }
   } 
}

void updateThreaded(){
   
   //calculation step
   sem_init(&s_main, 0, N_THREADS - 1);

   pthread_mutex_lock(&calc_lock);
      pthread_cond_broadcast(&c_calc);
   pthread_mutex_unlock(&calc_lock);

   pthread_mutex_lock(&main_lock);
      pthread_cond_wait(&c_main, &main_lock);
   pthread_mutex_unlock(&main_lock);

   //boid update step
   sem_init(&s_main, 0, N_THREADS - 1);

   pthread_mutex_lock(&up_lock);
      pthread_cond_broadcast(&c_update);
   pthread_mutex_unlock(&up_lock);

   pthread_mutex_lock(&main_lock);
      pthread_cond_wait(&c_main, &main_lock);
   pthread_mutex_unlock(&main_lock);

   //quadrant update step
   sem_init(&s_main, 0, N_THREADS - 1);

   pthread_mutex_lock(&q_lock);
      pthread_cond_broadcast(&c_quadrants);
   pthread_mutex_unlock(&q_lock);

   pthread_mutex_lock(&main_lock);
      pthread_cond_wait(&c_main, &main_lock);
   pthread_mutex_unlock(&main_lock);
}


void display(void)
{
   glClear(GL_COLOR_BUFFER_BIT);
   glMatrixMode(GL_MODELVIEW);      // To operate on Model-View matrix
   glLoadIdentity();                // Reset the model-view matrix

   updateThreaded();

   //showing graphics
   for(int i = 0; i < N; i++){
      boids[i].draw();
   }

   glutSwapBuffers();
}

void setupLogica(int argc, char** argv){
   srand(16);
   for(int i = 0; i < N; i++){
      boids[i] = Boid(boids, i, my_rand()*0.89, my_rand()*0.89, SPEED, BOID_SIZE);
   } 

   sem_init(&s_main, 0, 0);

   int *id;
   for(int i = 0; i < N_THREADS; i++){
      id = (int *) malloc(sizeof(int));
      *id = i;

      if(pthread_create(&calcThreads[i], NULL, calcTask, (void*) id))
      {
         printf("erro na criacao do thread %d\n", i);
         exit(1);
      }

      if(pthread_create(&updateThreads[i], NULL, updateTask, (void*) id))
      {
         printf("erro na criacao do thread %d\n", i);
         exit(1);
      }

      if(pthread_create(&quadrantThreads[i], NULL, quadrantTask, (void*) id))
      {
         printf("erro na criacao do thread %d\n", i);
         exit(1);
      }
   }
}

void setupGraficos(int argc, char** argv){
   glutInit(&argc, argv);           // Initialize GLUT
   glutInitWindowSize(L, L);        // Set the window's initial width & height - non-square
   glutCreateWindow("Trabalho 2");  // Create window with the given title
   glutInitDisplayMode(GLUT_DOUBLE);
   glEnable(GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glutDisplayFunc(display);        // Register callback handler for window re-paint event
   glutIdleFunc(display);
   glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void benchmark(){
   auto start = chrono::high_resolution_clock::now();
   
   ios_base::sync_with_stdio(false);

   for (int iter = 0; iter < BENCHMARK_ITERATIONS; iter ++){
      simpleUpdate();
   }

   auto end = chrono::high_resolution_clock::now();

   double time_taken = 
   chrono::duration_cast<chrono::nanoseconds>(end - start).count();

   time_taken *= 1e-9;

   cout << "Tempo com implementacao naive : " << fixed 
   << time_taken << setprecision(9);
   cout << " seg" << endl;

   start = chrono::high_resolution_clock::now();

   ios_base::sync_with_stdio(false);

   for (int iter = 0; iter < BENCHMARK_ITERATIONS; iter ++){
      updateWithQuadrants();
   }

   end = chrono::high_resolution_clock::now();

   time_taken = 
   chrono::duration_cast<chrono::nanoseconds>(end - start).count();

   time_taken *= 1e-9;

   cout << "Tempo com quadrantes : " << fixed 
   << time_taken << setprecision(9);
   cout << " seg" << endl;


   start = chrono::high_resolution_clock::now();

   for (int iter = 0; iter < BENCHMARK_ITERATIONS; iter ++){
      updateThreaded();
   }

   end = chrono::high_resolution_clock::now();

   time_taken = 
   chrono::duration_cast<chrono::nanoseconds>(end - start).count();

   time_taken *= 1e-9;

   cout << "Tempo com " << N_THREADS << " threads : " << fixed 
   << time_taken << setprecision(9);
   cout << " seg" << endl;
}

int main(int argc, char** argv)
{
   int b = 0;
   cout<<"modo de benchmark?\n(0) Nao\n(1) Sim\n";
   cin>>b;
   if(b == 1){
      cout << "quantas threads utilizar? (recomendado: 16)\n";
      cin >> N_THREADS;

      //alocando vetores para threads

      calcThreads = (pthread_t*)malloc(N_THREADS*sizeof(pthread_t));

      updateThreads = (pthread_t*)malloc(N_THREADS*sizeof(pthread_t));;

      quadrantThreads = (pthread_t*)malloc(N_THREADS*sizeof(pthread_t));;

      cout << "iniciando benchmark\n";
      setupLogica(argc, argv);
      benchmark();
   }
   else{
      calcThreads = (pthread_t*)malloc(N_THREADS*sizeof(pthread_t));

      updateThreads = (pthread_t*)malloc(N_THREADS*sizeof(pthread_t));;

      quadrantThreads = (pthread_t*)malloc(N_THREADS*sizeof(pthread_t));;
      setupLogica(argc, argv);
      setupGraficos(argc, argv);
      glutMainLoop();                  // Enter the infinite event-processing glutMainLoop
   }
   return 0;
}