#include <algorithm>
#include <deque>

#define SPEED 2
#define NB_DRONE 5
#define NB_VEH 4
#define TIME_LIMIT 0.5
#define TIME_LIMIT2 600
#define NB_NEIGHBOR 3

#define LIMIT_LAB_METHOD 3
#define SEUIL 20
#define SEUIL2 27
#define SEUIL_CMPT 10

//choose between the K closest neighbors

extern double** matrixTruck; //Represents the distance matrix for the truck (Manhattan distance)
extern double** matrixDrone; //Represents the distance matrix for the drone (Euclidean  distance)
extern std::deque<int> droneEligible; //Contains id's of drone eligible custom

// Foncteur servant à libérer un pointeur - applicable à n'importe quel type
struct Delete
{
   template <class T> void operator ()(T*& p) const
   {
      delete p;
      p = NULL;
   }
};


