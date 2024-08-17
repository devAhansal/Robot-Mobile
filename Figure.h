#ifndef FIGURE_H // Protection contre l'inclusion multiple
#define FIGURE_H

class Robot {
public:
    // Constructeur par défaut avec initialisation des membres
    Robot() : x(400), y(200), r(50), Dr(0), Dalpha(0), wg(0), wd(0), Dg(0), Dd(0), Rc(0), Dx(0), Dy(0) {}

    // Membres de la classe
    float x, y, r;
    float Dr;	  // Distance parcourue pendant Dt en m
    float Dalpha; // Reorientation pendant Dt en rad
    float wg;	  // Vitesse angulaire de la roue gauche en rad/s
    float wd;	  // Vitesse angulaire de la roue droite en rad/s
    float Dg;	  // Distance parcourue par la roue gauche pendant Dt en m
    float Dd;	  // Distance parcourue par la roue droite pendant Dt en m
    float Rc;	  // Rayon de courbure de la trajectoire du robot en m
    float Dx, Dy; // Distance instantanee du robot pendant Dt de x et y en pixel/s
};

class Goal {
public:
    int x;
    int y;
    int radius;
};

class Obstacle {
public:
    int x;
    int y;
    int radius;
};

#endif

