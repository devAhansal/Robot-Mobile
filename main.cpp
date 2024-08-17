#include <iostream>
#include <graphics.h>
#include <stdio.h>
#include <graphics.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#define M_PI 3.14159265358979323846
#define MAX_STATES 1000
#include <iostream>
#include "Figure.h"

static const float Dt = 0.1f;	  // Intervale de temps entre chaque mise ? jour de la position et l'orientation
static const float D = 0.05f;	  // Distance entre les roues D = 0.05 m
static const float Rr = 0.05f;	  // Rayon du robot Rr = 0.05 m
static const float R0 = 0.02f;	  // Rayon d'une roue R0 = 0.02 m
static const float w0Max = 10.0f; // Vitesse angulaire maximum des roues w0Max = 10 rad/s
static const float Dw0Max = 2.0f; // Acceleration angulaire maximum des roues Dw0Max = 2 rad/s

// Robot functions
float Robot_getDalpha(Robot *robot)
{
	return robot->Dalpha;
}

void Robot_updateDr(Robot *robot)
{
	robot->Dr = (robot->Dg + robot->Dd) / 2;
}

void Robot_updateDalpha(Robot *robot)
{
	robot->Dalpha += (robot->Dg - robot->Dd) / D;
}

void Robot_updateDg(Robot *robot)
{
	robot->Dg = robot->wg * Dt * R0;
}

void Robot_updateDd(Robot *robot)
{
	robot->Dd = robot->wd * Dt * R0;
}

void Robot_updateDx(Robot *robot)
{
	robot->Dx = robot->Dr * cos(robot->Dalpha);
	robot->Dx *= 5000;
}

void Robot_updateDy(Robot *robot)
{
	robot->Dy = robot->Dr * sin(robot->Dalpha);
	robot->Dy *= 5000;
}

float Robot_getX(Robot *robot)
{
	return robot->x;
}

void Robot_setX(Robot *robot, float x)
{
	robot->x = x;
}

float Robot_getY(Robot *robot)
{
	return robot->y;
}

void Robot_setY(Robot *robot, float y)
{
	robot->y = y;
}

float Robot_getR(Robot *robot)
{
	return robot->r;
}

void Robot_setR(Robot *robot, float r)
{
	robot->r = r;
}

void Robot_updateDalpha(Robot *robot, float Dalpha)
{
	robot->Dalpha = Dalpha;
}

// function that makes the robot move towards the goal:
void move_towards_goal(Robot *robot, Goal *goal)
{
	// Calculate the difference between the x and y coordinates of the goal and the robot
	float diff_x = goal->x - robot->x;
	float diff_y = goal->y - robot->y;
	// Update the dx and dy values of the robot based on the difference between the x and y coordinates
	if (abs(diff_x) > abs(diff_y))
	{
		robot->Dx = (diff_x > 0) ? 5 : -5;
		robot->Dy = (diff_y > 0) ? 5 * diff_y / abs(diff_x) : -5 * diff_y / abs(diff_x);
	}
	else
	{
		robot->Dx = (diff_x > 0) ? 5 * diff_x / abs(diff_y) : -5 * diff_x / abs(diff_y);
		robot->Dy = (diff_y > 0) ? 5 : -5;
	}

	float Alpha = atan2(robot->Dy, robot->Dx);
	robot->Dalpha = Alpha;
}

// Check if the robot is colliding with an obstacle and return the direction to avoid the obstacle
int check_collision(Robot robot, Obstacle obstacle, float *dx, float *dy)
{
	// Calculate the distance between the center of the robot and the center of the obstacle
	float distance = sqrt((robot.x - obstacle.x) * (robot.x - obstacle.x) +
						  (robot.y - obstacle.y) * (robot.y - obstacle.y)) -
					 5;

	// Return 1 if the distance is less than the sum of the radii, 0 otherwise
	if (distance < robot.r + obstacle.radius)
	{
		// Calculate the angle between the center of the robot and the center of the obstacle
		float angle = atan2(obstacle.y - robot.y, obstacle.x - robot.x);

		// Set the new direction of the robot to move around the obstacle
		*dx = 0.5 * cos(angle + M_PI);
		*dy = 0.5 * sin(angle + M_PI);
		return 1;
	}
	else
	{
		return 0;
	}
}

void avoidCollisionWithObstacle(Robot *robot, Obstacle *obstacle)
{
	// Check if the robot is colliding with the obstacle
	float dx, dy;
	if (check_collision(*robot, *obstacle, &dx, &dy))
	{
		// The robot is colliding with the obstacle, so update its position to avoid the collision
		robot->x += dx ; // Move the robot at half the speed to make it move slowly
		robot->y += dy ;
		Robot_updateDalpha(robot, atan2(dy, dx)); // Update the rotation angle of the robot to match the direction of the rotation
	}
}

// Check if the robot is colliding with a goal and return 1 if it is, 0 otherwise
int reached_goal(Robot robot, Goal goal)
{
	// Calculate the distance between the center of the robot and the center of the goal
	float distance = sqrt((robot.x - goal.x) * (robot.x - goal.x) +
						  (robot.y - goal.y) * (robot.y - goal.y));

	// Return 1 if the distance is less than the sum of the radii, 0 otherwise
	return distance < robot.r + goal.radius;
}

// Function to save the state of the robot to a file
int num_states_saved = 0;
void save_robot_state(Robot robot)
{

	FILE *fp = fopen("state.pts", "a");
	if (fp == NULL)
	{
		printf("Error opening file!\n");
		return;
	}

	if (num_states_saved < MAX_STATES)
	{
		// Write the state to the file in the specified format
		fprintf(fp, "< %d > < %f > < %f > < %f > < %f >\n", num_states_saved, robot.x, robot.y, robot.wg, robot.wd);
		num_states_saved++;
	}
	else
	{
		// Go to the beginning of the file
		rewind(fp);

		// Find the oldest state
		int i;
		for (i = 0; i < MAX_STATES - 1; i++)
		{
			fscanf(fp, "%*d %*f %*f %*f %*f %*f %*f\n");
		}

		// Overwrite the oldest state with the new state
		fprintf(fp, "< %d > < %f > < %f > < %f > < %f >\n", num_states_saved, robot.x, robot.y, robot.wg, robot.wd);
	}
	fclose(fp);
}

int main()
{
	int w = GetSystemMetrics(SM_CXSCREEN);
	int h = GetSystemMetrics(SM_CYSCREEN);
	initwindow(w, h, "Accueil");

	settextstyle(DEFAULT_FONT, HORIZ_DIR, 2);

	// Calculer les positions pour centrer le menu
	int centerX = w / 2;
	int centerY = h / 2;

	// Afficher les ?l?ments du menu centr?s
	outtextxy(centerX - 100, centerY - 50, "Choisissez le mode :");
	outtextxy(centerX - 100, centerY, "1. Manuel (F1)");
	outtextxy(centerX - 100, centerY + 50, "2. Automatique (F2)");

	Robot robot; // instance of Robot

	float alpha;

	// Initialize the obstacle
	// Load obstacles from .obs file
	FILE *obstacle_file = fopen("obstacles.obs", "r");
	if (obstacle_file == NULL)
	{
		printf("Error: Could not open obstacles file.\n");
		return 1;
	}
	int num_obstacles;

	fscanf(obstacle_file, "%d", &num_obstacles);

	Obstacle obstacles[num_obstacles];
	for (int i = 0; i <= num_obstacles; i++)
	{
		int x, y, r;
		fscanf(obstacle_file, "%d %d %d", &x, &y, &r);
		obstacles[i] = (Obstacle){x, y, r};
	}
	fclose(obstacle_file);
	// Goal
	Goal goal;
	goal.radius = 50;
	srand(time(0)); // Choose a different seed at each execution of the program
	goal.x = 550;
	goal.y = 400;
	while (1)
	{
		if (GetAsyncKeyState(VK_F1) & 0x8000)
		{
			cleardevice();

			while (1)
			{

				alpha = Robot_getDalpha(&robot);

				cleardevice();

				setlinestyle(0, 0, 2);
				// Draw the obstacles
				setcolor(RED);
				for (int i = 0; i <= num_obstacles; i++)
				{
					circle(obstacles[i].x, obstacles[i].y, obstacles[i].radius);
					setfillstyle(SOLID_FILL, RED);
					floodfill(obstacles[i].x, obstacles[i].y, RED);
				}

				// Draw the goal
				setcolor(GREEN);
				circle(goal.x, goal.y, goal.radius);
				setfillstyle(SOLID_FILL, GREEN);
				floodfill(goal.x, goal.y, GREEN);

				/**
				 *---------------------- Draw The Robot ----------------------------
				 ***/
				setcolor(WHITE);
				circle(Robot_getX(&robot), Robot_getY(&robot), Robot_getR(&robot));

				/**
				 *----------------------Draw TRIANGLE Using trigonometric ----------------------------
				 ***/
				// std::cout << "alpha: " << cos(alpha - (M_PI / 2)) << std::endl;
				line(Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha - (M_PI / 2)),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha - (M_PI / 2)),
					 Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha));

				line(Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha),
					 Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha + (M_PI / 2)),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha + (M_PI / 2)));

				line(Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha - (M_PI / 2)),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha - (M_PI / 2)),
					 Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha + (M_PI / 2)),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha + (M_PI / 2)));
				/**
				 *----------------------Draw wheels----------------------------
				 ***/

				// Calculate the center point of the existing line segment
				float center_x = (Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha - M_PI / 2) +
								  Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha + M_PI / 2)) /
								 2;
				float center_y = (Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha - M_PI / 2) +
								  Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha + M_PI / 2)) /
								 2;
				// // std::cout << "center_x: " << center_x << std::endl;
				// // std::cout << "center_y: " << center_y << std::endl;
				// Calculate the endpoints of the horizontal line segment on the top of the existing line segment
				float top_x1 = center_x + (int)(Robot_getR(&robot) / 2) * cos(alpha - M_PI / 3);
				float top_y1 = center_y + (int)(Robot_getR(&robot) / 2) * sin(alpha - M_PI / 3);
				float top_x2 = center_x + (int)(Robot_getR(&robot) / 2) * cos(alpha - 2 * M_PI / 3);
				float top_y2 = center_y + (int)(Robot_getR(&robot) / 2) * sin(alpha - 2 * M_PI / 3);

				// Calculate the endpoints of the horizontal line segment on the bottom of the existing line segment
				float bottom_x1 = center_x + (int)(Robot_getR(&robot) / 2) * cos(alpha + M_PI / 3);
				float bottom_y1 = center_y + (int)(Robot_getR(&robot) / 2) * sin(alpha + M_PI / 3);
				float bottom_x2 = center_x + (int)(Robot_getR(&robot) / 2) * cos(alpha + 2 * M_PI / 3);
				float bottom_y2 = center_y + (int)(Robot_getR(&robot) / 2) * sin(alpha + 2 * M_PI / 3);

				// Draw horizontal line segment on the top of the existing line segment
				line(top_x1, top_y1, top_x2, top_y2);

				// Draw horizontal line segment on the bottom of the existing line segment
				line(bottom_x1, bottom_y1, bottom_x2, bottom_y2);

				// Check if the robot is colliding with any of the obstacles and avoid them
				int collision = 0;
				for (int i = 0; i < num_obstacles; i++)
				{
					collision |= check_collision(robot, obstacles[i], &robot.Dx, &robot.Dy);
					avoidCollisionWithObstacle(&robot, &obstacles[i]);
				}

				// Check if the robot is at the goal
				if (reached_goal(robot, goal))
				{
					// If the robot is at the goal, create a new goal within the screen and avoid the edges and obstacles
					goal.radius = 50;
					int new_x, new_y;
					do
					{
						// Generate random x and y coordinates within the range of the screen
						new_x = (int)((rand() / (double)RAND_MAX) * (getmaxx() - 2 * goal.radius) + goal.radius);
						new_y = (int)((rand() / (double)RAND_MAX) * (getmaxy() - 2 * goal.radius) + goal.radius);

						// Check the distance between the new goal and all obstacles
						int collision = 0;
						for (int i = 0; i < num_obstacles; i++)
						{
							float distance = sqrt((new_x - obstacles[i].x) * (new_x - obstacles[i].x) +
												  (new_y - obstacles[i].y) * (new_y - obstacles[i].y));
							if (distance < goal.radius + obstacles[i].radius)
							{
								collision = 1;
								break;
							}
						}

						// If the new goal is not colliding with any obstacles, exit the loop
						if (!collision)
						{
							break;
						}
					} while (1);

					//			Set the new x and y coordinates for the goal
					goal.x = new_x;
					goal.y = new_y;
				}

				// MANUEL
				float nwg = 0.0f;
				float nwd = 0.0f;
				// float nDx, nDy;

				if (GetKeyState(VK_UP) & 0x8000)
				{
					if (nwg + 1 <= w0Max)
					{
						nwg += 1;
					}
					if (nwd + 1 <= w0Max)
					{
						nwd += 1;
					}
				}
				else if (GetKeyState(VK_LEFT) & 0x8000)
				{
					if (nwg - 1 >= -w0Max && nwd + 1 <= w0Max)
					{
						nwg -= 1;
						nwd += 1;
					}
				}
				else if (GetKeyState(VK_RIGHT) & 0x8000)
				{
					if (nwg - 1 >= -w0Max && nwd + 1 <= w0Max)
					{
						nwg += 1;
						nwd -= 1;
					}
				}
				else if (GetKeyState(VK_DOWN) & 0x8000)
				{
					if (nwg - 1 >= -w0Max)
					{
						nwg -= 1;
					}
					if (nwd - 1 >= -w0Max)
					{
						nwd -= 1;
					}
				}
				else
				{
					if (nwg > nwd)
					{
						nwg -= 1;
						nwd += 1;
					}
					if (nwg < nwd)
					{
						nwd -= 1;
						nwg += 1;
					}
				}

				robot.wg = nwg;
				robot.wd = nwd;
				Robot_updateDd(&robot);
				Robot_updateDg(&robot);
				Robot_updateDr(&robot);
				Robot_updateDalpha(&robot);
				Robot_updateDx(&robot);
				Robot_updateDy(&robot);

				if (!collision)
				{
					// Towards the Goal
					Robot_setX(&robot, robot.x + robot.Dx);
					Robot_setY(&robot, robot.y + robot.Dy);
				}
				else
				{
					robot.wg -= 0.02;
					robot.wd -= 0.02;
					// Save the State of the Robot :
					save_robot_state(robot);
				}

				// Save the State of the Robot :
				save_robot_state(robot);

				// delay
				delay(100);
			}
		}
		else if (GetAsyncKeyState(VK_F2) & 0x8000)
		{
			cleardevice();
			while (1)
			{

				alpha = Robot_getDalpha(&robot);

				cleardevice();

				setlinestyle(0, 0, 2);
				// Draw the obstacles
				setcolor(RED);
				for (int i = 0; i <= num_obstacles; i++)
				{
					circle(obstacles[i].x, obstacles[i].y, obstacles[i].radius);
					setfillstyle(SOLID_FILL, RED);
					floodfill(obstacles[i].x, obstacles[i].y, RED);
				}

				// Draw the goal
				setcolor(GREEN);
				circle(goal.x, goal.y, goal.radius);
				setfillstyle(SOLID_FILL, GREEN);
				floodfill(goal.x, goal.y, GREEN);

				/**
				 *---------------------- Draw The Robot ----------------------------
				 ***/
				setcolor(WHITE);
				circle(Robot_getX(&robot), Robot_getY(&robot), Robot_getR(&robot));

				/**
				 *----------------------Draw TRIANGLE Using trigonometric ----------------------------
				 ***/
				// std::cout << "alpha: " << cos(alpha - (M_PI / 2)) << std::endl;
				line(Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha - (M_PI / 2)),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha - (M_PI / 2)),
					 Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha));

				line(Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha),
					 Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha + (M_PI / 2)),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha + (M_PI / 2)));

				line(Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha - (M_PI / 2)),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha - (M_PI / 2)),
					 Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha + (M_PI / 2)),
					 Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha + (M_PI / 2)));
				/**
				 *----------------------Draw wheels----------------------------
				 ***/

				// Calculate the center point of the existing line segment
				float center_x = (Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha - M_PI / 2) +
								  Robot_getX(&robot) + (int)Robot_getR(&robot) * cos(alpha + M_PI / 2)) /
								 2;
				float center_y = (Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha - M_PI / 2) +
								  Robot_getY(&robot) + (int)Robot_getR(&robot) * sin(alpha + M_PI / 2)) /
								 2;
				// Calculate the endpoints of the horizontal line segment on the top of the existing line segment
				float top_x1 = center_x + (int)(Robot_getR(&robot) / 2) * cos(alpha - M_PI / 3);
				float top_y1 = center_y + (int)(Robot_getR(&robot) / 2) * sin(alpha - M_PI / 3);
				float top_x2 = center_x + (int)(Robot_getR(&robot) / 2) * cos(alpha - 2 * M_PI / 3);
				float top_y2 = center_y + (int)(Robot_getR(&robot) / 2) * sin(alpha - 2 * M_PI / 3);

				// Calculate the endpoints of the horizontal line segment on the bottom of the existing line segment
				float bottom_x1 = center_x + (int)(Robot_getR(&robot) / 2) * cos(alpha + M_PI / 3);
				float bottom_y1 = center_y + (int)(Robot_getR(&robot) / 2) * sin(alpha + M_PI / 3);
				float bottom_x2 = center_x + (int)(Robot_getR(&robot) / 2) * cos(alpha + 2 * M_PI / 3);
				float bottom_y2 = center_y + (int)(Robot_getR(&robot) / 2) * sin(alpha + 2 * M_PI / 3);

				// Draw horizontal line segment on the top of the existing line segment
				line(top_x1, top_y1, top_x2, top_y2);

				// Draw horizontal line segment on the bottom of the existing line segment
				line(bottom_x1, bottom_y1, bottom_x2, bottom_y2);

				// Check if the robot is colliding with any of the obstacles and avoid them
				int collision = 0;
				for (int i = 0; i < num_obstacles; i++)
				{
					collision |= check_collision(robot, obstacles[i], &robot.Dx, &robot.Dy);
					avoidCollisionWithObstacle(&robot, &obstacles[i]);
				}
				// Check if the robot is at the goal
				if (reached_goal(robot, goal))
				{
					// If the robot is at the goal, create a new goal within the screen and avoid the edges and obstacles
					goal.radius = 50;
					int new_x, new_y;
					do
					{
						// Generate random x and y coordinates within the range of the screen
						new_x = (int)((rand() / (double)RAND_MAX) * (getmaxx() - 2 * goal.radius) + goal.radius);
						new_y = (int)((rand() / (double)RAND_MAX) * (getmaxy() - 2 * goal.radius) + goal.radius);

						// Check the distance between the new goal and all obstacles
						int collision = 0;
						for (int i = 0; i < num_obstacles; i++)
						{
							float distance = sqrt((new_x - obstacles[i].x) * (new_x - obstacles[i].x) +
												  (new_y - obstacles[i].y) * (new_y - obstacles[i].y));
							if (distance < goal.radius + obstacles[i].radius)
							{
								collision = 1;
								break;
							}
						}

						// If the new goal is not colliding with any obstacles, exit the loop
						if (!collision)
						{
							break;
						}
					} while (1);

					//			Set the new x and y coordinates for the goal
					goal.x = new_x;
					goal.y = new_y;
				}

				float nwg = 0.0f;
				float nwd = 0.0f;
				Robot_updateDd(&robot);
				Robot_updateDg(&robot);
				Robot_updateDr(&robot);
				Robot_updateDalpha(&robot);

				if (!collision)
				{
					// Towards the Goal
					move_towards_goal(&robot, &goal);
					std::cout << "wg: " << robot.wg << std::endl;
					Robot_setX(&robot, robot.x + robot.Dx);
					Robot_setY(&robot, robot.y + robot.Dy);
				}
				else
				{
					robot.wg -= 0.02;
					robot.wd -= 0.02;
					// Save the State of the Robot :
					save_robot_state(robot);
				}

				// Save the State of the Robot :
				save_robot_state(robot);

				// delay
				delay(100);
			}
		}
	}

	delay(3000); // Attente pour que l'utilisateur puisse voir le message
	closegraph();
	return 0;
}

