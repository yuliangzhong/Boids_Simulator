# Assignment 3 - Boids

## Introduction

In this assignment, I implement a particle system for simulating the flocking behavior of birds based on *Boids [Reynolds(1987)]*. 

In the case *FreeFall* and *Circular Motion*, you can test the **Time Integration Algorithms** including **Basic Time Integration, Symplectic Euler** and **Explicit Midpoint**. In the case *Cohesion, Alignment, Separation* and *Collision Aviodance*, you can test the Flocking behavior of boids. In the case *Leading* and *Collaborative & Adversarial*, you can test how the boids follow the leader and how two boids fight with each other in a bounded habitat, which might be the most interesting part of this assignment.

You can see the demos below.

This assignment is accomplished by **C++**, heavily based on **Eigen**. The main implementation is in the file **boids.h** and **main.cpp**.

TAs provide a simple starter code skeleton including basic GUI functionality and particle drawing, but I changed them for my convenience.

## Demos
* FreeFall
Boids are initiated at random positions and have zero velocity. Then they fall because of gravity.

![freefall](https://user-images.githubusercontent.com/39910677/114870800-df601b80-9df8-11eb-8a39-f11e25cf5fa6.gif =100)

* Circular Motion
Boids are rotating w.r.t. the origin. In this case, you can test the difference between three time integration algorithms. (By change *updateMode* in *boids.h*.)

**All three integration schemes perform well when stepsize h is small (h=0.0005), but basic time integration and explicit midpoint diverge when the stepsize is big (h = 1). Midpoint diverges more slowly than basic integration. But symplectic euler method is still stable.**

https://user-images.githubusercontent.com/39910677/114868723-868f8380-9df6-11eb-92e5-d1868be5f33e.mp4

* Cohesion
Birds have the tendency to stay close to their neighbors. (Position Control)

https://user-images.githubusercontent.com/39910677/114868749-8db69180-9df6-11eb-8758-fa04779c7e81.mp4

* Alignment
In addition to moving toward the average position of neighboring birds, each bird now also wants to match the average direction of the others. (Velocity Control)

https://user-images.githubusercontent.com/39910677/114868801-9ad38080-9df6-11eb-987a-51d4002bd4d5.mp4

* Separation
When birds come too close to each other, they separate such as to avoid collisions. Add repulsive forces that avoid overcrowding.

https://user-images.githubusercontent.com/39910677/114867424-16ccc900-9df5-11eb-86e5-f8cfe001064e.mp4

* Collision Avoidance
Implement a collision avoidance strategy where birds should steer around a given circular obstacle.

https://user-images.githubusercontent.com/39910677/114868833-a32bbb80-9df6-11eb-9423-701700f347cc.mp4

* Leading
One blue bird is designated as the leader whose motion the remaining red birds should follow. 

https://user-images.githubusercontent.com/39910677/114868861-ab83f680-9df6-11eb-9d2d-9371bff44842.mp4

* Collaborative & Adversarial

 ***Assumptions:***

* Two boids only breed on April Fool's Day in the spring of each year. (Breeding Season)
* If two birds from the same group are sufficiently close, a third one is created. (Breed)
* The child bird's position and velocity are the means of its parents.
* If three birds from the same group are close to a bird from the other group, the latter one is killed, i.e. removed from the system. (Attack)
* The habitat is bounded and the resource is limitted.
* If one bird is surrounded by six birds from the same group, and the distance between them is smaller than 60% of the separation distance, the bird is dead of hunger. (Avoid Overcrowded)

https://user-images.githubusercontent.com/39910677/114868890-b3439b00-9df6-11eb-9e60-fd571a3dc3d4.mp4

As you can see, the numbers of two boids have logistic growth. Same initial conditions and same breed/attack rule, the fate of the two boids is random.

But what if you apply control strategies to the red boids? Can they win the death match?
 
 ***Strategy 1: Seize the origin, quick attack and quick retreat***
 
https://user-images.githubusercontent.com/39910677/114869017-d1a99680-9df6-11eb-94bf-b5c926141057.mp4

By quick attack, the red boids seize more habitat. By retreat, the red boids consolidate population superiority. Then they win the game.

 ***Strategy 2: Take advantage of local majority***
 
https://user-images.githubusercontent.com/39910677/114869033-d79f7780-9df6-11eb-9a34-a2dbaa0dfd08.mp4

By Lanchester's N-square law of war, the red boid should attack the border of the blue bird group and take advantage of the local majority. In implementation, the red boids are chasing the rightest blue bird. The winning probability is around 80% ( win 16 times in 20 trials).

***Strategy 3: Warriors and breeders***

https://user-images.githubusercontent.com/39910677/114869405-382eb480-9df7-11eb-8895-e2f504fbd0e7.mp4

Half of the red birds are male. They are stronger and more aggressive. They are warriors and attack the enemy. Half of the red birds are female. They are breeders, following the boids, breeding, and supporting the war. For warriors, they  won't go too deep into the enemy's boids. The code implementation is adapted from the *Collision Avoidance*
  
## How to get started

1. git clone the source code
2. ```~$ cd build/src/app```
3. open GUI ```~$ ./app```
4. Enjoy it. 

Tips: press *space* for pause, and press *R* for re-init. It automatically re-init if you change the current case.

## Code Annotation

The code is annotated in detail so you can easily understand each part and play around. 

Some highlights: 

* ```void drawNanoVG()```
 for drawing the boids, obstacles, leader target pos, ...
* ```void mouseButtonPressed(int button, int mods)```
for getting mouse click positions.
* ```void initializePositions(MethodTypes type = FREEFALL)```
for initiating boids pos and vel w.r.t. different case.
* ```TVStack Xupdate(TVStack pos, TVStack vel, bool if_half_h = false)```
```TVStack Vupdate(TVStack vel, TVStack acc, bool if_half_h = false)```
 for pos and vel update.
* ```TVStack getAcc(MethodTypes type, TVStack pos)```
for implementing different control strategies in different cases.
* ```void updateBehavior(MethodTypes type)```
for pos and vel updating.














