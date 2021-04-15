# Assignment 3 - Boids

## Introduction

In this assignment, I implement a particle system for simulating the flocking behavior of birds based on *Boids [Reynolds(1987)]*. 

In the case *FreeFall* and *Circular Motion*, you can test the **Time Integration Algorithms** including **Basic Time Integration, Symplectic Euler** and **Explicit Midpoint**. In the case *Cohesion, Alignment, Separation* and *Collision Aviodance*, you can test the Flocking behavior of boids. In the case *Leading* and *Collaborative & Adversarial*, you can test how the boids follow the leader and how two boids fight with each other in a bounded habitat, which might be the most interesting part of this assignment.

You can see the demos below.

This assignment is accomplished by **C++**, heavily based on **Eigen**. The main implementation is in the file **boids.h** and **main.cpp**.

TAs provide a simple starter code skeleton including basic GUI functionality and particle drawing, but I changed them for my convenience.

## Demos: check them in /mp4_media

### click pictures to see live demos

### FreeFall

Boids are initiated at random positions and have zero velocity. Then they fall because of gravity.

[![freefall](https://user-images.githubusercontent.com/39910677/114882364-2bfd2400-9e04-11eb-8b79-ce679c9bf399.png)](https://www.youtube.com/watch?v=JETY0oluqXg&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=7)

### Circular Motion

Boids are rotating w.r.t. the origin. In this case, you can test the difference between three time integration algorithms. (By change *updateMode* in *boids.h*.)

**All three integration schemes perform well when stepsize h is small (h=0.0005), but basic time integration and explicit midpoint diverge when the stepsize is big (h = 1). Midpoint diverges more slowly than basic integration. But symplectic euler method is still stable.**

[![circular](https://user-images.githubusercontent.com/39910677/114882683-7b435480-9e04-11eb-9c75-c4a7863ddeb8.png)](https://www.youtube.com/watch?v=Lnw2bfIW4pk&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=9)

### Cohesion

Birds have the tendency to stay close to their neighbors. (Position Control)

[![cohesion](https://user-images.githubusercontent.com/39910677/114882853-ab8af300-9e04-11eb-8562-155ae65a3b7a.png)](
https://www.youtube.com/watch?v=-V6rMuqULQo&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=8)

### Alignment

In addition to moving toward the average position of neighboring birds, each bird now also wants to match the average direction of the others. (Velocity Control)

[![alignment](https://user-images.githubusercontent.com/39910677/114883078-decd8200-9e04-11eb-80f4-0729224419a1.png)](
https://www.youtube.com/watch?v=2hg85VVZ670&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=11)

### Separation

When birds come too close to each other, they separate such as to avoid collisions. Add repulsive forces that avoid overcrowding.

[![separation](https://user-images.githubusercontent.com/39910677/114883230-03295e80-9e05-11eb-86b8-f54fa3a969cf.png)](
https://www.youtube.com/watch?v=re6e6dt_byg&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=1)

### Collision Avoidance

Implement a collision avoidance strategy where birds should steer around a given circular obstacle.

[![coll](https://user-images.githubusercontent.com/39910677/114883400-2e13b280-9e05-11eb-9bb3-f8601d88f8f0.png)](
https://www.youtube.com/watch?v=8ywFnugLdJE&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=7)

### Leading

One blue bird is designated as the leader whose motion the remaining red birds should follow. 

[![leading](https://user-images.githubusercontent.com/39910677/114883538-4edc0800-9e05-11eb-82eb-3cc62458e0a5.png)](
https://www.youtube.com/watch?v=fS_ndt58cZo&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=5)

### Collaborative & Adversarial

 ***Assumptions:***

* Two boids only breed on April Fool's Day in the spring of each year. (Breeding Season)
* If two birds from the same group are sufficiently close, a third one is created. (Breed)
* The child bird's position and velocity are the means of its parents.
* If three birds from the same group are close to a bird from the other group, the latter one is killed, i.e. removed from the system. (Attack)
* The habitat is bounded and the resource is limitted.
* If one bird is surrounded by six birds from the same group, and the distance between them is smaller than 60% of the separation distance, the bird is dead of hunger. (Avoid Overcrowded)

[![ca](https://user-images.githubusercontent.com/39910677/114883719-77640200-9e05-11eb-8616-ccda6a47ecae.png)](
https://www.youtube.com/watch?v=038QWXIv_R0&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=10)

As you can see, the numbers of two boids have logistic growth. Same initial conditions and same breed/attack rule, the fate of the two boids is random.

But what if you apply control strategies to the red boids? Can they win the death match?
 
 ### ***Strategy 1: Seize the origin, quick attack and quick retreat***
 
By quick attack, the red boids seize more habitat. By retreat, the red boids consolidate population superiority. Then they win the game.

[![s1](https://user-images.githubusercontent.com/39910677/114884054-c01bbb00-9e05-11eb-9b74-96103b2a4053.png)](
https://www.youtube.com/watch?v=waI9HHkfRm8&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=4)

### ***Strategy 2: Take advantage of local majority***
 
By Lanchester's N-square law of war, the red boid should attack the border of the blue bird group and take advantage of the local majority. In implementation, the red boids are chasing the rightest blue bird. The winning probability is around 80% ( win 16 times in 20 trials).

[![s2](https://user-images.githubusercontent.com/39910677/114884277-f0635980-9e05-11eb-877b-6a73bf384ae8.png)](
https://www.youtube.com/watch?v=NmD2O9MXY5k&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=3)

### ***Strategy 3: Warriors and breeders***

Half of the red birds are male. They are stronger and more aggressive. They are warriors and attack the enemy. Half of the red birds are female. They are breeders, following the boids, breeding, and supporting the war. For warriors, they  won't go too deep into the enemy's boids. The code implementation is adapted from the *Collision Avoidance*.

[![s3](https://user-images.githubusercontent.com/39910677/114884612-31f40480-9e06-11eb-8fff-56798316f97a.png)](
https://www.youtube.com/watch?v=SavlOi1UrcE&list=PLWVHPmzDfDplsOPVaa_Z4VhxtUqWyCyGT&index=2)

  
## How to get started

1. git clone the source code
2. ```~$ cd build/src/app```
3. open GUI ```~$ ./app```
4. Enjoy it. 

Tips: 

* Press *space* for pause, and press *R* for re-init. It automatically re-init if you change the current case.
* ```~$ make``` in the /build/src/app to compile

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














