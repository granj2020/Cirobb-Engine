# Cirobb V1.1.4

" Tested in Visual Studio 2012"

<b>Cirobb</b> is a <b>2D Rigid Body Physics Engine</b> inspired by <b>Box2D Lite</b> that was created with the purpose of helping people who are starting
in this wonderful world of physics for Videogames. The Engine uses the Erin Catto's Contact Persistence Algorithm
and it's implementation of the <b>PGS</b> (Projected Gauss Seidel) solver to solve the MLCP very well known as a <b>SI</b> (Sequential Impulse).<br>it also has a very simple <b>
2D Collision Detection</b> to Calculate Contact Points, Normal Direction and Penetration. <br>
The main Idea of this project is to explain in more detail how physics engines works,
I must clarify that not all physics engines use the same approach. This engine follows the Impulse Based Dynamics.



If you want to make more complex simulations with many Constraints and Shapes. 
I recommend you use better <b>Box2D</b> which in my opinion I think it's the best 2D open source physics engine that exists.


<h2> Images </h2>

![img1](/images/img1.gif?raw=true)
![img2](/images/img2.gif?raw=true)
![img3](/images/img3.gif?raw=true)

<br>
<h2>Demo Video</h2>

Cirobb Engine V1.1.4 : https://youtu.be/j2p6qmOVA7M

<h2>Books and Resources</h2>

* <b>Read math and physics books and Game Physics</b> 

If you want to better understand how the Physics Engine works I recommend you learn a lot about linear Algebra, Numerical Analysis,
Calculus and Classical Physics. And I Also recommend you these slides and books:

- <b> Erin Catto </b> for his incredible slides and knowledges about Game Physics:

	* Iterative Dynamics with Temporal Coherence.

 	* Modeling and Solving Constraints.
	
	* Soft Constraints.
	
	* Numerical_Methods.
	
	* Continuous Collision Detection.
	
	* Understanding Constraints. 


- <b> Dirk Gregorius </b> For his incredible slides about collision detection:

	* The Separating Axis test Between Convex Polyhedra.
	
	* Robust Contact Creation for physics Simulation.


- <b> Erwin Coumans </b> For his slides and Physics Engine:

	* Forum Bullet.

	* Exploring MLCP Solvers And Featherstone.


- <b> Tonge Richard </b> For his incredible Slides:

	* Iterative Rigid Body Solvers 2012.

	* Iterative Rigid Body Solvers 2013.


- <b> Brian Vincent Mirtich </b> For introducing a different approach to Game Physics.

	* Impulse-based Dynamic Simulation of Rigid Body Systems.


- <b> Kenny Erleben </b>: for his slides and thesis doctoral: 

	* Book of Physics-Based Animation 2005.

	* Numerical Methods for Linear Complementarity Problems in Physics-based Animation.


- <b> David Baraff </b> For his slides and Knowledges:

	* Analytical Methods for Dynamic Simulation of Non-penetrating Rigid Bodies.

	* Fast Contact Force Computation for Nonpenetrating Rigid Bodies.
	
	* Linear-Time Dynamics using Lagrange Multipliers.

	* Physically Based Modeling. Pixar Animation Studios.


- <b> Jim Van Verth </b> For his Slides:

	* Numerical Integration.

	* Understanding Rotations.


- <b> Matthias Müller </b> for his Slides About:

	* Position Based Dynamics.


- <b> Michael B. Cline </b> for his Slides:

	* Post-Stabilization for Rigid Body Simulation with Contact and Constraints.


- <b> Randy Gaul </b> for his Slide: 

	* Separating Axis Test (SAT) and Support Points in 2D.


- <b> Books about Game Physics and Collision Detection </b>: 

	* Game Physics Pearls Gino van den Bergen and Dirk Gregorius.

	* Real-Time Collision Detection Christer Ericson.

<h2> Future </h2>

Cirobb is a Physics Engine that is almost finished. Since the main idea is to create 
a simple introduction to 2D rigid body physics engines few modifications will be added to keep things Simple.

The Following Features will be added to the Cirobb Engine in the future:

* Direct Solver to improve the convergence of the Velocity Solver.

* The Distance Constraint. (Optional?)

* More internal documentation to improve understanding of the code.

* Modification of the .h and .cpp files to Improve understanding of the Code.

* Separate the graphics Library of the Physics Engine.

<h2> About the Author </h2>

Hi, my name is <b>Jonathan Peña</b>, I'm from Venezuela. I really like physics in Videogames,
I like the Dynamics of Physics Engines and how the equations predict movement. You can contact me here: granj215@gmail.com.
