## Nonlinear Dynamic Inversion

This project provides functions for control design when using Nonlinear Dynamic Inversion (NDI).

## Motivation

NDI (also known as feedback linearization) is a frequently chosen control design technique in the area of flight control.
Therefore, this project provides functions that simplify the design process of such controllers.

## Installation

- You must install [LADAC](../../../README.md).

## Example

### Relative degree 2 (with LQR)

Suppose we have a transformed system (relative degree 2) with NDI and want to control the error dynamics with a state feedback gain designed as linear-quadratic regulator.
Open-loop system as block diagram:  
<pre>
             -----     -----  
            |  1  |   |  1  |  
Delta_ny -->| --- |-->| --- |--> e_y  
            |  s  |   |  s  |  
             -----     -----  
</pre>
1. Define the maximum desired state errors and the maximum desired pseudo-control output (see [1], p. 406, "Maximum Desired Values of z(t) and u(t)"):
	```
	e_x_max = [0.1,1];
	Delta_ny_max = 1;
	```
2. Compute the optimal feedback gain:
	```
	[K,A,B] = ndiFeedbackGainLqr(e_x_max,Delta_ny_max,0)
	```
3. Visualize the open-loop transformed system as block diagram:
	```
	ndiPrintBlocksFromSs(A)
	```
4. Visualize the closed-loop system dynamics:
	```
	ndiPlotClosedLoopErrorDynamics( A, B, K )
	```

### Relative degree 2 with higher order dynamics (with LQR)

Suppose we want to consider higher order dynamics in the above system.
Open-loop system as block diagram:  
<pre>
             -----------     -----     -----   
            |     1     |   |  1  |   |  1  |  
Delta_ny -->| --------- |-->| --- |-->| --- |--> e_y  
            | 0.2*s + 1 |   |  s  |   |  s  |  
             -----------     -----     -----   
</pre>
1. Define the maximum desired state errors and the maximum desired pseudo-control input (see [1], p. 406, "Maximum Desired Values of z(t) and u(t)"):
	```
	e_x_max = [0.1,1,10];
	Delta_ny_max = 1;
	```
2. Define the first-order delay time constant:
	```
	T_h = 0.2;
	```
3. Define that we do not want to feed back the first-order delay input (same feedback variables as in the above example):
	```
	is_fb_deriv = false;
	```
4. Compute the optimal feedback gain:
	```
	[K_h,A_h,B_h] = ndiFeedbackGainLqr(e_x_max,Delta_ny_max,T_h,is_fb_deriv)
	```
5. Compare `K` (first example) and `K_h`. Use the feedback gain of the first example for this system and see that it is less stable due to the first-order delay (that is why it makes sense to consider a first-order delay for control design):
	```
	ndiPlotClosedLoopErrorDynamics( A_h, B_h, [K,0] )
	```
6. Set `is_fb_deriv = true` and compare the results (the performance gets better but you have to decide whether you can measure the input of the first integrator block).

### Pole placement

For the same system as above we want to use pole placement to design the feedback gain.
1. Define the first-order delay time constant:
	```
	T_h = 0.2;
	```
2. Define the desired close-loop eigenvalues:
	```
	p = [-5,-5,-5];
	```
3. Compute the feedback gain:
	```
	[K_p,A_p,B_p] = ndiFeedbackGainPlace(p,T_h)
	```
4. Visualize the closed-loop system dynamics:
	```
	ndiPlotClosedLoopErrorDynamics( A_p, B_p, K_p )
	```

## How to use?

- Read through the [Example](#Example) section.
- Read the function documentations.
- Read the referenced literature if you do not understand the methods.
- Choice of first-order delay time constant:
  - If your higher order dynamics is a first-order time delay, the computed error dynamics will be exact.
  - However, if your higher order dynamics include multiple states (e.g. multiple low pass filters), you should choose T_h such that it approximates the higher order dynamics well.  
    Note that in the real system, the error dynamics might be different because you will lose phase margin when you increase the feedback gain.  
	Thus, make sure that the closed-loop eigenfrequencies are smaller than the inverse neglected (not fed back) higher order dynamics time delay: `omega_max < 1/T_neglected` (output response is slower than neglected higher order dynamics response).
  - For a second order delay with damping ratio of 1: `T_h = 2/omega`
  - For multiple second order delays with damping ratio of 1: `T_h = 2/omega1 + 2/omega2 + ...`
  - Transport delays should be added to the time constant: `T_h = 2/omega + T_transport`

## Literature
[1] Stevens, B. L. et al. (2016): Aircraft Control and Simulation. Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
