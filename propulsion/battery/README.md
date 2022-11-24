# Battery Discharge Model

The battery discharge model of Tremblay is used [1].
However, the formulation is modified:

The battery discharge model of Tremblay uses absolute parameter values like battery capacity, voltage, internal resistance as well as hyper parameters that depend on these absolute values etc.
However, default battery parameters could be used for a wide range of different batteries as long as they are made of the same material (e.g. LiPo).
Here, LiPo battery parameters are normalized so that LiPo batteries of different capacities and voltages can be simulated based on the same default parameters.

## Example

Run the following script and take a look at the figures as well as the code:
```
battery_example
```

## How to use?

Beside the example script there are two Simulink blocks.
Since the coupling of the battery block `Dynamic Battery (Tremblay)` and the `Brushless DC Motor` block in the library `actuators_lib` causes an algebraic loop, the second battery block `Dynamic Battery (Tremblay) without algebraic loop` block was introduced which can be coupled with the `Brushless DC Motor`.
In the `actuators_lib` there is also a 2nd order motor model which can be coupled with the first battery model without algebraic loop.
Note that the algebraic loop causes a warning but neither decreases the simulation speed nor yields wrong results.

## Literature
[1] Tremblay, O., & Dessaint, L. A. (2009). Experimental validation of a battery dynamic model for EV applications. World electric vehicle journal, 3(2), 289-298.
