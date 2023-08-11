# pyMPC-ProxQP

### WIP alert: The code within this reposity is in a WIP state and may not have undergone rigorous testing and may exhibit instability. Use at your own discretion.
This repository is a fork from [forgi86/pyMPC](https://github.com/forgi86/pyMPC/blob/master/pyMPC/mpc.py) with an additional feature that replaces OSQP with ProxQP. Preliminary experiments on `examples/example_inverted_pendulum.py` sometime ran faster than OSQP.

## Requirements

pyMPC requires the following packages:
* numpy
* scipy
* [OSQP](https://osqp.org/)
* [proxsuite](https://github.com/Simple-Robotics/proxsuite)
* matplotlib


## Usage 

This code snippets illustrates the use of the MPCController class:

```
from pyMPC.mpc import MPCController

K = MPCController(Ad,Bd,Np=20, x0=x0,xref=xref,uminus1=uminus1,
                  Qx=Qx, QxN=QxN, Qu=Qu,QDu=QDu,
                  xmin=xmin,xmax=xmax,umin=umin,umax=umax,Dumin=Dumin,Dumax=Dumax)
K.setup()

...

xstep = x0
for i in range(nsim): 
  uMPC = K.output()
  xstep = Ad.dot(xstep) + Bd.dot(uMPC)  # system simulation steps
  K.update(xstep) # update with measurement
```
Full working examples are given in the [examples](examples) folder:
 * [Point mass with input force and friction](examples/example_point_mass.ipynb)
 * [Inverted pendulum on a cart](examples/example_inverted_pendulum.ipynb)
 * [Inverted pendulum on a cart with kalman filter](examples/example_inverted_pendulum_kalman.ipynb)

## Acknoledgment
This code have been directly source from [forgi86/pyMPC](https://github.com/forgi86/pyMPC/blob/master/pyMPC/mpc.py). We are grateful to the original authors and maintainers for their work.
