# Model Predictive Control


## 1. Student describes their model in detail. This includes the state, actuators and update equations.

Same to what is described in lecture, the model consists of five states:

* **px**: coordinates of vehicle.
* **py**: coordinates of vehicle.
* **psi**: orientation in radian of vehicle.
* **v**: speed in mph of vehicle.
* **cte**: cross track error.
* **epsi**: orientation error which indicates the difference between the current orientation and the desired orientation.

Before passing this state vector, I have them transformed to vehicle coordinates system as recommended. Therefore, **px**'
I pass the **state** into the model and the model will calculate the output actuator vectors based on the equations as below:

* x`t+1` = x`t` + v`t` * cos(psi`t`) * dt 
* y`t+1` = y`t` + v`t` * sin(psi`t`) * dt
* psi`t+1` = psi`t` - v`t`/Lf * delta`t` * dt
* v`t+1` = v`t` + a`t` * dt
* cte`t` = f(x`t`) - y`t` + v`t`*sin(epsi`t`)*dt
* epsi`t+1` = (psi`t` - psi`{des}t`) - v`t`/Lf * delta`t`*dt

Following the tips I substide 






## 2. Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

## 3. Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

## 4. The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
