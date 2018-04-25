# Model Predictive Control


## 1.The Model
### State, Actuators and Update Equations

First of all, let's discuss the state, actuators and update equations.

Same as what is described in lecture, the model consists of five states:

* **px**: coordinates of vehicle.
* **py**: coordinates of vehicle.
* **psi**: orientation in radian of vehicle.
* **v**: speed in mph of vehicle.
* **cte**: cross track error.
* **epsi**: orientation error which indicates the difference between the current orientation and the desired orientation.

In order to calculate actuators, we also need pass the coefficients(**coeffs** in code) of the approximate polynomial to our MPC Model. We can do this by passing the given reference points to the helper function _polyfit_. Followed the suggestions in lecture, I transformed the reference points and state points of vehicle with vehicle coordinates system. Therefore, **px**, **py** and **psi** are all zeros in the view of the vehicle coordinate system. Refered in code between 103~105 in main.cpp as follow:

```c++
//calculate reference point coordinates relative to car in map coordinate-axises
double shift_x = ptsx[i] - px;
double shift_y = ptsy[i] - py;

//convert to car coordinates-axis
ptsx[i] = shift_x * cos(0 - psi) - shift_y * sin(0 - psi);
ptsy[i] = shift_x * sin(0 - psi) + shift_y * cos(0 - psi);
```
I pass the **state** into the model and the model will calculate the output actuator vectors based on the equations as below:

* x`t+1` = x`t` + v`t` * cos(psi`t`) * dt  
* y`t+1` = y`t` + v`t` * sin(psi`t`) * dt
* psi`t+1` = psi`t` - v`t`/Lf * delta`t` * dt
* v`t+1` = v`t` + a`t` * dt
* cte`t` = f(x`t`) - y`t` + v`t`*sin(epsi`t`)*dt
* epsi`t+1` = (psi`t` - psi`{des}t`) - v`t`/Lf * delta`t`*dt

Here I followed the tips and substituted the *+* with *-* in **psi** and **epsi** update equations.
### Prediction and Update



## 2. Timestep Length and Elapsed Duration (N & dt)

### Reasoning behind the chosen N and dt values
### the previous values tried.

## 3. Polynomial Fitting and MPC Preprocessing

## 4. Model Predictive Control with Latency
The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
