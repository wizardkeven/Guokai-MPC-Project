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

* x`t+1` = x`t` + v`t` * _cos(_psi`t`_)_ * dt                      
* y`t+1` = y`t` + v`t` * _sin(_psi`t`_)_ * dt
* psi`t+1` = psi`t` - v`t`/Lf * delta`t` * dt
* v`t+1` = v`t` + a`t` * dt
* cte`t` = _f_(x`t`) - y`t` + v`t`*_sin(_epsi`t`_)_*dt
* epsi`t+1` = (psi`t` - psi`{des}t`) - v`t`/Lf * delta`t`*dt

Here I followed the tips and substituted the *+* with *-* in **psi** and **epsi** update equations. In the **cte** update equation, _f_ is the polynomial fitted from reference points on map in _main.cpp_. 


## 2. Timestep Length N and Elapsed Duration dt

### Reasoning behind the chosen N and dt values
At the beginning, we are just given the rough range of N and dt. As described on the lecture, T should be in a few seconds which means the product of N and dt should be in this range. Moreover, the simulator is added in a latency of a millisecond which means dt should be around 100 ms or 0.1 s. For N, it is related to the speed of the vehicle. Suppose we set **v** as 100 mps or 160 kph, then if N is set to 50 then the model will compute and predict a distance of **_160kph * 3.6s/m * 50 * 0.1 = 2880m_**. Well I don't believe our model can give appropriate prediction and actuation parameters on such a distance taking the potential **cte** and **epsi** into consideration. I set reference velocity as 50 mph. In this condition is about 1400 m which is still kinda too large for me. So I start with N = 10 and dt = 0.1 as shown in the Q&A video. 

Without considering the factors of weights in cost function, N and dt can also be observed the obvious effect on MPC. When the car is turning dramatically before or after a curve disproportionally to the current cte or keeping running straight upon a curve, this is probably caused by inappropriately set N and dt. I was not clear about this mechanism during I was tuning the weights of cost function. In fact, it took me many rounds running the simulator to come into this conclusion. 


## 3. Polynomial Fitting and MPC Preprocessing

As mentioned above, I c

## 4. Model Predictive Control with Latency

The final N is set to 10 and dt is set to 0.2. I was just setting N to 10 and dt to 0.1 as in Q&A video until I was about to finish tuning weights. I found that no matter how I changed the weights the vehicle was just not able to run correctly upon the curve and right after the curve. Then i start to think the ralation between dt, T and latency. As vehicle latency is set to 0.1 s, it took at least 0.1 s from sending control command to executing the movement on vehicle actuator. Taking the processing time and other mechanical latency, dt should be set around 0.2 s.

Based on the hypothesis above, I tried 0.05 for dt to prove my hypothesis on the minimum value 0.1 and the test result is that the vehicle is rushing into curb right away. Then I tried 0.3, 0.25, 0.2, 1.5, 1.3, 1.7. Among these values, only 0.2 works well on my computer. 

As to N, I tried 10, 7, 15, 20, 30. Too big N leads to nonsynchronous steering output not long after the starting point. While too small N will cause lagging.
