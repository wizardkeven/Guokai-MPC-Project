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

The final N is set to 10 and dt is set to 0.2. I was just setting N to 10 and dt to 0.1 as in Q&A video until I was about to finish tuning weights. I found that no matter how I changed the weights the vehicle was just not able to run correctly upon the curve and right after the curve. Then i start to think the ralation between dt, T and latency. As vehicle latency is set to 0.1 s, it took at least 0.1 s from sending control command to executing the movement on vehicle actuator. Taking the processing time and other mechanical latency, dt should be set around 0.2 s.

Based on the hypothesis above, I tried 0.05 for dt to prove my hypothesis on the minimum value 0.1 and the test result is that the vehicle is rushing into curb right away. Then I tried 0.3, 0.25, 0.2, 1.5, 1.3, 1.7. Among these values, only 0.2 works well on my computer. 

As to N, I tried 10, 7, 15, 20, 30. Too big N leads to nonsynchronous steering output not long after the starting point. While too small N will cause lagging.


## 3. Polynomial Fitting and MPC Preprocessing

The reference path polynomial is calculated from the given global coordinates of a series of points. I convert all these points to vehicle coordinate system before passing to the helper function _polyfit_ as below:
```c++
for(int i = 0; i< ptsx.size(); i++)
{
  //calculate reference point coordinates relative to car in map coordinate-axises
  double shift_x = ptsx[i] - px;
  double shift_y = ptsy[i] - py;

  //convert to car coordinates-axis
  ptsx[i] = shift_x * cos(0 - psi) - shift_y * sin(0 - psi);
  ptsy[i] = shift_x * sin(0 - psi) + shift_y * cos(0 - psi);
}
```

The MPC model will calculate a optimized control output **delta** and **a** based on current vehicle state and reference path from map. The main code on this part has been given by lecture and the Q&A video. What left to do is to set the factors and weights of cost functions. See below:

```c++
for(int i =0; i< N; i++)
{
  fg[0] += 4000*CppAD::pow(vars[cte_start + i] - ref_cte, 2);   ----1
  fg[0] += 2000*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2); ----2
  fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);            ----3
}

for(int t = 0; t < N-1; t++)
{
  fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);             ----4
  fg[0] += 5*CppAD::pow(vars[a_start + t], 2);                 ----5
}

// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) 
{
  fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2); ----6
  fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);          ----7
}
```
The main values that I observe is the state correctness which consists of **cte**(cross-track error), **epsi**(orientation error) and difference of current velocity from the set velocity (velocity error), security which consists of **delta**(steering angle) and **a**(acceleration) and driving stableness which consist of quantity of steering angle and acceleration difference between tow consecutive actuations.

Tuning these weights of cost function is definitely the core challenge and time consuming work in this project. Although the Q&A video really helps, that is just a starting point. If we set the parameter as it, the vehicle runs catastrophically. Additionally, we can only get the intuition on the effects every weights give through the real experimentations. 

Weights of 1,2,3 will decide how many efforts the model will put on keeping the vehicle with reference set values: cte_ref, epsi_ref and v_ref. In my reasoning, keeping the car on track is the most important thing compared to keeping the desired velocity. The related part are weight 1 and 2. As the fitted reference path polynomial could contain error resulted from inaccurate map coordinates. There could also be inevitable error on current orientation angle and desired orientation angle because the vehicle may be trying to turn towards the desired position as it is not deviated while the latency and other factors is causing latency or errors. So I put weight 1 to very large value 4000 and weight 2 to 2000 to tell the model to punish heavily and weight 3 to 1 to tell the model to pay some attention on velocity error.

Weight 4 and 5 will tell the model to punish dramatic steering and acceleration. As most of the road is straight line vehicle will not need to turn or accelerate violentely or 

## 4. Model Predictive Control with Latency

To take the latency into account, I pass the future state to model. Here I didn't use exactly 100 ms or 200 ms as the timestep discussed in first part but a 0.05 ms because I found this works well in the current model and my computer. Theoretically this _dt_ should be equal to the dt in model, but as it works well, I decide not to modified this. In fact I tried with 100 ms but the vehicle will always be ahead of the beginning of the planned path line and the reference path line which I think 100 ms might be too big to the current system. In addition I doubt the received state values from the simulator is not synchronous with the real vehicle state at that time point. 
