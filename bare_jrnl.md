::: IEEEkeywords
Multiple Target Tracking, Granger Causality, Kalman Filter, Radar
Systems
:::

# Introduction

field of Radar Systems has advanced over the last years both in research
and development. A major challenge in this field is real-time multi
object tracking and trajectory forecasting. Real time objects tracking
is the process of determining the position of moving objects over time
using some measurement and sensors in real time driven data. Trajectory
forecasting refers to the prediction of the described objects' future
positions. One of the challenges when tracking objects is interpreting a
noisy measurement, and understanding it comprehensively. We wish to
focus on a noisy Radar Detection of multiple objects.\
This work's goal is to develop and implement a tracking algorithm that
is able to detect surrounding objects, classify them, estimate their
dynamics, and model their movement to forecast their trajectories. The
immediate goal is to treat the objects' trajectories as an
auto-regression time series and estimate the model's weights by using
positions of detected objects as a measurement in an estimation
algorithm. Using a Kalman Filter (KF) algorithm, we intend to estimate
the position of the objects in the environment by using the Radar System
as a position measurement for each object.\
The problem arises when the objects interact or influence each other,
and the individual dynamics model for each object does not hold.
Moreover, each object's forecasted trajectory is time dependent, as new
measurements are obtained through the sensors (which also include
internal measurement noise). Objects may also hide one another from the
perspective of the Radar and moving objects could unite into one object
or split into two, making it even harder to track each and every
detected object. Thus, forecasting the expected trajectory of the
detected objects is a great challenge as all the factors above lead to
an expanding uncertainty along forecasted the trajectory.

## Objects' Influenced Trajectories

In addition to the difficulties of analyzing a noisy environment, the
forecasted trajectory of the objects may be influenced by other objects
-- an influence that may vary with time and be treated as a time series.
Specifically, one object's movement may be influenced by its' neighbor
objects and vise versa -- an influence that changes with the change of
the objects' positions.

## Granger Causality

Another field that also struggles with estimating the influence of one
time series on another is the field of Economics. For that purpose, the
Granger causality test was developed in 1969 by Clive Granger. The
Granger causality test is a statistical hypothesis test for determining
whether one (or more) time series information is useful in forecasting
another time series. Therefore, a detected objects' trajectory can be
modelled as an autoregressive (AR) time series and by using the Granger
causality test, the influence of one object's forecasted trajectory on
the other objects' forecasted trajectories could be estimated.

## Project's Suggestion

This project suggests estimating the forecasted states of objects
detected by Radar System, using a mathematical time series models and
methods, together with a two-layer estimation algorithm: one for the
targets' location and another for the correlation between their
trajectories.

## Essential Conclusions

Some main conclusions of this work are that the weight estimation and
prediction algorithm (presented in this work) lead to reasonably good
trajectory prediction results. Together with that, it is the slowest
thread in the flow between the sensors and the prediction solution, thus
using a fully generalized AR model could only be relevant to low
dimensional problems (in the case where the algorithm has to run in real
time). The measurement noise passed to the weight's estimation algorithm
by the position estimation influences its prediction abilities, leading
to a noisy RMSE -- but the algorithm still leads to good results in the
sense of RMSE.

# Problem Formulation

The object tracking and trajectory forecasting problem may be formulated
as follows: Assume that at time $t$ there are $n_t$ detected objects.
Let $\mathbf{x}_{1:t}={x_1,…,x_t }$ represent the true positions of all
$n_t$ detections and $\mathbf{z}_{1:t}={z_1,…,z_t}$ be the positions
measurement history up to time $t$. Each measurement $z_i$ is a position
of the object in cartesian co-ordinates, processed from a Radar System
measurement. We assume a dynamic model
$\mathbf{x}_k=f(\mathbf{x}_{k-1},\epsilon_{k})$ such that
$\mathbf{x}_k=F_k \cdot \mathbf{x}_{k-1}+\epsilon_k$, where
$\epsilon_{k}$ is a zero mean random Gaussian variable. We also assume
the measurement model $\mathbf{z}_k=h(\mathbf{x}_{k},\delta_{k})$ such
that $\mathbf{z}_k=\mathbf{x}_k+\delta_k$, where $\delta_k$ is a zero
mean random Gaussian variable. We are concerned with estimating the
expected position of all detected objects, i.e.,
$\hat{\mathbf{x}}_t = \mathop{\mathrm{\mathbb{E}}}[\mathbf{x}_t|\mathbf{z}_{1:t}]$.
Furthermore, we aim to predict the objects' future trajectories, i.e.,
$\hat{\mathbf{x}}_{t+1:t+p} = \mathop{\mathrm{\mathbb{E}}}[{\mathbf{x}}_{t+1:t+p} |\hat{\mathbf{x}}_{t},z_{1:t}]$
and their influence on one another. For that purpose, we are concerned
with estimating the causality of all $n_t$ detected objects on each
object by estimating the auto-regressive time series' last $n$ weights,
i.e.,
$\hat{\mathbf{\alpha}}_t = \mathop{\mathrm{\mathbb{E}}}[{\mathbf{\alpha}}_t|\mathbf{x}_{t-n:t}]$
for each of the detected objects.

# Methodologies

The following section reviews some important methodologies used along
this work, for example the Kalman Filter Gaussian Filter Estimator and
time series models such as the Autoregressive model and the Granger
Causality model.

## Kalman Filter Algorithm

The Kalman filter is a mathematical recursive estimator that works by
using a series of predictions and measurements to update an estimate of
the system state [@Probablistic_Robotics]. It uses a process model to
predict the state of the system at the next time step, and then compares
this prediction to the actual measurement of the system state to compute
the error between the prediction and the measurement. The Kalman filter
then uses this error to update the estimation of the system state,
considering the uncertainty in both the prediction and the measurement.
The Kalman Filter is using the "moments parametrization" a technique for
filtering and prediction of linear Gaussian systems. This means, that at
any time t the prediction is represented by two parameters: a mean
($\mu_n$) and a covariance ($\Sigma_{n\times n} or P$) of the predicted
state. The KF algorithm is shown in Algorithm
[\[alg:cap\]](#alg:cap){reference-type="ref" reference="alg:cap"}.

::: algorithm
::: algorithmic
$Set(Q, R)$ $\hat{\bf{x}}_{k-1|k-1} \gets \bf{x}_0$
$\hat{\bf{P}}_{k-1|k-1} \gets \bf{P}_0$
$\hat{\bf{x}}_{k|k-1} \gets \bf{F}_{k}\bf{x}_{k-1|k-1}$
$\hat{\bf{P}}_{k|k-1} \gets \bf{F}_{k}\bf{P}_{k-1|k-1}\bf{F}_{k}^{T} + Q$
$\Tilde{\bf{y}}_{k} \gets \bf{z}_{k}-\bf{H}_{k}\hat{\bf{x}}_{k|k-1}$
$\bf{S}_{k} \gets \bf{H}_{k}\bf{P}_{k|k-1}\bf{H}_{k}^{T} + R$
$\bf{K}_{k} \gets \bf{P}_{k|k-1}\bf{H}_{k}^{T}\bf{S}_{k}^{-1}$
$\hat{\bf{x}}_{k|k} \gets \hat{\bf{x}}_{k|k-1} + \bf{K}_{k}\Tilde{\bf{y}}_{k}$
$\hat{\bf{P}}_{k|k} \gets (\bf{I} - \bf{K}_{k}\bf{H}_{k})\hat{\bf{P}}_{k|k-1}$
$\hat{\bf{x}}_{k|k} \gets \hat{\bf{x}}_{k|k-1}$
$\hat{\bf{P}}_{k|k} \gets \hat{\bf{P}}_{k|k-1}$ $k-1 \gets k$
:::
:::

## Autoregressive Model

Analyzing the change over time of different sequentially recorded
parameters is referred to as time series analysis. Assuming the time
series is governed by a dynamical law, observations could be made to
study and reveal it. According to an autoregressive model, a future
state could be predicated using the past values in the series as they
are linearly correlated [@TimeSeries]. Assume that the current time
state (that is a part of a time series) $x_t$ is autocorrelated to its
previous time state $x_{t-1}$. It follows thus, that the previous time
state is highly informative when trying to predict the current time
state: $$x_t = \alpha_0 +\alpha_1 x_{t-1} + \epsilon_t$$ The previous
state could not always solely determine the current state, and thus the
model above should be generalized to higher orders of AR. For example,
using the $p$'s order of AR, denoted as AR(p) (assuming that p is a
natural number), the current state would be [@LinTimeSeries]:
$$\label{eq1}
    x_t = \alpha_0 + \sum_{i=1}^{p}\alpha_ix_{t-i} +\epsilon_t$$

## Granger Causality Theorem

The AR model could be generalized to a multi variable time series, and
together with the granger causality theorem [@Granger], could represent
the correlation of two or more time series on one another (their
influence on one another) in the following manner: A time series
granger-cause another time series if it \"helps\" predicting
(forecasting) its next state. Suppose two different time series denoted
as $x_t^{(1)}$ and $x_t^{(2)}$, each could be modeled using
autoregression. $x_t^{(1)}$ is said to "granger-cause" $x_t^{(2)}$, if
the previous states of $x_t^{(1)}$ helps predicting or forecasting
$x_t^{(2)}$ next state. This key idea is demonstrated in the equation
below, representing a bi-variate AR system: $$\label{eq2}
    \begin{cases}
      x_t^{(1)} = \alpha_0^{(1)} + \sum_{i=1}^{p}\alpha_{1,i}^{(1)} x_{t-i}^{(1)} + \sum_{i=1}^{p}\alpha_{2,i}^{(1)} x_{t-i}^{(2)} +\epsilon_t^{(1)}\\
      \\
      x_t^{(2)} = \alpha_0^{(2)} + \sum_{i=1}^{p}\alpha_{1,i}^{(2)} x_{t-i}^{(1)} + \sum_{i=1}^{p}\alpha_{2,i}^{(2)} x_{t-i}^{(2)} +\epsilon_t^{(2)}\\
    \end{cases}$$ Note, that if $\alpha_{i,j}^k=0$ for all $i$'s and for
all $j\neq k$, equation [\[eq2\]](#eq2){reference-type="ref"
reference="eq2"} simplifies to two independent equations similar to
equation [\[eq1\]](#eq1){reference-type="ref" reference="eq1"}. If this
situation does not hold, i.e. The mutual coefficients are not all 0,
that it is said that $x_t^{(1)}$ is said to "granger-cause" $x_t^{(2)}$
or vice versa.

## Coefficients (Weights) Estimator

::: figure*
![image](BlockDiagram.png){width="\\textwidth" height="6cm"}
:::

When generalizing equation [\[eq1\]](#eq1){reference-type="ref"
reference="eq1"} into N objects, of dimension D, with AR model of order
k, the total number of weights could be calculated by using the
following formula: $$\label{eq3}
    [weights] =(n\cdot d\cdot k)\cdot (n\cdot d)=k\cdot (n\cdot d)^2$$
In order to estimate the AR coefficients, this work focuses on using the
Bayesian estimation - and specifically the Kalman Filter. Once the
coefficients are estimated, prediction of the future states will be
possible, as shown in Fig
[\[BlockDiagram_fig\]](#BlockDiagram_fig){reference-type="ref"
reference="BlockDiagram_fig"}. This way, prediction of the forecasted
trajectory of the detected object could be calculated "p" steps ahead --
by propagating the (estimated) position just by multiplying it by the
estimated weights. This way, the algorithm is also considering the
relations (influence) between those objects, as it is reflected in the
weights' values. The objects' dynamics (The change in its position
state) would be treated as a time series that is to be modelled using a
multivariate auto-regressive model, giving each time step its own weight
and allowing a representation of the causality of one object on another.
The model's weights would be estimated by using a Kalman Filter
estimation method, under the assumption that the Posterior could be
represented as a Gaussian. Formally, The Filter solves the problem of
estimating $\bf{\alpha}_t$ i.e. finding the expected value
$\mathop{\mathrm{\mathbb{E}}}[{\mathbf{\alpha}}_t|\mathbf{x}_{t-n:t}]$.
Another Kalman Filter position estimator will be used directly to
estimate the location of the detected objects, by receiving a position
measurement for each object from the Radar, and treating it as point
object tracking. Eventually, the objects' forecasted trajectory will be
derived using both the position estimator and the estimated weights.

## State-Space Model

Assuming the models shown in previous chapters, the suggested weights
Kalman Filter's state space equations are as follows: The estimated
state is made of the coefficients: $\bar{\bf{\alpha}}_t$. The state
transition equation: $$\label{eq4}
    \dot{\bar{\bf{\alpha}{}}}_t = f(\bar{\bf{\alpha}}_t)+v_t$$ Whereas
$f(\bar{\bf{\alpha}}_t)=\bar{\bf{0}}$ is the state transition function
and $v\sim \mathcal{N}(\bar{\bf{0}},\,\sigma_v^2)$ is the process noise.
Equation [\[eq4\]](#eq4){reference-type="ref" reference="eq4"} implies
that the weights are considered as noise. Equation
[\[eq4\]](#eq4){reference-type="ref" reference="eq4"} could be
represented in the discrete form in the following way: $$\label{eq5}
    \bar{\bf{\alpha}}_{k+1} = \bf{I}\cdot\bar{\bf{\alpha}}_k+v_k$$ The
measurement equation could be represented in the following manner:
$$\label{eq6}
    \bar{\bf{y}}_t = h(\bar{\bf{\alpha}}_t)+w_t$$ Where
$h(\bar{\bf{\alpha}}_t)$ is the measurement function which is based on
the history of the position of the detected objects, and
$w\sim \mathcal{N}(\bar{\bf{0}},\,\sigma_w^2)$ is the measurement noise.
As the weights are independent of the location of the detected objects,
and by using the linearity of the AR model, $h(\bar{\bf{\alpha}}_t)$
could be represented as follows: $$\label{eq7}
    h(\bar{\bf{\alpha}}_t) = \bf{H}_t\cdot \bar{\bf{\alpha}}_t$$ Note,
that
$\bf{H}_t = blockdiag(\bar{\bf{x}}_{t-1}^{(1)},\hdots ,\bar{\bf{x}}_{t-K}^{(1)},\hdots, \bar{\bf{x}}_{t-K}^{(N)})$
Where $\bar{\bf{x}}_{t-t_i}^{(j)}$ is the position vector of the $j$'th
object at time $t-t_i$. Next, by receiving the estimated current
position of the objects, the Kalman filter algorithm could be
implemented to estimate the weights. Once having the estimated weights,
the objects' positions could be propagated through time by simply
multiplying them by the weights to predict their future location.

# Simulation

In order to test the algorithm, a simulation was created in MATLAB in
the following manner: The simulation used a "flock" movement using a
model called "Boids Model" [@Boids] -- which is unknown to the
estimation algorithm. The Boids model is a simulation of the movement of
a flock of birds. It was developed by Craig Reynolds in 1986 as a way to
simulate the seemingly chaotic movement of a flock of birds. The model
is based on three simple rules: separation, alignment, and cohesion. The
separation rule causes each bird to try to maintain a certain distance
from its neighbors, the alignment rule causes each bird to try to match
the velocity of its neighbors, and the cohesion rule causes each bird to
try to move towards the center of mass of its neighbors. By following
these simple rules, the movements of a flock of birds can be simulated
in a realistic way. The Boids model has been used in many different
fields, including computer vision, robotics, to study the behavior of
flocks, schools, and swarms. The modelled output data according to the
Boids model in 2D movement was used as ground truth to be compared to.
This ground truth was then noised with an AWGN to demonstrate a
measurement noise.\
In this tested simulation there was only use in the trajectories of the
flock i.e., the birds. There were N=10 boids, K=15 history timesteps,
P=20 predicting steps and D=2 is the dimensionality taken. Note that the
algorithm was written in a generic form making each of these parameter
adjustable. A video representing the "real time" (in simulation)
prediction of the bird was also created, and a snapshot from such video
is shown in Fig [1](#Flock_fig){reference-type="ref"
reference="Flock_fig"}.

![Predicted location of the boids](Flock.png){#Flock_fig
width="\\linewidth"}

In Fig [1](#Flock_fig){reference-type="ref" reference="Flock_fig"}, the
black dots represent each objects' K history steps taken into
consideration when calculating the weights. The colored dots are the P
predicted steps by using the AR model with the calculated weights. An
example of a full trajectory and Kalman Filter Solution is shown in Fig
[2](#ARKF_fig){reference-type="ref" reference="ARKF_fig"}.

![Auto-regressive Kalman Filter Solution and
Reference](ARKF.png){#ARKF_fig width="\\linewidth"}

The dotted black line is the reference ground truth, and the colored
lines are the estimated movement of the boids. Note that the
imperfection of estimation is due the both the mismatch between the AR
model and the Boids model, and to measurement noise implemented.

# Results

To analyze the performance of the suggested algorithm, shown below in
Fig [3](#RMSE_fig){reference-type="ref" reference="RMSE_fig"} an RMSE
error of the estimated current state, while the measurement error is 10
\[m\] (std) along every axis:

![Estimation RMSE](RMSE.png){#RMSE_fig width="\\linewidth"}

It could be easily seen that the performance of the algorithm is better
than just taking the measurement into account, that would lead to a
10\[m\] error. Furthermore, The RMSE of the 20 timesteps predicted
states along each axis could be seen in Fig
[4](#PredRMSE_fig){reference-type="ref" reference="PredRMSE_fig"}.

![Prediction RMSE](PredRMSE.png){#PredRMSE_fig width="\\linewidth"}

Besides a few peaks, the predicted state settles at a value of
approximately 20\[m\], which is also a very good result as it manages to
keep a small error even when predicting 20 states ahead. The peaks are
mostly due to sudden changes in the boids movement, most of which were
causes by a geometric constraint implemented in the simulation (map
boundaries). Lastly, in order to examine the influence of the sensors
noise (measurement noise), a compare between a few noise orders was
conducted, leading to the RMSE results shown in Fig
[5](#SensPredRMSE){reference-type="ref" reference="SensPredRMSE"}

![Prediction RMSE for different measurement
noises](SensPredRMSE.png){#SensPredRMSE width="\\linewidth"}

As expected, the higher measurement noise leads to a less smooth
prediction RMSE. Note, that the prediction RMSE is actually larger at
most of the iterations when the measurement noise is larger but is still
reasonably close to the scenarios with more accurate measurements. This
result implies that the assumptions taken when using the AR model are
reasonable and could handle difficult scenarios in the sense of
measurement noise.

# Conclusions

The main conclusions that come up from the work done are that the weight
estimation algorithm, according to the AR model, allowed an accurate
prediction which was mainly limited by the sensors' accuracy (the
measurement noise). The measurement noise passed to the weight's
estimation algorithm by the position estimation Kalman filter influences
its prediction abilities, leading to a noisy RMSE -- but still keep its
reasonably good. Once Estimating the coefficients, the causality of one
object on another is qualitifed, and could be interpreted mathematically
(for example: the direction of causality, the influce amount and more).

::: thebibliography
1

Anton Vančo, \"Quantification of causal interactions in complex
systems\", Master Thesis, Comenius University, Bratislava. Faculty of
Mathematics, Physics and Informatics. Department of Applied Informatics.

Sebastian Thrun, Wolfram Burgard, Dieter Fox. "Probabilistic Robotics".
The MIT Press, Cambridge, Massachusetts, London, England.

G.E.P. Box, G.M. Jenkins, G.C. Reinsel. (1994). "Time Series Analysis:
Forecasting and Control". Prentice Hall, Englewood Cliffs.

Ruey S. Tsay. (2010). "Linear Time Series Analysis and Its
Applications". Wiley Series in Probability and Statistics.

Granger, C. W. J. (1969). \"Investigating Causal Relations by
Econometric Models and Cross-spectral Methods\". Econometrica. 37 (3):
424--438.

Reynolds, Craig (1987). Flocks, herds and schools: A distributed
behavioral model. SIGGRAPH '87: Proceedings of the 14th Annual
Conference on Computer Graphics and Interactive Techniques. Association
for Computing Machinery. pp. 25--34.
:::
