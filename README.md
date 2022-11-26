# AutoRegKF
Auto-Regressive model Kalman Filter Estimation:

The field of computer vision and image processing has advanced over the last years both in research and development. A major challenge in this field is real time multi object tracking and trajectory forecasting.
Real time objects tracking is the process of determining the position of moving objects over time using a camera and other sensors in real time driven data.

Trajectory forecasting refers to the prediction of the described objects’ positions.
Tracking objects holds the challenge of analyzing a stochastic environment and understanding it comprehensively.
To sense the environment, one may use  different sensors simultaneously – each yielding its own observations.

In addition to the difficulties of analyzing a stochastic environment, the forecasted trajectory of the objects may be influenced by other objects – and influence that may vary with time and be treated as a time series.
The field of Economics also deals with the struggle of estimating the influence of one time series on another.
For that purpose, the Granger causality test was developed in 1969 by Clive Granger.

The Granger causality test is a statistical hypothesis test for determining whether one (or more) time series information is useful in forecasting another time series.
Therefore, a detected objects’ trajectory can be modelled as an autoregressive time series and by using the Granger causality test, the influence of one objects on the other can be predicted.

This project suggests estimating the forecasted states of objects, using a mathematical time series models and methods, together with a two-layer estimation algorithm: one for the targets’ location and another for the correlation between them.


**The ground truch data was created using Boids model:

https://github.com/ahn1340/Boids.git