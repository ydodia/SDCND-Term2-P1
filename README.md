# Extended Kalman Filter - SDCND Term 2 P1

In this project, I used an *Extended Kalman Filter* to track a vehicle moving in a figure-eight pattern. The example two datasets start the vehicle in opposite directions.

To compile and run the C++ code, follow the steps below:

1. cd build
2. cmake ..
3. make
4. ./ExtendedKF

Then, run the simulator and my code will track the simluated vehicles path.

## Results
My implementation results in RMSE values as follows for Dataset 1:

`
X  : 0.0973
Y  : 0.0854
Vx : 0.4512
Vy : 0.4396 .
`

