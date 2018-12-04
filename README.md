# Estimation Project - Writeup

### 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data. 

*The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements)*

I've determined the standar deviation of GPS and Accelerometer data as follows: 

I took the data from `Graph1.txt` and `Graph2.txt` and extracted the standar devitiation using the python code:

```
import numpy as np

gps_x = np.loadtxt('./config/log/Graph1.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
acc_x = np.loadtxt('./config/log/Graph2.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]

gps_x_std = np.std(gps_x)
print(gps_x_std)
acc_x_std = np.std(acc_x)
print(acc_x_std)
```

The results:

```
MeasuredStdDev_GPSPosXY = 0.710771593538
MeasuredStdDev_AccelXY = 0.489659566399
```

### 2. Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.

*The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.*

The purpose of this task is to find the estimated Pitch and Roll angles.

I completed this task in C++ as follows:

```
float phi = rollEst;
float theta = pitchEst;

Mat3x3F rot = Mat3x3F::Zeros();
rot(0, 0) = 1;
rot(0, 1) = sin(phi) * tan(theta);
rot(0, 2) = cos(phi) * tan(theta);
rot(1, 1) = cos(phi);
rot(1, 2) = -sin(phi);
rot(2, 1) = sin(phi) / cos(theta);
rot(2, 2) = cos(phi) / cos(theta);

V3F angle_dot = rot * gyro;

float predictedRoll = rollEst + dtIMU * angle_dot.x;
float predictedPitch = pitchEst + dtIMU * angle_dot.y;
ekfState(6) = ekfState(6) + dtIMU * angle_dot.z;

// normalize yaw to -pi .. pi
if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;
```

### 3. Implement all of the elements of the prediction step for the estimator.

*The prediction step should include the state update element (PredictState() function), a correct calculation of the Rbg prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.*

I implemented this step in C++ as follows:

In the `PredictState()` function:

```
predictedState(0) = curState(0) + dt * curState(3);
predictedState(1) = curState(1) + dt * curState(4);
predictedState(2) = curState(2) + dt * curState(5);

V3F acc_world = attitude.Rotate_BtoI(accel);

predictedState(3) = curState(3) + dt * acc_world.x;
predictedState(4) = curState(4) + dt * acc_world.y;
predictedState(5) = curState(5) + dt * (acc_world.z - CONST_GRAVITY);
```


Rbg Prime is calculated in `GetRbgPrime` function:


```
float cosTheta = cos(pitch);
float sinTheta = sin(pitch);

float cosPhi = cos(roll);
float sinPhi = sin(roll);

float sinPsi = sin(yaw);
float cosPsi = cos(yaw);

RbgPrime(0, 0) = -cosTheta * sinPsi;
RbgPrime(0, 1) = -sinPhi * sinTheta * sinPsi - cosTheta * cosPsi;
RbgPrime(0, 2) = -cosPhi * sinTheta * sinPsi + sinPhi * cosPsi;

RbgPrime(1, 0) = cosTheta * cosPsi;
RbgPrime(1, 1) = sinPhi * sinTheta * cosPsi - cosPhi * sinPsi;
RbgPrime(1, 2) = cosPhi * sinTheta * cosPsi + sinPhi * sinPsi;
```

implement the rest of the prediction step (predict the state covariance forward) in `Predict()`:

```
gPrime(0, 3) = dt;
gPrime(1, 4) = dt;
gPrime(2, 5) = dt;

gPrime(3, 6) = (RbgPrime(0) * accel).sum() * dt;
gPrime(4, 6) = (RbgPrime(1) * accel).sum() * dt;
gPrime(5, 6) = (RbgPrime(2) * accel).sum() * dt;

ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
```

### 4. Implement the magnetometer update.

*The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).*

I implemented this step in C++ as follows:

```
zFromX(0) = ekfState(6);
float yaw_diff = magYaw - ekfState(6);
if (yaw_diff > F_PI) zFromX(0) += 2.f*F_PI;
if (yaw_diff < -F_PI) zFromX(0) -= 2.f*F_PI;

hPrime(0, 6) = 1;
```

### 5. Implement the GPS update.

*The estimator should correctly incorporate the GPS information to update the current state estimate.*

I implemented this step in C++ as follows:

```
zFromX(0) = ekfState(0);
zFromX(1) = ekfState(1);
zFromX(2) = ekfState(2);
zFromX(3) = ekfState(3);
zFromX(4) = ekfState(4);
zFromX(5) = ekfState(5);

for (int i = 0; i < 6; i++) {
  hPrime(i, i) = 1;
}
```

### 6. Meet the performance criteria of each step.

*For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.*

The project passes all performance criteria.

### 7. De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.

*The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight).*

I decreased the velocity gains a little bit and it worked well:

```
# Position control gains
kpPosXY = 3
kpPosZ = 4
KiPosZ = 11

# Velocity control gains
kpVelXY = 7
kpVelZ = 11

# Angle control gains
kpBank = 15
kpYaw = 3

# Angle rate gains
kpPQR = 80, 80, 10

# limits
maxAscentRate = 5
maxDescentRate = 2
maxSpeedXY = 5
maxHorizAccel = 12
maxTiltAngle = .7

```