---
layout: default
title: Autonomous Navigation
nav_order: 1
---

# Autonomous Vehicle Navigation

## Particle Filter (Known Map)
Monte-Carlo is used for determining the true pose of an autonomous vehicle, **given a map**.

### Particle Vs Kalman

1. Particle filter assumes no linearity in model and measurement models.
2. Particle filter can represent multi-modal distributions — multiple likely robot positions at once.
3. Particle filter can handle non-Gaussian noise.

However,
1. Kalman filter is faster.

### Steps of a Particle filter

**Step 1**: Generate a large number of discrete poses around the map. Uniformly randomize (x, y) and yaw.

**Step 2**: Lidar measurement returns. Using the map, each particle returns its own estimates of lidar reading. This is what the lidar would've have returned if that pose was the true pose.

**Step 3**: The filter then compares the two scenes - one given by actual lidar reading, the other given by each filter's estimate - and determines the likelihood of each particle being the true state. Updating the probability of each particle.

Note that after Step 3, the probability distribution is no longer uniform, but skewed towards the more probable poses.

**Step 4**: Resample based on the new probability distribution after the update.

**Step 5**: The vehicle dead-reckons its pose based on onboard odometry sensors (same as the prediction step in Kalman). Apply the same action to all particles. Since dead-reckoning is extremely **noisy**, when propagating the particles through the estimated dynamics, we add a process noise by injecting a random motion.

**Step 6**: As we are honing into the true pose, reduce the number of particles to speed up processing time. Adaptive Monte Carlo computes the new number of particles needed at each iteration.

**Step 7**: repeat the process until convergence is observed.

---

## SLAM (Unkown Map)
Simultaneously building a map and localizing.

### Main issue - uncertainty
Lidar is accurate by odometry would have large uncertainty which would make the map deviates more and more at each time step.

![]()