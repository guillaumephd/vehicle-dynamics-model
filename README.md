# Vehicle Dynamics Model

This repository holds a python implementation of the vehicle dynamics model used in the article [Coupled longitudinal and lateral control of a vehicle using deep learning](https://arxiv.org/abs/1810.09365) from G. Devineau, P. Polack, F. Altché and F. Moutarde.

## Install

The model requires `python3`. To setup, run:

```bash
# 1. Get the repo
git clone https://github.com/guillaumephd/vehicle-dynamics-model

# 2. Make the vehicle model available everywhere
sudo cp vehiclesim.so `python -c "import site; print(site.getsitepackages()[-1])"`
```

## Getting started

```bash
import numpy
import vehiclesim

# ---------------------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------------------
def get_simulation(x=0, vx=0, y=0, vy=0, z=0.715, vz=0, r=0, vr=0, p=0, vp=0, yaw=0, vyaw=0, T=1200., delta=0.):

    """
    Returns
    -------
    _run: numpy.ndarray of the vehicle state at each timestep
          shape: (sim_duration=100, 12) 
    """

    sim_dt = 0.001      # in s
    sim_duration = 100  # in ms, since dt=0.001s
    
    # vehicle initial state
    state = numpy.array([x, vx, y, vy, z, vz, r, vr, p, vp, yaw, vyaw]).astype(numpy.double)

    # controls
    #    T     = couple,         in N.m
    #    delta = steering angle, in rad
    controls = numpy.array([
        T,  # front left wheel
        T,  # front right wheel
        T if T < 0 else 0.,  # rear left wheel. can only brake and not accelerate
        T if T < 0 else 0.,  # rear right wheel. can only brake and not accelerate
        delta
    ])
    mu = numpy.array([1., 1., 1., 1.])
    
    # Run sim
    # Note: sim columns: dt x vx y vy z vz r vr p vp yaw vyaw om_fl om_fr om_rl om_rr ax ay sr1 sr2 sr3 sr4 sa1 sa2 sa3 sa4
    _run = vehiclesim.run(sim_duration, sim_dt, state, controls, mu)        

    # Drop the first column (time)
    _run = _run[:, 1:]
    
    # Shape: (sim_duration, 12).
    #        Rows format: x vx y vy z vz roll droll pitch dpitch yaw dyaw
    _run = _run[:, 0:12]

    return _run


# Drive the vehicle at 10m/s for 100ms:
demo_state_trajectory = get_simulation(0., 10., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., .0)
assert demo_state_trajectory.shape == (100, 12)
```

## Input and output shapes

```
    input:
        sim_duration:   int (duration in timesteps)
        sim_dt:         double (timestep in ms)
        input_state:    array of double values, shape: (12,)
        input_controls: array of double values, shape: (5,)
        mu:             array of double values, shape: (4,)

    output:
        state_trajectory: array of float values, shape: (sim_duration x len(state_columns))

    state_columns:
        dt x vx y vy z vz r vr p vp yaw vyaw om_fl om_fr om_rl om_rr ax ay sr1 sr2 sr3 sr4 sa1 sa2 sa3 sa4
```

## Edit the model

First edit the source. Then, install `boost` with `boost_numpy`. Then run:

```
# Compile again. Requires boost.
make
```

## Help and tips

The vehicle simulator is written in `c++` and called from `python` using `libboost` under the hood. A few tips:

    - Check that you have `python3`.
    - Check that the `boost` library is linked to the correct version of `python`.
    - Use an `anaconda` environment to get `boost` if needed.

### Citation

If you find this code useful in your research, please consider citing:

```
@inproceedings{devineau2018coupled,
  title={Coupled longitudinal and lateral control of a vehicle using deep learning},
  author={Devineau, Guillaume and Polack, Philip and Altch{\'e}, Florent and Moutarde, Fabien},
  booktitle={2018 21st International Conference on Intelligent Transportation Systems (ITSC)},
  pages={642--649},
  year={2018},
  organization={IEEE}
}
```
