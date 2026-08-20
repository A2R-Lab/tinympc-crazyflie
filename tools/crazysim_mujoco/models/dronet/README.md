# DroNet baseline

`dronet.onnx` contains the weights published by the University of Zurich
Robotics and Perception Group for *DroNet: Learning to Fly by Driving*
(Loquercio et al., 2018), converted from the official Keras 2.0.2 HDF5 file
without retraining. The model accepts `float32[1,1,200,200]` grayscale images
normalized to `[0,1]` and returns steering in `[-1,1]` and collision
probability in `[0,1]`.

Source: https://github.com/uzh-rpg/rpg_public_dronet at commit
`ac19c54bd6ac5a8fd1d220405ecf6f51af55d1f4`.

The original code and weights are MIT licensed; see `LICENSE`.
