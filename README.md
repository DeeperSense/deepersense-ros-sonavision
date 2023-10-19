# ros1_sonavision_inference

This package implements sonavision inference node, which listens for rectangular sonar image and stereo camera image (in stereo format).

## Prerequisites

`Tensorflow` is must to successfully run this package.

## Dynamic Params
There are two dynamic params `blur_level` and `darkness_level` which can be (re)configured using dynamic reconfigure. They can also be changed using keybindings defined below using `sonavision_keyboard_bindings` node.

## Keybindings
```
- UP KEY:    INCREASE DARKNESS FACTOR BY 0.1
- DOWN KEY:  DECREASE DARKNESS FACTOR BY 0.1
- LEFT KEY:  INCREASE BLUR FACTOR BY 0.1
- RIGHT KEY: DECREASE BLUR FACTOR BY 0.1
```

## Usage <a name = "usage"></a>

### run `ros1_sonavision_inference` node
```
rosrun ros1_sonavision_inference Sonavision.py
```

### run `sonavision_keyboard_bindings`
```
rosrun ros1_sonavision_inference SonavisionKeyboardBindings.py
```

### killing `sonavision_keyboard_bindings`
```
rosnode kill /sonavision_keyboard_bindings
```