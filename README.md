# QMC5883L

A library for the QMC5883L, written with the pico-sdk.
Supports:

- Reading the x, y, and z values from the compass
- Calibration of the compass
- Calculating the azimuth (heading)

## API

`qmc5883l_read_data`:

- Get the current x, y, and z values read by the compass.

`qmc5883l_get_azimuth`:

- Get the current heading of the compass

`qmc5883l_calibrate`:

- Calibrate the compass (see `examples/calibrated/calibrated.c`)

`qmc5883l_set_calibration`:

- If calibration values are already known, set them.

`qmc5883l_init`:

- Initialize a new QMC5883L struct.

## Usage

See `examples/xyz.c` for sample API usage.
Make sure to include a pico_sdk_import.cmake, and ensure that `$PICO_SDK_PATH` is set.
Then simply:

```
[qmc5883l/examples/xyz]$ mkdir build; cd build; cmake ..`
```

## TODO

- Add smoothing
