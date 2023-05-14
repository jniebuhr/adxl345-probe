# ADXL345 Probe

The ADXL345 has an interesting feature called tap detection. With the appropriate tuning, this can be used to implement a nozzle probe on 3D printers.
This project aims to support nozzle probing through tap detection for printers using Klipper.

## Installation

```bash
cd $HOME
git clone https://github.com/jniebuhr/adxl345-probe
cd adxl345-probe
./scripts/install.sh
```

## Configuration

```
[adxl345_probe]
probe_pin: <pin for either int1 or int2>
int_pin: int1 # select either int1 or int2, depending on your choice of wiring
tap_thresh: 12000 # this needs to be tuned
tap_dur: 0.01 # this needs to be tuned
speed: 20 # this needs to be tuned
# Adjust this to your liking
samples: 3
sample_retract_dist: 3.0
samples_result: median
samples_tolerance: 0.01
samples_tolerance_retries: 20
```

## License

This project is licensed as
![image](https://user-images.githubusercontent.com/37383368/139769027-7267da5b-7f58-499d-96bc-e41d164a3aac.png)

https://creativecommons.org/licenses/by-nc/4.0/
