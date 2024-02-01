# ADXL345 Probe
[![](https://dcbadge.vercel.app/api/server/APw7rgPGPf)](https://discord.gg/APw7rgPGPf)
[![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

**!!! This project is in a BETA state, use at your own risk !!!**

The ADXL345 has an interesting feature called tap detection. With the appropriate tuning, this can be used to implement a nozzle probe on 3D printers.
This project aims to support nozzle probing through tap detection for printers using Klipper.

You can watch this thing in action here:
[https://www.youtube.com/shorts/_qd0kMkrVZw](https://www.youtube.com/shorts/_qd0kMkrVZw)

Results you can expect for a properly tuned system (This was on a Voron Trident):

```
probe accuracy results: maximum 0.007500, minimum 0.000937, range 0.006563, average 0.004031, median 0.004219, standard deviation 0.001841
```

I also measured the force on the bed using a standard kitchen scale, this was approximately 200g. A CAN bus board was used, so a direct connection might result in a quicker stop (See Multi MCU homing in the Klipper docs for more information on this).

## Installation

```bash
cd $HOME
git clone https://github.com/jniebuhr/adxl345-probe
cd adxl345-probe
./scripts/install.sh
```

## Physical setup

This code requires the ADXL int1 or int2 pins to be wired to one of your boards (preferrably the one that controls Z motion).
For a ADXL345 breakout board, simply run a wire. If you're using a CAN toolboard, the following boards are supported as they have wired the pins:

- [Mellow Fly SB2040](https://aliexpress.com/item/1005004675264551.html)
- [Mellow Fly SHT36 v2](https://aliexpress.com/item/1005004687165673.html)
- Huvud

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

## Tuning guide

In progress...

## License

This work is licensed under a
[Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License][cc-by-nc-sa].

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg?style=for-the-badge
