from . import probe, adxl345

REG_THRESH_TAP = 0x1D
REG_DUR = 0x21
REG_INT_MAP = 0x2F
REG_TAP_AXES = 0x2A
REG_INT_ENABLE = 0x2E
REG_INT_SOURCE = 0x30

DUR_SCALE = 0.000625  # 0.625 msec / LSB
TAP_SCALE = 0.0625 * adxl345.FREEFALL_ACCEL  # 62.5mg/LSB * Earth gravity in mm/s**2

ADXL345_REST_TIME = .1


class ADXL345Probe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.toolhead = None
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(config, 'deactivate_gcode', '')
        int_pin = config.get('int_pin').strip()
        self.inverted = False
        self.is_measuring = False
        self.last_probe_loc = [-100000, -100000]
        if int_pin.startswith('!'):
            self.inverted = True
            int_pin = int_pin[1:].strip()
        if int_pin != 'int1' and int_pin != 'int2':
            raise config.error('int_pin must specify one of int1 or int2 pins')
        probe_pin = config.get('probe_pin')
        self.int_map = 0x40 if int_pin == 'int2' else 0x0
        self.tap_thresh = config.getfloat('tap_thresh', 5000, minval=TAP_SCALE, maxval=100000.)
        self.tap_dur = config.getfloat('tap_dur', 0.01, above=DUR_SCALE, maxval=0.1)
        self.tare_tap_ratio = config.getfloat('tare_tap_ratio', 0, minval=0, maxval=10.)
        self.tare_z_move = config.getfloat('tare_z_move', 4.0, minval=-10, maxval=10.0)
        self.tap_detect_delay = config.getfloat('tap_detect_delay', 0, minval=0, maxval=2.0)
        try:
            self.tap_axes = sorted({"xyz".index(name) for name in config.getlist("tap_axes", default=list("xyz"))})
        except IndexError:
            raise config.error("Invalid tap_axes")
        self.position_endstop = config.getfloat('z_offset')
        self.disable_fans = config.get("disable_fans", "").split(",")

        self.adxl345 = self.printer.lookup_object('adxl345')
        self._patch_adxl345()
        self.next_cmd_time = self.action_end_time = 0.
        # Create an "endstop" object to handle the sensor pin
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(probe_pin, can_invert=True,
                                      can_pullup=True)
        mcu = pin_params['chip']
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        # Add wrapper methods for endstops
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        # Register commands and callbacks
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("SET_ACCEL_PROBE", "CHIP", None, self.cmd_SET_ACCEL_PROBE, desc=self.cmd_SET_ACCEL_PROBE_help)
        self.printer.register_event_handler('klippy:connect', self.init_adxl)
        self.printer.register_event_handler('klippy:mcu_identify', self.handle_mcu_identify)
        self.printer_probe = probe.PrinterProbe(config, self)
        self.printer.add_object('probe', self.printer_probe)
    
    def _patch_adxl345(self):
        chip = self.adxl345
        def set_reg(reg, val, minclock=0, reqclock=0):
            # taken from adxl345.py, with the addition of reqclock
            chip.spi.spi_send([reg, val & 0xFF], minclock=minclock, reqclock=reqclock)
            stored_val = chip.read_reg(reg)
            if stored_val != val:
                raise self.printer.command_error(
                        "Failed to set ADXL345 register [0x%x] to 0x%x: got 0x%x. "
                        "This is generally indicative of connection problems "
                        "(e.g. faulty wiring) or a faulty adxl345 chip." % (
                            reg, val, stored_val))
        chip.set_reg = set_reg

    def init_adxl(self):
        chip = self.adxl345
        chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
        chip.set_reg(adxl345.REG_DATA_FORMAT, 0x0B)
        if self.inverted:
            chip.set_reg(adxl345.REG_DATA_FORMAT, 0x2B)
        chip.set_reg(REG_INT_MAP, self.int_map)
        tap_axes_reg_value = sum((1 << (2 - a)) for a in self.tap_axes)
        chip.set_reg(REG_TAP_AXES, tap_axes_reg_value)
        chip.set_reg(REG_THRESH_TAP, int(self.tap_thresh / TAP_SCALE))
        chip.set_reg(REG_DUR, int(self.tap_dur / DUR_SCALE))

    def handle_mcu_identify(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        kin = self.toolhead.get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)
    
    def control_fans(self, disable=True):
        for fan in self.disable_fans:
            fan = self.printer.lookup_object(fan)
            if disable:
                fan._fan_speed = fan.fan_speed
                fan.fan_speed = 0
            else:
                fan.fan_speed = fan._fan_speed
                fan._fan_speed = 0
    
    def tare(self):
        client = self.adxl345.start_internal_client()
        current_pos = self.toolhead.get_position()
        new_pos = list(current_pos)
        new_pos[2] += self.tare_z_move # TODO: perhaps infer the z tare move from the probe request, might need to monkeypatch PrinterProbe
        self.toolhead.move(new_pos, self.printer_probe.speed)
        self.toolhead.move(current_pos, self.printer_probe.speed)
        self.toolhead.wait_moves()
        client.finish_measurements()
        samples = client.get_samples()
        axis_max = max(abs(s[a + 1]) for s in samples for a in self.tap_axes) # TODO: experiment with other statistics
        tap_thresh = max(axis_max * self.tare_tap_ratio, self.tap_thresh)
        self.toolhead.dwell(ADXL345_REST_TIME)
        self.toolhead.wait_moves() # TODO: no idea why this is needed, otherwise adxl coomplaints a lousy error
        print_time = self.toolhead.get_last_move_time()
        clock = self.adxl345.mcu.print_time_to_clock(print_time)
        self._set_accel_probe(tap_thresh=tap_thresh, minclock=clock, reqclock=clock)

    def multi_probe_begin(self):
        self._in_multi_probe = True
        self.control_fans(disable=True)

    def multi_probe_end(self):
        self.control_fans(disable=False)
        self._in_multi_probe = False

    def get_position_endstop(self):
        return self.position_endstop

    def _try_clear_tap(self):
        chip = self.adxl345
        tries = 8
        while tries > 0:
            val = chip.read_reg(REG_INT_SOURCE)
            if not (val & 0x40):
                return True
            tries -= 1
        return False

    def probe_prepare(self, hmove):
        self.activate_gcode.run_gcode_from_command()
        chip = self.adxl345
        self.toolhead.flush_step_generation()
        self.toolhead.dwell(ADXL345_REST_TIME)
        probe_loc = self.toolhead.get_position()[:2]
        if self.tare_tap_ratio > 1 and max(abs(probe_loc[0] - self.last_probe_loc[0]), abs(probe_loc[1] - self.last_probe_loc[1])) > 1e-3:
            self.tare()
            self.last_probe_loc = probe_loc
        print_time = self.toolhead.get_last_move_time()
        clock = self.adxl345.mcu.print_time_to_clock(print_time)
        chip.set_reg(REG_INT_ENABLE, 0x00, minclock=clock)
        chip.read_reg(REG_INT_SOURCE)
        delayed_clock = self.adxl345.mcu.print_time_to_clock(print_time + self.tap_detect_delay)
        chip.set_reg(REG_INT_ENABLE, 0x40, minclock=clock, reqclock=delayed_clock)
        self.is_measuring = (chip.read_reg(adxl345.REG_POWER_CTL) == 0x08)
        if not self.is_measuring:
            chip.set_reg(adxl345.REG_POWER_CTL, 0x08, minclock=clock)
        if not self._try_clear_tap():
            raise self.printer.command_error("ADXL345 tap triggered before move, it may be set too sensitive.")
        if not self._in_multi_probe:
            self.control_fans(disable=True)

    def probe_finish(self, hmove):
        chip = self.adxl345
        self.toolhead.dwell(ADXL345_REST_TIME)
        print_time = self.toolhead.get_last_move_time()
        clock = chip.mcu.print_time_to_clock(print_time)
        chip.set_reg(REG_INT_ENABLE, 0x00, minclock=clock)
        if not self.is_measuring:
            chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
        self.deactivate_gcode.run_gcode_from_command()
        if not self._try_clear_tap():
            raise self.printer.command_error("ADXL345 tap triggered after move, it may be set too sensitive.")
        if not self._in_multi_probe:
            self.control_fans(disable=False)

    cmd_SET_ACCEL_PROBE_help = "Configure ADXL345 parameters related to probing"

    def cmd_SET_ACCEL_PROBE(self, gcmd):
        self.tap_thresh = gcmd.get_float('TAP_THRESH', self.tap_thresh,
                                         minval=TAP_SCALE, maxval=100000.)
        self.tap_dur = gcmd.get_float('TAP_DUR', self.tap_dur,
                                      above=DUR_SCALE, maxval=0.1)
        self.tare_tap_ratio = gcmd.get_float('TARE_TAP_RATIO', self.tare_tap_ratio,
                                            minval=0, maxval=10.)
        self.tap_detect_delay = gcmd.get_float('TAP_DETECT_DELAY', self.tap_detect_delay,
                                                minval=0, maxval=2.0)
        self._set_accel_probe()

    def _set_accel_probe(self, tap_thresh=None, tap_dur=None, minclock=0, reqclock=0):
        tap_thresh = tap_thresh or self.tap_thresh
        tap_dur = tap_dur or self.tap_dur
        chip = self.adxl345
        chip.set_reg(REG_THRESH_TAP, int(tap_thresh / TAP_SCALE), minclock=minclock, reqclock=reqclock)
        chip.set_reg(REG_DUR, int(tap_dur / DUR_SCALE), minclock=minclock, reqclock=reqclock)

def load_config(config):
    return ADXL345Probe(config)
