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
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(config, 'deactivate_gcode', '')
        int_pin = config.get('int_pin').strip()
        self.inverted = False
        self.is_measuring = False
        if int_pin.startswith('!'):
            self.inverted = True
            int_pin = int_pin[1:].strip()
        if int_pin != 'int1' and int_pin != 'int2':
            raise config.error('int_pin must specify one of int1 or int2 pins')
        probe_pin = config.get('probe_pin')
        self.int_map = 0x40 if int_pin == 'int2' else 0x0
        self.tap_thresh = config.getfloat('tap_thresh', 5000, minval=TAP_SCALE, maxval=100000.)
        self.tap_dur = config.getfloat('tap_dur', 0.01, above=DUR_SCALE, maxval=0.1)
        self.position_endstop = config.getfloat('z_offset')

        self.adxl345 = self.printer.lookup_object('adxl345')
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
        self.phoming = self.printer.lookup_object('homing')
        # Register commands and callbacks
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("SET_ACCEL_PROBE", "CHIP", None, self.cmd_SET_ACCEL_PROBE, desc=self.cmd_SET_ACCEL_PROBE_help)
        self.printer.register_event_handler('klippy:connect', self.init_adxl)
        self.printer.register_event_handler('klippy:mcu_identify', self.handle_mcu_identify)
        self.printer.add_object('probe', probe.PrinterProbe(config, self))

    def init_adxl(self):
        chip = self.adxl345
        chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
        chip.set_reg(adxl345.REG_DATA_FORMAT, 0x0B)
        if self.inverted:
            chip.set_reg(adxl345.REG_DATA_FORMAT, 0x2B)
        chip.set_reg(REG_INT_MAP, self.int_map)
        chip.set_reg(REG_TAP_AXES, 0x7)
        chip.set_reg(REG_THRESH_TAP, int(self.tap_thresh / TAP_SCALE))
        chip.set_reg(REG_DUR, int(self.tap_dur / DUR_SCALE))

    def handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    def multi_probe_begin(self):
        pass

    def multi_probe_end(self):
        pass

    def probing_move(self, pos, speed):
        return self.phoming.probing_move(self, pos, speed)

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
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        toolhead.dwell(ADXL345_REST_TIME)
        print_time = toolhead.get_last_move_time()
        clock = self.adxl345.mcu.print_time_to_clock(print_time)
        chip.set_reg(REG_INT_ENABLE, 0x00, minclock=clock)
        chip.read_reg(REG_INT_SOURCE)
        chip.set_reg(REG_INT_ENABLE, 0x40, minclock=clock)
        self.is_measuring = (chip.read_reg(adxl345.REG_POWER_CTL) == 0x08)
        if not self.is_measuring:
            chip.set_reg(adxl345.REG_POWER_CTL, 0x08, minclock=clock)
        if not self._try_clear_tap():
            raise self.printer.command_error("ADXL345 tap triggered before move, it may be set too sensitive.")

    def probe_finish(self, hmove):
        chip = self.adxl345
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.dwell(ADXL345_REST_TIME)
        print_time = toolhead.get_last_move_time()
        clock = chip.mcu.print_time_to_clock(print_time)
        chip.set_reg(REG_INT_ENABLE, 0x00, minclock=clock)
        if not self.is_measuring:
            chip.set_reg(adxl345.REG_POWER_CTL, 0x00)
        self.deactivate_gcode.run_gcode_from_command()
        if not self._try_clear_tap():
            raise self.printer.command_error("ADXL345 tap triggered after move, it may be set too sensitive.")

    cmd_SET_ACCEL_PROBE_help = "Configure ADXL345 parameters related to probing"

    def cmd_SET_ACCEL_PROBE(self, gcmd):
        chip = self.adxl345
        self.tap_thresh = gcmd.get_float('TAP_THRESH', self.tap_thresh,
                                         minval=TAP_SCALE, maxval=100000.)
        self.tap_dur = gcmd.get_float('TAP_DUR', self.tap_dur,
                                      above=DUR_SCALE, maxval=0.1)
        chip.set_reg(REG_THRESH_TAP, int(self.tap_thresh / TAP_SCALE))
        chip.set_reg(REG_DUR, int(self.tap_dur / DUR_SCALE))


def load_config(config):
    return ADXL345Probe(config)
