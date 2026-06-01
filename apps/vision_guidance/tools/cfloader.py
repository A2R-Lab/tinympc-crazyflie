#!/usr/bin/env python3
"""
Minimal cfloader replacement for flashing the Crazyflie STM32 firmware.

Modern cflib (>=0.1.3x) no longer ships the standalone `cfloader` module that
the crazyflie-firmware Makefile's `cload` target invokes via `python -m cfloader`.
The old standalone script only came bundled with the heavyweight cfclient (Qt)
package. This shim reproduces just the `flash` action using cflib.bootloader so
you don't need cfclient installed.

Usage (matches what `make cload` passes):
    python3 cfloader.py [radio://uri] flash <file.bin> <target> [<target> ...]

  - No radio URI  -> COLD boot: put the Crazyflie in bootloader mode manually
    (power off, then hold the power button ~3 s until the blue LEDs blink) within
    the scan window. The shim scans for a CF already in bootloader mode.
  - radio://uri   -> WARM boot: the shim connects to the running CF over that
    link and resets it into the bootloader automatically. Faster, no button.

  <target> is like "stm32-fw" => Target(platform='cf2', target='stm32', type='fw').

Wire it into the build with:
    make cload CLOAD_SCRIPT="python3 $(pwd)/tools/cfloader.py"
"""
import sys

import cflib.crtp
from cflib.bootloader import Bootloader, Target


def parse_target(spec):
    # "stm32-fw" -> ('stm32', 'fw'); default type 'fw' if omitted.
    parts = spec.split('-')
    name = parts[0]
    typ = parts[1] if len(parts) > 1 else 'fw'
    # platform 'cf2' covers the Crazyflie 2.x / Bolt STM32F4 builds here.
    return Target('cf2', name, typ, [], [])


def main(argv):
    args = list(argv)

    clink = None
    if args and (args[0].startswith('radio://') or args[0].startswith('usb://')):
        clink = args[0]
        args = args[1:]

    if not args or args[0] != 'flash':
        print(__doc__)
        return 2

    args = args[1:]
    if not args:
        print('error: flash requires a <file.bin>', file=sys.stderr)
        return 2

    filename = args[0]
    target_specs = args[1:] or ['stm32-fw']
    targets = [parse_target(s) for s in target_specs]

    cflib.crtp.init_drivers()

    warm = clink is not None
    bl = Bootloader(clink)
    try:
        if warm:
            print('Warm boot: connecting to {} and resetting to bootloader...'.format(clink))
        else:
            print('Cold boot: put the Crazyflie in bootloader mode now '
                  '(power off, hold power button ~3 s until blue LEDs blink)...')

        if not bl.start_bootloader(warm_boot=warm):
            print('error: could not enter/Find bootloader. '
                  'Check the radio dongle and that the CF is in bootloader mode.',
                  file=sys.stderr)
            return 1

        print('Connected to bootloader, flashing {} -> {}'.format(
            filename, ', '.join(target_specs)))
        bl.flash(filename, targets)
        print('Flashing done, resetting to firmware.')
        bl.reset_to_firmware()
        return 0
    finally:
        bl.close()


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
