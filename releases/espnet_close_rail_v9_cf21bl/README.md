# CF21BL firmware: ESPNetV2 close-rail v9

This directory contains the flashable STM32 firmware built from
`apps/controller_tinympc_eigen` for the Crazyflie 2.1 Brushless (`cf21bl`).
It accepts the CRC-protected TinyRacer vision v6 packet (`90 19 08 3d`) from
GAP8 and preserves collision-first recovery:

1. Validate all three collision and six rail probabilities.
2. Select the sector with maximum collision probability.
3. Consult only that sector's rail-presence and pass-right outputs.
4. Require a stopped/near-hover state and two consistent rail cues before a
   gate opening can replace the ordinary lower-risk escape direction.

Artifacts:

```text
6ce405f3cdfd2fa5d4ae3bcbc12adde8a9d70e18f23d5f4d15e6f3e5f3d9cd3b  cf21bl.bin
518392f132f5a790d88778f35b2388eca563e5d8d5c6146ca07710752ece178f  cf21bl.hex
```

The release build used GCC ARM Embedded 13.2.1 and:

```sh
make clean
make -j2 KCFLAGS='-Wno-error=unused-variable -Wno-error=unused-function'
```

The warning exemptions cover existing configuration-dependent symbols in
`controller_tinympc.cpp`; the v6 receiver itself compiles without a warning.
Reported use was 442,788/1,032,192 bytes flash, 121,116/131,072 bytes RAM,
and 62,248/65,536 bytes CCM.

Flash over radio/USB using the normal Crazyflie tooling, for example:

```sh
cfloader flash cf21bl.bin stm32-fw
```

Verify the target is CF21BL before flashing. The app configuration requires
DShot and motor arming; it deliberately disables the stock AI-deck CPX driver
because this deployment uses the custom UART vision link.
