# ESPHome Zehnder fan

ESPHome external component to wirelessly control a Zehnder ComfoFan / ComfoAir mechanical
ventilation unit from Home Assistant. It runs on an ESP32-C6 paired with an nRF905 radio module
and behaves like an extra RF remote control.

## Usage

See [`zehnder_fan_controller.yaml`](zehnder_fan_controller.yaml) for a complete example.

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/srslynow/esphome-zehnder
      ref: main
    components: [zehnder_fan]

spi:
  id: spi_bus
  clk_pin: GPIO21
  mosi_pin: GPIO19
  miso_pin: GPIO20

fan:
  - platform: zehnder_fan
    id: ventilation_fan
    name: Mechanical Ventilation
    spi_id: spi_bus
    cs_pin: GPIO18
    pwr_pin: GPIO15
    ce_pin: GPIO5
    txen_pin: GPIO7
    dr_pin: GPIO23
```

A recent ESPHome release is required (developed against 2026.10).

## Pairing

1. Power cycle the ventilation unit. It accepts new remotes for a short while after power-up.
2. Press the **Pair with Fan** button.

The pairing is stored in flash and survives restarts and firmware updates. The
**Forget Fan Pairing** button removes it from the controller; the ventilation unit itself keeps
its list of remotes.

## Behaviour

- The fan has four speeds: low, medium, high and max.
- **Off** selects the unit's lowest ("auto") setting. A ventilation unit never stops completely.
- A speed change is only reported to Home Assistant after the unit has acknowledged it. When a
  new request arrives while an earlier one is still being sent, the newest one wins.
- After a restart the last known state is shown again; nothing is sent to the unit. Change this
  with the standard `restore_mode` option.
- Changes made with another remote control are not detected.
- `id(ventilation_fan).set_speed_with_timer(speed, minutes)` runs at a speed (1-4) for a number
  of minutes, after which the unit returns to its previous speed by itself. The state of the fan
  entity is not changed by this.

## Credits

The radio protocol is based on the work of Eelco Huininga in
[nRF905-API](https://github.com/eelcohn/nRF905-API) (MIT licence, © 2020 eelcohn).

## Licence

[MIT](LICENSE)
