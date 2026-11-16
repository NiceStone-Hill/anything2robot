"""Pico 2 electrical adaptation; preserves motor variant uncertainty.

The CAN design uses external 3.3 V SPI controller + transceiver, not native CAN.
VSYS is fed from 5 V through a separately reserved reverse-feed protection stage.
"""

from .checks import review
from .topology import port

SOURCE = "https://pip.raspberrypi.com/categories/1005-raspberry-pi-pico-2"
PROFILE = "raspberry_pi_pico2"
# Physical board pin numbers, separate GPIO identity and planned peripheral use.
PINS = {
    "GP0": 1,
    "GP1": 2,
    "GP2": 4,
    "GP3": 5,
    "GP4": 6,
    "GP5": 7,
    "GP6": 9,
    "GP7": 10,
    "GP16": 21,
    "GP17": 22,
    "GP18": 24,
    "GP19": 25,
    "GND": 38,
    "3V3": 36,
    "VSYS": 39,
}


def catalog_profile():
    return {
        "model": "Raspberry Pi Pico 2 (no headers)",
        "source_url": SOURCE,
        "physical_pads": {
            str(pin): {
                "id": str(pin),
                "pad_number": str(pin),
                "signal": signal,
                "board_signal": signal,
                "connector_reference": "direct_solder_castellated_pad",
                "pad_xy_mm": None,
                "geometry_status": "official_pin_number_known_physical_cable_exit_pending",
                "evidence": {"source_url": SOURCE},
            }
            for signal, pin in PINS.items()
        },
        "aliases": {k: str(v) for k, v in PINS.items()},
    }


def scenario(bus, motors, controller, regulator, catalog):
    from .topology import scenario as uno_scenario

    plan = uno_scenario(bus, motors, controller, regulator, catalog)
    by_id = {c["id"]: c for c in plan["components"]}
    host = by_id[controller]
    bridge = by_id["PW_BUS_INTERFACE"]
    protection = by_id["PW_CONTROLLER_SUPPLY"]
    alias = catalog["profiles"]["arduino_uno_rev3e"]["aliases"]
    if bus == "CAN":
        signals = {
            "MOSI": "GP19",
            "MISO": "GP16",
            "SCK": "GP18",
            "CS": "GP17",
            "D2": "GP2",
        }
    else:
        signals = {"TX": "GP0", "RX": "GP1", "D2": "GP2", "D3": "GP3"}
    replacements = {
        alias["VIN"]: str(PINS["VSYS"]),
        alias["GND_POWER_1"]: str(PINS["GND"]),
    }
    replacements.update(
        {alias[key]: str(PINS[value]) for key, value in signals.items()}
    )
    new_ports = {replacements[key]: dict(value) for key, value in host["ports"].items()}
    for pin, value in new_ports.items():
        value["physical_pin"] = int(pin)
        value["voltage_range_v"] = (
            [1.8, 5.5]
            if pin == str(PINS["VSYS"])
            else [0, 3.3]
            if value["kind"].startswith("signal")
            else value.get("voltage_range_v")
        )
        if value.get("voltage_range_v") is None:
            value.pop("voltage_range_v", None)
    new_ports[str(PINS["3V3"])] = port("power_out", bounds=[3.3, 3.3])
    host.update(
        model="Raspberry Pi Pico 2 (no headers)",
        ports=new_ports,
        profile_id=PROFILE,
        evidence=SOURCE,
        pending=["direct_solder_harness_retention_and_usb_access_pending"],
    )
    bridge["model"] = (
        "3.3 V SPI CAN controller AND CAN transceiver, exact module TBD"
        if bus == "CAN"
        else "3.3 V RS485 transceiver, exact module TBD"
    )
    bridge["ports"]["VCC"]["voltage_range_v"] = [3.3, 3.3]
    for spec in bridge["ports"].values():
        if spec["kind"].startswith("signal"):
            spec["voltage_range_v"] = [0, 3.3]
    bridge["pending"] += ["verify_total_Pico_3V3_load_before_selecting_module"]
    protection.update(
        model="5 V reverse-feed protection diode or power mux TBD",
        evidence="Pico 2 datasheet power supply section; external VSYS feed must coexist with USB",
        ports={
            "IN": port("power_in", bounds=[4.8, 5.2]),
            "GND": port("ground"),
            "OUT": port("power_out", bounds=[4.0, 5.2]),
        },
        pending=[
            "select_protection_part_verify_drop_current_and_USB_backfeed",
            "new_module_mount_and_connectors_pending",
        ],
    )
    for collection in (plan["cables"], plan["nets"]):
        for item in collection:
            terminals = item.get("terminals", [item.get("source"), item.get("target")])
            for terminal in terminals:
                if terminal and terminal["instance_id"] == controller:
                    terminal["port_id"] = replacements.get(
                        terminal["port_id"], terminal["port_id"]
                    )
            if item["id"].startswith("host_"):
                item["voltage_range_v"] = [0, 3.3]
    # Update both representations of every changed connection, then review.
    changes = {
        "CTRL_REG_input": (
            (regulator, "VOUT"),
            ("PW_CONTROLLER_SUPPLY", "IN"),
            [4.8, 5.2],
        ),
        "controller_power": (
            ("PW_CONTROLLER_SUPPLY", "OUT"),
            (controller, str(PINS["VSYS"])),
            [4.0, 5.2],
        ),
        "bus_interface_power": (
            (controller, str(PINS["3V3"])),
            ("PW_BUS_INTERFACE", "VCC"),
            [3.3, 3.3],
        ),
    }
    for key, (a, b, voltage) in changes.items():
        terminals = [{"instance_id": i, "port_id": p} for i, p in (a, b)]
        net = next(n for n in plan["nets"] if n["id"] == key)
        net.update(terminals=terminals, voltage_range_v=voltage)
        cable = next(c for c in plan["cables"] if c["id"] == key)
        cable.update(source=terminals[0], target=terminals[1])
    # Former Uno controller regulator feed must not leave an unused distribution lug required.
    by_id["PW_POWER_DISTRIBUTION"]["ports"]["CTRL_REG"]["required"] = False
    plan["controller_profile"] = PROFILE
    plan["controller_signal_assignment"] = {
        name: {"gpio": gpio, "physical_pin": PINS[gpio]}
        for name, gpio in signals.items()
    }
    plan["supply_policy"] = (
        "Motor rail remains separately specified. 5 V regulator feeds VSYS through reverse-feed protection. External bus electronics use 3.3 V logic; Pico 3V3 current budget pending."
    )
    plan["pending"] += [
        "Pico 2 selected by user; motor protocol and all new power/interface modules remain candidates",
        "Pico firmware peripheral configuration and total 3V3 current pending",
    ]
    plan["review"] = review(plan)
    return plan
