"""Candidate bus-motor circuits. No automatic hardware purchase or variant choice."""

from __future__ import annotations

from .checks import review


def port(kind="passive", protocol=None, bounds=None, role=None):
    value = {"kind": kind}
    if protocol:
        value["protocol"] = protocol
    if bounds:
        value["voltage_range_v"] = bounds
    if role:
        value["conductor_role"] = role
    return value


def scenario(
    bus,
    motor_instances,
    controller,
    regulator,
    catalog,
    controller_profile="arduino_uno_rev3e",
):
    if controller_profile == "raspberry_pi_pico2":
        from .pico import scenario as pico_scenario

        return pico_scenario(bus, motor_instances, controller, regulator, catalog)
    if controller_profile != "arduino_uno_rev3e":
        raise ValueError("Unsupported controller profile")
    if bus not in {"CAN", "RS485"}:
        raise ValueError("Unsupported candidate bus")
    if not motor_instances or len(set(motor_instances)) != len(motor_instances):
        raise ValueError("A scenario requires unique motor instances")
    profile = catalog["profiles"]["mg4005v2_" + bus.lower()]
    components, nets, cables = [], [], []
    identities = set()

    def add(identity, model, ports, evidence, pending=(), groups=(), profile_id=None):
        if identity in identities:
            raise ValueError(
                "Generated component ID collides with an assembly instance"
            )
        identities.add(identity)
        value = {
            "id": identity,
            "model": model,
            "ports": ports,
            "evidence": evidence,
            "pending": list(pending),
            "conductive_groups": list(groups),
            "profile_id": profile_id,
        }
        components.append(value)
        return value

    def link(identity, a, b, voltage=None, protocol=None, bundle=None):
        terminals = [{"instance_id": x, "port_id": y} for x, y in (a, b)]
        net = {"id": identity, "terminals": terminals}
        if voltage:
            net["voltage_range_v"] = voltage
        if protocol:
            net["protocol"] = protocol
        nets.append(net)
        cables.append(
            {
                "id": identity,
                "net": identity,
                "source": terminals[0],
                "target": terminals[1],
                "bundle_id": bundle,
                "protocol": protocol,
            }
        )

    alias = catalog["profiles"]["arduino_uno_rev3e"]["aliases"]
    host = {"VIN": port("power_in", bounds=[7, 12]), "GND_POWER_1": port("ground")}
    if bus == "CAN":
        signals = [
            ("MOSI", "signal_out"),
            ("MISO", "signal_in"),
            ("SCK", "signal_out"),
            ("CS", "signal_out"),
            ("D2", "signal_in"),
        ]
        bridge_ports = {
            "MOSI": port("signal_in", "SPI_MOSI"),
            "MISO": port("signal_out", "SPI_MISO"),
            "SCK": port("signal_in", "SPI_CLOCK"),
            "CS": port("signal_in", "SPI_CS"),
            "IRQ": port("signal_out", "IRQ"),
        }
        protocols = {
            "MOSI": "SPI_MOSI",
            "MISO": "SPI_MISO",
            "SCK": "SPI_CLOCK",
            "CS": "SPI_CS",
            "D2": "IRQ",
        }
        bridge_signals = {"D2": "IRQ"}
        bridge_requirement = (
            "5 V compatible SPI CAN controller PLUS CAN transceiver, exact module TBD"
        )
        conductors = ["CAN_H", "CAN_L"]
    else:
        signals = [
            ("TX", "signal_out"),
            ("RX", "signal_in"),
            ("D2", "signal_out"),
            ("D3", "signal_out"),
        ]
        bridge_ports = {
            "DI": port("signal_in", "UART_TX"),
            "RO": port("signal_out", "UART_RX"),
            "DE": port("signal_in", "GPIO_DE"),
            "RE_N": port("signal_in", "GPIO_RE"),
        }
        protocols = {"TX": "UART_TX", "RX": "UART_RX", "D2": "GPIO_DE", "D3": "GPIO_RE"}
        bridge_signals = {"TX": "DI", "RX": "RO", "D2": "DE", "D3": "RE_N"}
        bridge_requirement = (
            "5 V compatible half-duplex RS485 transceiver, exact module TBD"
        )
        conductors = ["RS485_A", "RS485_B"]
    for name, kind in signals:
        host[name] = port(kind, protocols[name], [0, 5])
    add(
        controller,
        "Arduino Uno R3 TH Rev3e",
        {alias[k]: v for k, v in host.items()},
        "official Uno PCB and pinout",
        ["board_revision_and_connector_style_unconfirmed"],
        profile_id="arduino_uno_rev3e",
    )
    bridge_ports.update({name: port("bus", bus, role=name) for name in conductors})
    bridge_ports.update(VCC=port("power_in", bounds=[4.8, 5.2]), GND=port("ground"))
    add(
        "PW_BUS_INTERFACE",
        bridge_requirement,
        bridge_ports,
        "design requirement, not a selected/installed part",
        [
            "select_compatible_interface_module",
            "add_carrier_and_connector_geometry",
            "logic_levels_current_and_firmware_pending",
        ],
    )
    add(
        "PW_MOTOR_SUPPLY",
        "Motor power source TBD",
        {"OUT": port("power_out"), "RETURN": port("ground")},
        "unselected source; existing battery nominal voltage alone is insufficient",
        [
            "select_motor_voltage_and_peak_current",
            "battery_discharge_voltage_sag_and_regeneration_pending",
        ],
    )
    add(
        "PW_FUSE",
        "Fuse and holder TBD",
        {"IN": port(), "OUT": port()},
        "required protection, rating TBD",
        ["select_fuse_and_holder_rating"],
        groups=[["IN", "OUT"]],
    )
    add(
        "PW_MAIN_SWITCH",
        "Motor-power-rated disconnect TBD",
        {"IN": port(), "OUT": port()},
        "required switch, existing S1 not assumed motor-rated",
        ["select_dc_switch_rating_and_external_access"],
        groups=[["IN", "OUT"]],
    )
    power_ports = {"IN": port(), "LOGIC_REG": port(), "CTRL_REG": port()}
    ground_ports = {
        "IN": port("ground"),
        "LOGIC_REG": port("ground"),
        "CTRL_REG": port("ground"),
        "CONTROLLER": port("ground"),
        "BUS_INTERFACE": port("ground"),
    }
    for i in range(len(motor_instances)):
        power_ports[f"M{i + 1}"] = port()
        ground_ports[f"M{i + 1}"] = port("ground")
    add(
        "PW_POWER_DISTRIBUTION",
        "Positive distribution block TBD",
        power_ports,
        "distinct physical connection positions required",
        ["select_distribution_part_and_current_rating", "place_distribution_block"],
        groups=[list(power_ports)],
    )
    add(
        "PW_GROUND_DISTRIBUTION",
        "Return distribution block TBD",
        ground_ports,
        "distinct return connection positions required",
        ["select_distribution_part_and_current_rating", "place_distribution_block"],
        groups=[list(ground_ports)],
    )
    add(
        regulator,
        "Pololu D24V50F5",
        {
            "VIN": port("power_in", bounds=[6, 38]),
            "GND_1": port("ground"),
            "VOUT": port("power_out", bounds=[4.8, 5.2]),
        },
        "Pololu official product 2851",
        ["physical_connector_exit_and_input_headroom_pending"],
        profile_id="pololu_d24v50f5",
    )
    add(
        "PW_CONTROLLER_SUPPLY",
        "7–12 V regulated controller supply TBD",
        {
            "IN": port("power_in"),
            "GND": port("ground"),
            "OUT": port("power_out", bounds=[7, 12]),
        },
        "design requirement from Uno VIN recommended range; not a selected regulator",
        [
            "select_input_compatible_regulator",
            "verify_thermal_current_and_usb_power_coexistence",
        ],
    )
    # Treat the bus loom as a new physical assembly, NOT an assumed star of bare wires.
    harness_ports = {}
    tap_names = [
        "HOST",
        *[f"M{i + 1}" for i in range(len(motor_instances))],
        "TERM_HOST",
        "TERM_END",
    ]
    for tap in tap_names:
        for name in conductors:
            harness_ports[f"{tap}:{name}"] = port("bus", bus, role=name)
    groups = [[f"{tap}:{name}" for tap in tap_names] for name in conductors]
    add(
        "PW_BUS_LOOM",
        "Linear bus trunk with short taps; physical loom TBD",
        harness_ports,
        "candidate linear topology requirement; no cable dimensions inferred",
        [
            "trunk_and_tap_lengths_pending",
            "cable_spec_and_shield_grounding_pending",
            "loom_connector_geometry_pending",
        ],
        groups=groups,
    )
    for which in ("HOST", "END"):
        add(
            "PW_TERM_" + which,
            "Bus termination component TBD",
            {name: port("passive", bus, role=name) for name in conductors},
            "termination positions proposed; value and existing built-in termination unverified",
            ["verify_total_termination_and_bus_characteristics"],
        )
    for motor in motor_instances:
        add(
            motor,
            profile["model"],
            {
                "V+": port("power_in", bounds=profile["driver_input_range_v"]),
                "V-": port("ground"),
                **{name: port("bus", bus, role=name) for name in conductors},
            },
            "candidate only: LK manufacturer catalogue page 51",
            [
                "motor_model_variant_unconfirmed",
                "physical_pin_mapping_pending",
                "motor_mount_side_pending",
            ],
            profile_id="mg4005v2_" + bus.lower(),
        )
    link("supply_fuse", ("PW_MOTOR_SUPPLY", "OUT"), ("PW_FUSE", "IN"))
    link("fuse_switch", ("PW_FUSE", "OUT"), ("PW_MAIN_SWITCH", "IN"))
    link(
        "switch_distribution",
        ("PW_MAIN_SWITCH", "OUT"),
        ("PW_POWER_DISTRIBUTION", "IN"),
    )
    link(
        "source_return",
        ("PW_MOTOR_SUPPLY", "RETURN"),
        ("PW_GROUND_DISTRIBUTION", "IN"),
        [0, 0],
    )
    for target, label in [
        (regulator, "LOGIC_REG"),
        ("PW_CONTROLLER_SUPPLY", "CTRL_REG"),
    ]:
        link(
            label + "_input",
            ("PW_POWER_DISTRIBUTION", label),
            (target, "VIN" if label == "LOGIC_REG" else "IN"),
        )
        link(
            label + "_return",
            ("PW_GROUND_DISTRIBUTION", label),
            (target, "GND_1" if label == "LOGIC_REG" else "GND"),
            [0, 0],
        )
    link(
        "controller_power",
        ("PW_CONTROLLER_SUPPLY", "OUT"),
        (controller, alias["VIN"]),
        [7, 12],
    )
    link(
        "controller_return",
        ("PW_GROUND_DISTRIBUTION", "CONTROLLER"),
        (controller, alias["GND_POWER_1"]),
        [0, 0],
    )
    link(
        "bus_interface_power",
        (regulator, "VOUT"),
        ("PW_BUS_INTERFACE", "VCC"),
        [4.8, 5.2],
    )
    link(
        "bus_interface_return",
        ("PW_GROUND_DISTRIBUTION", "BUS_INTERFACE"),
        ("PW_BUS_INTERFACE", "GND"),
        [0, 0],
    )
    for name, _ in signals:
        link(
            "host_" + name,
            (controller, alias[name]),
            ("PW_BUS_INTERFACE", bridge_signals.get(name, name)),
            [0, 5],
            protocols[name],
        )
    targets = [
        ("PW_BUS_INTERFACE", "HOST"),
        *[(m, f"M{i + 1}") for i, m in enumerate(motor_instances)],
        ("PW_TERM_HOST", "TERM_HOST"),
        ("PW_TERM_END", "TERM_END"),
    ]
    for target, tap in targets:
        for conductor in conductors:
            link(
                tap + "_" + conductor,
                ("PW_BUS_LOOM", tap + ":" + conductor),
                (target, conductor),
                protocol=bus,
                bundle="bus_pair_" + tap,
            )
    for i, motor in enumerate(motor_instances):
        link(
            f"motor_{i + 1}_power",
            ("PW_POWER_DISTRIBUTION", f"M{i + 1}"),
            (motor, "V+"),
        )
        link(
            f"motor_{i + 1}_return",
            ("PW_GROUND_DISTRIBUTION", f"M{i + 1}"),
            (motor, "V-"),
            [0, 0],
        )
    plan = {
        "schema_version": 1,
        "protocol": bus,
        "selection_status": "candidate_not_selected",
        "components": components,
        "nets": nets,
        "cables": cables,
        "pending": [
            "physical_hardware_not_selected",
            "power_budget_not_verified",
            "no_motion_or_electrical_hardware_test",
        ],
        "supply_policy": "Motor rail must satisfy all selected input ranges; 5 V regulator supplies bus electronics only, never the candidate MG4005V2 motors.",
    }
    plan["review"] = review(plan)
    return plan
