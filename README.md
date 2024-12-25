# The Governor

The **Governor** is an experiment state manager: it manages the transitions between
different positioners positions in a beamline endstation in a safe and reliable way.
This is especially useful if such positioners share motion envelopes and, therefore, might
collide with each other if not driven carefully. To achieve this goal, the **Governor**
implements a fully configurable state machine.

The **Governor** is controlled via [EPICS](http://www.aps.anl.gov/epics/index.php)
PVs. The devices controlled by the **Governor**, in turn, must be controllable through
EPICS PVs.

This document starts with an example and then expands on the **Governor** concepts.

## Example Instance

This example is a trimmed down and modified version of actual configuration files used in NSLS-II's FMX and
AMX beamlines. These beamlines have a very tight endstation and instruments that can collide with each other.

They have two modes for loading samples: Human and Robot. Human mode **Transitions** are slower because
the instruments are moved a long distance to to ease the mounting of a sample by a person. Robot mode,
however, can have a tighter mounting space due to the precision and repeatability of the sample mounting
robot.

In this hypothetical example, let's assume that there are three instruments: a sample backlight (`li`,
Light), a beam stop (`bs`, Beam Stop) and a detector cover (`dc`, Detector Cover). Let's also assume
that the beam stop `In` Target collides with the backlight's `Up` Target. Therefore, whenever we want to
move the light `Up`, we have to first take the beam stop `Out`. Conversely, whenever we want to move the beam
stop to the `In` Target, we have to first make sure that the light is `Down`. Here are the configuration files:

### Configuration Files

File `human.yaml`:

    name: Human

    devices:
      bs:
        type: Motor
        name: Beam Stop
        pv: FMX{BS:1-Ax:Z}Mtr
        tolerance: 5
        timeout: 5
        positions:
          In: 40.0
          Out: 15.0
      li:
        type: Motor
        name: Light
        pv: FMX{Light:1-Ax:Y}Mtr
        tolerance: 5
        timeout: 5
        positions:
          Up: 8.0
          Down: -100.0
      dc:
        type: Valve
        name: Detector Cover
        pv: FMX{Det:FMX-Cover}
        timeout: 5

    states:
      M:
        name: Maintenance
      SE:
        name: Sample Exchange
        targets:
          bs: {target: Out, limits: [0, 0.0]}
          li: {target: Up, limits: [0, 0]}
          dc: {target: Closed, limits: [0, 0]}
      SA:
        name: Sample Alignment
        targets:
          bs: {target: In, limits: [0, 0]}
          li: {target: Down, limits: [-101.0, 1.0], updateAfter: True}
          dc: {target: Open, limits: [0, 0]}

    init_state: M

    transitions:
      M:
        SE: [dc, li, bs]
      SE:
        SA: [[dc, bs], li]
      SA:
        SE: [[dc, li], bs]

File `robot.yaml`: same as `human.yaml` but with a different name and different Target values.

Let's break the `human.yaml` file down:

#### Setting the State Machine's name

First, the name of the State Machine is set. This is what will appear in the `Config-Sel` PV.

    name: Human

#### Declaring the Devices

    devices:
      bs:
        type: Motor
        name: Beam Stop
        pv: FMX{BS:1-Ax:Z}Mtr
        tolerance: 5
        timeout: 5
        positions:
          In: 40.0
          Out: 15.0
      li:
        type: Motor
        name: Light
        pv: FMX{Light:1-Ax:Y}Mtr
        tolerance: 5
        timeout: 5
        positions:
          Up: 8.0
          Down: -100.0
      dc:
        type: Valve
        name: Detector Cover
        pv: FMX{Det:FMX-Cover}
        timeout: 5

Here, three **Devices** are declared. Note that the `dc` **Device** is a Valve-like device
and the `pv` field is actually a prefix. The actual PVs that will be used for this
particular device are:

    FMX{Det:FMX-Cover}Pos-Sts
    FMX{Det:FMX-Cover}Cmd:Opn-Cmd
    FMX{Det:FMX-Cover}Cmd:Cls-Cmd

A `tolerance` of `5` for `bs` means that, for example, when in `In`, the **Governor** will
allow `bs` to go between `35.0` and `45.0` before complaining.

The `timeout` specifies the time that the **Governor** will wait for the device to reach the desired
position in a transition. A slow (or sputtering) motor can still use a short `timeout` value
because the timeout timer only starts if the motor is standing still according to its readback value.

Here both `li` and `bs` have two Targets each. Any number of targets can be specified.

Note how `dc` doesn't specify any Target. Valve-like **Devices** have two implicit Targets: `Open` and
`Closed`.

#### Declaring the States

    states:
      M:
        name: Maintenance
      SE:
        name: Sample Exchange
        targets:
          bs: {target: In, limits: [0, 0.0]}
          li: {target: Down, limits: [0, 0]}
          dc: {target: Closed, limits: [0, 0]}
      SA:
        name: Sample Alignment
        targets:
          bs: {target: Out, limits: [0, 0]}
          li: {target: Up, limits: [-101.0, 1.0], updateAfter: True}
          dc: {target: Open, limits: [0, 0]}

    init_state: M

Note that the first **State** is special. When things go wrong, the **Governor** will immediately move to this
**State**, and stay there while the fault condition persists. There should be **NO** incoming **Transitions**
into this **State**. However, this **State** can always be reached from any other **State** immediately,
without moving any **Device**.

Each **State** takes a short name, a long name and the list of Targets that define the **State**. The
`limits` entry defines the allowed range of motion for that particular **Device** when in that particular
**State**. For example:

    li: {target: Up, limits: [-101.0, 1.0], updateAfter: True}

When in the `SA` **State**, the `li` **Device** will be in the `Up` Target (which is the position `8.0`).
Now, the `tolerance` for this particular **Device** is `5`, which means that `li` has an allowed range of
motion of `[3.0, 13.0]` without complaints by the **Governor**. Furthermore, since the `limits` are set to
`[-101.0, 1.0]`, the final allowed range is `[-98.0, 14.0]`.

The last interesting bit on this line is the `updateAfter` setting. When `True`, it means that the Target will
be updated after leaving this **State**. For example, if we are in the `SA` **State** (Target `Up: 8.0`) and we
make a small adjustment to the motor position, say, to `7.0` (which must be in the allowed range `[-98.0, 14.0]`),
then when the **Governor** *leaves* the **State** `SA`, the Target `Up` will be set to `7.0` instead of `8.0`.

#### Declaring the Transitions

    transitions:
      M:
        SE: [dc, li, bs]
      SE:
        SA: [[dc, bs], li]
      SA:
        SE: [[dc, li], bs]

This set of **Transitions** is the last necessary piece of information to fully define the State Machine:

<center> M → SE ↔ SA </center>

The first transition, M → SE is declared as follows:

      M:
        SE: [dc, li, bs]

This means that first `dc` (Detector Cover) will be moved (to the `Closed` Target), then, *after it is done*,
`li` will be moved to `Down` and finally, *after `li` is done*, `bs` will be moved to `In`.

Sometimes more than one **Device** can be moved in parallel, and the **Governor** allows for that. The next lines
illustrate the syntax:

      SE:
        SA: [[dc, bs], li]

In this transition, SE → SA, `dc` and `bs` are moved in parallel. Only when *both* reach their targets, `li` is
moved. The transition SA → SE is similar, but `li` is moved before `bs` to satisfy physical constraints.

### Synchronization File

Assuming that we want to keep all `li` Targets and one of the `bs` Targets in sync between `Human` and `Robot` State
Machines, we can use a synchronization file (`sync.yaml`):

    li: [Up, Down]
    bs: [In]

If this file is loaded, whenever one of these three Targets is changed in the `Human` State Machine, its counterpart
in the `Robot` State Machine will reflect the changes. The converse is also true. `bs`'s `Out`, however, will have
potentially different values in `Human` and in `Robot`.

### Command line invocation

    python3 main.py -c human.yaml robot.yaml -s sync.yaml --prefix FMX

### Resulting PVs

The following PVs will be created after the previous invocation:

    FMX{Gov}Active-Sel                    # enum values: "Inactive", "Active". "Inactive" prevents all changes
    FMX{Gov}Config-Sel                    # enum values: "Human", "Robot"
    FMX{Gov}Sts:Configs-I                 # list of all state machines: ["Human", "Robot"]
    FMX{Gov}Cmd:Abort-Cmd                 # when written to, abort the current transition in active Governor instance
    FMX{Gov}Cmd:Kill-Cmd                  # when written to, forces the IOC to exit

    FMX{Gov:Human}Sts:States-I            # a sorted list of all state names: ["M", "SA", "SE"]
    FMX{Gov:Human}Sts:Devs-I              # a sorted list of all device names: ["bs", "dc", "li"]
    FMX{Gov:Human}Sts:State-I             # the name of the current state: "M" (at first)
    FMX{Gov:Human}Sts:Reach-I             # a list of all currently reachable states: ["SE"] (at first)
    FMX{Gov:Human}Sts:Busy-Sts            # 1 if performing a transition, 0 otherwise

    FMX{Gov:Human}Cmd:Abort-Cmd           # when written to, abort the current transition
    FMX{Gov:Human}Cmd:Go-Cmd              # write the name of the desired state to start a transition
    FMX{Gov:Human}Sts:Status-Sts          # current state machine status (Idle/Busy/Disabled/FAULT)
    FMX{Gov:Human}Sts:Msg-Sts             # message from the Governor

    FMX{Gov:Human-Dev:bs}Sts:Tgts-I       # value: ["In", "Out"]
    FMX{Gov:Human-Dev:li}Sts:Tgts-I       # value: ["Up", "Down"]

    FMX{Gov:Human-Dev:bs}Pos:In-Pos       # value: 40.0
    FMX{Gov:Human-Dev:bs}Pos:Out-Pos      # value: 15.0
    FMX{Gov:Human-Dev:li}Pos:Up-Pos       # value: 8.0
    FMX{Gov:Human-Dev:li}Pos:Down-Pos     # value: -100.0

    FMX{Gov:Human-Dev:bs}SE:LLim-Pos      # value: 0.0, lower limit of the bs device when in state SE
    FMX{Gov:Human-Dev:bs}SE:HLim-Pos      # value: 0.0, upper limit of the bs device when in state SE
    FMX{Gov:Human-Dev:bs}SA:LLim-Pos      # value: 0.0
    FMX{Gov:Human-Dev:bs}SA:HLim-Pos      # value: 0.0
    FMX{Gov:Human-Dev:li}SE:LLim-Pos      # value: 0.0
    FMX{Gov:Human-Dev:li}SE:HLim-Pos      # value: 0.0
    FMX{Gov:Human-Dev:li}SA:LLim-Pos      # value: -101.0
    FMX{Gov:Human-Dev:li}SA:HLim-Pos      # value: 1.0

    FMX{Gov:Human-St:M}Sts:Reach-Sts      # if 1, the state M is reachable from the current state
    FMX{Gov:Human-St:M}Sts:Active-Sts     # if 1, M is the current state
    FMX{Gov:Human-St:SE}Sts:Reach-Sts
    FMX{Gov:Human-St:SE}Sts:Active-Sts
    FMX{Gov:Human-St:SA}Sts:Reach-Sts
    FMX{Gov:Human-St:SA}Sts:Active-Sts

    FMX{Gov:Human-Tr:M-SE}Sts:Active-Sts  # when 1, the transition M->SE is in progress
    FMX{Gov:Human-Tr:M-SE}Sts:Reach-Sts   # when 1, the transition M->SE can be requested
    FMX{Gov:Human-Tr:SE-SA}Sts:Active-Sts
    FMX{Gov:Human-Tr:SE-SA}Sts:Reach-Sts
    FMX{Gov:Human-Tr:SA-SE}Sts:Active-Sts
    FMX{Gov:Human-Tr:SA-SE}Sts:Reach-Sts

    # Also, corresponding PVs will be generated with the
    # prefix "FMX{Gov:Robot"


## State Machines

The **Governor** can manage several State Machines at the same time. However, only one
State Machine can be enabled at any given time. This is useful if one or more State
Machines that share devices and setpoints are to be used.

## Entities

There are three kinds of entities in the **Governor**:

### 1. Devices

A **Device** entity represents, quite simply, a positioner that will be driven by the
**Governor**. Currently a **Device** can be of one of two types:

  1. `Motor`: a motor that is available as an EPICS PV for a motor record.
  2. `Valve`: a valve-like device that is available as a few PVs that follow the
              NSLS-II Naming Standard.

A third type exists, called simply `Device`, which represents a dummy **Device**. Dummy devices always move
immediately and successfully to their Targets. No communication with a real device is done, even if the `pv`
parameter is specified. This is useful for debugging, for temporarily disabling a device without having
to modify the entire State Machine and for testing new devices before issuing real "move" commands.

### 2. States

A **State** entity represents a particular experiment state with the specific positions
the positioners are expected to be in. **States** are the vertices in the State Machine.

The *initial* **State** is special: it is a fallback that is moved into when a transition
fails.

### 3. Transitions

A **Transition** specifies which positioners will be moved between **States** and in
which order.

## Command line arguments

This is a list of command line arguments accepted by the **Governor**.

    usage: main.py [-h] -c CONFIG [CONFIG ...] [--check_config]
                   [-l {DEBUG,INFO,WARNING,ERROR,CRITICAL}] [--prefix PREFIX]
                   [-s SYNC]


* `-h`: Shows the help message and exit
* `-c CONFIG [CONFIG ...]`: Configuration files to be loaded. Each configuration file
   will result in a different State Machine being instantiated. At least one configuration
   file must be passed in.
* `--check-config`: Exits right after checking that the configuration files passed in
  the `-c` option are valid. Any errors in the configuration files are reported.
* `-l, --log-level`: The log level to be used. One of `DEBUG`, `INFO`, `WARNING`,
  `ERROR` or `CRITICAL`. The default log level is `INFO`.
* `--prefix`: The prefix to be prepended to the **Governor**'s own PVs. Default is an
  empty string.
* `-s SYNC, --sync SYNC`: The name of the Synchronization Configuration file to be used.

## Configuration files

Each **Governor** State Machine is generated from a [YAML](http://www.yaml.org/)
configuration file.


### Configuration name
### Devices
### States
### Transitions
### Initial State

## Exposed PVs

The **Governor** exposes the following PVs (here shown without prefix). The PVs follow the
NSLS-II Naming Convention.

### Base PVs:

* `{Gov}Active-Sel`: R/W, enumeration. Allowed values: `Active`, `Inactive`. Controls whether
  the **Governor** will accept any command to any State Machine.

* `{Gov}Config-Sel`: R/W, enumeration. Allowed values: names of the loaded State Machines.
  Controls which State Machine is active at the moment.

* `{Gov}Sts:Configs-I`: R/O, list of string. Holds the list of existing State Machines.

* `{Gov}Cmd:Abort-Cmd`: W/O. Writing any value to this PV will abort the current in progress
  **Transition**.

* `{Gov}Cmd:Kill-Cmd`: W/O. Writing any value to this PV will cause the Governor to exit.

### Per State Machine PVs

Assuming a State Machine named "SM".

* `{Gov:SM}Cmd:Go-Cmd`: W/O, string. Write the name of a **State** to this PV to start a **Transition**.

* `{Gov:SM}Sts:Status-Sts`: R/O, enumeration. Possible values:
  * `Idle`: when holding a state.
  * `Busy`: when transitioning between states.
  * `Disabled`: when a State Machine other that SM is enabled.
  * `FAULT`: when at least one device is either Disconnected or Not Homed.

* `{Gov:SM}Sts:Msg-Sts`: R/O, string. A descriptive string for the State Machine status:
  * When status is `Idle` or `Disabled`, this PV will display the name of "SM"'s
  current **State**.
  * When status is `Busy`, this PV will display the names of the states involved in the
  **Transition** in progress.
  * When status is `FAULT`, this PV will display the conditions that are causing the State
  Machine to be in a Fault state.

* `{Gov:SM}Sts:Busy-Sts`: R/O, enumeration. Possible values:
  * `No`: a **Transition** is not in progress.
  * `Yes`: a **Transition** is in progress.
* `{Gov:SM}Sts:State-I`: R/O, string. Holds current **State** name.
* `{Gov:SM}Sts:Devs-I`: R/O, list of string. Holds all **Device** names.
* `{Gov:SM}Sts:States-I`: R/O, list of string. Holds all **State** names.
* `{Gov:SM}Sts:Reach-I`: R/O, list of string. Holds all **State** names that are reachable
  from the current state.

### Per State PVs

Assuming a State Machine named "SM" and a **State** named "A".

* `{Gov:SM-St:A}Sts:Reach-Sts`: R/O, binary. Indicates whether the "A" **State** is reachable from
  the current **State**.
* `{Gov:SM-St:A}Sts:Active-Sts`: R/O, binary. Indicates whether "A" **is** the current **State** of
  the "SM" State Machine

### Per Device PVs

Assuming a **Device** named "dev".

* `{Gov:SM-Dev:dev}Sts:Tgts-I`: R/O, list of string. Holds all **Target** names of this **Device**.

### Per Device and State PVs

Assuming a State Machine named "SM", a **State** named "A" and a **Device** named "dev".

* `{Gov:SM-Dev:dev}A:LLim-Pos`: R/W, number. Holds the lower limit, relative to the target position,
  for the device "dev" when in state "A".
* `{Gov:SM-Dev:dev}A:HLim-Pos`: R/W, number. Holds the upper limit, relative to the target position,
  for the device "dev" when in state "A".

### Per Device and Target PV

Assuming a State Machine named "SM", a **Device** named "dev" and a Target named "Tgt".

* `{Gov:SM-Dev:dev}Pos:Tgt-Pos`: R/W, number. The actual position for the Target "Tgt" of the **Device**
  "dev".

### Per Transition PVs

Assuming a transition between states A and B.

* `{Gov:SM-Tr:A-B}Sts:Reach-Sts`: R/O, binary. Indicates whether the **Transition** A → B can be
  performed.
* `{Gov:SM-Tr:A-B}Sts:Active-Sts`: R/O, binary. Indicates whether the **Transition** A → B is
  currently under way.

## Synchronization Configuration

When more than one State Machine is loaded into the Governor, it is sometimes desirable to keep some
Targets for the same **Device** across State Machines in sync. In order to achieve this goal, a
Synchronization Configuration file can be used. The syntax for this YAML file is very simple: it is a
list of entries of the form `dev_name: [List of Targets]`. For example:

    device1: [Up, Down]
    device2: [In]

If this synchronization file is loaded, then whenever one of the specified Targets is changed in one
State Machine the change will be propagated to all other State Machines currently loaded.json



