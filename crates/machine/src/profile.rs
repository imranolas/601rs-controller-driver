//! The machine profile (PROGRAM_FORMAT §8): the facts about one machine that turn board
//! millimetres into controller commands. It is separate from programs, so recalibrating the
//! machine never invalidates a stored program. [`Profile::gunnar_601`] holds the 601rs's values,
//! from GMC's parameters [GCF] and its captures [CAP]; the protocol reference cites each.

use program::Head as HeadKind;

use crate::frame::{CarriageMm, MachineMm};
use protocol::{Axis, Setting, SignalAcks, Units};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Profile {
    pub machine: String,
    pub scale: Scale,
    /// Where the board frame's origin (the board's bottom-left corner, PROGRAM_FORMAT §2) lies
    /// in machine millimetres. A head's blade is at machine position `carriage + head offset`,
    /// where the carriage position is its steps ÷ scale.
    ///
    /// The 601's value is where GMC places the board: its straight-edge cuts in the captures
    /// lie at X 601–602 and Y 761–762 steps (protocol reference §4.2), in the front-left
    /// corner by the red button, where boards register; board +Y is machine +Y, away from the
    /// operator (protocol reference §4.1; `gunnar check` stage 3 confirms it).
    pub board_origin_mm: MachineMm,
    pub bevel: Head,
    pub straight: Head,
    pub clamps: Clamps,
    /// The outputs with the heads up and nothing clamped (`$A02`): bit 1 (`i_bh`) holds the
    /// bevel head up; the straight head is up while its bit is off (protocol reference §2.5).
    /// Every output this crate sends outside a bevel cut keeps `i_bh` on.
    pub neutral_outputs: u8,
    /// Steps/s at 100 % (`#M`, GMC's `AbsMaxSpeed`): speeds are percentages of it.
    pub max_speed: u32,
    /// Head-up travel speed, % of the maximum (GMC's `PosSpeed`).
    pub travel_speed_pct: f64,
    pub accel: Accel,
    /// `#S`, steps/s: the speed the motors start at without a ramp (GMC's `StartStop`).
    pub start_speed: u32,
    pub soft_start: SoftStart,
    /// A cut whose first segment runs off the axes gets a longer ramp: the cutting ramp ×
    /// (1 + the angle off the nearest axis ÷ 45°) ÷ this factor, at most `max_diagonal_ramp_ms`
    /// (GMC's `DiagonalRampeSmoother` and `DiagonalRampeSmoothFactor`). `None`: no such ramp.
    pub diagonal_ramp_factor: Option<f64>,
    pub max_diagonal_ramp_ms: u16,
    /// `#C1` dead time (ms) before a cut starts moving, so the controller has the path in hand.
    pub dead_time_ms: DeadTime,
    /// `#B`: the braking angle, degrees. A sharper turn between vectors stops the carriage.
    pub brake_angle_deg: u8,
    pub radius_slowdown: RadiusSlowdown,
    /// The sharpest turn a cut may make at one point, degrees (PROGRAM_FORMAT V7). A sharper
    /// corner must be separate cuts.
    pub break_angle_deg: f64,
    /// Where the carriage may go, machine millimetres (soft limits, V10).
    pub envelope_mm: Envelope,
    /// Where the carriage parks at the end of a job, steps.
    pub park_steps: [i32; 2],
    /// The bevel head's V-groove switch, if a stub at the back of the table sets it: the
    /// runner sets it to normal cutting after homing, as GMC does (protocol reference §4.1).
    pub vgroove_switch: Option<VGrooveSwitch>,
    pub line: Line,
    /// The controller configuration everything here assumes.
    pub controller: ControllerConfig,
    pub homing: Homing,
    /// What `gunnar check` has shown this controller does beyond GMC's form.
    pub capabilities: Capabilities,
}

/// The controller configuration the machine model assumes, beyond what each job sets: GMC
/// writes it at connect [CAP g_init], and the runner checks it there (protocol reference §10:
/// whether the box boots with it is for the probe to say). See
/// [`Profile::controller_settings`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerConfig {
    /// `#ZZ`: the blade axis's steps per revolution. With `#C13` (rotary) the controller turns
    /// it the short way round to each angle; the blade angles assume it.
    pub z_steps_per_rev: u32,
    /// `#NZ`: its µm per revolution.
    pub z_um_per_rev: u32,
    /// `#C14`: dynamic path speed, bits 1 diagonal, 2 radius, 4 tangential (the radius from the
    /// blade's turn), 8 buffer fill. Curves slow down only with 2 or 4; the run-time estimate
    /// assumes 4.
    pub dynamic_speed: u8,
    /// `#C2`: extra braking criteria. Bit 2 (stop on a change of speed entry) must be off for
    /// the one-stream form, where travel and cutting use different entries.
    pub brake_criteria: u8,
}

/// How the machine homes: GMC's sequence, Z first, then X and Y [CAP g_init; protocol
/// reference §4.1].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Homing {
    /// `#G10` and `#G11`: the speeds (steps/s) of the switch search and the back-off.
    pub search_speed: u32,
    pub back_off_speed: u32,
    /// `#R` while homing X and Y.
    pub ramp_ms: u16,
    /// `#O`: how far each axis moves off its switch after homing, steps (X, Y, Z).
    pub offset_steps: [i32; 3],
}

/// Where the bevel head's V-groove switch (GMC's DBD) is set: the carriage pushes the head
/// against a stub at the back of the table, and the blade's angle picks the way the switch goes
/// [GCF `VGChangeY`, `VGChangeZ`; protocol reference §4.1].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VGrooveSwitch {
    /// The carriage's Y against the stub, steps. It lies past the envelope: the one place the
    /// carriage goes outside it.
    pub y_steps: i32,
    /// The blade's angle on the bevel head that sets normal cutting, degrees; 180° on from it
    /// sets V-groove.
    pub normal_deg: f64,
}

/// The serial line to the controller.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Line {
    /// What it carries: 115,200 baud, 10 bits a byte.
    pub bytes_per_sec: f64,
    /// The share of it a path may need, leaving room for the replies and the unexpected
    /// (crate docs, "Keeping up with the cut").
    pub budget_share: f64,
}

/// Controller features the one-stream-per-cut form uses. Each needs firmware 2.00 to do what the
/// v1.00 spec says, so each stays off until `gunnar check` stage 5 has shown it on the box
/// (protocol reference §10). With none, a job is lowered in GMC's form.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Capabilities {
    /// A path as one absolute vector and then relative steps without `L20,` (follow-on
    /// vectors): half the bytes, so curves fit the serial line.
    pub compact_paths: bool,
    /// `A` and `W` in the stream: the head goes down, settles and comes up in sequence with
    /// the path, with no mode switch and no host wait.
    pub stream_head: bool,
    /// A second speed entry (`L21`) runs at its own feed, so the travel to a cut joins the
    /// cut's stream.
    pub travel_entry: bool,
}

impl Capabilities {
    /// Whether consecutive cuts can share a stream.
    pub fn streams(self) -> bool {
        self.stream_head || self.travel_entry
    }

    /// Everything on: what the simulator, which follows the spec, supports.
    pub fn all() -> Self {
        Self {
            compact_paths: true,
            stream_head: true,
            travel_entry: true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scale {
    pub x_steps_per_mm: f64,
    pub y_steps_per_mm: f64,
    /// Steps per degree of blade rotation. The 601's is GMC's `ScaleZ`, 8.3333 as written:
    /// GMC truncates `8.3333 × degrees`, and 25/3 would round some angles up a step.
    pub z_steps_per_deg: f64,
}

/// A cutting head.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Head {
    /// Where its blade is relative to the carriage, mm (GMC's `StartPX`/`StartPY`).
    pub offset_mm: [f64; 2],
    /// The blade's direction of travel when Z reads 0, degrees (GMC's `StartPZ`).
    pub blade_mount_deg: f64,
    /// The outputs (`$A` bits) that lower it.
    pub down: u8,
    /// The outputs that lower it for a closed curve, if they differ: GMC's slow head-down,
    /// `i_t` alone (`$A41`, not `$A45`). The blade comes down through the Z head's throttle and
    /// enters the board gradually while the carriage moves, since on a closed curve it goes in
    /// on the edge that shows (protocol reference §2.5).
    pub down_closed: Option<u8>,
    /// The outputs that take the blade out of the board before the head goes up, if the head
    /// does that in two steps (the bevel head: `i_b` and `i_t` off).
    pub lift: Option<u8>,
    /// The outputs that hold it up (`i_bh`).
    pub up: u8,
}

/// The clamps' output bits.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Clamps {
    pub left: u8,
    pub right: u8,
}

/// Accelerations, mm/s². The ramp time `#R` is the time to reach the maximum speed at one.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Accel {
    pub travel: f64,
    pub cut: f64,
    /// Short travel with a large turn of the blade (GMC's `_FastZRampe`).
    pub fast_turn: f64,
    /// The longest ramp, ms (GMC caps `#R` at 700).
    pub max_ramp_ms: u16,
}

/// GMC's soft start for the straight head on thick board (protocol reference §3.3): from
/// `thin_mm` up, the ramp lengthens toward `max_ramp_ms` and `#S` falls toward `slow_speed`,
/// by the factor thickness ÷ (`thin_mm` + `thick_mm`), or fully above `thick_mm`. Below
/// `thin_mm`, `#S` is `thin_speed`. The bevel head gets none.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SoftStart {
    pub thin_mm: f64,
    pub thick_mm: f64,
    /// `#S` on the thickest board, steps/s (GMC's `StartSpeed`).
    pub slow_speed: u32,
    /// `#S` below `thin_mm`, steps/s (GMC's `CBStartStop`, for drawn shapes).
    pub thin_speed: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeadTime {
    /// A cut of one straight segment.
    pub line: u16,
    /// A cut along a curve.
    pub curve: u16,
}

/// `#C20`–`#C28` (protocol reference §2.4.1).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RadiusSlowdown {
    pub min_radius_steps: u32,
    pub max_radius_steps: u32,
    pub lower_pct: u32,
    pub upper_pct: u32,
    pub faster_count: u32,
    pub slower_count: u32,
    pub steepness: u32,
}

/// The carriage's range: where it may go, in carriage mm.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Envelope {
    pub x: [f64; 2],
    pub y: [f64; 2],
}

impl Envelope {
    pub fn contains(&self, CarriageMm([x, y]): CarriageMm) -> bool {
        (self.x[0]..=self.x[1]).contains(&x) && (self.y[0]..=self.y[1]).contains(&y)
    }
}

impl Head {
    /// Where the carriage is with this head's blade at `blade`.
    pub fn carriage(&self, MachineMm([x, y]): MachineMm) -> CarriageMm {
        let [dx, dy] = self.offset_mm;
        CarriageMm([x - dx, y - dy])
    }

    /// Where this head's blade is with the carriage at `carriage`.
    pub fn blade(&self, CarriageMm([x, y]): CarriageMm) -> MachineMm {
        let [dx, dy] = self.offset_mm;
        MachineMm([x + dx, y + dy])
    }
}

impl Profile {
    /// The Gunnar 601rs, as GMC drives it.
    pub fn gunnar_601() -> Self {
        Self {
            machine: "gunnar-601rs".to_owned(),
            // [GCF] MachineScale 400/cm; ScaleZ 8.3333.
            scale: Scale {
                x_steps_per_mm: 40.0,
                y_steps_per_mm: 40.0,
                z_steps_per_deg: 8.3333,
            },
            board_origin_mm: MachineMm([21.225, 20.725]),
            // [GCF] Heads[*].ToolHeadCorrection StartPX/StartPY/StartPZ; outputs [CAP].
            bevel: Head {
                offset_mm: [-71.5, 1.9],
                blade_mount_deg: 0.0,
                down: 0x05,
                down_closed: Some(0x01),
                lift: Some(0x00),
                up: 0x02,
            },
            straight: Head {
                offset_mm: [6.2, 1.7],
                blade_mount_deg: 180.0,
                down: 0x0A,
                down_closed: None,
                lift: None,
                up: 0x02,
            },
            // GMC closes 0x40 for a board on the left half of the table and 0x20 on the right,
            // both for a board wider than half (protocol reference §2.5); stage 3 confirms.
            clamps: Clamps {
                left: 0x40,
                right: 0x20,
            },
            neutral_outputs: 0x02,
            max_speed: 36_000,
            travel_speed_pct: 100.0,
            // [GCF] _PosRampe, _RampeCut, _FastZRampe; GMC's 700 ms cap.
            accel: Accel {
                travel: 4000.0,
                cut: 3600.0,
                fast_turn: 8000.0,
                max_ramp_ms: 700,
            },
            start_speed: 4000,
            // [GCF] DynamicMatSetting MatLow 0.2 cm, MatHigh 0.35 cm; StartSpeed 400;
            // ImportParameters.CBStartStop 1000.
            soft_start: SoftStart {
                thin_mm: 2.0,
                thick_mm: 3.5,
                slow_speed: 400,
                thin_speed: 1000,
            },
            // [GCF] DiagonalRampeSmoother true, DiagonalRampeSmoothFactor 1; GMC's cap.
            diagonal_ramp_factor: Some(1.0),
            max_diagonal_ramp_ms: 1500,
            dead_time_ms: DeadTime {
                line: 100,
                curve: 200,
            },
            brake_angle_deg: 90,
            radius_slowdown: RadiusSlowdown {
                min_radius_steps: 400,
                max_radius_steps: 12_000,
                lower_pct: 30,
                upper_pct: 100,
                faster_count: 5,
                slower_count: 5,
                steepness: 1,
            },
            break_angle_deg: 15.0,
            // [GCF] DRX 156 cm, DRY 101.5 cm: GMC's own soft-limit basis, until the diagnostic
            // tool measures the envelope.
            envelope_mm: Envelope {
                x: [0.0, 1560.0],
                y: [0.0, 1015.0],
            },
            // [GCF] AutoParkX/Y 140 × 20 cm, the bevel blade's position: its carriage less the
            // bevel's offset, 147.15 × 19.81 cm [CAP].
            park_steps: [58_860, 7_923],
            // [GCF] VGChangeY 104.5 cm, VGChangeZ 90°.
            vgroove_switch: Some(VGrooveSwitch {
                y_steps: 41_800,
                normal_deg: 90.0,
            }),
            controller: ControllerConfig {
                z_steps_per_rev: 3000,
                z_um_per_rev: 10_000,
                dynamic_speed: 13,
                brake_criteria: 0,
            },
            homing: Homing {
                search_speed: 1750,
                back_off_speed: 750,
                ramp_ms: 100,
                offset_steps: [300, 300, 0],
            },
            line: Line {
                bytes_per_sec: 11_520.0,
                budget_share: 0.7,
            },
            capabilities: Capabilities::default(),
        }
    }

    pub fn head(&self, kind: HeadKind) -> &Head {
        match kind {
            HeadKind::Bevel => &self.bevel,
            HeadKind::Straight => &self.straight,
        }
    }

    /// Steps/s for a speed in % of the maximum (`#G20 = 360 × speed %` on the 601).
    pub fn speed(&self, pct: f64) -> u32 {
        (self.max_speed as f64 * pct / 100.0) as u32
    }

    /// The ramp time `#R` (ms) for an acceleration: the time to reach the maximum speed,
    /// at most `max_ramp_ms` (protocol reference §3.1).
    pub fn ramp_ms(&self, accel: f64) -> u16 {
        let top = self.max_speed as f64 / self.scale.y_steps_per_mm;
        let ms = (1000.0 * top / accel) as u32;
        u16::try_from(ms.min(u32::from(self.accel.max_ramp_ms))).unwrap_or(self.accel.max_ramp_ms)
    }

    /// The controller configuration to check at connect, and to write where it differs:
    /// steps as units, the maximum speed the ramps are measured against, the blade axis's
    /// resolution and rotary turning, the dynamic speed and braking, the homing ramp, and the
    /// signals the session uses (the green button's as bytes, `#C35,1`, as GMC sets it; the
    /// buffer's flow control only for the one-stream form).
    pub fn controller_settings(&self) -> Vec<Setting> {
        let c = self.controller;
        vec![
            Setting::Units(Units::Steps),
            Setting::MaxSpeed(self.max_speed),
            Setting::StepsPerRev {
                axis: Axis::Z,
                value: c.z_steps_per_rev,
            },
            Setting::MicrometresPerRev {
                axis: Axis::Z,
                value: c.z_um_per_rev,
            },
            Setting::RotaryZ(true),
            Setting::DynamicSpeed(c.dynamic_speed),
            Setting::BrakeCriteria(c.brake_criteria),
            Setting::HomingRamp(true),
            Setting::PositionedSignal(true),
            Setting::HomingDoneSignal(true),
            Setting::SignalAcks(SignalAcks {
                input1: true,
                input2: false,
            }),
            // The buffer's 0x03/0x02 flow control: GMC never enables it and doesn't expect it,
            // so GMC's form keeps it off; one stream per cut sends the next cut while the last
            // one runs, and needs it (gunnar check stage 6).
            Setting::FlowControl(self.capabilities.streams()),
        ]
    }

    /// Where the carriage is at `steps`.
    pub fn carriage_mm(&self, [x, y]: [i32; 2]) -> CarriageMm {
        let s = self.scale;
        CarriageMm([
            f64::from(x) / s.x_steps_per_mm,
            f64::from(y) / s.y_steps_per_mm,
        ])
    }

    /// The steps that put the carriage nearest `c`.
    pub fn carriage_steps(&self, CarriageMm([x, y]): CarriageMm) -> [i32; 2] {
        let s = self.scale;
        [
            (x * s.x_steps_per_mm).round() as i32,
            (y * s.y_steps_per_mm).round() as i32,
        ]
    }
}
