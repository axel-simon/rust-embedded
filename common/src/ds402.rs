//! The DS402 (CiA 402) device-control state.
//!
//! This module only defines the standard's vocabulary (the states
//! themselves); which of the standard's 17 transitions (0-16, below) an
//! application actually drives, and how (e.g. what triggers each one),
//! is application-specific and deliberately lives outside `common`.
//!
//! The full state diagram (edge colors: grey = main sequence, purple =
//! return transitions, amber = quick stop, red/dashed = fault handling —
//! numbers match the transition table below):
#![doc = include_str!("ds402_state_machine.svg")]
//!
//! Full transition table, for reference: 0 Reset -> initialization; 1
//! initialization complete -> Switch On Disabled; 2 Shutdown -> Ready to
//! Switch On; 3 Switch On -> Switched On; 4 Enable Operation -> Operation
//! Enable; 5 Disable Operation -> Switched On; 6 Shutdown -> Ready to
//! Switch On; 7 Disable Voltage -> Switch On Disabled; 8 Shutdown ->
//! Ready to Switch On; 9 Disable Voltage -> Switch On Disabled; 10
//! Disable Voltage/Quick Stop cleared -> Switch On Disabled; 11 Quick
//! Stop bit cleared -> Quick Stop Active; 12 Disable Voltage -> Switch On
//! Disabled; 13 fault occurs -> Fault Reaction Active; 14 fault reaction
//! completed -> Fault; 15 Fault Reset -> Switch On Disabled; 16 Quick
//! Stop bit set again -> Operation Enable.

/// The DS402 device-control state — see the module doc comment for the
/// standard's full transition table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// Axis is not ready to switch on, initialization has not completed.
    NotReadyToSwitchOn,
    /// Axis is ready to switch on, parameters can be transferred, the bus
    /// voltage can be switched on, motion functions cannot be carried out
    /// yet.
    SwitchOnDisabled,
    /// Bus voltage may be switched on, parameters can be transferred,
    /// motion functions cannot be carried out yet.
    ReadyToSwitchOn,
    /// Bus voltage must be switched on, parameters can be transferred,
    /// motion functions cannot be carried out yet.
    SwitchedOn,
    /// No fault present, output stage and motion functions are enabled.
    OperationEnabled,
    /// Drive has been stopped with the emergency ramp, output stage is
    /// enabled, motion functions are not enabled.
    QuickStopActive,
    /// A fault has occurred, the drive is in process of stopping with the
    /// quick stop ramp.
    FaultReactionActive,
    /// A fault is active, the drive has been stopped and disabled.
    Fault,
}
