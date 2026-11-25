package frc.robot.testbench;

import java.util.List;

/**
 * ★ THE ONLY FILE YOU NORMALLY EDIT ★
 *
 * <p>Mechanical brings a new mechanism? Copy one of the templates at the bottom, fill in the CAN IDs
 * and gear ratio, add it to the list, deploy ONCE. Everything else (speeds, setpoints, gains,
 * current limits, test settings) is then changed live from Elastic.
 *
 * <p>Comment out any mechanism that is not plugged in, otherwise its motors show CAN errors.
 *
 * <p>Units: speed = mechanism RPM, position = mechanism degrees (or the unit you pick),
 * gains = volts. See {@link MechanismConfig}.
 */
public final class MechanismPresets {
    private MechanismPresets() {}

    public static List<TestMechanism> create() {
        return List.of(

            // ── Current robot (same CAN IDs / inverts / limits as frc.robot.subsystems) ─────────

            new RollerMechanism(MechanismConfig.named("Intake Rollers")
                .leader(17).follower(61, true)          // 61 faces the other way
                .inverted(false).brake(false)
                .maxVolts(6.0)                          // = the old 0.5 peak duty cycle
                .currentLimits(80, 40)
                .motorFreeSpeedRPM(6380)                 // for x60 = 6000, for x44 = 7530, for falcon = 6380
                .pid(0.2, 0, 0)
                .feedforward(0.0, 0.113, 0.0)           // kS, kV, kA. kV = 12 V / (6380/60 rps). Run Speed Ramp to measure
                .targetRPM(1800)),

            new PositionMechanism(MechanismConfig.named("Intake Pivot")
                .leader(1)
                .inverted(true).brake(true)
                .gearRatio(208.28)                      // motor rotations per pivot rotation
                .currentLimits(40, 40)
                .range(0, 140).startPosition(0)         // sits at 0 deg (stowed) at power-on
                .presets(5, 120)                        // A = stow, Y = deploy
                .pid(250, 0, 0)                         // old 0.1 duty/motor-rot converted to volts/pivot-rot
                .feedforward(0.0, 25.0, 0.0)            // kS, kV, kA. kV = 12 V / (6000/60/208.28 rps). Run "Find kS + kG"
                .motionMagic(345, 345, 864)),           // old Motion Magic values converted to deg/s

            new RollerMechanism(MechanismConfig.named("Indexer")
                .leader(10)
                .inverted(true)
                .maxVolts(9.6)                          // = the old 0.8 peak duty cycle
                .currentLimits(80, 40)
                .pid(0.1, 0, 0)
                .feedforward(0.0, 0.12, 0.0)            // kS, kV, kA. kV = 12 V / (6000/60 rps)
                .targetRPM(4000)),

            new RollerMechanism(MechanismConfig.named("Feeder")
                .leader(57).follower(51, true)
                .inverted(true)
                .maxVolts(9.6)
                .currentLimits(80, 40)
                .pid(0.1, 0, 0)
                .feedforward(0.0, 0.12, 0.0)            // kS, kV, kA. kV = 12 V / (6000/60 rps)
                .targetRPM(4200)),

            new RollerMechanism(MechanismConfig.named("Shooter")
                .leader(54)                             // RShooterUp
                .follower(55, true)                     // LShooterUp
                .follower(56, true)                     // LShooterDown
                .follower(53, false)                    // RShooterDown
                .inverted(true)
                .maxVolts(9.6)
                .currentLimits(80, 40)
                .pid(0.4, 0, 0)
                .feedforward(0.25, 0.15, 0.09)          // kS, kV, kA (from the old shooter code)
                .targetRPM(2000)),

            new PositionMechanism(MechanismConfig.named("New Arm")
                .leader(2)
                .brake(true)
                .gearRatio(100.0)
                .maxVolts(6.0)
                .currentLimits(40, 40)
                .range(-30, 110).startPosition(-30) // ARM gravity: 0 deg MUST be horizontal
                .presets(0, 90)
                .gravity(MechanismConfig.Gravity.ARM, 0.0)          // run "Find kS + kG" to get kG
                .pid(30, 0, 0)
                .feedforward(0.0, 12.0, 0.0)            // kS, kV, kA. kV = 12 V / (6000/60/100 rps). Run "Find kS + kG"
                .motionMagic(90, 180, 0)),

            new PositionMechanism(MechanismConfig.named("Hood")
                .leader(52)
                .inverted(false).brake(true)
                .gearRatio(60.8417)
                .currentLimits(80, 40)
                .range(0, 38).startPosition(0)
                .presets(5, 25)
                .pid(584, 0, 0)                         // old 0.8 duty/motor-rot converted
                .feedforward(0.0, 7.30, 0.0)            // kS, kV, kA. kV = 12 V / (6000/60/60.8417 rps)
                .motionMagic(1183, 1775, 1775))

            // ── TEMPLATES: copy, fill in, add above ──────────────────────────────────────────
            //
            // A roller (intake, feeder, conveyor, gripper wheels, flywheel):
            //
            // new RollerMechanism(MechanismConfig.named("New Roller")
            //     .leader(20).follower(21, true)      // follower optional; true = opposite direction
            //     .gearRatio(3.0)                     // 3:1 reduction -> RPM shown is ROLLER RPM
            //     .maxVolts(6.0)                      // start gentle on a new mechanism
            //     .currentLimits(60, 40)
            //     .pid(0.1, 0, 0)
            //     .feedforward(0.0, 0.36, 0.0)        // kS, kV, kA. kV = 12 V / (6000/60/3 rps) = 0.36
            //     .targetRPM(1000)),
            //
            // An arm / wrist / pivot (degrees):
            //
            // new PositionMechanism(MechanismConfig.named("New Arm")
            //     .leader(30)
            //     .brake(true)
            //     .gearRatio(100.0)
            //     .maxVolts(6.0)
            //     .currentLimits(40, 40)
            //     .range(-30, 110).startPosition(-30) // ARM gravity: 0 deg MUST be horizontal
            //     .presets(0, 90)
            //     .gravity(MechanismConfig.Gravity.ARM, 0.0)          // run "Find kS + kG" to get kG
            //     .pid(30, 0, 0)
            //     .feedforward(0.0, 12.0, 0.0)        // kS, kV, kA. kV = 12 V / (6000/60/100 rps) = 12
            //     .motionMagic(90, 180, 0)),
            //
            // An elevator (inches):
            //
            // new PositionMechanism(MechanismConfig.named("New Elevator")
            //     .leader(40).follower(41, true)
            //     .brake(true)
            //     .gearRatio(12.0)
            //     .units("in", Math.PI * 1.751)       // inches per drum/sprocket rotation (pitch dia x pi)
            //     .range(0, 50).startPosition(0)
            //     .presets(5, 40)
            //     .gravity(MechanismConfig.Gravity.ELEVATOR, 0.0)
            //     .pid(2, 0, 0)
            //     .feedforward(0.0, 1.2, 0.0)         // kS, kV, kA. kV = 12 V / (6000/60/12 rps) = 1.2
            //     .motionMagic(20, 40, 0)),
        );
    }
}
