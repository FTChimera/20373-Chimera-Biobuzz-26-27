/**
 * Test-only FTC hardware; never deploy these devices.
 *
 * <p>FakeHardwareMap.register(name, device) populates the overall map and all applicable
 * SDK DeviceMappings. SDK put remains overall-only; mapping.put populates that mapping
 * and the overall map. Names are trimmed, case sensitive, and must be unique, including
 * repeated registration of the same object. Different names may alias one physical device.
 * get/getMapping fail informatively, while tryGet and serial lookup preserve SDK optional
 * null semantics. getAll/overall iteration deduplicate physical identities; mapping values
 * retain aliases. Snapshots and lookup histories support assertions. The strict map rejects
 * SDK multi-driver names, replacement, and unregistered mapping.putLocal.
 *
 * <p>advance(seconds) integrates each registered FakeSteppable identity once, including
 * SDK DcMotorImplEx wrappers backed by FakeDcMotorController. Controllers themselves
 * are not plants. Registering only a controller does not advance unattached motor ports.
 * setBatteryVoltage explicitly updates currently registered motor plants and fake voltage
 * sensors; newly registered devices retain their own defaults until the next stimulus.
 * Motor plants integrate first-order shaft velocity exactly for a fixed open-loop command,
 * with encoder direction/reset, target tolerance, BRAKE/FLOAT, load/stall/fault, current,
 * angular units, configuration snapshots, and write/call histories. Configured target
 * tolerance takes effect on the next target write, matching the SDK wrapper. PID gains are retained,
 * not simulated as REV firmware. This is not a coupled chassis, trajectory or electrical
 * model: fixtures must explicitly feed localization/IMU/color/distance as the robot moves.
 *
 * <p>Use {@link org.firstinspires.ftc.teamcode.testing.OpModeRunner} for deterministic
 * iterative/linear lifecycle simulation and {@link org.firstinspires.ftc.teamcode.testing.RecordingTelemetry}
 * for Driver Station recording. Inject its FakeClock into robot/controllers and advance
 * this map in its beforeTick callback; advance never changes the caller's clock.
 * Raw System.nanoTime, ElapsedTime, Thread.sleep, gamepad effects and arbitrary busy loops
 * are outside deterministic stepping. Native vendor drivers need explicit boundary adapters.
 *
 * <p>Sensor setters supply instantaneous readings. There is no automatic time integration,
 * noise, calibration, servo travel, coupled electrical load, I2C transport, or vendor-specific
 * concrete sensor emulation. Sensor and servo fakes do not implement FakeSteppable.
 *
 * <p>Defaults are 12 V supply, 0 V analog input (3.3 V maximum), high digital input,
 * zero distance/color/orientation/rates, and servo controller positions of 0.5
 * (zero CR-servo power). A distance out-of-range reading requires explicit selection.
 * Color channels use an explicit eight-bit raw model; normalized gain clips to [0,1]
 * and does not change raw readings. Returned normalized readings support toColor()
 * without Android graphics code. I2C addresses must be explicitly supplied.
 *
 * <p>FakeAnalogInput extends the SDK's concrete AnalogInput through a one-port controller.
 * That controller deliberately rejects other ports and physical serial-number queries.
 * FakeServoController implements ServoControllerEx for SDK ServoImpl, ServoImplEx and
 * CRServoImpl control APIs; SDK wrapper localized device names still require Android
 * context. Controller histories retain physical commands, while FakeServo/FakeCRServo
 * histories retain clipped logical commands. Changing direction/range reinterprets an
 * existing command, as in the SDK, without issuing a new position write.
 *
 * <p>FakeIMU.setOrientation takes intrinsic FTC Z-X-Y yaw/pitch/roll in robot coordinates;
 * setAngularVelocity takes unnormalized robot X/Y/Z rates. Both setters require explicit
 * units and acquisition timestamps. initialize stores mounting parameters but does not
 * transform already robot-centric stimuli. resetYaw preserves pitch, roll and rates,
 * and its offset survives an OpMode configuration reset. Euler/quaternion results are
 * consistent with yaw reset and returned as independent objects.
 *
 * <p>Writes have immutable snapshot histories/counts where useful; configuration resets
 * preserve histories and externally supplied stimuli. Direct fake operations reject
 * nonfinite inputs and closed devices. SDK wrappers retain their own input semantics
 * (for example clipping infinity before it reaches a controller). A servo close does
 * not close its shared controller. Concrete vendor devices (RevColorSensorV3, Limelight,
 * GoBildaPrism, Lynx hubs and cameras) are not impersonated: configure explicit fixtures
 * or existing mechanism stub factories, rather than requesting those classes from the map.
 * A successful JVM test is not robot safety validation; verify motor direction, encoder
 * scale/reset, target stopping, PWM range, sensor axes and stop behavior on the robot.
 */
package org.firstinspires.ftc.teamcode.fakeHw;
