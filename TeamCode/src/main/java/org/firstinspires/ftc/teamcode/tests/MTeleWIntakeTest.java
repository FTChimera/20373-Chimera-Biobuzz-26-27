/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.Button;
import org.firstinspires.ftc.teamcode.library.hardware.gamepad.GamepadEx;

/*
 * This file includes a teleop (driver-controlled) file for a
 * Mecanum drivetrain. This teleop has field centric and robot centric control
 */

@Utility(name = "M Tele W Intake Test")
public class MTeleWIntakeTest extends OpMode {

    /*
     * Enum to represent the different control types. This is used to switch between field centric and robot centric control, or tests
     * We put a method to easily switch between the different control types.
     */

    enum Mode {
        ROBOT_CENTRIC_TELEOP,
        FIELD_CENTRIC_TELEOP,
        MOTOR_DIRECTION_TEST;

        public Mode next(int amt) {
            int returnIndex = (this.ordinal() + amt) % values().length;
            while (returnIndex < 0) {
                returnIndex += values().length;
            }
            return values()[returnIndex];
        }
    }

    // Declare OpMode members.
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private DcMotor intake = null;
    private Intake.INTAKE_PARAMS intakeParams = null;
    private IMU imu = null;
    private GamepadEx driver;
    Mode teleopMode = Mode.ROBOT_CENTRIC_TELEOP;
    double[] motorPowers = new double[4];

    /* Constants */
    private final double DRIVE_POWER_SCALAR = Constants.currentBot.MAX_POWER; // Scalar for drive power to prevent overpowering the motors, and easier driving.
    private final double MOTOR_DIRECTION_POWER = 0.3; // power for motor direction test mode to see clearly if the motors are going in the right direction or not.
    private final double STRAFE_POWER_COMPENSATION = 1.1; // Compensation for imperfect strafing on most mecanum drives. This is multiplied by the strafing power to make the robot strafe more effectively.
    private final double JOYSTICK_DEADZONE = 0.05; // Deadzone for joysticks to prevent drift. Inputs within the deadzone are set to 0.
    private final double JOYSTICK_CURVE_EXPONENT = 1.5; // Exponent for joystick curve. This makes the controls less sensitive at low inputs for finer control, while still allowing full power at max input.
    private final double TRIGGER_THRESHOLD = 0.4; // Threshold for triggers to prevent accidental activation. Inputs below the threshold are set to 0, and inputs above the threshold are set to 1 to make it act like a button.
    /*
     * Code to run ONCE when the driver hits INIT
     * This initializes hardware
     */
    @Override
    public void init() {

        /*
         * Initialize the drive motor variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFront = hardwareMap.get(DcMotor.class, Constants.currentBot.FRONT_LEFT_MOTOR_NAME);
        rightFront = hardwareMap.get(DcMotor.class, Constants.currentBot.FRONT_RIGHT_MOTOR_NAME);
        leftBack = hardwareMap.get(DcMotor.class, Constants.currentBot.BACK_LEFT_MOTOR_NAME);
        rightBack = hardwareMap.get(DcMotor.class, Constants.currentBot.BACK_RIGHT_MOTOR_NAME);
        /*
         * To drive forward, most robots need the motors on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFront.setDirection(Constants.currentBot.FRONT_LEFT_MOTOR_DIRECTION.toFtcSdk());
        rightFront.setDirection(Constants.currentBot.FRONT_RIGHT_MOTOR_DIRECTION.toFtcSdk());
        leftBack.setDirection(Constants.currentBot.BACK_LEFT_MOTOR_DIRECTION.toFtcSdk());
        rightBack.setDirection(Constants.currentBot.BACK_RIGHT_MOTOR_DIRECTION.toFtcSdk());

        imu = Constants.createIMU(hardwareMap);
        imu.resetYaw();
        driver = new GamepadEx(gamepad1, 0.0, x -> x, TRIGGER_THRESHOLD);

        // Initialize intake motor directly from currentBot mechanism params (no full Mechanism)
        intakeParams = Constants.currentBot.getMechanismParams(Intake.class, new Intake.INTAKE_PARAMS());
        if (intakeParams != null && intakeParams.ISBEINGUSED && intakeParams.HARDWAREMAP_NAME != null && !intakeParams.HARDWAREMAP_NAME.isEmpty()) {
            intake = hardwareMap.get(DcMotor.class, intakeParams.HARDWAREMAP_NAME);
            intake.setDirection(intakeParams.INTAKE_MOTOR_DIRECTION.toFtcSdk());
        }

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized Hardware Components");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     * This runs the selection process for field centric vs robot centric control.
     */
    @Override
    public void init_loop() {
        driver.update();
        /*
         * Here we use the dpad Left/Right to switch between teleop modes.
         * The selection is shown on the driver station via telemetry.
         */
        if (driver.wasJustPressed(Button.DPAD_LEFT)) {
            teleopMode = teleopMode.next(1);
        }
        if (driver.wasJustPressed(Button.DPAD_RIGHT)) {
            teleopMode = teleopMode.next(-1);
        }

        telemetry.addData("TeleOp Mode", teleopMode);
        telemetry.addData("Instructions", "Press DPAD Left/Right to switch modes");
        telemetry.addData("IMU Yaw", "(%.3f) degrees", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     * Right now, this method is blank.
     */
    @Override
    public void start() {
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     * Here we call our main teleop code to control the robot. We also show telemetry data on the driver station.
     */
    @Override
    public void loop() {
        driver.update();

        /*
         * Here, lets add an IMU reset option
         * This is useful in field centric, to reset the heading so that the robot has a more accurate heading reference.
         * It is a good idea to keep it as a global method in loop() and not just for field centric
         * Here we use Back to reset yaw. This is also the same as share in PS4/PS5 controllers.
         */
        if (driver.wasJustPressed(Button.BACK)) {
            imu.resetYaw();
        }

        /*
         * Here we use the left stick for translation and the right stick for rotation. The controls are squared to allow for finer control at low speeds.
         * We also negate the inputs because by default, pushing the stick forward gives a negative value, and we want that to correspond to positive power.
         * The method of passing in the inputs depends on the teleop mode, which is selected in the init_loop method.
         * We keep these global variables through the method for telemetry
         */

        double forward=0, strafe=0, rotate=0;
        double leftStickX, leftStickY, rightStickX; // We declare these here to avoid redeclaring them in each case of the switch statement, since they are used in multiple cases.

        /*
         * Here we implement the code for each teleop mode to set drive powers
         */
        switch (teleopMode) {
            case ROBOT_CENTRIC_TELEOP:
                // Implement Joystick deadzone and curve
                leftStickX = Math.abs(driver.leftStick.getRawX()) > JOYSTICK_DEADZONE ? driver.leftStick.getRawX() : 0;
                leftStickY = Math.abs(-driver.leftStick.getRawY()) > JOYSTICK_DEADZONE ? -driver.leftStick.getRawY() : 0;
                rightStickX = Math.abs(driver.rightStick.getRawX()) > JOYSTICK_DEADZONE ? driver.rightStick.getRawX() : 0;
                leftStickX = Math.copySign(Math.pow(Math.abs(leftStickX), JOYSTICK_CURVE_EXPONENT), leftStickX);
                leftStickY = Math.copySign(Math.pow(Math.abs(leftStickY), JOYSTICK_CURVE_EXPONENT), leftStickY);
                rightStickX = Math.copySign(Math.pow(Math.abs(rightStickX), JOYSTICK_CURVE_EXPONENT), rightStickX);
                forward = -leftStickY;
                strafe = leftStickX * STRAFE_POWER_COMPENSATION;
                rotate = rightStickX;
                break;
            case FIELD_CENTRIC_TELEOP:
                // Implement Joystick deadzone and curve
                leftStickX = Math.abs(driver.leftStick.getRawX()) > JOYSTICK_DEADZONE ? driver.leftStick.getRawX() : 0;
                leftStickY = Math.abs(-driver.leftStick.getRawY()) > JOYSTICK_DEADZONE ? -driver.leftStick.getRawY() : 0;
                rightStickX = Math.abs(driver.rightStick.getRawX()) > JOYSTICK_DEADZONE ? driver.rightStick.getRawX() : 0;
                leftStickX = Math.copySign(Math.pow(Math.abs(leftStickX), JOYSTICK_CURVE_EXPONENT), leftStickX);
                leftStickY = Math.copySign(Math.pow(Math.abs(leftStickY), JOYSTICK_CURVE_EXPONENT), leftStickY);
                rightStickX = Math.copySign(Math.pow(Math.abs(rightStickX), JOYSTICK_CURVE_EXPONENT), rightStickX);
                forward = -leftStickY;
                strafe = leftStickX;
                rotate = rightStickX;
                // For field centric, we need to rotate the inputs by the direction of the robot to turn them into robot centric inputs
                double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double cosA = Math.cos(-robotHeading);
                double sinA = Math.sin(-robotHeading);
                double tempForward = forward * cosA + strafe * sinA;
                strafe = -forward * sinA + strafe * cosA;
                strafe *= STRAFE_POWER_COMPENSATION;
                forward = tempForward;
                break;
            case MOTOR_DIRECTION_TEST:
                // In this mode, the motors move forward based on gamepad right trigger. This allows you to easily see if any motors are reversed or not, and adjust the motor directions accordingly.
                forward = (driver.rightTrigger.isPressed() ? 1 : 0) * MOTOR_DIRECTION_POWER;
                strafe = 0;
                rotate = 0;
                if (driver.wasJustPressed(Button.A)) {
                    leftFront.setDirection(leftFront.getDirection().inverted());
                }
                if (driver.wasJustPressed(Button.B)) {
                    rightFront.setDirection(rightFront.getDirection().inverted());
                }
                if (driver.wasJustPressed(Button.X)) {
                    leftBack.setDirection(leftBack.getDirection().inverted());
                }
                if (driver.wasJustPressed(Button.Y)) {
                    rightBack.setDirection(rightBack.getDirection().inverted());
                }

                telemetry.addLine("Use A to invert left front motor direction");
                telemetry.addLine("Use B to invert right front motor direction");
                telemetry.addLine("Use X to invert left back motor direction");
                telemetry.addLine("Use Y to invert right back motor direction");
                telemetry.addData("Motor Directions", getMotorDirectionsString());
                break;
        }

        // Now, calculate drive powers based on the forward, strafe, and rotate values, and set the motor powers.

        motorPowers[0] = (forward + strafe + rotate); // Front Left
        motorPowers[1] = (forward - strafe - rotate); // Front Right
        motorPowers[2] = (forward - strafe + rotate); // Back Left
        motorPowers[3] = (forward + strafe - rotate); // Back Right
        double max = Math.max(
                1,
                Math.max(
                        Math.abs(motorPowers[0]),
                        Math.max(
                                Math.abs(motorPowers[1]),
                                Math.max(Math.abs(motorPowers[2]), Math.abs(motorPowers[3]))
                        )
                )
        );

        for (int i = 0; i < 4; i++) {
            motorPowers[i] /= max;
            motorPowers[i] *= DRIVE_POWER_SCALAR;
        }
        // Now, set powers to motors
        leftFront.setPower(motorPowers[0]);
        rightFront.setPower(motorPowers[1]);
        leftBack.setPower(motorPowers[2]);
        rightBack.setPower(motorPowers[3]);
        /*
         * Here we use the dpad Left/Right to switch between teleop modes.
         * The selection is shown on the driver station via telemetry.
         */
        if (driver.wasJustPressed(Button.DPAD_LEFT)) {
            teleopMode = teleopMode.next(1);
        }
        if (driver.wasJustPressed(Button.DPAD_RIGHT)) {
            teleopMode = teleopMode.next(-1);
        }

        // Intake control (direct DcMotor using currentBot params)
        double intakePower = 0;
        if (driver.rightTrigger.isPressed()) {
            intakePower = intakeParams != null ? intakeParams.FULL_POWER : 1.0;
        } else if (driver.leftTrigger.isPressed()) {
            intakePower = intakeParams != null ? intakeParams.REVERSE_FULL_POWER : -1.0;
        } else {
            intakePower = intakeParams != null ? intakeParams.STOP_POWER : 0.0;
        }
        if (intake != null) {
            intake.setPower(intakePower);
        }

        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        telemetry.addData("TeleOp Mode", teleopMode);
        telemetry.addData("Instructions", "Press DPAD Left/Right to switch modes");
        telemetry.addData("Drive Power", "forward (%.2f), strafe (%.2f), rotate (%.2f)", forward, strafe, rotate);
        telemetry.addData("IMU Yaw", "(%.3f) degrees", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addLine("Use Back/Share to reset IMU Yaw");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     * Here we stop the motors to prevent the robot from moving after teleop ends.
     */
    @Override
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        if (intake != null) intake.setPower(0);
    }

    String getMotorDirectionsString() {
        return getDirString(0) + " " + getDirString(1) + " " + getDirString(2) + " " + getDirString(3);
    }

    String getDirString(int idx) {
        DcMotor motor;
        switch (idx) {
            case 0: motor = leftFront; break;
            case 1: motor = rightFront; break;
            case 2: motor = leftBack; break;
            default: motor = rightBack;
        }
        return motor.getDirection() == DcMotorSimple.Direction.FORWARD ? "F" : "R";}
}