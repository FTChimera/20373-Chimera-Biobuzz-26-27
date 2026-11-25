## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## BIOBUZZ ftcsim component diagnostics

`robotDev/sim/` contains six independent, numbered diagnostics. They are the
default `TeamCodeSim` menu; competition OpModes are not run automatically.
With ftcsim running, connect using `./gradlew runSimulator --console=plain`,
then use `http://localhost:9015/ds/` to INIT and START one diagnostic at a time.
In ftcsim, apply the team's hardware export with **Apply to SDK**, and apply
mechanisms with **Update simulation**, while connected and no OpMode initialized.
These tests do not supply or guess hub ports or wheelbase measurements.

| Menu / class | Measurements and local diagnostic criteria |
| --- | --- |
| #1 `SimPathPinpointTest` | RED garden start (9,25,0 radians), one line to shared garden shoot pose. PASS requires full route completion, endpoint error <=2 inches and <=8 degrees, raw Pinpoint and follower translation >=0.5 inches, and wheel encoder movement >=12 ticks. Timeout 20 seconds. |
| #2 `SimSotmAimTest` | Two 3-second drive-and-Y-aim phases, SOTM OFF then ON; 0.15 forward power is a command, not a measured speed. Logs both wheel targets/velocities, heading error, and actual feed-gate decisions. PASS in each phase requires measured movement, moving heading error <=3 degrees, simultaneous auto-fire gates, and a 0.3-second dual-wheel settle. |
| #3 `SimShooterStepTest` | Both real SmartFlywheel controllers, current positive shot-table targets, from verified rest. Logs first entry and sustained 0.3-second settle inside 40 ticks/second; timeout 8 seconds. About 1.5 seconds is a prior, not a robot specification. |
| #4 `SimHoldJitterTest` | SOTM ON, neutral Y-aim, no feed. After bounded heading acquisition, records a full 5-second sampled window, circular peak-to-peak and maximum heading error. PASS requires every error <=3 degrees, drift <=0.5 inches and speed <=0.5 inches/second. |
| #5 `SimAStarParkingTest` | Production 6-inch-cell A* route from RED garden, exact followed waypoints in telemetry. PASS requires full route completion, live Pinpoint/encoder movement and measured robot-body overlap with the park zone; timeout 40 seconds. |
| #6 `SimIntakeTransferTest` | 5.5-second STOP/STOW, deploy-only, intake, intake-plus-transfer, STOP/STOW sequence. Compares observed applied powers and SDK servo command with expected values (tolerance 0.005), requires sampled phase coverage and measured motion from both motors. No follower required. |

**Run #1 first.** If wheels move but Pinpoint remains fixed for 2 seconds,
the test stops and reports `FAIL: Pinpoint dead; stop other path diagnostics`.
Do not interpret #2, #4, or #5 as valid localization/aiming evidence until #1
passes. Wheel encoders prove wheel motion, not independent chassis ground truth.
ftcsim sees no AprilTags or game pieces; feed gates are not proof of scoring,
and a servo's SDK position is not physical ramp feedback.

All diagnostics stop their commanded outputs on completion or STOP. Final
numbers remain visible until STOP; SOTM and modified follower settings are
restored. Wheel/rest/acquisition dwell restarts across sampling gaps >0.1 seconds.
Jitter and sequencing reject incomplete sampling coverage. Diagnostic thresholds
stay local to `robotDev`, not in a production `Constants.robotDev` section.

To register only one diagnostic, for example:

```zsh
./gradlew runSimulator --console=plain \
  -PsimOpmodes=org.firstinspires.ftc.teamcode.robotDev.sim.SimPathPinpointTest
```

Pure diagnostic checks are registered sim-first in `UnitTestAll`:
`./gradlew :TeamCode:testDebugUnitTest --tests org.firstinspires.ftc.teamcode.UnitTestAll --console=plain`.
Actual INIT-to-completion behavior still needs the owner's configured ftcsim
run. Before any on-robot run, use an empty shooter, clear moving mechanisms and
travel area, physically place the robot at the displayed RED start pose, and
keep Driver Station STOP accessible. Confirm motor directions, real timing and
odometry with measurements; simulator results are only priors.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""