(function (root) {
    const defaultSettings = Object.freeze({
        packageName: "org.firstinspires.ftc.teamcode.pedroPathing",
        constantsPackage: "org.firstinspires.ftc.teamcode",
        className: "Autonomous",
        opModeName: "Auto",
        opModeGroup: "Pedro",
        useIvyCommandBase: true
    });

    function normalizeText(value, fallback) {
        return typeof value === "string" && value.trim() ? value.trim() : fallback;
    }

    function normalizeSettings(settings = {}) {
        const packageName = normalizeText(settings.packageName, defaultSettings.packageName);

        return {
            packageName,
            constantsPackage: normalizeText(settings.constantsPackage, defaultSettings.constantsPackage),
            className: normalizeText(settings.className, defaultSettings.className),
            opModeName: normalizeText(settings.opModeName, defaultSettings.opModeName),
            opModeGroup: normalizeText(settings.opModeGroup, defaultSettings.opModeGroup),
            useIvyCommandBase: settings.useIvyCommandBase === undefined
                ? defaultSettings.useIvyCommandBase
                : Boolean(settings.useIvyCommandBase)
        };
    }

    function deg(value) {
        return value !== undefined ? `Math.toRadians(${value})` : "0";
    }

    function sanitize(name) {
        const normalized = String(name ?? "")
            .toLowerCase()
            .replace(/[^a-z0-9]/g, "_")
            .replace(/_+/g, "_")
            .replace(/^_+|_+$/g, "");

        return normalized || "item";
    }

    function buildConstantsImport(settings) {
        const packageName = settings.packageName;
        const constantsPackage = settings.constantsPackage || packageName;

        return constantsPackage === packageName ? "" : `\nimport ${constantsPackage}.Constants;`;
    }

    function normalizePathData(parsedData) {
        if (!parsedData || typeof parsedData !== "object") {
            throw new Error("The selected file does not contain a valid path object.");
        }

        if (!parsedData.startPoint || !Array.isArray(parsedData.lines)) {
            throw new Error("Expected a Pedro Pathing or Turtle Tracer export with startPoint and lines.");
        }

        return {
            ...parsedData,
            poses: Array.isArray(parsedData.poses) ? parsedData.poses : [],
            lines: parsedData.lines,
            sequence: Array.isArray(parsedData.sequence)
                ? parsedData.sequence
                : parsedData.lines
                    .filter(line => line && line.id)
                    .map(line => ({ kind: "path", lineId: line.id }))
        };
    }

    function parsePathFile(fileContent) {
        if (typeof fileContent !== "string") {
            throw new Error("Path file content must be a string.");
        }

        let parsedData;

        try {
            parsedData = JSON.parse(fileContent);
        } catch {
            throw new Error("The selected file is not valid JSON.");
        }

        return normalizePathData(parsedData);
    }

    function buildStandardJava(settings, constantsImport, poses, paths, build, enumDeclaration, follow, initialState) {
        return `
package ${settings.packageName};

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;${constantsImport}

@Autonomous(name = "${settings.opModeName}", group = "${settings.opModeGroup}")
public class ${settings.className} extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private PathState pathState;

    public enum PathState {
        IDLE${enumDeclaration}
    }

${poses.join("\n\n")}

${paths.join("\n")}

    public void buildPaths() {
${build.join("\n\n")}
    }

    public void autonomousPathUpdate() {
        if (follower.isBusy()) return;

        switch (pathState) {
${follow.join("\n\n")}
        }
    }

    public void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        setPathState(PathState.${initialState});
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
`.trim();
    }

    function buildIvyStartBody(pathSequence) {
        if (pathSequence.length === 0) {
            return `        schedule(
            infinite(follower::update)
        );`;
        }

        const followCommands = pathSequence
            .map(pathName => `                follow(${pathName})`)
            .join(",\n");

        return `        schedule(
            infinite(follower::update),
            sequential(
${followCommands}
            )
        );`;
    }

    function buildIvyJava(settings, constantsImport, poses, paths, build, pathSequence) {
        return `
package ${settings.packageName};

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;${constantsImport}

@Autonomous(name = "${settings.opModeName}", group = "${settings.opModeGroup}")
public class ${settings.className} extends OpMode {

    private Follower follower;

${poses.join("\n\n")}

${paths.join("\n")}

    public void buildPaths() {
${build.join("\n\n")}
    }

    @Override
    public void init() {
        reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
${buildIvyStartBody(pathSequence)}
    }

    @Override
    public void loop() {
        Scheduler.execute();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        reset();
    }

    public void schedule(Command... commands) {
        Scheduler.schedule(commands);
    }

    public Command follow(PathChain pathChain) {
        return PedroCommands.follow(follower, pathChain);
    }

    public void reset() {
        Scheduler.reset();
    }

    public Command infinite(Runnable run) {
        return Commands.infinite(run);
    }

    public Command sequential(Command... commands) {
        return Groups.sequential(commands);
    }
}
`.trim();
    }

    function buildPathSegment(lastPose, poseName, controlPointPoses) {
        if (controlPointPoses.length > 0) {
            const controlPointsStr = controlPointPoses.join(",\n                    ");

            return `new BezierCurve(
                    ${lastPose},
                    ${controlPointsStr},
                    ${poseName}
                )`;
        }

        return `new BezierLine(
                    ${lastPose},
                    ${poseName}
                )`;
    }

    function buildPoseDeclaration(name, x, y, heading) {
        return `    public static final Pose ${name} = new Pose(
        ${x},
        ${y},
        ${deg(heading)}
    );`;
    }

    function createUniqueName(baseName, existingNames) {
        let suffix = "";
        let counter = 1;

        while (existingNames.has(baseName + suffix)) {
            suffix = "_" + counter;
            counter++;
        }

        return baseName + suffix;
    }

    function generate(fileContent, settings = {}) {
        const data = typeof fileContent === "string" ? parsePathFile(fileContent) : normalizePathData(fileContent);
        const resolvedSettings = normalizeSettings(settings);

        const linesById = {};
        data.lines.forEach(line => {
            linesById[line.id] = line;
        });

        const poses = [];
        const paths = [];
        const build = [];
        const follow = [];
        const enumValues = [];
        const pathSequence = [];

        const createdPoses = new Map();
        const createdPaths = new Map();
        const poseRefsById = new Map();

        const firstPathStep = data.sequence.find(step => step.kind === "path");
        const firstLine = firstPathStep ? linesById[firstPathStep.lineId] : null;
        const startDegree = firstLine && firstLine.endPoint && firstLine.endPoint.startDeg !== undefined
            ? firstLine.endPoint.startDeg
            : data.startPoint.startDeg;

        const robotStartPose = {
            id: "start",
            x: data.startPoint.x,
            y: data.startPoint.y,
            heading: startDegree,
            varName: "startPose"
        };

        poses.push(buildPoseDeclaration(
            robotStartPose.varName,
            robotStartPose.x,
            robotStartPose.y,
            robotStartPose.heading
        ));

        poseRefsById.set(robotStartPose.id, robotStartPose);

        data.poses.forEach((pose, poseIndex) => {
            if (!pose || typeof pose !== "object") {
                return;
            }

            const poseId = pose.id || `pose-${poseIndex + 1}`;
            const poseName = createUniqueName(`${sanitize(pose.name || `pose_${poseIndex + 1}`)}Pose`, createdPoses);
            const poseInfo = {
                id: poseId,
                x: pose.x,
                y: pose.y,
                heading: pose.heading || 0,
                varName: poseName
            };

            createdPoses.set(poseInfo.varName, poseInfo);
            poseRefsById.set(poseInfo.id, poseInfo);
            poses.push(buildPoseDeclaration(poseInfo.varName, poseInfo.x, poseInfo.y, poseInfo.heading));
        });

        let lastPose = robotStartPose;

        data.sequence.forEach(step => {
            if (step.kind !== "path") {
                return;
            }

            const line = linesById[step.lineId];
            if (!line) {
                return;
            }

            const lineLabel = line.name || line.id || "Unnamed Path";
            const basePoseName = sanitize(line.name) + "Pose";
            const basePathName = sanitize(line.name) + "Path";
            const explicitStartPose = line.startPoseId ? poseRefsById.get(line.startPoseId) : null;
            const segmentStartPose = explicitStartPose || lastPose;

            if (line.startPoseId && !explicitStartPose) {
                throw new Error(`Path "${lineLabel}" references an unknown start pose.`);
            }

            let poseInfo;
            if (line.endPoseId) {
                poseInfo = poseRefsById.get(line.endPoseId);
                if (!poseInfo) {
                    throw new Error(`Path "${lineLabel}" references an unknown end pose.`);
                }
            } else {
                if (!line.endPoint) {
                    throw new Error(`Path "${lineLabel}" is missing an end point.`);
                }

                const existingPose = Array.from(createdPoses.values()).find(pose =>
                    pose.x === line.endPoint.x
                    && pose.y === line.endPoint.y
                    && pose.heading === (line.endPoint.endDeg || 0)
                );

                if (existingPose) {
                    poseInfo = existingPose;
                } else {
                    poseInfo = {
                        x: line.endPoint.x,
                        y: line.endPoint.y,
                        heading: line.endPoint.endDeg || 0,
                        varName: createUniqueName(basePoseName, createdPoses)
                    };

                    createdPoses.set(poseInfo.varName, poseInfo);
                    poses.push(buildPoseDeclaration(poseInfo.varName, poseInfo.x, poseInfo.y, poseInfo.heading));
                }
            }

            const poseName = poseInfo.varName;

            const controlPointPoses = [];
            if (line.controlPoints && line.controlPoints.length > 0) {
                line.controlPoints.forEach((controlPoint, controlPointIndex) => {
                    const controlPoseName = `${sanitize(line.name)}ControlPoint${controlPointIndex + 1}`;

                    const existingControlPose = Array.from(createdPoses.values()).find(pose =>
                        pose.x === controlPoint.x
                        && pose.y === controlPoint.y
                        && pose.heading === 0
                    );

                    if (existingControlPose) {
                        controlPointPoses.push(existingControlPose.varName);
                        return;
                    }

                    let suffix = "";
                    let counter = 1;
                    while (createdPoses.has(controlPoseName + suffix)) {
                        suffix = "_" + counter;
                        counter++;
                    }
                    const finalControlPoseName = controlPoseName + suffix;

                    createdPoses.set(finalControlPoseName, {
                        x: controlPoint.x,
                        y: controlPoint.y,
                        heading: 0,
                        varName: finalControlPoseName
                    });

                    poses.push(
                        buildPoseDeclaration(finalControlPoseName, controlPoint.x, controlPoint.y, 0)
                    );
                    controlPointPoses.push(finalControlPoseName);
                });
            }

            let pathName;
            const pathSignature = {
                fromPose: segmentStartPose.varName,
                toPose: poseName,
                controlPoints: controlPointPoses
            };

            const existingPath = Array.from(createdPaths.values()).find(path =>
                path.fromPose === segmentStartPose.varName
                && path.toPose === poseName
                && JSON.stringify(path.controlPoints) === JSON.stringify(pathSignature.controlPoints)
            );

            if (existingPath) {
                pathName = existingPath.varName;
            } else {
                pathName = createUniqueName(basePathName, createdPaths);

                createdPaths.set(pathName, {
                    ...pathSignature,
                    varName: pathName
                });

                paths.push(`    public PathChain ${pathName};`);

                build.push(
`        ${pathName} = follower.pathBuilder()
            .addPath(
                ${buildPathSegment(segmentStartPose.varName, poseName, controlPointPoses)}
            )
            .setLinearHeadingInterpolation(
                ${segmentStartPose.varName}.getHeading(),
                ${poseName}.getHeading()
            )
            .build();`
                );
            }

            const enumName = sanitize(line.name).toUpperCase();
            let finalEnumName = enumName;
            let counter = 1;
            while (enumValues.includes(finalEnumName)) {
                finalEnumName = `${enumName}_${counter}`;
                counter++;
            }
            enumValues.push(finalEnumName);

            follow.push(
`        case ${finalEnumName}:
            follower.followPath(${pathName});
            setPathState(PathState.IDLE);
            break;`
            );

            pathSequence.push(pathName);
            lastPose = poseInfo;
        });

        for (let index = 0; index < follow.length; index++) {
            const nextState = index < follow.length - 1 ? `PathState.${enumValues[index + 1]}` : "PathState.IDLE";
            follow[index] = follow[index].replace("setPathState(PathState.IDLE)", `setPathState(${nextState})`);
        }

        let enumDeclaration = "";
        if (enumValues.length > 0) {
            enumDeclaration = `,
        ${enumValues.join(",\n        ")}`;
        }

        const initialState = enumValues.length > 0 ? enumValues[0] : "IDLE";
        const constantsImport = buildConstantsImport(resolvedSettings);

        return resolvedSettings.useIvyCommandBase
            ? buildIvyJava(resolvedSettings, constantsImport, poses, paths, build, pathSequence)
            : buildStandardJava(resolvedSettings, constantsImport, poses, paths, build, enumDeclaration, follow, initialState);
    }

    const api = Object.freeze({
        defaultSettings,
        normalizeSettings,
        normalizePathData,
        parsePathFile,
        generate
    });

    if (typeof module !== "undefined" && module.exports) {
        module.exports = api;
    }

    if (root) {
        root.PedroPathingJavaGenerator = api;
    }
})(typeof globalThis !== "undefined" ? globalThis : this);