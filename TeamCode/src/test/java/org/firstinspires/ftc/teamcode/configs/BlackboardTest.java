package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.library.math.MathUtils;
import org.firstinspires.ftc.teamcode.library.robot.Driver;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThrows;

public class BlackboardTest {
    @Before
    public void configureInitialValues() {
        Blackboard.resetAll();
        Blackboard.put(Blackboard.Caption.POSE, new Pose());
        Blackboard.put(Blackboard.Caption.DRIVER, Driver.DEFAULT);
    }

    @After
    public void resetBlackboard() {
        Blackboard.resetAll();
    }

    @Test
    public void storesUpdatesAndResetsConfiguredItems() {
        Pose pose = new Pose(1.0, 2.0, 3.0);
        Blackboard.put(Blackboard.Caption.POSE, pose);
        Blackboard.put(Blackboard.Caption.DRIVER, Driver.ATHARVA);

        assertEquals(pose, Blackboard.get(Blackboard.Caption.POSE).getValue());
        assertEquals(Driver.ATHARVA, Blackboard.get(Blackboard.Caption.DRIVER).getValue());

        Blackboard.resetAll();

        Pose resetPose = Blackboard.get(Blackboard.Caption.POSE).getValue();
        assertNotNull(resetPose);
        assertEquals(0.0, resetPose.getX(), MathUtils.EPSILON);
        assertEquals(0.0, resetPose.getY(), MathUtils.EPSILON);
        assertEquals(0.0, resetPose.getHeading(), MathUtils.EPSILON);
        assertEquals(Driver.DEFAULT, Blackboard.get(Blackboard.Caption.DRIVER).getValue());
    }

    @Test
    public void rejectsValuesWhoseTypesDoNotMatchTheirCaption() {
        assertThrows(IllegalArgumentException.class, () ->
                Blackboard.put(Blackboard.Caption.DRIVER, new Pose(0.0, 0.0, 0.0)));
        assertThrows(IllegalArgumentException.class, () ->
                new Blackboard.Item(Blackboard.Caption.POSE, Driver.DEFAULT));
    }
}
