// Docs: https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html
import static org.junit.Assert.*;
import org.junit.*;

import frc.robot.Vector;

public class TestVector {

    @Before // this method will run before each test
    public void setup() {
  
    }

    @After // this method will run after each test
    public void shutdown() throws Exception {

    }

    @Test // marks this method as a test
    public void noError() {

        for(double dir = -5*Math.PI; dir < 5*Math.PI; dir += Math.PI/6) {
            var vec = new Vector(dir, 5);
            System.out.printf("Mag: %f  Dir: %f pi/6\n", vec.getMagnitude(), vec.getDirection()/(Math.PI/6));
        }

    }

}
