package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        OTOSConstants.useCorrectedOTOSClass = true;
        OTOSConstants.hardwareMapName = "otos";
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(0.0, (2.0/25.4), (3*Math.PI/2));
        OTOSConstants.linearScalar = 1.0;
        OTOSConstants.angularScalar = 1.0;
        /*
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(0.25, 0.1875, 3*Math.PI/2);
        OTOSConstants.linearScalar = 0.97348;
        OTOSConstants.angularScalar = 0.934383;
        */
    }
}




