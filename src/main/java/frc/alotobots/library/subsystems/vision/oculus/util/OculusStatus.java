package frc.alotobots.library.subsystems.vision.oculus.util;

import lombok.experimental.UtilityClass;

@UtilityClass
public final class OculusStatus {
    /** Status indicating system is ready for commands */
    public static final int STATUS_READY = 0;

    /** Status indicating heading reset completion */
    public static final int STATUS_HEADING_RESET_COMPLETE = 99;

    /** Status indicating pose reset completion */
    public static final int STATUS_POSE_RESET_COMPLETE = 98;

    /** Status indicating ping response receipt */
    public static final int STATUS_PING_RESPONSE = 97;
}
