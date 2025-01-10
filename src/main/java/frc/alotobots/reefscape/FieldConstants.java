package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.UtilityClass;
import java.util.HashMap;
import java.util.Map;

@UtilityClass
public class FieldConstants {
  // Reef branches (counterclockwise A->L)
  public enum ReefBranch {
    A, B, C, D, E, F, G, H, I, J, K, L
  }

  // Algae positions between reef branches
  public enum ReefAlgaePosition {
    AB, CD, EF, GH, IJ, KL
  }

  public enum GamePieceFloorPosition {
    // Blue alliance side CORAL floor positions (looking from center to blue alliance)
    CORAL_LEFT, CORAL_MIDDLE, CORAL_RIGHT,

    // Blue alliance side ALGAE floor positions (looking from center to blue alliance)
    ALGAE_LEFT, ALGAE_MIDDLE, ALGAE_RIGHT
  }

  // Branch levels
  public enum Level {
    L2,   // 2'7⅞" height, 35° angle
    L3,   // 3'11⅝" height, 35° angle
    L4    // 6' height, vertical
  }

  @RequiredArgsConstructor
  @Getter
  private static class FieldPose {
    private final Pose3d bluePose;

    public Pose3d getRed() {
      Translation3d redTranslation = new Translation3d(
              -bluePose.getX(),
              bluePose.getY(),
              bluePose.getZ()
      );

      Rotation3d blueRot = bluePose.getRotation();
      Rotation3d redRotation = new Rotation3d(
              blueRot.getX(),
              blueRot.getY(),
              Math.PI - blueRot.getZ()
      );

      return new Pose3d(redTranslation, redRotation);
    }
  }

  // Maps to store positions
  private final Map<String, FieldPose> branchPoses = new HashMap<>();
  private final Map<String, FieldPose> reefAlgaePoses = new HashMap<>();
  private final Map<GamePieceFloorPosition, FieldPose> floorPoses = new HashMap<>();

  private String makeBranchKey(ReefBranch branch, Level level) {
    return branch.toString() + "_" + level.toString();
  }

  static {
    // Initialize all positions
    // Branches (Edge)
    // L2
    addBranchPose(ReefBranch.A, Level.L2, new Pose3d(3.71, 4.19, .788, new Rotation3d()));
    addBranchPose(ReefBranch.B, Level.L2, new Pose3d(3.71, 3.862, .788, new Rotation3d()));
    // addBranchPose(ReefBranch.A, Level.L2, BRANCH_A_L2_BLUE);

    // Reef algae
    // addReefAlgaePose(ReefAlgaePosition.AB, REEF_ALGAE_AB_BLUE);

    // Floor pieces
    // Algae
    addFloorPose(GamePieceFloorPosition.ALGAE_LEFT, new Pose3d(1.227, 5.937, .483, Rotation3d.kZero));
    addFloorPose(GamePieceFloorPosition.ALGAE_MIDDLE,new Pose3d(1.227, 4.026, .483, Rotation3d.kZero));
    addFloorPose(GamePieceFloorPosition.ALGAE_RIGHT, new Pose3d(1.227, 2.197, .483, Rotation3d.kZero));

    // Coral
    addFloorPose(GamePieceFloorPosition.CORAL_LEFT, new Pose3d(1.227, 5.937, 0, Rotation3d.kZero));
    addFloorPose(GamePieceFloorPosition.CORAL_MIDDLE,new Pose3d(1.227, 4.026, 0, Rotation3d.kZero));
    addFloorPose(GamePieceFloorPosition.CORAL_RIGHT, new Pose3d(1.227, 2.197, 0, Rotation3d.kZero));
  }

  /**
   * Get the blue alliance pose for a branch at a specific level
   */
  public Pose3d getBranchBlue(ReefBranch branch, Level level) {
    FieldPose pos = branchPoses.get(makeBranchKey(branch, level));
    if (pos == null) {
      throw new IllegalArgumentException(
              String.format("No position found for branch %s at level %s", branch, level)
      );
    }
    return pos.getBluePose();
  }

  /**
   * Get the red alliance pose for a branch at a specific level
   */
  public Pose3d getBranchRed(ReefBranch branch, Level level) {
    return branchPoses.get(makeBranchKey(branch, level)).getRed();
  }

  /**
   * Get the blue alliance pose for an algae position between reef branches
   */
  public Pose3d getReefAlgaeBlue(ReefAlgaePosition position) {
    FieldPose pos = reefAlgaePoses.get(position.toString());
    if (pos == null) {
      throw new IllegalArgumentException(
              String.format("No position found for reef algae position %s", position)
      );
    }
    return pos.getBluePose();
  }

  /**
   * Get the red alliance pose for an algae position between reef branches
   */
  public Pose3d getReefAlgaeRed(ReefAlgaePosition position) {
    return reefAlgaePoses.get(position.toString()).getRed();
  }

  /**
   * Get the blue alliance pose for a game piece floor position
   */
  public Pose3d getFloorBlue(GamePieceFloorPosition position) {
    FieldPose pos = floorPoses.get(position);
    if (pos == null) {
      throw new IllegalArgumentException(
              String.format("No position found for floor position %s", position)
      );
    }
    return pos.getBluePose();
  }

  /**
   * Get the red alliance pose for a game piece floor position
   */
  public Pose3d getFloorRed(GamePieceFloorPosition position) {
    return floorPoses.get(position).getRed();
  }

  private void addBranchPose(ReefBranch branch, Level level, Pose3d bluePose) {
    branchPoses.put(makeBranchKey(branch, level), new FieldPose(bluePose));
  }

  private void addReefAlgaePose(ReefAlgaePosition position, Pose3d bluePose) {
    reefAlgaePoses.put(position.toString(), new FieldPose(bluePose));
  }

  private void addFloorPose(GamePieceFloorPosition position, Pose3d bluePose) {
    floorPoses.put(position, new FieldPose(bluePose));
  }
}