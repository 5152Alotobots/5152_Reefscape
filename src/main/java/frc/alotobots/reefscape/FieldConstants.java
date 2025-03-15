/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2025 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.reefscape;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.UtilityClass;

/**
 * Constants class containing field-specific positions and measurements for the game field. This
 * includes positions for reef branches, algae positions, and game piece floor positions.
 */
@UtilityClass
public class FieldConstants {
  /** The length of the field in meters */
  public static final double FIELD_LENGTH = 17.548; // Meters

  /** The width of the field in meters */
  public static final double FIELD_WIDTH = 8.052;

  /** Enum representing the available coral stations. Used primarily for pathfinding */
  public enum CoralStationSide {
    LEFT,
    RIGHT
  }

  /** Enum representing the available position in coral stations. Used primarily for pathfinding */
  public enum CoralStationPickupPosition {
    P1,
    P2,
    P3
  }

  /**
   * Enum representing the available reef branches on the field, labeled A through L in
   * counterclockwise order.
   */
  public enum ReefBranch {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L
  }

  /** Enum representing the algae positions between reef branches. */
  public enum ReefAlgaePosition {
    AB, // Between branches A and B
    CD, // Between branches C and D
    EF, // Between branches E and F
    GH, // Between branches G and H
    IJ, // Between branches I and J
    KL // Between branches K and L
  }

  /**
   * Enum representing the possible floor positions for game pieces. Includes positions for both
   * coral and algae on the blue alliance side.
   */
  public enum GamePieceFloorPosition {
    // Blue alliance side CORAL floor positions (looking from center to blue alliance)
    CORAL_LEFT,
    CORAL_MIDDLE,
    CORAL_RIGHT,

    // Blue alliance side ALGAE floor positions (looking from center to blue alliance)
    ALGAE_LEFT,
    ALGAE_MIDDLE,
    ALGAE_RIGHT
  }

  /** Enum representing the different height levels for branches. */
  public enum Level {
    L2,
    L3,
    L4
  }

  /** Internal class representing a field position that can be mirrored between alliances. */
  @RequiredArgsConstructor
  @Getter
  private static class FieldPose {
    /** The pose for the blue alliance side */
    private final Pose3d bluePose;

    /**
     * Gets the equivalent pose for the red alliance side.
     *
     * @return The mirrored Pose3d for the red alliance
     */
    public Pose3d getRed() {
      // Convert Pose3d to Pose2d for flipping
      Pose2d pose2d =
          new Pose2d(
              bluePose.getX(), bluePose.getY(), new Rotation2d(bluePose.getRotation().getZ()));

      // Use PathPlanner's flip utility
      Pose2d flippedPose2d = FlippingUtil.flipFieldPose(pose2d);

      // Create new Pose3d with flipped X and rotation, keeping Z coordinate and X/Y rotations
      return new Pose3d(
          flippedPose2d.getX(),
          flippedPose2d.getY(),
          bluePose.getZ(),
          new Rotation3d(
              bluePose.getRotation().getX(),
              bluePose.getRotation().getY(),
              flippedPose2d.getRotation().getRadians()));
    }
  }

  /** Map storing branch positions keyed by branch and level */
  private final Map<String, FieldPose> branchPoses = new HashMap<>();

  /** Map storing reef algae positions keyed by position name */
  private final Map<String, FieldPose> reefAlgaePoses = new HashMap<>();

  /** Map storing floor positions keyed by position enum */
  private final Map<GamePieceFloorPosition, FieldPose> floorPoses = new HashMap<>();

  /**
   * Creates a unique key for storing branch positions.
   *
   * @param branch The reef branch
   * @param level The height level
   * @return A unique string key combining branch and level
   */
  private String makeBranchKey(ReefBranch branch, Level level) {
    return branch.toString() + "_" + level.toString();
  }

  static {
    // Branches (swerve position for aligning)
    // L2
    addBranchPose(
        ReefBranch.A,
        Level.L2,
        new Pose3d(
            3.15,
            4.19,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(0))));
    addBranchPose(
        ReefBranch.B,
        Level.L2,
        new Pose3d(
            3.15,
            3.862,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(0))));
    addBranchPose(
        ReefBranch.C,
        Level.L2,
        new Pose3d(
            3.309,
            2.383,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(60))));
    addBranchPose(
        ReefBranch.D,
        Level.L2,
        new Pose3d(
            3.640,
            2.201,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(60))));
    addBranchPose(
        ReefBranch.E,
        Level.L2,
        new Pose3d(
            4.992,
            2.757,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(120))));
    addBranchPose(
        ReefBranch.F,
        Level.L2,
        new Pose3d(
            5.285,
            2.943,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(120))));
    addBranchPose(
        ReefBranch.G,
        Level.L2,
        new Pose3d(
            5.821,
            3.855,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(180))));
    addBranchPose(
        ReefBranch.H,
        Level.L2,
        new Pose3d(
            5.811,
            4.185,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(180))));
    addBranchPose(
        ReefBranch.I,
        Level.L2,
        new Pose3d(
            5.304,
            5.097,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(-120))));
    addBranchPose(
        ReefBranch.J,
        Level.L2,
        new Pose3d(
            5.002,
            5.283,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(-120))));
    addBranchPose(
        ReefBranch.K,
        Level.L2,
        new Pose3d(
            3.968,
            5.26,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(-60))));
    addBranchPose(
        ReefBranch.L,
        Level.L2,
        new Pose3d(
            3.680,
            5.092,
            .788,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(-60))));
    // L3
    addBranchPose(
        ReefBranch.A,
        Level.L3,
        new Pose3d(
            3.15,
            4.19,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(0))));
    addBranchPose(
        ReefBranch.B,
        Level.L3,
        new Pose3d(
            3.15,
            3.862,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(0))));
    addBranchPose(
        ReefBranch.C,
        Level.L3,
        new Pose3d(
            3.309,
            2.383,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(60))));
    addBranchPose(
        ReefBranch.D,
        Level.L3,
        new Pose3d(
            3.640,
            2.201,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(60))));
    addBranchPose(
        ReefBranch.E,
        Level.L3,
        new Pose3d(
            4.992,
            2.757,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(120))));
    addBranchPose(
        ReefBranch.F,
        Level.L3,
        new Pose3d(
            5.285,
            2.943,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(120))));
    addBranchPose(
        ReefBranch.G,
        Level.L3,
        new Pose3d(
            5.821,
            3.855,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(180))));
    addBranchPose(
        ReefBranch.H,
        Level.L3,
        new Pose3d(
            5.811,
            4.185,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(180))));
    addBranchPose(
        ReefBranch.I,
        Level.L3,
        new Pose3d(
            5.304,
            5.097,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(-120))));
    addBranchPose(
        ReefBranch.J,
        Level.L3,
        new Pose3d(
            5.002,
            5.283,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(-120))));
    addBranchPose(
        ReefBranch.K,
        Level.L3,
        new Pose3d(
            3.968,
            5.26,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(-60))));
    addBranchPose(
        ReefBranch.L,
        Level.L3,
        new Pose3d(
            3.680,
            5.092,
            1.179,
            new Rotation3d(0, Units.degreesToRadians(125), Units.degreesToRadians(-60))));
    // L4
    addBranchPose(
        ReefBranch.A,
        Level.L4,
        new Pose3d(
            3.15,
            4.19,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(0))));
    addBranchPose(
        ReefBranch.B,
        Level.L4,
        new Pose3d(
            3.15,
            3.862,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(0))));
    addBranchPose(
        ReefBranch.C,
        Level.L4,
        new Pose3d(
            3.309,
            2.383,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(60))));
    addBranchPose(
        ReefBranch.D,
        Level.L4,
        new Pose3d(
            3.640,
            2.201,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(60))));
    addBranchPose(
        ReefBranch.E,
        Level.L4,
        new Pose3d(
            4.992,
            2.757,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(120))));
    addBranchPose(
        ReefBranch.F,
        Level.L4,
        new Pose3d(
            5.285,
            2.943,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(120))));
    addBranchPose(
        ReefBranch.G,
        Level.L4,
        new Pose3d(
            5.821,
            3.855,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(180))));
    addBranchPose(
        ReefBranch.H,
        Level.L4,
        new Pose3d(
            5.811,
            4.185,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(180))));
    addBranchPose(
        ReefBranch.I,
        Level.L4,
        new Pose3d(
            5.304,
            5.097,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(-120))));
    addBranchPose(
        ReefBranch.J,
        Level.L4,
        new Pose3d(
            5.002,
            5.283,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(-120))));
    addBranchPose(
        ReefBranch.K,
        Level.L4,
        new Pose3d(
            3.968,
            5.26,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(-60))));
    addBranchPose(
        ReefBranch.L,
        Level.L4,
        new Pose3d(
            3.680,
            5.092,
            1.829,
            new Rotation3d(0, Units.degreesToRadians(90), Units.degreesToRadians(-60))));

    // Reef algae
    addReefAlgaePose(
        ReefAlgaePosition.AB,
        new Pose3d(3.81, 4.026, 1.313, new Rotation3d(0, 0, Units.degreesToRadians(0))));
    addReefAlgaePose(
        ReefAlgaePosition.CD,
        new Pose3d(4.15, 4.348, .909, new Rotation3d(0, 0, Units.degreesToRadians(60))));
    addReefAlgaePose(
        ReefAlgaePosition.EF,
        new Pose3d(4.829, 3.438, 1.313, new Rotation3d(0, 0, Units.degreesToRadians(120))));
    addReefAlgaePose(
        ReefAlgaePosition.GH,
        new Pose3d(5.169, 4.026, .909, new Rotation3d(0, 0, Units.degreesToRadians(180))));
    addReefAlgaePose(
        ReefAlgaePosition.IJ,
        new Pose3d(4.829, 4.614, 1.313, new Rotation3d(0, 0, Units.degreesToRadians(-120))));
    addReefAlgaePose(
        ReefAlgaePosition.KL,
        new Pose3d(4.15, 4.614, .909, new Rotation3d(0, 0, Units.degreesToRadians(-60))));

    // Floor pieces
    // Algae
    addFloorPose(
        GamePieceFloorPosition.ALGAE_LEFT, new Pose3d(1.227, 5.937, .483, Rotation3d.kZero));
    addFloorPose(
        GamePieceFloorPosition.ALGAE_MIDDLE, new Pose3d(1.227, 4.026, .483, Rotation3d.kZero));
    addFloorPose(
        GamePieceFloorPosition.ALGAE_RIGHT, new Pose3d(1.227, 2.197, .483, Rotation3d.kZero));

    // Coral
    addFloorPose(GamePieceFloorPosition.CORAL_LEFT, new Pose3d(1.227, 5.937, 0, Rotation3d.kZero));
    addFloorPose(
        GamePieceFloorPosition.CORAL_MIDDLE, new Pose3d(1.227, 4.026, 0, Rotation3d.kZero));
    addFloorPose(GamePieceFloorPosition.CORAL_RIGHT, new Pose3d(1.227, 2.197, 0, Rotation3d.kZero));
  }

  /**
   * Gets the blue alliance pose for a branch at a specific level.
   *
   * @param branch The reef branch to get the pose for
   * @param level The height level of the branch
   * @return The Pose3d for the specified branch and level on the blue alliance side
   * @throws IllegalArgumentException if no position exists for the specified branch and level
   */
  public Pose3d getBranchBlue(ReefBranch branch, Level level) {
    FieldPose pos = branchPoses.get(makeBranchKey(branch, level));
    if (pos == null) {
      throw new IllegalArgumentException(
          String.format("No position found for branch %s at level %s", branch, level));
    }
    return pos.getBluePose();
  }

  /**
   * Gets the red alliance pose for a branch at a specific level.
   *
   * @param branch The reef branch to get the pose for
   * @param level The height level of the branch
   * @return The Pose3d for the specified branch and level on the red alliance side
   */
  public Pose3d getBranchRed(ReefBranch branch, Level level) {
    return branchPoses.get(makeBranchKey(branch, level)).getRed();
  }

  /**
   * Gets the blue alliance pose for an algae position between reef branches.
   *
   * @param position The algae position to get the pose for
   * @return The Pose3d for the specified algae position on the blue alliance side
   * @throws IllegalArgumentException if no position exists for the specified position
   */
  public Pose3d getReefAlgaeBlue(ReefAlgaePosition position) {
    FieldPose pos = reefAlgaePoses.get(position.toString());
    if (pos == null) {
      throw new IllegalArgumentException(
          String.format("No position found for reef algae position %s", position));
    }
    return pos.getBluePose();
  }

  /**
   * Gets the red alliance pose for an algae position between reef branches.
   *
   * @param position The algae position to get the pose for
   * @return The Pose3d for the specified algae position on the red alliance side
   */
  public Pose3d getReefAlgaeRed(ReefAlgaePosition position) {
    return reefAlgaePoses.get(position.toString()).getRed();
  }

  /**
   * Gets the blue alliance pose for a game piece floor position.
   *
   * @param position The floor position to get the pose for
   * @return The Pose3d for the specified floor position on the blue alliance side
   * @throws IllegalArgumentException if no position exists for the specified position
   */
  public Pose3d getFloorBlue(GamePieceFloorPosition position) {
    FieldPose pos = floorPoses.get(position);
    if (pos == null) {
      throw new IllegalArgumentException(
          String.format("No position found for floor position %s", position));
    }
    return pos.getBluePose();
  }

  /**
   * Gets the red alliance pose for a game piece floor position.
   *
   * @param position The floor position to get the pose for
   * @return The Pose3d for the specified floor position on the red alliance side
   */
  public Pose3d getFloorRed(GamePieceFloorPosition position) {
    return floorPoses.get(position).getRed();
  }

  /**
   * Adds a branch pose to the branch poses map.
   *
   * @param branch The reef branch
   * @param level The height level
   * @param bluePose The pose for the blue alliance side
   */
  private void addBranchPose(ReefBranch branch, Level level, Pose3d bluePose) {
    branchPoses.put(makeBranchKey(branch, level), new FieldPose(bluePose));
  }

  /**
   * Adds a reef algae pose to the reef algae poses map.
   *
   * @param position The algae position
   * @param bluePose The pose for the blue alliance side
   */
  private void addReefAlgaePose(ReefAlgaePosition position, Pose3d bluePose) {
    reefAlgaePoses.put(position.toString(), new FieldPose(bluePose));
  }

  /**
   * Adds a floor pose to the floor poses map.
   *
   * @param position The floor position
   * @param bluePose The pose for the blue alliance side
   */
  private void addFloorPose(GamePieceFloorPosition position, Pose3d bluePose) {
    floorPoses.put(position, new FieldPose(bluePose));
  }

  /**
   * Determines if a branch is a "left" branch (A, C, E, G, I, K).
   *
   * @param branch The reef branch to check
   * @return true if the branch is a left branch, false otherwise
   */
  public boolean isLeftBranch(ReefBranch branch) {
    // Left branches are A, C, E, G, I, K (odd indices in the enum)
    return branch.ordinal() % 2 == 0;
  }

  /**
   * Determines if a branch is a "right" branch (B, D, F, H, J, L).
   *
   * @param branch The reef branch to check
   * @return true if the branch is a right branch, false otherwise
   */
  public boolean isRightBranch(ReefBranch branch) {
    // Right branches are B, D, F, H, J, L (even indices in the enum)
    return branch.ordinal() % 2 == 1;
  }

  /**
   * Gets all left branch poses for the blue alliance at a specific level.
   *
   * @param level The height level for the branches
   * @return An array of Pose3d objects for all left branches (A, C, E, G, I, K) at the specified
   *     level
   */
  public Pose3d[] getLeftBranchesBlue(Level level) {
    Pose3d[] leftBranches = new Pose3d[6];
    int index = 0;

    for (ReefBranch branch : ReefBranch.values()) {
      if (isLeftBranch(branch)) {
        leftBranches[index++] = getBranchBlue(branch, level);
      }
    }

    return leftBranches;
  }

  /**
   * Gets all right branch poses for the blue alliance at a specific level.
   *
   * @param level The height level for the branches
   * @return An array of Pose3d objects for all right branches (B, D, F, H, J, L) at the specified
   *     level
   */
  public Pose3d[] getRightBranchesBlue(Level level) {
    Pose3d[] rightBranches = new Pose3d[6];
    int index = 0;

    for (ReefBranch branch : ReefBranch.values()) {
      if (isRightBranch(branch)) {
        rightBranches[index++] = getBranchBlue(branch, level);
      }
    }

    return rightBranches;
  }

  /**
   * Gets all left branch poses for the red alliance at a specific level.
   *
   * @param level The height level for the branches
   * @return An array of Pose3d objects for all left branches (A, C, E, G, I, K) at the specified
   *     level
   */
  public Pose3d[] getLeftBranchesRed(Level level) {
    Pose3d[] leftBranches = new Pose3d[6];
    int index = 0;

    for (ReefBranch branch : ReefBranch.values()) {
      if (isLeftBranch(branch)) {
        leftBranches[index++] = getBranchRed(branch, level);
      }
    }

    return leftBranches;
  }

  /**
   * Gets all right branch poses for the red alliance at a specific level.
   *
   * @param level The height level for the branches
   * @return An array of Pose3d objects for all right branches (B, D, F, H, J, L) at the specified
   *     level
   */
  public Pose3d[] getRightBranchesRed(Level level) {
    Pose3d[] rightBranches = new Pose3d[6];
    int index = 0;

    for (ReefBranch branch : ReefBranch.values()) {
      if (isRightBranch(branch)) {
        rightBranches[index++] = getBranchRed(branch, level);
      }
    }

    return rightBranches;
  }

  /**
   * Gets a specific left branch pose for the blue alliance at a specific level.
   *
   * @param leftIndex The index of the left branch (0-5, where 0=A, 1=C, 2=E, 3=G, 4=I, 5=K)
   * @param level The height level for the branch
   * @return The Pose3d for the specified left branch on the blue alliance side
   * @throws IllegalArgumentException if leftIndex is out of bounds
   */
  public Pose3d getLeftBranchBlue(int leftIndex, Level level) {
    if (leftIndex < 0 || leftIndex > 5) {
      throw new IllegalArgumentException("Left branch index must be between 0 and 5 inclusive");
    }

    ReefBranch[] leftBranches = {
      ReefBranch.A, ReefBranch.C, ReefBranch.E,
      ReefBranch.G, ReefBranch.I, ReefBranch.K
    };

    return getBranchBlue(leftBranches[leftIndex], level);
  }

  /**
   * Gets a specific right branch pose for the blue alliance at a specific level.
   *
   * @param rightIndex The index of the right branch (0-5, where 0=B, 1=D, 2=F, 3=H, 4=J, 5=L)
   * @param level The height level for the branch
   * @return The Pose3d for the specified right branch on the blue alliance side
   * @throws IllegalArgumentException if rightIndex is out of bounds
   */
  public Pose3d getRightBranchBlue(int rightIndex, Level level) {
    if (rightIndex < 0 || rightIndex > 5) {
      throw new IllegalArgumentException("Right branch index must be between 0 and 5 inclusive");
    }

    ReefBranch[] rightBranches = {
      ReefBranch.B, ReefBranch.D, ReefBranch.F,
      ReefBranch.H, ReefBranch.J, ReefBranch.L
    };

    return getBranchBlue(rightBranches[rightIndex], level);
  }

  /**
   * Gets a specific left branch pose for the red alliance at a specific level.
   *
   * @param leftIndex The index of the left branch (0-5, where 0=A, 1=C, 2=E, 3=G, 4=I, 5=K)
   * @param level The height level for the branch
   * @return The Pose3d for the specified left branch on the red alliance side
   * @throws IllegalArgumentException if leftIndex is out of bounds
   */
  public Pose3d getLeftBranchRed(int leftIndex, Level level) {
    if (leftIndex < 0 || leftIndex > 5) {
      throw new IllegalArgumentException("Left branch index must be between 0 and 5 inclusive");
    }

    ReefBranch[] leftBranches = {
      ReefBranch.A, ReefBranch.C, ReefBranch.E,
      ReefBranch.G, ReefBranch.I, ReefBranch.K
    };

    return getBranchRed(leftBranches[leftIndex], level);
  }

  /**
   * Gets a specific right branch pose for the red alliance at a specific level.
   *
   * @param rightIndex The index of the right branch (0-5, where 0=B, 1=D, 2=F, 3=H, 4=J, 5=L)
   * @param level The height level for the branch
   * @return The Pose3d for the specified right branch on the red alliance side
   * @throws IllegalArgumentException if rightIndex is out of bounds
   */
  public Pose3d getRightBranchRed(int rightIndex, Level level) {
    if (rightIndex < 0 || rightIndex > 5) {
      throw new IllegalArgumentException("Right branch index must be between 0 and 5 inclusive");
    }

    ReefBranch[] rightBranches = {
      ReefBranch.B, ReefBranch.D, ReefBranch.F,
      ReefBranch.H, ReefBranch.J, ReefBranch.L
    };

    return getBranchRed(rightBranches[rightIndex], level);
  }

  /**
   * Gets the closest left branch pose to a given pose for the blue alliance at a specific level.
   *
   * @param robotPose The current robot pose (Pose2d)
   * @param level The height level for the branches
   * @return The closest left branch (A, C, E, G, I, K) Pose3d
   */
  public Pose3d getClosestLeftBranchBlue(Pose2d robotPose, Level level) {
    ReefBranch closestBranch = null;
    double minDistance = Double.MAX_VALUE;

    // Check all left branches (A, C, E, G, I, K)
    for (ReefBranch branch : ReefBranch.values()) {
      if (isLeftBranch(branch)) {
        Pose3d branchPose = getBranchBlue(branch, level);
        double distance =
            robotPose
                .getTranslation()
                .getDistance(new Translation2d(branchPose.getX(), branchPose.getY()));

        if (distance < minDistance) {
          minDistance = distance;
          closestBranch = branch;
        }
      }
    }

    return getBranchBlue(closestBranch, level);
  }

  /**
   * Gets the closest right branch pose to a given pose for the blue alliance at a specific level.
   *
   * @param robotPose The current robot pose (Pose2d)
   * @param level The height level for the branches
   * @return The closest right branch (B, D, F, H, J, L) Pose3d
   */
  public Pose3d getClosestRightBranchBlue(Pose2d robotPose, Level level) {
    ReefBranch closestBranch = null;
    double minDistance = Double.MAX_VALUE;

    // Check all right branches (B, D, F, H, J, L)
    for (ReefBranch branch : ReefBranch.values()) {
      if (isRightBranch(branch)) {
        Pose3d branchPose = getBranchBlue(branch, level);
        double distance =
            robotPose
                .getTranslation()
                .getDistance(new Translation2d(branchPose.getX(), branchPose.getY()));

        if (distance < minDistance) {
          minDistance = distance;
          closestBranch = branch;
        }
      }
    }

    return getBranchBlue(closestBranch, level);
  }

  /**
   * Gets the closest left branch pose to a given pose for the red alliance at a specific level.
   *
   * @param robotPose The current robot pose (Pose2d)
   * @param level The height level for the branches
   * @return The closest left branch (A, C, E, G, I, K) Pose3d
   */
  public Pose3d getClosestLeftBranchRed(Pose2d robotPose, Level level) {
    ReefBranch closestBranch = null;
    double minDistance = Double.MAX_VALUE;

    // Check all left branches (A, C, E, G, I, K)
    for (ReefBranch branch : ReefBranch.values()) {
      if (isLeftBranch(branch)) {
        Pose3d branchPose = getBranchRed(branch, level);
        double distance =
            robotPose
                .getTranslation()
                .getDistance(new Translation2d(branchPose.getX(), branchPose.getY()));

        if (distance < minDistance) {
          minDistance = distance;
          closestBranch = branch;
        }
      }
    }

    return getBranchRed(closestBranch, level);
  }

  /**
   * Gets the closest right branch pose to a given pose for the red alliance at a specific level.
   *
   * @param robotPose The current robot pose (Pose2d)
   * @param level The height level for the branches
   * @return The closest right branch (B, D, F, H, J, L) Pose3d
   */
  public Pose3d getClosestRightBranchRed(Pose2d robotPose, Level level) {
    ReefBranch closestBranch = null;
    double minDistance = Double.MAX_VALUE;

    // Check all right branches (B, D, F, H, J, L)
    for (ReefBranch branch : ReefBranch.values()) {
      if (isRightBranch(branch)) {
        Pose3d branchPose = getBranchRed(branch, level);
        double distance =
            robotPose
                .getTranslation()
                .getDistance(new Translation2d(branchPose.getX(), branchPose.getY()));

        if (distance < minDistance) {
          minDistance = distance;
          closestBranch = branch;
        }
      }
    }

    return getBranchRed(closestBranch, level);
  }

  /**
   * Gets the closest branch (of any type) pose to a given pose for the blue alliance at a specific
   * level.
   *
   * @param robotPose The current robot pose (Pose2d)
   * @param level The height level for the branches
   * @return The closest branch Pose3d
   */
  public Pose3d getClosestBranchBlue(Pose2d robotPose, Level level) {
    ReefBranch closestBranch = null;
    double minDistance = Double.MAX_VALUE;

    // Check all branches
    for (ReefBranch branch : ReefBranch.values()) {
      Pose3d branchPose = getBranchBlue(branch, level);
      double distance =
          robotPose
              .getTranslation()
              .getDistance(new Translation2d(branchPose.getX(), branchPose.getY()));

      if (distance < minDistance) {
        minDistance = distance;
        closestBranch = branch;
      }
    }

    return getBranchBlue(closestBranch, level);
  }

  /**
   * Gets the closest branch (of any type) pose to a given pose for the red alliance at a specific
   * level.
   *
   * @param robotPose The current robot pose (Pose2d)
   * @param level The height level for the branches
   * @return The closest branch Pose3d
   */
  public Pose3d getClosestBranchRed(Pose2d robotPose, Level level) {
    ReefBranch closestBranch = null;
    double minDistance = Double.MAX_VALUE;

    // Check all branches
    for (ReefBranch branch : ReefBranch.values()) {
      Pose3d branchPose = getBranchRed(branch, level);
      double distance =
          robotPose
              .getTranslation()
              .getDistance(new Translation2d(branchPose.getX(), branchPose.getY()));

      if (distance < minDistance) {
        minDistance = distance;
        closestBranch = branch;
      }
    }

    return getBranchRed(closestBranch, level);
  }

  /**
   * Gets the branch identifier (ReefBranch enum) for the closest branch to a given pose.
   *
   * @param robotPose The current robot pose (Pose2d)
   * @param level The height level for the branches
   * @param isBlue Whether to use blue alliance positions (true) or red alliance positions (false)
   * @return The closest ReefBranch enum value
   */
  public ReefBranch getClosestBranchIdentifier(Pose2d robotPose, Level level, boolean isBlue) {
    ReefBranch closestBranch = null;
    double minDistance = Double.MAX_VALUE;

    // Check all branches
    for (ReefBranch branch : ReefBranch.values()) {
      Pose3d branchPose = isBlue ? getBranchBlue(branch, level) : getBranchRed(branch, level);
      double distance =
          robotPose
              .getTranslation()
              .getDistance(new Translation2d(branchPose.getX(), branchPose.getY()));

      if (distance < minDistance) {
        minDistance = distance;
        closestBranch = branch;
      }
    }

    return closestBranch;
  }
}
