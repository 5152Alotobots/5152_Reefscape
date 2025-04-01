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
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.experimental.UtilityClass;

/** Constants class containing field-specific positions and measurements for the game field. */
@UtilityClass
public class FieldConstants {

  public enum BranchType {
    LEFT,
    RIGHT,
    ANY
  }

  // Helper Class
  @RequiredArgsConstructor
  @Getter
  private static class FieldPose {
    private final Pose2d bluePose;

    public Pose2d getRed() {
      return FlippingUtil.flipFieldPose(bluePose);
    }
  }

  // Branch class and related code
  @UtilityClass
  public static class BranchPositions {
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

    private final Map<ReefBranch, FieldPose> branchPoses = new HashMap<>();

    static {
      // Branch positions
      addBranchPose(
          ReefBranch.A, new Pose2d(3.71, 4.19, new Rotation2d(Units.degreesToRadians(0))));
      addBranchPose(
          ReefBranch.B, new Pose2d(3.71, 3.862, new Rotation2d(Units.degreesToRadians(0))));
      addBranchPose(
          ReefBranch.C, new Pose2d(3.939, 3.434, new Rotation2d(Units.degreesToRadians(60))));
      addBranchPose(
          ReefBranch.D, new Pose2d(4.243, 3.269, new Rotation2d(Units.degreesToRadians(60))));
      addBranchPose(
          ReefBranch.E, new Pose2d(4.736, 3.272, new Rotation2d(Units.degreesToRadians(120))));
      addBranchPose(
          ReefBranch.F, new Pose2d(5.02, 3.435, new Rotation2d(Units.degreesToRadians(120))));
      addBranchPose(
          ReefBranch.G, new Pose2d(5.267, 3.862, new Rotation2d(Units.degreesToRadians(180))));
      addBranchPose(
          ReefBranch.H, new Pose2d(5.267, 4.19, new Rotation2d(Units.degreesToRadians(180))));
      addBranchPose(
          ReefBranch.I, new Pose2d(5.02, 4.615, new Rotation2d(Units.degreesToRadians(-120))));
      addBranchPose(
          ReefBranch.J, new Pose2d(4.737, 4.783, new Rotation2d(Units.degreesToRadians(-120))));
      addBranchPose(
          ReefBranch.K, new Pose2d(4.243, 4.782, new Rotation2d(Units.degreesToRadians(-60))));
      addBranchPose(
          ReefBranch.L, new Pose2d(3.959, 4.616, new Rotation2d(Units.degreesToRadians(-60))));
    }

    private void addBranchPose(ReefBranch branch, Pose2d bluePose) {
      branchPoses.put(branch, new FieldPose(bluePose));
    }

    public Pose2d getBranchBlue(ReefBranch branch) {
      FieldPose pos = branchPoses.get(branch);
      if (pos == null) {
        throw new IllegalArgumentException(
            String.format("No position found for branch %s", branch));
      }
      return pos.getBluePose();
    }

    public Pose2d getBranchRed(ReefBranch branch) {
      return branchPoses.get(branch).getRed();
    }

    public boolean isLeftBranch(ReefBranch branch) {
      return branch.ordinal() % 2 == 0;
    }

    public boolean isRightBranch(ReefBranch branch) {
      return branch.ordinal() % 2 == 1;
    }

    public Pose2d getClosestBranch(
        Pose2d robotPose, DriverStation.Alliance alliance, BranchType branchType) {
      Map<Pose2d, ReefBranch> pose2dToBranch = new HashMap<>();

      for (Map.Entry<ReefBranch, FieldPose> entry : branchPoses.entrySet()) {
        if (branchType == BranchType.LEFT && !isLeftBranch(entry.getKey())) {
          continue;
        }
        if (branchType == BranchType.RIGHT && !isRightBranch(entry.getKey())) {
          continue;
        }

        Pose2d pose2d =
            alliance == DriverStation.Alliance.Blue
                ? getBranchBlue(entry.getKey())
                : getBranchRed(entry.getKey());

        pose2dToBranch.put(pose2d, entry.getKey());
      }

      return robotPose.nearest(pose2dToBranch.keySet().stream().toList());
    }

    public Pose2d getClosestBranch(Pose2d robotPose, DriverStation.Alliance alliance) {
      return getClosestBranch(robotPose, alliance, BranchType.ANY);
    }

    public ReefBranch getClosestBranchIdentifier(
        Pose2d robotPose, DriverStation.Alliance alliance, BranchType branchType) {
      Map<Pose2d, ReefBranch> pose2dToBranch = new HashMap<>();

      for (Map.Entry<ReefBranch, FieldPose> entry : branchPoses.entrySet()) {
        if (branchType == BranchType.LEFT && !isLeftBranch(entry.getKey())) {
          continue;
        }
        if (branchType == BranchType.RIGHT && !isRightBranch(entry.getKey())) {
          continue;
        }

        Pose2d pose2d =
            alliance == DriverStation.Alliance.Blue
                ? getBranchBlue(entry.getKey())
                : getBranchRed(entry.getKey());

        pose2dToBranch.put(pose2d, entry.getKey());
      }

      Pose2d nearestPose = robotPose.nearest(pose2dToBranch.keySet().stream().toList());
      return pose2dToBranch.get(nearestPose);
    }

    public ReefBranch getClosestBranchIdentifier(
        Pose2d robotPose, DriverStation.Alliance alliance) {
      return getClosestBranchIdentifier(robotPose, alliance, BranchType.ANY);
    }
  }

  // Algae class and related code
  @UtilityClass
  public static class AlgaePositions {
    public enum ReefAlgaePosition {
      AB,
      CD,
      EF,
      GH,
      IJ,
      KL
    }

    private final Map<ReefAlgaePosition, FieldPose> reefAlgaePoses = new HashMap<>();

    static {
      // Reef algae
      addReefAlgaePose(
          ReefAlgaePosition.AB, new Pose2d(3.81, 4.026, new Rotation2d(Units.degreesToRadians(0))));
      addReefAlgaePose(
          ReefAlgaePosition.CD,
          new Pose2d(4.15, 4.348, new Rotation2d(Units.degreesToRadians(60))));
      addReefAlgaePose(
          ReefAlgaePosition.EF,
          new Pose2d(4.829, 3.438, new Rotation2d(Units.degreesToRadians(120))));
      addReefAlgaePose(
          ReefAlgaePosition.GH,
          new Pose2d(5.169, 4.026, new Rotation2d(Units.degreesToRadians(180))));
      addReefAlgaePose(
          ReefAlgaePosition.IJ,
          new Pose2d(4.829, 4.614, new Rotation2d(Units.degreesToRadians(-120))));
      addReefAlgaePose(
          ReefAlgaePosition.KL,
          new Pose2d(4.15, 4.614, new Rotation2d(Units.degreesToRadians(-60))));
    }

    private void addReefAlgaePose(ReefAlgaePosition position, Pose2d bluePose) {
      reefAlgaePoses.put(position, new FieldPose(bluePose));
    }

    public Pose2d getReefAlgaeBlue(ReefAlgaePosition position) {
      FieldPose pos = reefAlgaePoses.get(position);
      if (pos == null) {
        throw new IllegalArgumentException(
            String.format("No position found for reef algae position %s", position));
      }
      return pos.getBluePose();
    }

    public Pose2d getReefAlgaeRed(ReefAlgaePosition position) {
      return reefAlgaePoses.get(position).getRed();
    }

    public Pose2d getClosestReefAlgae(Pose2d robotPose, DriverStation.Alliance alliance) {
      Map<Pose2d, ReefAlgaePosition> pose2dToAlgae = new HashMap<>();

      for (Map.Entry<ReefAlgaePosition, FieldPose> entry : reefAlgaePoses.entrySet()) {
        Pose2d pose2d =
            alliance == DriverStation.Alliance.Blue
                ? getReefAlgaeBlue(entry.getKey())
                : getReefAlgaeRed(entry.getKey());

        pose2dToAlgae.put(pose2d, entry.getKey());
      }

      return robotPose.nearest(pose2dToAlgae.keySet().stream().toList());
    }

    public ReefAlgaePosition getClosestReefAlgaeIdentifier(
        Pose2d robotPose, DriverStation.Alliance alliance) {
      Map<Pose2d, ReefAlgaePosition> pose2dToAlgae = new HashMap<>();

      for (Map.Entry<ReefAlgaePosition, FieldPose> entry : reefAlgaePoses.entrySet()) {
        Pose2d pose2d =
            alliance == DriverStation.Alliance.Blue
                ? getReefAlgaeBlue(entry.getKey())
                : getReefAlgaeRed(entry.getKey());

        pose2dToAlgae.put(pose2d, entry.getKey());
      }

      Pose2d nearestPose = robotPose.nearest(pose2dToAlgae.keySet().stream().toList());
      return pose2dToAlgae.get(nearestPose);
    }
  }

  // Floor class and related code
  @UtilityClass
  public static class FloorPositions {
    public enum GamePieceFloorPosition {
      CORAL_LEFT,
      CORAL_MIDDLE,
      CORAL_RIGHT,
      ALGAE_LEFT,
      ALGAE_MIDDLE,
      ALGAE_RIGHT
    }

    private final Map<GamePieceFloorPosition, FieldPose> floorPoses = new HashMap<>();

    static {
      // Floor pieces - Algae
      addFloorPose(GamePieceFloorPosition.ALGAE_LEFT, new Pose2d(1.227, 5.937, Rotation2d.kZero));
      addFloorPose(GamePieceFloorPosition.ALGAE_MIDDLE, new Pose2d(1.227, 4.026, Rotation2d.kZero));
      addFloorPose(GamePieceFloorPosition.ALGAE_RIGHT, new Pose2d(1.227, 2.197, Rotation2d.kZero));

      // Floor pieces - Coral
      addFloorPose(GamePieceFloorPosition.CORAL_LEFT, new Pose2d(1.227, 5.937, Rotation2d.kZero));
      addFloorPose(GamePieceFloorPosition.CORAL_MIDDLE, new Pose2d(1.227, 4.026, Rotation2d.kZero));
      addFloorPose(GamePieceFloorPosition.CORAL_RIGHT, new Pose2d(1.227, 2.197, Rotation2d.kZero));
    }

    private void addFloorPose(GamePieceFloorPosition position, Pose2d bluePose) {
      floorPoses.put(position, new FieldPose(bluePose));
    }

    public Pose2d getFloorBlue(GamePieceFloorPosition position) {
      FieldPose pos = floorPoses.get(position);
      if (pos == null) {
        throw new IllegalArgumentException(
            String.format("No position found for floor position %s", position));
      }
      return pos.getBluePose();
    }

    public Pose2d getFloorRed(GamePieceFloorPosition position) {
      return floorPoses.get(position).getRed();
    }

    public Pose2d getClosestFloor(Pose2d robotPose, DriverStation.Alliance alliance) {
      Map<Pose2d, GamePieceFloorPosition> pose2dToFloor = new HashMap<>();

      for (Map.Entry<GamePieceFloorPosition, FieldPose> entry : floorPoses.entrySet()) {
        Pose2d pose2d =
            alliance == DriverStation.Alliance.Blue
                ? getFloorBlue(entry.getKey())
                : getFloorRed(entry.getKey());

        pose2dToFloor.put(pose2d, entry.getKey());
      }

      return robotPose.nearest(pose2dToFloor.keySet().stream().toList());
    }

    public GamePieceFloorPosition getClosestFloorIdentifier(
        Pose2d robotPose, DriverStation.Alliance alliance) {
      Map<Pose2d, GamePieceFloorPosition> pose2dToFloor = new HashMap<>();

      for (Map.Entry<GamePieceFloorPosition, FieldPose> entry : floorPoses.entrySet()) {
        Pose2d pose2d =
            alliance == DriverStation.Alliance.Blue
                ? getFloorBlue(entry.getKey())
                : getFloorRed(entry.getKey());

        pose2dToFloor.put(pose2d, entry.getKey());
      }

      Pose2d nearestPose = robotPose.nearest(pose2dToFloor.keySet().stream().toList());
      return pose2dToFloor.get(nearestPose);
    }
  }

  // Coral Station class and related code
  @UtilityClass
  public static class CoralStationPositions {
    public enum CoralStation {
      A_LEFT,
      A_RIGHT,
      B_LEFT,
      B_RIGHT,
      C_LEFT,
      C_RIGHT
    }

    private final Map<CoralStation, FieldPose> stationPoses = new HashMap<>();

    static {
      // Coral stations
      addStationPose(
          CoralStation.A_LEFT, new Pose2d(.35, 7, Rotation2d.fromDegrees(125)));
      addStationPose(
          CoralStation.A_RIGHT, new Pose2d(.35, 1, Rotation2d.fromDegrees(-125)));
      addStationPose(
          CoralStation.B_LEFT, new Pose2d(.875, 7.4, Rotation2d.fromDegrees(125)));
      addStationPose(
          CoralStation.B_RIGHT, new Pose2d(.875, .6, Rotation2d.fromDegrees(-125)));
      addStationPose(
          CoralStation.C_LEFT, new Pose2d(1.4, 7.83, Rotation2d.fromDegrees(125)));
      addStationPose(
          CoralStation.C_RIGHT, new Pose2d(1.4, .23, Rotation2d.fromDegrees(-125))); 
    }

    private void addStationPose(CoralStation station, Pose2d bluePose) {
      stationPoses.put(station, new FieldPose(bluePose));
    }

    public Pose2d getStationBlue(CoralStation station) {
      FieldPose pos = stationPoses.get(station);
      if (pos == null) {
        throw new IllegalArgumentException(
            String.format("No position found for coral station %s", station));
      }
      return pos.getBluePose();
    }

    public Pose2d getStationRed(CoralStation station) {
      return stationPoses.get(station).getRed();
    }

    public Pose2d getClosestStation(Pose2d robotPose, DriverStation.Alliance alliance) {
      Map<Pose2d, CoralStation> pose2dToStation = new HashMap<>();

      for (Map.Entry<CoralStation, FieldPose> entry : stationPoses.entrySet()) {
        Pose2d pose2d =
            alliance == DriverStation.Alliance.Blue
                ? getStationBlue(entry.getKey())
                : getStationRed(entry.getKey());

        pose2dToStation.put(pose2d, entry.getKey());
      }

      return robotPose.nearest(pose2dToStation.keySet().stream().toList());
    }

    public CoralStation getClosestStationIdentifier(
        Pose2d robotPose, DriverStation.Alliance alliance) {
      Map<Pose2d, CoralStation> pose2dToStation = new HashMap<>();

      for (Map.Entry<CoralStation, FieldPose> entry : stationPoses.entrySet()) {
        Pose2d pose2d =
            alliance == DriverStation.Alliance.Blue
                ? getStationBlue(entry.getKey())
                : getStationRed(entry.getKey());

        pose2dToStation.put(pose2d, entry.getKey());
      }

      Pose2d nearestPose = robotPose.nearest(pose2dToStation.keySet().stream().toList());
      return pose2dToStation.get(nearestPose);
    }
  }
}
