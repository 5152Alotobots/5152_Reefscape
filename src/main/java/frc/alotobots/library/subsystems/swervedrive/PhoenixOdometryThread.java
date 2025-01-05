/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.swervedrive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import frc.alotobots.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 */
public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock = new ReentrantLock();
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static PhoenixOdometryThread instance = null;
  private boolean isCANFD;

  private void configure() {
    this.isCANFD =
        new CANBus(Constants.tunerConstants.getDrivetrainConstants().CANBusName).isNetworkFD();
  }

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    SwerveDriveSubsystem.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      SwerveDriveSubsystem.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    SwerveDriveSubsystem.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      SwerveDriveSubsystem.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    SwerveDriveSubsystem.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      SwerveDriveSubsystem.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      // Wait for updates from all signals
      signalsLock.lock();
      try {
        if (isCANFD && phoenixSignals.length > 0) {
          BaseStatusSignal.waitForAll(
              2.0 / Constants.tunerConstants.getOdometryFrequency(), phoenixSignals);
        } else {
          Thread.sleep((long) (1000.0 / Constants.tunerConstants.getOdometryFrequency()));
          if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      // Save new data to queues
      SwerveDriveSubsystem.odometryLock.lock();
      try {
        // Sample timestamp is current FPGA time minus average CAN latency
        double timestamp = RobotController.getFPGATime() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : phoenixSignals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (phoenixSignals.length > 0) {
          timestamp -= totalLatency / phoenixSignals.length;
        }

        // Add new samples to queues
        for (int i = 0; i < phoenixSignals.length; i++) {
          phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        SwerveDriveSubsystem.odometryLock.unlock();
      }
    }
  }
}
