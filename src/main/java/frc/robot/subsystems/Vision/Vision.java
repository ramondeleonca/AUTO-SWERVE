// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  //! IO IMPLEMENTATION
  VisionIO io;

  //! INPUTS LOG TABLE (Generated )
  VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  /** Creates a new Vision. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.periodic();

    //! AUTO-LOGGED INPUTS
    // the inputs parameters is the previously created VisionIOInputsAutoLogged object
    // the inputs object is updated with the current values of the VisionIO object
    io.updateInputs(inputs);
  }

  public Pose2d getEstimatedPose() {
    return io.getEstimatedPose();
  }

  public double getTimestampSeconds() {
    return io.getTimestampSeconds();
  }
}
