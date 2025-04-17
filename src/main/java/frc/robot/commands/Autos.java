// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {

  /*public static Command coralAuto1(SwerveSubsystem swerveSubsystem){
    return new SequentialCommandGroup(
                                      //swerveSubsystem.turnToInitCommand().until(swerveSubsystem.isCameraInitalized),
                                      new CoralAuto(swerveSubsystem),
                                      swerveSubsystem.oaRunPathTest2(),
                                      Commands.waitSeconds(2),
                                      swerveSubsystem.oaRunPathTest());*/
    /* 
    return Commands.sequence(swerveSubsystem.oaRunPathTest(),
                            Commands.waitSeconds(2),
                            swerveSubsystem.oaRunPathTest2());
                            */
  //}

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
