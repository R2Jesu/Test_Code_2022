// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/PS4Controller.h>
#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <rev/CANEncoder.h>
#include <rev/CANAnalog.h>
#include <ctre/Phoenix.h>
#include <cmath>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  frc::PS4Controller m_Drivestick{0};
  rev::CANSparkMax m_SwerveDrive{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder m_DriveEncoder = m_SwerveDrive.GetEncoder();
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_SwerveTurn{1};
  rev::SparkMaxAnalogSensor m_SwerveAnalog = m_SwerveDrive.GetAnalog();
  double conversion = 360.0/3.3;
  double x, y, z;
  double correctionPID;
  double LENGTH = 4;
double WIDTH = 4;
double R = sqrt((LENGTH*LENGTH) + (WIDTH*WIDTH));
double angleChoice;
double Ppid = .05;
double Ipid = 0.0;
double Dpid = .00115;
frc2::PIDController m_angleController{ Ppid , Ipid, Dpid, 20_ms};
};
