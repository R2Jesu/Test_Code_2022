// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

//#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutNumber("Angle Choice Manual", 0.0);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  //fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
   m_SwerveAnalog.SetPositionConversionFactor(conversion);
}

void Robot::TeleopPeriodic() {
    //x = m_Drivestick.GetRightX();
    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    x = m_Drivestick.GetR2Axis();
    y = m_Drivestick.GetRightY() * -1;
    z = m_Drivestick.GetLeftX();
  printf("-------\n");
  printf("x=%f\ny=%f\nz=%f\n", x, y, z);

  double A = y - z*(LENGTH/R);
	double B = y + z*(LENGTH/R);
	double C = x - z*(WIDTH/R);
	double D = x + z*(WIDTH/R);

	double wSpeed1 = .25*(sqrt(B*B + C*C));

  //printf("B=%f\nC=%f\n", B, C);
	double wAngle1 = atan2(B,C) * 180/M_PI;
  if (wAngle1 < 0.0)
  {
    wAngle1 = wAngle1 + 360;
  }
  //printf("wAngle1=%f\n", wAngle1);
  printf("-------\n");

	double wSpeed2 = .25*(sqrt(B*B + D*D));
	double wAngle2 = atan2(B,D) * 180/M_PI;
  if (wAngle2 < 0.0)
  {
    wAngle2 = wAngle2 + 360;
  }

	double wSpeed3 = .25*(sqrt(A*A + D*D));
	double wAngle3 = atan2(A,D) * 180/M_PI;
  if (wAngle3 < 0.0)
  {
    wAngle3 = wAngle3 + 360;
  }

	double wSpeed4 = .25*(sqrt(A*A + C*C));
	double wAngle4 = atan2(A,C) * 180/M_PI;
  if (wAngle4 < 0.0)
  {
    wAngle4 = wAngle4 + 360;
  }

  double pidOutput;

	
frc::SmartDashboard::PutNumber("Current Angle", m_SwerveAnalog.GetPosition());
frc::SmartDashboard::PutNumber("Current Speed", wSpeed1);
frc::SmartDashboard::PutNumber("Angle Choice", wAngle1);
angleChoice = frc::SmartDashboard::GetNumber("Angle Choice Manual", 0);
//if ((m_Drivestick.GetRightX() > .1) || (m_Drivestick.GetRightX() < -.1))
//if ((m_Drivestick.GetRightY() > .2) || (m_Drivestick.GetRightY() < -.2))
//{
  m_SwerveDrive.Set(wSpeed1);
  printf("m_SwerveAnalog.GetPosition() %f\n", m_SwerveAnalog.GetPosition());
  pidOutput = m_angleController.Calculate(m_SwerveAnalog.GetPosition(), wAngle1);
  //pidOutput = m_angleController.Calculate(m_SwerveAnalog.GetPosition(), angleChoice);
  m_SwerveTurn.Set(pidOutput);
  frc::SmartDashboard::PutNumber("pidOutput", pidOutput);
  //Kept turning because we never stopped it added this
//}
//Kept running because we never stopped it
//else
//{
//  m_SwerveDrive.Set(0);
//  m_SwerveTurn.Set(0);
//}
 /*if ((m_Drivestick.GetLeftX() > .1) || (m_Drivestick.GetLeftX() < -.1))
 {
   m_SwerveTurn.Set(m_Drivestick.GetLeftX());
 }else
 {
  m_SwerveTurn.Set(0);
 }
 if ((m_Drivestick.GetRightY() > .1) || (m_Drivestick.GetRightY() < -.1))
  {
    m_SwerveDrive.Set(m_Drivestick.GetRightY());
  }else
  {
    m_SwerveDrive.Set(0);
  }
  if (m_Drivestick.GetSquareButtonPressed() == true)
  {
    while (m_SwerveAnalog.GetPosition() != 90)
    {
      m_SwerveTurn.Set(.2);
      frc::SmartDashboard::PutNumber("Current Angle", m_SwerveAnalog.GetPosition());
    }
      m_SwerveTurn.Set(0);
  }
  if (m_Drivestick.GetCircleButtonPressed() == true)
  {
    while (m_SwerveAnalog.GetPosition() != 270)
    {
      m_SwerveTurn.Set(.2);
      frc::SmartDashboard::PutNumber("Current Angle", m_SwerveAnalog.GetPosition());
    } 
      m_SwerveTurn.Set(0);
  }*/
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
