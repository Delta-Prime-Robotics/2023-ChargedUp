package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmSubsystem extends SubsystemBase {
    //defining the spark max arm motor
    private CANSparkMax m_armMotor;
    
    //arm speed scale factor
    private final double kScaleFactor = 0.5;
    
    //Defineing the arm encoder
    private RelativeEncoder m_armEncoder;
    
    public ArmSubsystem() {
        m_armMotor = new CANSparkMax(RoboRio.CanID.kArmControler, MotorType.kBrushless);
        // Need  value for Arm Encoder in Constents
        m_armEncoder = m_armMotor.getEncoder();
        
        m_armEncoder.setPositionConversionFactor(0);

        m_armMotor.setSmartCurrentLimit(30, 90, 10);
    }
    
     /**
   * This command resets the arm Encoders
   * 
   * @return a runOnce
   */
    public CommandBase resetEncoders() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
                m_armEncoder.setPosition(0.0);
            });
      }
    
      @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor Speed", m_armMotor.get());
  }


  private double applyLinearConstraints(double forwardSpeed) {
    double result = forwardSpeed;

    final double kDeadzone = 0.05;
    final double kMinNeededToMove = 0.1;
    final double kSpeedLimit = 0.8;       // This should be a value <= 1.0

    double absSpeed = Math.abs(forwardSpeed);

    if (absSpeed < kDeadzone) {
      result = 0.0;
    }
    else if (absSpeed < kMinNeededToMove) {
      result = 0.1 * (forwardSpeed/Math.abs(forwardSpeed));
    }
    else if (absSpeed > kSpeedLimit)
    {
      result = kSpeedLimit * (forwardSpeed/Math.abs(forwardSpeed));
    }

    return result;
  }


  public void stop() {

    m_armMotor.set(0.0);
  }

  public void ArmGo(double forwardSpeed) {

    forwardSpeed = applyLinearConstraints(forwardSpeed);

    m_armMotor.set(forwardSpeed);
  }


  // public Command ArmForward(double speed) {
  //   return startEnd(() -> {this.m_armMotor.set(0.5);}, () -> {m_armMotor.set(0.0);});

  // }

  // public Command ArmBackward(double speed) {
  //   return startEnd(() -> {this.m_armMotor.set(-0.5);}, () -> {m_armMotor.set(0.0);});
  // }


//   public CommandBase ArmGo(double speed) {
//     return run(
//       () -> {
//         double armSpeed = speed;
//         if (armSpeed < -1) {
//           armSpeed = -1;
//         }
//         else if (armSpeed > 1){
//           armSpeed = 1;
//         }
//         m_armMotor.set(speed * kScaleFactor);
//         SmartDashboard.putNumber("Arm Speed", speed);
//       });
//   };
}