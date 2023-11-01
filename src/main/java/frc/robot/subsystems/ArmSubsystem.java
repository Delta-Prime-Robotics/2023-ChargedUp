package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;


public class ArmSubsystem extends SubsystemBase {
    //defining the spark max arm motor
    private CANSparkMax m_armLeader;
    private CANSparkMax m_armFollower;
    
    private MotorControllerGroup m_MotorControllerGroup;
    //arm speed scale factor
    
    
    //Defineing the arm encoder
    public RelativeEncoder m_armEncoder;
    public RelativeEncoder m_armFollowerEncoder;
   
   
    public ArmSubsystem() {
        m_armLeader = new CANSparkMax(RoboRio.CanID.kArmLeader, MotorType.kBrushless);
        m_armFollower = new CANSparkMax(RoboRio.CanID.kArmFollower, MotorType.kBrushless);
        m_armFollower.setInverted(true);
        m_MotorControllerGroup = new MotorControllerGroup(m_armLeader, m_armFollower);

        // Need  value for Arm Encoder in Constents
        m_armEncoder = m_armLeader.getEncoder();
        m_armFollowerEncoder = m_armFollower.getEncoder();
        m_armEncoder.setPosition(0);
        m_armFollowerEncoder.setPosition(0);
       // m_armEncoder.setPositionConversionFactor(0);

        //m_armMotor.setSmartCurrentLimit(30, 90, 10);
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
    

  /**
   * Makes sure that the motor will run with the imput you have given.
   * @param forwardSpeed
   * @return the constrainted forwardSpeed
   */
  private double applyLinearConstraints(double forwardSpeed) {
    double result = forwardSpeed;

    final double kDeadzone = 0.05;
    final double kMinNeededToMove = 0.1;
    final double kSpeedLimit = 1;       // This should be a value <= 1.0

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

  /**
   * ArmGo applies linear constraints which is a method that
   * "does math to limit the motor and make sure you don't screw up"
   * to the imputed forwardSpeed variable.
   * It then sets m_armMotor to this speed.
   * @param forwardSpeed
   **/
  public void ArmGo(double forwardSpeed) {

    forwardSpeed = applyLinearConstraints(forwardSpeed);
    if ( m_armEncoder.getPosition() < 125 || forwardSpeed < 0) {
      m_MotorControllerGroup.set(forwardSpeed);
    }
    else{
      m_MotorControllerGroup.set(0);
    }
    
  }

  // public void ArmGoEncoder(double speed) {

  //   speed = applyLinearConstraints(speed);
  //   if ( m_armEncoder.getPosition() < 10000) {
  //      m_armMotor.set(speed);}
  //   else
  //     m_armMotor.set(0);
  // }
  
  /**
   * Sets m_armMotor speed to zero
   */
  public void stop() {

    m_MotorControllerGroup.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Motor Speed", m_armLeader.get());
    SmartDashboard.putNumber("arm Encoder", m_armEncoder.getPosition());
    SmartDashboard.putNumber("arm follower Encoder", m_armFollowerEncoder.getPosition());
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