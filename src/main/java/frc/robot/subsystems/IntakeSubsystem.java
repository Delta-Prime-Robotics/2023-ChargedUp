package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
    //Defining the Intake motor
    private CANSparkMax m_intakeMotor;

    //Defiding the intake encoder
    public RelativeEncoder m_intakeEncoder;

    //Intake speed scale factor
    private final double kScaleFactor = 0.5;


    public IntakeSubsystem() {
        m_intakeMotor = new CANSparkMax(RoboRio.CanID.kIntakeControler, MotorType.kBrushless);
        
        m_intakeEncoder = m_intakeMotor.getEncoder();
        // Need  value for Intake Encoder in Constents
        m_intakeEncoder.setPositionConversionFactor(1);

        //m_intakeMotor.setSmartCurrentLimit(30, 90, 10);
    }

    /**
   * This command resets the intake encoders
   * 
   * @return a runOnce
   */
  public CommandBase resetEncoders() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
            m_intakeEncoder.setPosition(0.0);
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
    SmartDashboard.putNumber("right Stick", forwardSpeed);
    return result;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Motor Speed", m_intakeMotor.get());
    SmartDashboard.putString("New Reminder \n Everything is under control", "True");
    

  }


  /**
   * Scales imputed speed and sets speed to either -1 or 1
   * @param speed
   * @return a run command
   */
  public CommandBase IntakeMove(double speed) {
      return run(
      () -> {
        double intakeSpeed = speed;
        if (intakeSpeed > -1) {
          intakeSpeed = -1;
        }
        else if (intakeSpeed < 1){
          intakeSpeed = 1;
        }
        m_intakeMotor.set(speed * kScaleFactor);
      });
  }

  public void IntakeGo(double speed) {

    speed = applyLinearConstraints(speed);

    m_intakeMotor.set(speed);
  }

  public void IntakeGoEncoder(double speed) {

    speed = applyLinearConstraints(speed);
    if ( m_intakeEncoder.getPosition() < 10000) {
       m_intakeMotor.set(speed);}
    else
      m_intakeMotor.set(0);
  }
  
}
