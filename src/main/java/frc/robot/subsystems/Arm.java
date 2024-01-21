package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.ArmConstants;

public class Arm {
    
    final TalonFX arm;
    // final DutyCycleEncoder throughBore;

    final VoltageOut m_pivotVoltageRequest = new VoltageOut(0);
    final PositionVoltage m_pivotPositionRequest = new PositionVoltage(0, 0, true, 0,0, false, false, false);
    final MotionMagicVoltage m_pivotMotionMagicRequest = new MotionMagicVoltage(0, true, 0, 0, false, false, false);

    final NeutralOut m_brake = new NeutralOut();

    public Arm(){
        arm = new TalonFX(10);
        //throughBore = new DutyCycleEncoder(16);
        init();
    }

    public void configurePID() {
        TalonFXConfiguration armConfiguration = new TalonFXConfiguration();
        
        arm.getConfigurator().refresh(armConfiguration);

        ArmConstants.kP_Arm.loadPreferences();
        ArmConstants.mmAcceleration_Arm.loadPreferences();
        ArmConstants.mmCruiseVelocity_Arm.loadPreferences();

        armConfiguration.Slot0.kP = ArmConstants.kP_Arm.get();

        MotionMagicConfigs armMMConfigs = armConfiguration.MotionMagic;
        armMMConfigs.MotionMagicCruiseVelocity = ArmConstants.mmCruiseVelocity_Arm.get();
        armMMConfigs.MotionMagicAcceleration = ArmConstants.mmAcceleration_Arm.get();

        armConfiguration.Voltage.PeakForwardVoltage = 11.5;
        armConfiguration.Voltage.PeakReverseVoltage = -11.5;

        StatusCode statusArm = arm.getConfigurator().apply(armConfiguration);

        if (!statusArm.isOK()){
            DriverStation.reportError("Could not apply pivot configs, error code:"+ statusArm.toString(), null);
        }
    }

    public void init() {
        configurePID();
    }

    public Command setPosition(double position) {
        return Commands.runOnce(() -> {
            arm.setControl(m_pivotMotionMagicRequest.withPosition(position));
        });
    }

}