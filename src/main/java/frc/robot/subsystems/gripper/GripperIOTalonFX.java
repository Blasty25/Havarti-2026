package frc.robot.subsystems.gripper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap.Arm;

public class GripperIOTalonFX implements GripperIO {
  private final TalonFX talon;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;

  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public GripperIOTalonFX() {
    talon = new TalonFX(Arm.endEffector); // TODO replace this with the actual outtake id;

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        GripperIOConstants.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = GripperConstants.Current.maxSupply;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = GripperConstants.Current.maxStator;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    tryUntilOk(5, () -> (talon.getConfigurator().apply(config, 0.25)));

    // Getting Status Signals
    voltage = talon.getMotorVoltage();
    temperature = talon.getDeviceTemp();
    statorCurrent = talon.getTorqueCurrent();
    supplyCurrent = talon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, voltage, temperature, statorCurrent, supplyCurrent);

    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    inputs.isConnected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(voltage, temperature, statorCurrent, supplyCurrent));

    BaseStatusSignal.refreshAll(voltage, temperature, statorCurrent, supplyCurrent);
    inputs.Voltage = voltage.getValueAsDouble();
    inputs.Temperature = temperature.getValueAsDouble();
    inputs.StatorCurrent = statorCurrent.getValueAsDouble();
    inputs.SupplyCurrent = supplyCurrent.getValueAsDouble();

    inputs.isDetected = inputs.StatorCurrent > GripperConstants.Current.freeCurrent;
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setControl(voltageControl.withOutput(voltage));
  }
}
