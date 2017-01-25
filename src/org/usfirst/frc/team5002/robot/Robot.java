package org.usfirst.frc.team5002.robot;
/**
 * Example demonstrating the Position closed-loop.
 * 
 * No buttons pressed, it should drive and steer one wheel as if it were in teleop.
 * button1 should steer the wheel forward with one set of PID gains.
 * button2 should steer the wheel backward with no PID gain.
 * No drive wheel motors while buttons pressed.
 * 
 * Be sure to select the correct feedback sensor using SetFeedbackDevice() below.
 *
 * After deploying/debugging this to your RIO, first use the stick 
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving counterclockwise the 
 * position sensor is moving in a positive direction.  If this is not the case
 * flip the boolean input to the reverseSensor() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target position.  
 *
 * Tweak the PID gains accordingly.
 */

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.StatusFrameRate;
import com.ctre.CANTalon.TalonControlMode;

public class Robot extends IterativeRobot {
	int steerTalonID = 1;
	int driveTalonID = 2;
	
	int lfOffset = 0;
	int lrOffset = 0;
	int rfOffset = 0;
	int rrOffset = 0;
	
	double kP = 0.0;
	double kI = 0.0;
	double kD = 0.0;
	double kF = 0.0;
  
	int btn1SteerValue = 511;
	int btn2SteerValue = 0;
	
	CANTalon steerTalon = new CANTalon(steerTalonID);
	CANTalon driveTalon = new CANTalon(driveTalonID);
	Joystick _joy = new Joystick(0);	
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	boolean _lastButton1 = false;
	
	
	public void robotInit() {		
		/* lets grab the Analog Encoder's starting position.  The wheel should already be positioned straight ahead.*/
		int initialRaw = steerTalon.getAnalogInRaw();
		steerTalon.setEncPosition(511);
		int offset = 511 - initialRaw;
		int absolutePosition = steerTalon.getAnalogInPosition();// if all goes as expected, this should be 511
		/* prepare line to print */
		_sb.append("\tinitialRaw:");
		_sb.append(initialRaw);
		_sb.append("\toffset:");
		_sb.append(offset);
		absolutePosition = steerTalon.getAnalogInPosition();
		_sb.append("\tnew absolutePosition:");
		_sb.append(absolutePosition);
                
        /* choose the sensor and sensor direction */
        steerTalon.setFeedbackDevice(FeedbackDevice.AnalogEncoder);
        steerTalon.reverseSensor(false);
        //_talon.configEncoderCodesPerRev(XXX), // if using FeedbackDevice.QuadEncoder
        steerTalon.configPotentiometerTurns(1); // if using FeedbackDevice.AnalogEncoder or AnalogPot

        /* set the peak and nominal outputs, 12V means full */
        steerTalon.configNominalOutputVoltage(+0f, -0f);
        steerTalon.configPeakOutputVoltage(+12f, -12f);
        /* set the allowable closed-loop error 0-1024   */      
        steerTalon.setAllowableClosedLoopErr(0);
        steerTalon.enableBrakeMode(true);
        steerTalon.changeControlMode(TalonControlMode.Position);
        /* set closed loop gains in slot0 */
        steerTalon.setProfile(0);
        steerTalon.setF(kF);
        steerTalon.setP(kP);
        steerTalon.setI(kI); 
        steerTalon.setD(kD); 
        
        steerTalon.setProfile(1);
        steerTalon.setF(0.0);
        steerTalon.setP(0.0);
        steerTalon.setI(0.0); 
        steerTalon.setD(0.0); 
        
        steerTalon.setProfile(0);
        
        /**
         * P gain is specified in throttle per error unit.  
         * For example, a value of 102 is ~9.97% (which is 102/1023) throttle per 1 unit Closed Loop Error.
         * I gain is specified in throttle per integrated error.  
         * For example, a value of 10 equates to ~0.97% for each accumulated error(IntegralAccumulator).
         * Integral accumulation is done every 1ms.
         * D gain is specified in throttle per derivative error. For example a value of 102 equates to ~9.97%(which is 102/1023)per change of Sensor Position/Velocity unit per 1ms. 
         * F gain is multiplied directly by the set point passed into the programming API.
         */
        _sb.append("\tP:");
		_sb.append(kP);
		_sb.append("\tI:");
		_sb.append(kI);
		_sb.append("\tD:");
		_sb.append(kD);
		_sb.append("\tF:");
		_sb.append(kF);
        System.out.println(_sb.toString());
	}
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	/* get stick axis */
    	double xStickValue = _joy.getAxis(AxisType.kX);
    	double yStickValue = _joy.getAxis(AxisType.kX);
    	
    	boolean button1 = _joy.getRawButton(1);
    	boolean button2 = _joy.getRawButton(2);
    	int steerSetpoint = (yStickValue==0.0 || xStickValue==0.0) ? 0 : (int) ((Math.atan2(yStickValue, xStickValue) + Math.PI) * 512.0/Math.PI);
    	int lfSetpoint = steerSetpoint + lfOffset;
    	int lrSetpont = steerSetpoint + lrOffset;
    	int rfSetpoint = steerSetpoint + rfOffset;
    	int rrSetpoint = steerSetpoint + rrOffset;
    	//int stickAngle = (int) Math.toDegrees(Math.atan2(yStickValue, xStickValue));
    	steerTalon.changeControlMode(TalonControlMode.Position);
    	/* prepare line to print */
		_sb.append("\tstickAngle:");
		_sb.append(Math.toDegrees(Math.atan2(yStickValue, xStickValue)));
        _sb.append("\twheelAngle:");
        _sb.append(Math.toDegrees(steerTalon.getPosition()));
        /* on button1 press enter closed-loop mode on target position */
        if(!_lastButton1 && button1) {
        	/* Straight Ahead Position mode - button just pressed */        	
        	steerTalon.setProfile(0);    
        	steerTalon.set(btn1SteerValue); /* hold at a single position */
        	driveTalon.set(0);       	
        }
        /* on button2 use 0 gain PID profile */
        else if(button2) {
        	steerTalon.setProfile(1);        	
        	steerTalon.set(btn2SteerValue);/* hold at a single position */
        	driveTalon.set(0);
        }   
        else
        	steerTalon.set(lfSetpoint);
        	driveTalon.set(Math.hypot(xStickValue, yStickValue));
        	_sb.append("\tClosedLoopError:");
        	_sb.append(steerTalon.getClosedLoopError());
        	_sb.append("\tsteerSpeed:");
        	_sb.append(steerTalon.getSpeed());
        
        /* print every ten loops, printing too much too fast is generally bad for performance */ 
        if(++_loops >= 10) {
        	_loops = 0;
        	System.out.println(_sb.toString());
        }
        _sb.setLength(0);
        /* save button state for on press detect */
        _lastButton1 = button1;
    }
}