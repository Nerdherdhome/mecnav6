/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import com.kauailabs.nav6.frc.BufferingSerialPort;
import com.kauailabs.nav6.frc.IMU;
import com.kauailabs.nav6.frc.IMUAdvanced;
import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.visa.VisaException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class MecanumNav6 extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    BufferingSerialPort serial_port;
    IMU imu;  // Alternatively, use IMUAdvanced for advanced features
    //IMUAdvanced imu;
    boolean first_iteration;
    CANJaguar frontLeft;
    CANJaguar frontRight;
    CANJaguar backRight;
    CANJaguar backLeft;
    Joystick joy;
    Joystick joy2;
//    Gyro gyro;
    double gyroAngle = 0, staticAngle = 0;
    NerdyPIDController pid;
    PIDRotate pidRotate;
    PIDController pidController;
    double kP = -0.002222;
    double kI = 0.00005;
    
    public void robotInit() {
        try {
            frontLeft = new CANJaguar(3);
            frontRight = new CANJaguar(4);
            backRight = new CANJaguar(5);
            backLeft = new CANJaguar(2);
            joy = new Joystick(1);
            joy2 = new Joystick(2);

            
//            gyro = new Gyro(2);
        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
        pid = new NerdyPIDController();
        try {
            serial_port = new BufferingSerialPort(57600);
            
            // You can add a second parameter to modify the 
            // update rate (in hz) from 4 to 100.  The default is 100.
            // If you need to minimize CPU load, you can set it to a
            // lower value, as shown here, depending upon your needs.
            
            // You can also use the IMUAdvanced class for advanced
            // features.

            byte update_rate_hz = 50;
            imu = new IMU(serial_port,update_rate_hz);
            //imu = new IMUAdvanced(serial_port,update_rate_hz);
            pidRotate = new PIDRotate();
            pidController = new PIDController(-0.002222, 0.000005,0, imu, pidRotate);
            pidController.setTolerance(2.0);
        } catch (VisaException ex) {
            ex.printStackTrace();
        }
        if ( imu != null ) {
            LiveWindow.addSensor("IMU", "Gyro", imu);
        }
        first_iteration = true;

    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
            
            // When calibration has completed, zero the yaw
            // Calibration is complete approaximately 20 seconds
            // after the robot is powered on.  During calibration,
            // the robot should be still
            
            boolean is_calibrating = imu.isCalibrating();
            if ( first_iteration && !is_calibrating ) {
                Timer.delay( 0.3 );
                imu.zeroYaw();
                first_iteration = false;
            }
            
            // Update the dashboard with status and orientation
            // data from the nav6 IMU
            
            SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
            SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
            SmartDashboard.putNumber(   "IMU_Yaw",              imu.getYaw());
            SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
            SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
            SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
            SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
            SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());

            // If you are using the IMUAdvanced class, you can also access the following
            // additional functions, at the expense of some extra processing
            // that occurs on the CRio processor
            
//            SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
//            SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
//            SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
//            SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());
            
            Timer.delay(0.2);
            
    }

    /**
     * This function is called periodically during operator control
     */
    double gyroConstant = 1;
    public void teleopPeriodic() {
            
            // When calibration has completed, zero the yaw
            // Calibration is complete approaximately 20 seconds
            // after the robot is powered on.  During calibration,
            // the robot should be still
            kP = SmartDashboard.getDouble("kP", kP);
            kI = SmartDashboard.getDouble("kI", kI);
            
            pidController.setPID(kP, 0, 0);
            boolean is_calibrating = imu.isCalibrating();
            if ( first_iteration && !is_calibrating ) {
                Timer.delay( 0.3 );
                imu.zeroYaw();
                first_iteration = false;
            }
            
            // Update the dashboard with status and orientation
            // data from the nav6 IMU
            
            SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
            SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
            SmartDashboard.putNumber(   "IMU_Yaw",              (imu.getYaw()));
            SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
            SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
            SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
            SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
            SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());

            // If you are using the IMUAdvanced class, you can also access the following
            // additional functions, at the expense of some extra processing
            // that occurs on the CRio processor
            
//            SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
//            SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
//            SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
//            SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());
            
            Timer.delay(0.2);
            
        ///////////////////////////////////////////////////////////
            
        //////////////////////////////////////////////////////////
        if(joy.getRawButton(4)) {
//            gyro.reset();
            imu.zeroYaw();
        }
        
        //SmartDashboard.putNumber("Robot Angle", gyroAngle);
        gyroAngle = (-imu.getYaw());
        double gyroAngleRads = gyroAngle * Math.PI / 180 * gyroConstant;
        double desiredAngle = (MathUtils.atan2(-joy.getY(), joy.getX()) + 3*Math.PI/2) % (2*Math.PI);
        double desiredRotateAngle = (MathUtils.atan2(-joy2.getY(), joy2.getX()) - Math.PI/2) % (2*Math.PI);
        double relativeAngle = (-(gyroAngleRads) + (desiredAngle) + (Math.PI/2)) % (2*Math.PI);
        double forward = Math.sin(relativeAngle);
        double strafe = Math.cos(relativeAngle);
        SmartDashboard.putNumber("Desired Angle", desiredAngle );
        SmartDashboard.putNumber("Relative Angle", relativeAngle);
//        double rotate = joy2.getX()/2;
        double unscaledJoy[] = {Math.sin(desiredAngle), Math.cos(desiredAngle)};
        double maxJoy[] = normalize(unscaledJoy, true);
        double scalar = threshhold((sqr(joy.getY()) + sqr(joy.getX())) / (sqr(maxJoy[0]) + sqr(maxJoy[1])));
        SmartDashboard.putNumber("Scalar", scalar);
        pid.setGyroAngle(gyroAngle);
        double rotate;
        pidController.setSetpoint(desiredRotateAngle * 180 / Math.PI);
        pidController.setContinuous();
        pidController.enable();
        SmartDashboard.putDouble("ANGLE ROTATE", (desiredRotateAngle * 180 / Math.PI));
        if(joy2.getRawButton(2))    {
            double rot = 0;
            if(!pidController.onTarget()){
                double x = 0.0;
                double y = 0.0;
                rotate = pidRotate.getValue();
                SmartDashboard.putDouble("ROTATE", rotate);
            }else{
                rotate = 0;
                pidController.disable();
            }
        }   else    {
            rotate = joy2.getX()/2;
        }
//        double kP = 0.1;
//        double error;
//        gyroAngleRads += 2*Math.PI;
//        if((((2*Math.PI-(desiredRotateAngle)+(gyroAngleRads))%(2*Math.PI))>Math.PI))  {
//            error = (2*Math.PI - ((2*Math.PI - (desiredRotateAngle)+gyroAngleRads)))%Math.PI;
//        }   else    {
//            error = (2*Math.PI-(2*Math.PI - (desiredRotateAngle) + (gyroAngleRads))%Math.PI) - Math.PI;
//        }
//        
//        SmartDashboard.putNumber("Error",(error)*180/Math.PI);
//        double rotate;
//        if(Math.abs(error * 180/Math.PI)> 5)    {
//            if(joy2.getRawButton(2))    {
//                rotate = error*kP;
//            }   else    {
//                rotate = joy2.getX()/2;
//            }
//        }   else    {
//            rotate = 0;
//        }
        
//        double ftLeft = (forward - strafe)*scalar + rotate;
//        double ftRight = (-forward - strafe)*scalar + rotate;
//        double bkLeft = (forward + strafe)*scalar + rotate;
//        double bkRight = (-forward + strafe)*scalar + rotate;
        double ftLeft = (forward + strafe)*scalar + rotate;
        double ftRight = (-forward + strafe)*scalar + rotate;
        double bkLeft = (forward - strafe)*scalar + rotate;
        double bkRight = (-forward - strafe)*scalar + rotate;
        
        SmartDashboard.putNumber("Strafe",strafe);
        SmartDashboard.putNumber("Forward",forward);
        
        double unnormalizedValues[] = {ftLeft, ftRight, bkLeft, bkRight};
        double output[] = normalize(unnormalizedValues, false);
        
        ftLeft = output[0];
        ftRight = output[1];
        bkLeft = output[2];
        bkRight = output[3];
        
        SmartDashboard.putNumber("Front Left" , ftLeft);
        SmartDashboard.putNumber("Front Right" , ftRight);
        SmartDashboard.putNumber("Back Left" , bkLeft);
        SmartDashboard.putNumber("Back Right" , bkRight);
        
        try{
            frontLeft.set(ftLeft);
            frontRight.set(ftRight);
            backLeft.set(bkLeft);
            backRight.set(bkRight);
        }catch(Exception e){
            System.out.println("Hey Listen");
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    
    public double sqr(double value){
        return value*value;
    }
    
    public static double threshhold(double value){
        if(value > 0){
            return Math.min(value, 1);
        }else{
            return Math.max(value, -1);
        }
    }
    
    public double[] normalize(double[] values, boolean scaleUp){
        double[] normalizedValues = new double[values.length];
        double max = Math.max(Math.abs(values[0]), Math.abs(values[1]));
        for(int i = 2; i < values.length; i++){
            max = Math.max(Math.abs(values[i]), max);
        }
        if(max < 1 && scaleUp == false) {
            for(int i = 0; i < values.length; i++){
                normalizedValues[i] = values[i];
            }
        }   else    {
            for(int i = 0; i < values.length; i++){
                normalizedValues[i] = values[i] / max;
            }
        }
        
        return normalizedValues;
    }
    
}
