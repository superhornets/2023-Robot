package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Arm {

    private SparkMaxPIDController m_pidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
    private double currentPos = 0;
    private RelativeEncoder m_encoder;
    private int driveSpeed = 4800;
    private final double STARTING_ANGLE = 0;
    private double flatAngle = 90-STARTING_ANGLE;
    
    private final double LEFT_FRONT_ANGLE = 0;
    private final double RIGHT_FRONT_ANGLE = 0;
    private final double LEFT_REAR_ANGLE = 0;
    private final double RIGHT_REAR_ANGLE = 0;
    private final double FRONT_LIMIT = 0;
    private final double LEFT_LIMIT = 0;
    private final double RIGHT_LIMIT = 0;
    private final double REAR_LIMIT = 0;



    private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);
    private final DigitalInput m_armLimitDown = new DigitalInput(0);
    private final DigitalInput m_armLimitUp = new DigitalInput(1);
    private Tower tower;
    private Grabber grabber;

    private double currentAngle = STARTING_ANGLE;
    private final double ARM_LENGTH = 0;
    private final double EXTENDER_GEAR_RATIO = 150;
    private final double TOWER_HEIGHT = 0;


    public Arm(){
        m_pidController=m_arm.getPIDController();
        m_encoder = m_arm.getEncoder();
        kP = 5e-5; 
        kI = 8e-7;
        kD = 0; 
        kIz = 0; 
        kFF = 0;
        maxVel = 2000; // rpm
        maxAcc = 1500;
        kMaxOutput = 1; 
        kMinOutput = -1;
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        m_pidController.setSmartMotionMaxAccel(maxAcc, 0);
        m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
        currentPos = m_encoder.getPosition();
    }
    public void updatePosition(){
    }
    public void setPickerUpper(Tower tower, Grabber grabber){
        this.tower = tower;
        this.grabber = grabber;
    }
    public void moveArm(double speed) {
        speed = speed * 0.2;
        if(speed == 0) {
            /*if(m_armLimitDown.get()) {
                m_arm.set(0);
            } else {*/
                m_pidController.setReference(currentPos, ControlType.kSmartMotion);            //}
        } else if(speed >0){
            /*if(m_armLimitUp.get()) {
                m_arm.set(0);
            } else {*/
                m_pidController.setReference(speed*driveSpeed, ControlType.kSmartVelocity);
                currentPos = m_encoder.getPosition();
            //}
        }
        else if(speed <0){
            /*if(m_armLimitUp.get()) {
                m_arm.set(0);
            } else {*/
                m_pidController.setReference(speed*driveSpeed, ControlType.kSmartVelocity);
                currentPos = m_encoder.getPosition();
    
            //}
        }
    
    }
    public boolean isAtHeightLimit(){
        currentAngle = m_encoder.getPosition()*2;
        if(currentAngle < flatAngle){
            return false;
        }
        else if(armYDistance() > 77){
            return true;
        }
        else{
            return false;
        }
        

    }
    public boolean isOverHeightLimit(){
        if(armYDistance()>77.5){
            return true;
        }
        else{
            return false;
        }
    }
    public String checkQuadrant(){
        if(tower.returnAngle()<LEFT_FRONT_ANGLE && tower.returnAngle()>RIGHT_FRONT_ANGLE){
            return "front";
        }
        else if(tower.returnAngle()>RIGHT_FRONT_ANGLE && tower.returnAngle()<RIGHT_REAR_ANGLE){
            return "right";
        }
        else if(tower.returnAngle() > RIGHT_REAR_ANGLE && tower.returnAngle() < LEFT_REAR_ANGLE){
            return "rear";
        }
        else{
            return "left";
        }
    }
    public boolean isAtExtentionLimit(){
        String quadrant = checkQuadrant();
        double limit = 0;
        if(quadrant == "front"){
            limit = FRONT_LIMIT;
        }
        else if(quadrant == "left"){
            limit = LEFT_LIMIT;
        }
        else if(quadrant == "right"){
            limit = RIGHT_LIMIT;
        }
        else{
            limit = REAR_LIMIT;
        }
        
        if(armXDistance() > limit-1){
            return true;
        }
        else{
            return false;
        }

    }
    public boolean isOverExtentionLimit(){
        String quadrant = checkQuadrant();
        double limit = 0;
        if(quadrant == "front"){
            limit = FRONT_LIMIT;
        }
        else if(quadrant == "left"){
            limit = LEFT_LIMIT;
        }
        else if(quadrant == "right"){
            limit = RIGHT_LIMIT;
        }
        else{
            limit = REAR_LIMIT;
        }
        
        if(armXDistance() > limit-.5){
            return true;
        }
        else{
            return false;
        }

    }

    public double armXZDistance(){
        double armX = (grabber.returnExtension()+ARM_LENGTH) * Math.sin(currentAngle+STARTING_ANGLE);
        return armX;
    }

    public double armYDistance(){
        double armY = ((grabber.returnExtension()+ARM_LENGTH) * Math.cos(currentAngle+STARTING_ANGLE-90))+TOWER_HEIGHT;
        return armY;
    }
    public double armXDistance(){
        double angle = Math.abs(tower.returnAngle()%45);
        double distance = armXZDistance()*Math.cos(angle);
        return distance;
    }
}

