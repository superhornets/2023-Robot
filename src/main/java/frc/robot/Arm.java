package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Arm {

    private SparkMaxPIDController m_pidController;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, maxAcc;
    private double currentPos = 0;
    private RelativeEncoder m_encoder;
    private int driveSpeed = 4800;
    private final double STARTING_ANGLE = 20;
    private double flatAngle = 90-STARTING_ANGLE;


    private final double EXTENSION_LIMIT = 48;
    private final double LEFT_FRONT_ANGLE = -27.84;
    private final double RIGHT_FRONT_ANGLE = 27.84;
    private final double LEFT_REAR_ANGLE = -131.78;
    private final double RIGHT_REAR_ANGLE = 131.78;
    private final double FRONT_LIMIT = 22.25+EXTENSION_LIMIT;
    private final double LEFT_LIMIT = 11.75+EXTENSION_LIMIT;
    private final double RIGHT_LIMIT = 11.75+EXTENSION_LIMIT;
    private final double REAR_LIMIT = 10.5+EXTENSION_LIMIT;



    private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);
    private final DigitalInput m_armLimitDown = new DigitalInput(0);
    private final DigitalInput m_armLimitUp = new DigitalInput(1);
    private Tower tower;
    private Grabber grabber;

    public double currentAngle = STARTING_ANGLE;
    private final double ARM_LENGTH = 46.25;
    private final double EXTENDER_GEAR_RATIO = 150;
    private final double TOWER_HEIGHT = 50.5;
    private final double GRABBER_WIDTH = 7.75;


    public Arm(){
        m_pidController=m_arm.getPIDController();
        m_encoder = m_arm.getEncoder();
        m_arm.setInverted(true);
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
        m_encoder.setPosition(0);
        updatePosition();
    }
    public void updatePosition(){
        currentPos = m_encoder.getPosition()*2+STARTING_ANGLE;
    }
    public void reseZero() {
        m_encoder.setPosition(0);
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
                m_pidController.setReference(0, ControlType.kSmartVelocity);            //}
        } else if(speed >0){
            /*if(m_armLimitUp.get()) {
                m_arm.set(0);
            } else {*/
                m_pidController.setReference(speed*driveSpeed, ControlType.kSmartVelocity);

            //}
        }
        else if(speed <0){
            /*if(m_armLimitUp.get()) {
                m_arm.set(0);
            } else {*/
                m_pidController.setReference(speed*driveSpeed, ControlType.kSmartVelocity);

    
            //}
        }
    
    }
    public boolean isAtHeightLimit(){
        if(armYDistance() > 77){
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
        double armX = (grabber.returnExtension()+ARM_LENGTH) * Math.sin(Math.toRadians(currentPos));
        return armX;
    }

     public double armYDistance(){
        double armY = Math.copySign((grabber.returnExtension()+ARM_LENGTH) *Math.sin(Math.toRadians(currentPos-90)), (currentPos-90))+TOWER_HEIGHT;
        return armY;
    }
    public double armXDistance(){
        double angle = normalizeAngle(tower.returnAngle());
        double distance = armXZDistance()*Math.cos(Math.toRadians(angle)) + GRABBER_WIDTH*Math.sin(Math.toRadians(angle));
        return distance;
    }
    public double armZDistance(){
        double angle = normalizeAngle(tower.returnAngle());
        double distance = armXZDistance()*Math.sin(Math.toRadians(angle)) + GRABBER_WIDTH*Math.cos(Math.toRadians(angle));
        return distance;
    }
    public double normalizeAngle(double angle) {
        angle = Math.abs(angle%90);
        if (angle > 45){
            angle = 90-angle;   
        }
        return angle;
    }
    public boolean isAtLeftFrame(){
        double angle = tower.returnAngle()%90;
        String quadrant = checkQuadrant();
        double limit = 0;
        if(angle > 0){
            return false;
        }
        if(quadrant == "front"){
            limit = 11.25;
        }
        else if(quadrant == "left"){
            limit = 10;
        }
        else if(quadrant == "right"){
            limit = 21.75;
        }
        else{
            limit = 11.25;
        }

        if(armZDistance()>limit){
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isAtRightFrame(){
        double angle = tower.returnAngle()%90;
        String quadrant = checkQuadrant();
        double limit = 0;
        if(angle < 0){
            return false;
        }
        if(quadrant == "front"){
            limit = 11.25;
        }
        else if(quadrant == "left"){
            limit = 21.75;
        }
        else if(quadrant == "right"){
            limit = 10;
        }
        else{
            limit = 11.25;
        }

        if(armZDistance()>limit){
            return true;
        }
        else{
            return false;
        }
    }
    public void SmartDashboard() {
        SmartDashboard.putNumber("arm x distance ", armXDistance());
        SmartDashboard.putNumber("arm y distance", armYDistance());
        SmartDashboard.putNumber("current angle", currentPos);
        SmartDashboard.putNumber("grabber extension", grabber.returnExtension());
        SmartDashboard.putNumber("arm encoder", m_encoder.getPosition());
    }
}

