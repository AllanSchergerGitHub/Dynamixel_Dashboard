/*

started to refactor the code at 9:20PM on Dec 13, 2021
https://stackoverflow.com/questions/26265002/pause-and-resume-swingworker-doinbackground

Several modes of operation:
    1) manually move the motor rotation to desired position; using visual observation of end effector to know xyz
    2) manually move the xyz variables and use math (or a precalc'd table) to derive the required motor rotation positions.
            Incorporate speed for each phase of the movement; as well as delays on some directions to control the path?
    3) move from one stored xyz to another stored xyz position by clicking on the start and end position
        3a) move to home position, etc
    4) pattern moves - ie cover a plane with a hatch pattern, etc (move back and forth over a square area to touch every point within the square.
    5) sequences of pattern moves - ie cover a plane at level z; then move to level z+1 and cover the plan; etc
    6) sequences of moves involving the rover and the arm. ie. cover one square area with rover arm; move the rover forward; repeat.
    7) these examples could be expanded to include visual or machine learning overvations to either avoid or go to certain areas. (ie avoid good plants; go to bad plants).

Questions:
    1) do I need to create a queue of commands that need to be completed one after another sequentially? Or can I interupt the current queue of commands before the queue is completed?

Layers:
    Goals
    Path Planning
        incremental positions; speed; how will we know if we reached one goal before going to the next goal?
    Monitoring and Control

Fixes:
    some of the button action events have too much going on inside them. 
    Limit the action to the setting of variables (ie target positions) and let the motion happen in the doinbackground section? or does this affect the GUI?
    May need to incorporate some threads to carry out actions. Also, for the arm dynamixels there is a readall capability that is faster. https://github.com/ROBOTIS-GIT/DynamixelSDK/issues/506

 */
package javaapplication1;

import org.apache.commons.lang3.ArrayUtils;
import com.sun.jna.Native;
import java.awt.Color;
import static java.lang.Math.abs;
import static java.lang.Thread.sleep;
import java.text.SimpleDateFormat;
import java.util.Date;

import java.awt.BorderLayout;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.prefs.BackingStoreException;
import java.util.prefs.Preferences;
import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.BorderFactory;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.SwingWorker;

public class JFrameClass extends javax.swing.JFrame {
    Preferences prefs;
    Preferences prefsGlobal; // technically this isn't 'Global' but it has a broad scope.
    public String Batch_time_stamp_into_mysql = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss.SSS").format(new Date());
    String time_stamp_into_mysql = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss.SSS").format(new Date());
    Dynamixel dynamixel = new Dynamixel();
    String DEVICENAME                   = "COM3";   
    
    short position1 = 0;
    short position2 = 0;
    short position3 = 0;
    short position4 = 0;
    Double ChangeBand = 0.01; // if the change in position is within this % range a new position is not printed to screen to reduce printouts
    
    private String scenarioSaveName = "initialized";
    private int recordPositions = 0; // 0 means do not record; 1 means yes to record
    private int recordOnce = 0; // records one record to mySQL and then stops recording.
    int HomePosMtr1 = 200; // HomePosition is the "ideal" place to send the arm when idle
    int HomePosMtr2 = 770;
    int HomePosMtr3 = 500;
    int HomePosMtr4 = 520;
    
    String ActualsOrTarget = "Undecided"; // which values from the prior session should be used to move the arm to? Prior Actuals or Prior Targets? Start with Undecided and wait for user input.
    int TargetPosMtr1 = HomePosMtr1; // TargetPos is the next place the arm should move to
    int TargetPosMtr2 = HomePosMtr2;
    int TargetPosMtr3 = HomePosMtr3;
    int TargetPosMtr4 = HomePosMtr4;
    int port_num = 0;
    int load1 = 0;
    int load2 = 0;
    int load3 = 0;
    int load4 = 0;
    int load1_movingaverage = 0;
    int load2_movingaverage = 0;
    int load3_movingaverage = 0;
    int load4_movingaverage = 0;
    //int[] load_movingaverage_array = new int[]{0, 0, 0, 0};    
    List<Long> load1_movingaverage_list = new ArrayList<>();
    List<Long> load2_movingaverage_list = new ArrayList<>();
    List<Long> load3_movingaverage_list = new ArrayList<>();
    List<Long> load4_movingaverage_list = new ArrayList<>();
    int mGoalPosLabelMotor1             = 0;
    int PROTOCOL_VERSION                = 1;
    int group_num = dynamixel.groupBulkRead(port_num, PROTOCOL_VERSION);
    byte DXL_ID                         = 6;    
    short ADDR_MX_TORQUE_ENABLE         = 24;
    short ADDR_MX_GOAL_POSITION         = 30;
    byte TORQUE_ENABLE                  = 1;
    int motorNumber                     = 0;
    short dxl_goal_position             = 300;
    Boolean FollowDriverFlag = false;
    Boolean RunEndEffectorContinuously = false;
    
    MotorClass motor1 = new MotorClass();
    MotorClass motor2 = new MotorClass();
    MotorClass motor3 = new MotorClass();
    MotorClass motor4 = new MotorClass();
    
    
    public void startupMethod(){
        System.out.println("did i print this?");
        JFrameClassUpdater mJFrameClassUpdater = new JFrameClassUpdater();
        mJFrameClassUpdater.execute();
        initComponents();
        prefsGlobal = Preferences.userRoot().node(this.getClass().getName());
        jTextFieldSavedPosition1.setText(prefsGlobal.get("SavedPosition1_Name", "default"));
        System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath() + " " + prefsGlobal.get("SavedPosition1_Name", "default2"));
    }
    
    /*
        Returns zero if the list size is zero to avoid div by zero error
    */
    public Long movingAverage(List<Long> myList) {
        Long value = (long)0;
        value = value.valueOf(value);
        for(int i = 0; i<myList.size(); i++){
            value = value + myList.get(i);
        }
        if(myList.size()!=0) {
            value = value / myList.size();}
        else {myList.size();}
        return value; 
    }
    
    final class JFrameClassUpdater extends SwingWorker<Void, Void> {
        @Override
        protected Void doInBackground()
                {
                    System.out.println("Make sure power is connected and working. Can I automate this validation step?");
                    
                    while (true) { 
                    Native.setProtected(true);
                    port_num = dynamixel.portHandler(DEVICENAME);
                    dynamixel.packetHandler();
                    dynamixel.closePort(port_num);         
                    dynamixel.openPort(port_num); 
                    System.err.println("BaudRate: "+1000000);
                    dynamixel.setBaudRate(port_num, 1000000);
                    if(dynamixel.openPort(port_num)){
                        System.err.println("Port Open: "+port_num);
                    }
                    else
                    {
                      System.err.println("Failed to open the port!");
                    }

                    motorNumber = 0;
                    motor1.setBatchTime(Batch_time_stamp_into_mysql);
                    motor1.makeDynamix(dynamixel, motorNumber);
                    motor1.setMovingSpeed();
                    motor1.setTorque();

                    motorNumber = 1;
                    motor2.setBatchTime(Batch_time_stamp_into_mysql);
                    motor2.makeDynamix(dynamixel, motorNumber);
                    motor2.setMovingSpeed();
                    motor2.setTorque();

                    motorNumber = 2; // wrist
                    motor3.setBatchTime(Batch_time_stamp_into_mysql);
                    motor3.makeDynamix(dynamixel, motorNumber);
                    motor3.setMovingSpeed();
                    motor3.setTorque();
                    System.err.println("motor3 pos: "+motor3.readPosition());

                    try {
                        sleep(25);
                    } catch (InterruptedException ex) {
                        Logger.getLogger(MotorClass.class.getName()).log(Level.SEVERE, null, ex);
                    }

                    motorNumber = 3; // shoulder
                    motor4.setBatchTime(Batch_time_stamp_into_mysql);
                    motor4.makeDynamix(dynamixel, motorNumber);
                    motor4.setMovingSpeed();
                    motor4.setTorque();
                    System.err.println("motor4 pos: "+motor4.readPosition());

                    try {
                        sleep(400);
                    } catch (InterruptedException ex) {
                        Logger.getLogger(MotorClass.class.getName()).log(Level.SEVERE, null, ex);
                    }

                    //prefs = Preferences.userRoot().node(this.getClass().getName());
                    //prefs.putInt("motor1ActualPosition", position1);
                    //prefs.putInt("motor1TargetPosition", TargetPosMtr1);

                    System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " Last Target1 from last session "+prefsGlobal.getInt("motor1TargetPosition", 99999));
                    System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " Last Target2 from last session "+prefsGlobal.getInt("motor2TargetPosition", 99999));
                    System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " Last Target3 from last session "+prefsGlobal.getInt("motor3TargetPosition", 99999));
                    System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " Last Target4 from last session "+prefsGlobal.getInt("motor4TargetPosition", 99999));

                    System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " Last Actuals1 from last session "+prefsGlobal.getInt("motor1ActualPosition", 99999));
                    System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " Last Actuals2 from last session "+prefsGlobal.getInt("motor2ActualPosition", 99999));
                    System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " Last Actuals3 from last session "+prefsGlobal.getInt("motor3ActualPosition", 99999));
                    System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " Last Actuals4 from last session "+prefsGlobal.getInt("motor4ActualPosition", 99999));

                    jRadioButtonPriorActualPositions.setSelected(true);
                    
                    int waitForInput = 0;
                    while(ActualsOrTarget=="Undecided"){
                        
                           try {
                               sleep(1000);
                           } catch (InterruptedException ex) {
                               Logger.getLogger(JFrameClass.class.getName()).log(Level.SEVERE, null, ex);
                           }
                           if(ActualsOrTarget!="Undecided"){
                               System.out.println("Using " + ActualsOrTarget + " as the desired starting point.");
                               break;
                           }
                           waitForInput = waitForInput + 1;
                           System.out.println("Waiting for input on which starting position to use (Prior Actuals or Prior Targets or Home) " + waitForInput 
                                    + " ActualButton: " +jRadioButtonPriorActualPositions.isSelected()
                                    + " TargetButton: " +jRadioButtonPriorTargetPositions.isSelected()
                                    + " HomeButton: " +jRadioButtonSavedPositions1.isSelected());
                           
                    
                    }
                    
                    if(ActualsOrTarget == "Actuals"){
                        TargetPosMtr1 = prefsGlobal.getInt("motor1ActualPosition", 99999);
                        TargetPosMtr2 = prefsGlobal.getInt("motor2ActualPosition", 99999);
                        TargetPosMtr3 = prefsGlobal.getInt("motor3ActualPosition", 99999);
                        TargetPosMtr4 = prefsGlobal.getInt("motor4ActualPosition", 99999);
                    }
                    
                    if(ActualsOrTarget == "Target"){
                        TargetPosMtr1 = prefsGlobal.getInt("motor1TargetPosition", 99999);
                        TargetPosMtr2 = prefsGlobal.getInt("motor2TargetPosition", 99999);
                        TargetPosMtr3 = prefsGlobal.getInt("motor3TargetPosition", 99999);
                        TargetPosMtr4 = prefsGlobal.getInt("motor4TargetPosition", 99999);
                    }
                    
                    if(TargetPosMtr1>1300 || TargetPosMtr1<=0){TargetPosMtr1=HomePosMtr1;} // if the value read from the stored preferences from a prior session is outside the range of the dynamixel assume it is an error and use the home position instead.
                    if(TargetPosMtr2>1300 || TargetPosMtr2<=0){TargetPosMtr2=HomePosMtr2;} // if the value read from the stored preferences from a prior session is outside the range of the dynamixel assume it is an error and use the home position instead.
                    if(TargetPosMtr3>1300 || TargetPosMtr3<=0){TargetPosMtr3=HomePosMtr3;} // if the value read from the stored preferences from a prior session is outside the range of the dynamixel assume it is an error and use the home position instead.
                    if(TargetPosMtr4>1300 || TargetPosMtr4<=0){TargetPosMtr4=HomePosMtr4;} // if the value read from the stored preferences from a prior session is outside the range of the dynamixel assume it is an error and use the home position instead.
                    
                    dxl_goal_position = (short)TargetPosMtr1; // end effector
                    motor1.moveMotor(dxl_goal_position);

                    dxl_goal_position = (short)TargetPosMtr2; // elbow
                    motor2.moveMotor(dxl_goal_position);

                    dxl_goal_position = (short)TargetPosMtr3; // wrist
                    motor3.moveMotor(dxl_goal_position);

                    dxl_goal_position = (short)TargetPosMtr4; // shoulder
                    motor4.moveMotor(dxl_goal_position);

                    System.out.println("------------------------------------------------------------------ end of jclass");
                    jSlider1_ActualTemp.setOpaque(true);
                    jSlider2_ActualTemp.setOpaque(true);
                    jSlider3_ActualTemp.setOpaque(true);
                    jSlider4_ActualTemp.setOpaque(true);

                    int x1 = 0;

                    while(true){
                        x1 = x1 + 1;
                        try {
                            System.err.println("made it here");
                            sleep(450); // give motors time to connect before reading from them
                        } catch (InterruptedException ex) {
                            Logger.getLogger(JFrameClass.class.getName()).log(Level.SEVERE, null, ex);
                        }
                        int x = 0;
                        while(true) {
                            x = x + 1;
                            try {    
                                if (FollowDriverFlag) { 
                                    // if this flag is true then arm 2 will follow the manual movement of arm 1

                                    // this section is designed to prevent extreme motions based on potentially erroneous readings
//                                        if (position5 < oldPosition5 *.8 && position5 > 0) {
//                                            position5 = (short) (oldPosition5 * 0.8);
//                                            //System.out.println("POSITION5 " + position5);
//                                        }
//                                        if (position6 < oldPosition6 *.8 && position6 > 0 ) {
//                                            position6 = (short) (oldPosition6 * 0.8);
//                                            //System.out.println("POSITION6 " + position6);
//                                        }
//                                        if (position7 < oldPosition7 *.8 && position7 > 0 ) {
//                                            position7 = (short) (oldPosition7 * 0.8);
//                                        }
//                                        if (position8 < oldPosition8 *.8 && position8 > 0 ) {
//                                            position8 = (short) (oldPosition8 * 0.8);
//                                        }
//
//                                        if (position5 > oldPosition5 *1.2 && position5 > 0) {
//                                            position5 = (short) (oldPosition5 * 1.2);
//                                            //System.out.println("POSITION5 TWO " + position5);
//                                        }
//                                        if (position6 > oldPosition6 *1.2 && position6 > 0) {
//                                            position6 = (short) (oldPosition6 * 1.2);
//                                        }
//                                        if (position7 > oldPosition7 *1.2 && position7 > 0) {
//                                            position7 = (short) (oldPosition7 * 1.2);
//                                        }
//                                        if (position8 > oldPosition8 *1.2 && position8 > 0) {
//                                            position8 = (short) (oldPosition8 * 1.2);
//                                        }
//                                        if (position5 == 0) {
//                                            position5 = (short) (oldPosition5);
//                                            //System.out.println("POSITION5 Three " + position5);
//                                        }
//                                        if (position6 == 0) {
//                                            position6 = (short) (oldPosition6);
//                                            //System.out.println("POSITION5 Three " + position5);
//                                        }
//                                        if (position7 == 0) {
//                                            position7 = (short) (oldPosition7);
//                                            //System.out.println("POSITION5 Three " + position5);
//                                        }
//                                        if (position8 == 0) {
//                                            position8 = (short) (oldPosition8);
//                                            //System.out.println("POSITION5 Three " + position5);
//                                        }
//
//                                        oldPosition5 = position5;
//                                        oldPosition6 = position6;
//                                        oldPosition7 = position7;
//                                        oldPosition8 = position8;
//
//                                    String time_stamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss.SSS").format(new Date());
//                                    System.out.println(time_stamp);
//                                    System.out.print("move motor 5 ");
//                                    motor1.moveMotor((short)(position5));
//                                    time_stamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss.SSS").format(new Date());
//                                    System.out.println(time_stamp);
//                                    System.out.print("move motor 6 ");
//                                    motor2.moveMotor((short)(position6));
//                                    time_stamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss.SSS").format(new Date());
//                                    System.out.println(time_stamp);
//                                    System.out.print("move motor 7 ");
//                                    motor3.moveMotor((short)(position7));
//                                    time_stamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss.SSS").format(new Date());
//                                    System.out.println(time_stamp);
//                                    System.out.print("move motor 8 ");
//                                    motor4.moveMotor((short)(position8));
//                                    time_stamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss.SSS").format(new Date());
//                                    System.out.println("Last time_stamp " + time_stamp);
                                    }
                                
                                
                                    position1 = motor1.readPosition(); // error here may mean this is starting before devices are connected.
                                    if(RunEndEffectorContinuously){
                                            if(position1<300){TargetPosMtr1=950;} // if less than 300 then switch to a large target
                                            if(position1>800){TargetPosMtr1=100;} // if over 800 then switch to a lower target; otherwise (implied) keep moving toward the existing target
                                            jScrollBarMtr1_Target.setValue(TargetPosMtr1);
                                            motor1.moveMotor((short)TargetPosMtr1);
                                            TargetPosMtr1 = (int)TargetPosMtr1;
                                            jLabelMotor1.setText(""+TargetPosMtr1);
                                        }
                                    
                                    jTextField_PositionMtr1.setText(position1+"");
                                    sleep(1);
                                    Long load1 = motor1.readLoad();
                                    
                                    if(load1>=-1023 & load1<=1023){
                                        load1_movingaverage_list.add(load1);
                                    }
                                    if(load1_movingaverage_list.size()>9) {
                                        load1_movingaverage_list.remove(0);
                                    }
                                    Long movingAverage1 = movingAverage(load1_movingaverage_list);
                                    
                                    jTextField_LoadEndEffector.setText(movingAverage1 + "");
                                    if (load1 == 16959){
                                        jLabelMotor1.setOpaque(true);
                                        jLabelMotor1.setBackground(Color.red);
                                    }
                                    else {
                                        jLabelMotor1.setOpaque(true);
                                        jLabelMotor1.setBackground(Color.green);
                                    }
                                    int Temp1C = motor1.readPresentTemperature(); // this also has read errors that appear to be high temps
                                    int Temp1F = Temp1C*9/5+32;
                                    jSlider1_ActualTemp.setValue(Temp1F);
                                    jTextFieldTempMtr1.setText(""+Temp1F);
                                    if (Temp1F>140) {
                                        //System.err.println("Fahrenheit: " + Temp1F + " motor1 Temperature is high " + Temp1C);
                                            jSlider1_ActualTemp.setBackground(Color.red);
                                        }
                                        else {
                                            jSlider1_ActualTemp.setBackground(Color.green);
                                        }
                                    sleep(2); // RxTx takes 100 ms? need to reread documentation.
                                    position2 = motor2.readPosition();	
                                    jTextField_PositionMtr2.setText(position2+"");	
                                    Long load2 = motor2.readLoad();
                                    
                                    if(load2>=-1023 & load2<=1023){
                                        load2_movingaverage_list.add(load2);
                                    }
                                    if(load2_movingaverage_list.size()>9) {
                                        load2_movingaverage_list.remove(0);
                                    }
                                    Long movingAverage2 = movingAverage(load2_movingaverage_list);
                                    jTextField_Load_2.setText(movingAverage2 + "");
                                    
                                    if (load2 == 999999){	
                                        jLabelMotor2.setOpaque(true);
                                        jLabelMotor2.setBackground(Color.red);	
                                    }	
                                    else {	
                                        jLabelMotor2.setOpaque(true);
                                        jLabelMotor2.setBackground(Color.green);	
                                    }	
                                    int Temp2C = motor2.readPresentTemperature(); // this also has read errors that appear to be high temps
                                    int Temp2F = Temp2C*9/5+32;
                                    jSlider2_ActualTemp.setValue(Temp2F);
                                    jTextFieldTempMtr2.setText(""+Temp2F);
                                    if (Temp2F>140) {
                                        //System.err.println("Fahrenheit: " + Temp2F + " motor2 Temperature is high " + Temp2C);
                                            jSlider2_ActualTemp.setBackground(Color.RED);
                                        }
                                        else {
                                            jSlider2_ActualTemp.setBackground(Color.GREEN);
                                        }
                                    sleep(2); // RxTx takes 100 ms? need to reread documentation.	
                                    position3 = motor3.readPosition();	
                                    jTextField_PositionMtr3.setText(position3+"");	
                                    Long load3 = motor3.readLoad();
                                    
                                    if(load3>=-1023 & load3<=1023){
                                        load3_movingaverage_list.add(load3);
                                    }
                                    if(load3_movingaverage_list.size()>9) {
                                        load3_movingaverage_list.remove(0);
                                    }
                                    Long movingAverage3 = movingAverage(load3_movingaverage_list);
                                    //System.out.println(movingAverage3 + " " + load3_movingaverage_list + " " + load3_movingaverage_list.size() + " most recent load = " + load3);
                                    
                                    jTextField_Load_3.setText(movingAverage3 + "");
                                    
                                    if (load3 == 999999){
                                        jLabelMotor3.setOpaque(true);
                                        jLabelMotor3.setBackground(Color.RED);	
                                    }	
                                    else {
                                        jLabelMotor3.setOpaque(true);
                                        jLabelMotor3.setBackground(Color.GREEN);	
                                    }
                                    int Temp3C = motor3.readPresentTemperature(); // this also has read errors that appear to be high temps
                                    int Temp3F = Temp3C*9/5+32;
                                    jSlider3_ActualTemp.setValue(Temp3F);
                                    jTextFieldTempMtr3.setText(""+Temp3F);
                                    if (Temp3F>140) {
                                        //System.err.println("Fahrenheit: " + Temp3F + " motor3 Temperature is high " + Temp3C);
                                            jSlider3_ActualTemp.setBackground(Color.RED);
                                        }
                                        else {
                                            jSlider3_ActualTemp.setBackground(Color.GREEN);
                                        }
                                    sleep(2); // RxTx takes 100 ms? need to reread documentation.
                                    position4 = motor4.readPosition();	
                                    jTextField_PositionMtr4.setText(position4+"");	
                                    jScrollBarMtr4_Actual.setValue(position4);
                                    long load4 = motor4.readLoad();
                                    
                                    if(load4>=-1023 & load4<=1023){
                                        load4_movingaverage_list.add(load4);
                                    }
                                    if(load4_movingaverage_list.size()>9) {
                                        load4_movingaverage_list.remove(0);
                                    }
                                    Long movingAverage4 = movingAverage(load4_movingaverage_list);
                                    
                                    jTextField_Load_4.setText(movingAverage4 + "");
                                    if (load4 == 999999){
                                        jLabelMotor4.setOpaque(true);
                                        jLabelMotor4.setBackground(Color.red);	
                                    }	
                                    else {	
                                        jLabelMotor4.setOpaque(true);
                                        jLabelMotor4.setBackground(Color.green);	
                                    }	
                                    int Temp4C = motor4.readPresentTemperature(); // this also has read errors that appear to be high temps
                                    int Temp4F = Temp4C*9/5+32;
                                    jSlider4_ActualTemp.setValue(Temp4F);
                                    jTextFieldTempMtr4.setText(""+Temp4F);
                                    if (Temp4F>140) {
                                        //System.err.println("Fahrenheit: " + Temp4F + " motor4 Temperature is high " + Temp4C);
                                            jSlider4_ActualTemp.setBackground(Color.RED);
                                        }
                                        else {
                                            jSlider4_ActualTemp.setBackground(Color.GREEN);
                                        }

                                time_stamp_into_mysql = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss.SSS").format(new Date());
                                scenarioSaveName = jTextFieldRecordScenarioName.getText();

                                if(position1!=0 && position2!=0 && position3!=0 && position4!=0){ // only record when positions are not zero	
                                     if(recordPositions==1 || recordOnce == 1){	
                                        recordOnce = 0;	
                                    }	
                                }                    

                                try {
                                    Thread.sleep(50);
                                } catch (InterruptedException ex) {
                                    Logger.getLogger(JFrameClass.class.getName()).log(Level.SEVERE, null, ex);
                                }
                            } catch (InterruptedException ex) {
                                Logger.getLogger(JFrameClass.class.getName()).log(Level.SEVERE, null, ex);
                            
                            }
                            catch(Exception ex1){
                                System.err.println("Big Fail Here?");
                                Logger.getLogger(JFrameClass.class.getName()).log(Level.SEVERE, null, ex1);
                            }
                        
                            //prefs = Preferences.userRoot().node(this.getClass().getName());
                            //prefs = Preferences.userRoot();
                            
                            if(position1!=0 && position2!=0 && position3!=0 && position4!=0){ // only record when positions are not zero; zeros occur when shutting down and other times.	
                                prefsGlobal.putInt("motor1ActualPosition", position1);
                                prefsGlobal.putInt("motor2ActualPosition", position2);
                                prefsGlobal.putInt("motor3ActualPosition", position3);
                                prefsGlobal.putInt("motor4ActualPosition", position4);
                                }
                            
                            prefsGlobal.putInt("motor1TargetPosition", TargetPosMtr1);
                            prefsGlobal.putInt("motor2TargetPosition", TargetPosMtr2);
                            prefsGlobal.putInt("motor3TargetPosition", TargetPosMtr3);
                            prefsGlobal.putInt("motor4TargetPosition", TargetPosMtr4);
//                            System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " 'motor1 motor1ActualPosition from last session '"+prefsGlobal.getInt("motor1ActualPosition", 99999));
//                            System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " 'motor1 motor2ActualPosition from last session '"+prefsGlobal.getInt("motor2ActualPosition", 99999));
//                            System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " 'motor1 motor3ActualPosition from last session '"+prefsGlobal.getInt("motor3ActualPosition", 99999));
//                            System.out.println("prefsGlobal.name() " + prefsGlobal.name()+" "+ prefsGlobal.absolutePath()+ " 'motor1 motor4ActualPosition from last session '"+prefsGlobal.getInt("motor4ActualPosition", 99999));
                        }
                    }
                }
            }
        };    
            
    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        buttonGroup1 = new javax.swing.ButtonGroup();
        jButtonMoveMtr1ToPresetHigh = new javax.swing.JButton();
        jButtonMoveMtr1ToPresetLow = new javax.swing.JButton();
        jScrollBarMtr1_Target = new javax.swing.JScrollBar();
        jLabelMotor1 = new javax.swing.JLabel();
        jScrollBarMtr2_Target = new javax.swing.JScrollBar();
        jScrollBarMtr3_Target = new javax.swing.JScrollBar();
        jScrollBarMtr4_Target = new javax.swing.JScrollBar();
        jLabelMotor2 = new javax.swing.JLabel();
        jButtonStepUpMtr1 = new javax.swing.JButton();
        jButtonStepDownMtr1 = new javax.swing.JButton();
        jButtonExit = new javax.swing.JButton();
        jToggleButtonMtr1 = new javax.swing.JToggleButton();
        jButtonResetTorqueMtr1 = new javax.swing.JButton();
        jButtonMoveMtr2ToPresetHigh = new javax.swing.JButton();
        jButtonStepUpMtr2 = new javax.swing.JButton();
        jButtonStepDownMtr2 = new javax.swing.JButton();
        jButtonMoveMtr2ToPresetLow = new javax.swing.JButton();
        jToggleButtonMtr2 = new javax.swing.JToggleButton();
        jButtonResetTorqueMtr2 = new javax.swing.JButton();
        jLabelMotor3 = new javax.swing.JLabel();
        jLabelMotor4 = new javax.swing.JLabel();
        jButtonMoveMtr3ToPresetHigh = new javax.swing.JButton();
        jButtonStepUpMtr3 = new javax.swing.JButton();
        jButtonStepDownMtr3 = new javax.swing.JButton();
        jButtonMoveMtr3ToPresetLow = new javax.swing.JButton();
        jToggleButtonMtr3 = new javax.swing.JToggleButton();
        jButtonResetTorqueMtr3 = new javax.swing.JButton();
        jButtonMoveMtr4ToPresetHigh = new javax.swing.JButton();
        jButtonStepUpMtr4 = new javax.swing.JButton();
        jButtonStepDownMtr4 = new javax.swing.JButton();
        jButtonMoveMtr4ToPresetLow = new javax.swing.JButton();
        jToggleButtonMtr4 = new javax.swing.JToggleButton();
        jButtonResetTorqueMtr4 = new javax.swing.JButton();
        jButtonEnableTorqueAllMotors = new javax.swing.JButton();
        jButtonDisableTorqueAllMotors = new javax.swing.JButton();
        jToggleButtonRecorYorN = new javax.swing.JToggleButton();
        jButtonRecord1Position = new javax.swing.JButton();
        jButtonRunASequence1 = new javax.swing.JButton();
        jTextFieldRecordScenarioName = new javax.swing.JTextField();
        jButtonRunASequence2 = new javax.swing.JButton();
        jTextFieldSpeedMotor1 = new javax.swing.JTextField();
        jTextFieldSpeedMotor2 = new javax.swing.JTextField();
        jTextFieldSpeedMotor3 = new javax.swing.JTextField();
        jTextFieldSpeedMotor4 = new javax.swing.JTextField();
        jLabel1 = new javax.swing.JLabel();
        jTextField_IncrementMtr1 = new javax.swing.JTextField();
        jTextField_IncrementMtr2 = new javax.swing.JTextField();
        jTextField_IncrementMtr3 = new javax.swing.JTextField();
        jTextField_IncrementMtr4 = new javax.swing.JTextField();
        jLabel_Increment = new javax.swing.JLabel();
        jTextField_PositionMtr4 = new javax.swing.JTextField();
        jTextField_PositionMtr1 = new javax.swing.JTextField();
        jTextField_PositionMtr2 = new javax.swing.JTextField();
        jTextField_PositionMtr3 = new javax.swing.JTextField();
        jLabel2 = new javax.swing.JLabel();
        jLabel3 = new javax.swing.JLabel();
        jToggleButtonPrintDebugging = new javax.swing.JToggleButton();
        jLabel4 = new javax.swing.JLabel();
        jToggleButton_FollowDriverButton = new javax.swing.JToggleButton();
        jLabel5 = new javax.swing.JLabel();
        jLabel6 = new javax.swing.JLabel();
        jLabel7 = new javax.swing.JLabel();
        jTextField_LoadEndEffector = new javax.swing.JTextField();
        jTextField_Load_2 = new javax.swing.JTextField();
        jTextField_Load_4 = new javax.swing.JTextField();
        jTextField_Load_3 = new javax.swing.JTextField();
        jLabel8 = new javax.swing.JLabel();
        jScrollBarMtr4_Actual = new javax.swing.JScrollBar();
        jSlider4_ActualTemp = new javax.swing.JSlider();
        jSlider1_ActualTemp = new javax.swing.JSlider();
        jSlider2_ActualTemp = new javax.swing.JSlider();
        jSlider3_ActualTemp = new javax.swing.JSlider();
        jScrollPane1 = new javax.swing.JScrollPane();
        jTextArea1 = new javax.swing.JTextArea();
        jTextField2 = new javax.swing.JTextField();
        jTextField3 = new javax.swing.JTextField();
        jTextField4 = new javax.swing.JTextField();
        jToggleButton1 = new javax.swing.JToggleButton();
        jTextFieldTempMtr4 = new javax.swing.JTextField();
        jTextFieldTempMtr3 = new javax.swing.JTextField();
        jTextFieldTempMtr2 = new javax.swing.JTextField();
        jTextFieldTempMtr1 = new javax.swing.JTextField();
        jRadioButtonPriorActualPositions = new javax.swing.JRadioButton();
        jRadioButtonSavedPositions1 = new javax.swing.JRadioButton();
        jButtonGoToDesiredStartPosition = new javax.swing.JButton();
        jRadioButtonPriorTargetPositions = new javax.swing.JRadioButton();
        jRadioButtonHomePositions1 = new javax.swing.JRadioButton();
        jTextFieldSavedPosition1 = new javax.swing.JTextField();
        jButtonUpdateSavedPosition1 = new javax.swing.JButton();
        jButtonListAllSavedPositions = new javax.swing.JButton();
        jButtonClearThisSavedPreference = new javax.swing.JButton();
        jScrollPane2 = new javax.swing.JScrollPane();
        jListSavedPositions = new javax.swing.JList<>();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        getContentPane().setLayout(new org.netbeans.lib.awtextra.AbsoluteLayout());

        jButtonMoveMtr1ToPresetHigh.setText("Max Up  ");
        jButtonMoveMtr1ToPresetHigh.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonMoveMtr1ToPresetHighActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonMoveMtr1ToPresetHigh, new org.netbeans.lib.awtextra.AbsoluteConstraints(188, 350, 84, 58));

        jButtonMoveMtr1ToPresetLow.setText("Max Down");
        jButtonMoveMtr1ToPresetLow.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonMoveMtr1ToPresetLowActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonMoveMtr1ToPresetLow, new org.netbeans.lib.awtextra.AbsoluteConstraints(188, 470, 84, 51));

        jScrollBarMtr1_Target.setBlockIncrement(20);
        jScrollBarMtr1_Target.setMaximum(1000);
        jScrollBarMtr1_Target.setValue(390);
        jScrollBarMtr1_Target.setVisibleAmount(20);
        jScrollBarMtr1_Target.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                jScrollBarMtr1_TargetAdjustmentValueChanged(evt);
            }
        });
        getContentPane().add(jScrollBarMtr1_Target, new org.netbeans.lib.awtextra.AbsoluteConstraints(220, 53, 30, 130));

        jLabelMotor1.setText("Position");
        getContentPane().add(jLabelMotor1, new org.netbeans.lib.awtextra.AbsoluteConstraints(210, 280, 57, 30));

        jScrollBarMtr2_Target.setBlockIncrement(20);
        jScrollBarMtr2_Target.setMaximum(1000);
        jScrollBarMtr2_Target.setMinimum(700);
        jScrollBarMtr2_Target.setValue(850);
        jScrollBarMtr2_Target.setVisibleAmount(20);
        jScrollBarMtr2_Target.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                jScrollBarMtr2_TargetAdjustmentValueChanged(evt);
            }
        });
        getContentPane().add(jScrollBarMtr2_Target, new org.netbeans.lib.awtextra.AbsoluteConstraints(340, 53, 30, 130));

        jScrollBarMtr3_Target.setBlockIncrement(20);
        jScrollBarMtr3_Target.setMaximum(720);
        jScrollBarMtr3_Target.setMinimum(340);
        jScrollBarMtr3_Target.setValue(500);
        jScrollBarMtr3_Target.setVisibleAmount(20);
        jScrollBarMtr3_Target.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                jScrollBarMtr3_TargetAdjustmentValueChanged(evt);
            }
        });
        getContentPane().add(jScrollBarMtr3_Target, new org.netbeans.lib.awtextra.AbsoluteConstraints(480, 53, 33, 130));

        jScrollBarMtr4_Target.setBlockIncrement(20);
        jScrollBarMtr4_Target.setMaximum(-385);
        jScrollBarMtr4_Target.setMinimum(-700);
        jScrollBarMtr4_Target.setToolTipText("");
        jScrollBarMtr4_Target.setUnitIncrement(-1);
        jScrollBarMtr4_Target.setValue(-500);
        jScrollBarMtr4_Target.addAdjustmentListener(new java.awt.event.AdjustmentListener() {
            public void adjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {
                jScrollBarMtr4_TargetAdjustmentValueChanged(evt);
            }
        });
        getContentPane().add(jScrollBarMtr4_Target, new org.netbeans.lib.awtextra.AbsoluteConstraints(620, 53, 36, 130));

        jLabelMotor2.setText("Position");
        getContentPane().add(jLabelMotor2, new org.netbeans.lib.awtextra.AbsoluteConstraints(340, 280, 67, 32));

        jButtonStepUpMtr1.setText("step up");
        jButtonStepUpMtr1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonStepUpMtr1ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonStepUpMtr1, new org.netbeans.lib.awtextra.AbsoluteConstraints(185, 410, 90, -1));

        jButtonStepDownMtr1.setText("step down");
        jButtonStepDownMtr1.setPreferredSize(new java.awt.Dimension(72, 22));
        jButtonStepDownMtr1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonStepDownMtr1ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonStepDownMtr1, new org.netbeans.lib.awtextra.AbsoluteConstraints(185, 440, 90, -1));

        jButtonExit.setText("Exit");
        jButtonExit.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonExitActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonExit, new org.netbeans.lib.awtextra.AbsoluteConstraints(430, 610, 175, 55));

        jToggleButtonMtr1.setText("Lower Torque");
        jToggleButtonMtr1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jToggleButtonMtr1ActionPerformed(evt);
            }
        });
        getContentPane().add(jToggleButtonMtr1, new org.netbeans.lib.awtextra.AbsoluteConstraints(180, 530, 110, -1));

        jButtonResetTorqueMtr1.setText("Reset Torque to Normal");
        jButtonResetTorqueMtr1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonResetTorqueMtr1ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonResetTorqueMtr1, new org.netbeans.lib.awtextra.AbsoluteConstraints(180, 570, 100, -1));

        jButtonMoveMtr2ToPresetHigh.setText("Max Up  ");
        jButtonMoveMtr2ToPresetHigh.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonMoveMtr2ToPresetHighActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonMoveMtr2ToPresetHigh, new org.netbeans.lib.awtextra.AbsoluteConstraints(333, 350, 84, 58));

        jButtonStepUpMtr2.setText("step up");
        jButtonStepUpMtr2.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonStepUpMtr2ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonStepUpMtr2, new org.netbeans.lib.awtextra.AbsoluteConstraints(330, 410, 90, -1));

        jButtonStepDownMtr2.setText("step down");
        jButtonStepDownMtr2.setPreferredSize(new java.awt.Dimension(72, 22));
        jButtonStepDownMtr2.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonStepDownMtr2ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonStepDownMtr2, new org.netbeans.lib.awtextra.AbsoluteConstraints(330, 440, 90, -1));

        jButtonMoveMtr2ToPresetLow.setText("Max Down");
        jButtonMoveMtr2ToPresetLow.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonMoveMtr2ToPresetLowActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonMoveMtr2ToPresetLow, new org.netbeans.lib.awtextra.AbsoluteConstraints(333, 470, 84, 51));

        jToggleButtonMtr2.setText("Lower Torque");
        jToggleButtonMtr2.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jToggleButtonMtr2ActionPerformed(evt);
            }
        });
        getContentPane().add(jToggleButtonMtr2, new org.netbeans.lib.awtextra.AbsoluteConstraints(320, 530, 114, -1));

        jButtonResetTorqueMtr2.setText("Reset Torque to Normal");
        jButtonResetTorqueMtr2.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonResetTorqueMtr2ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonResetTorqueMtr2, new org.netbeans.lib.awtextra.AbsoluteConstraints(320, 570, 110, -1));

        jLabelMotor3.setText("Position");
        getContentPane().add(jLabelMotor3, new org.netbeans.lib.awtextra.AbsoluteConstraints(480, 280, 67, 32));

        jLabelMotor4.setText("Position");
        getContentPane().add(jLabelMotor4, new org.netbeans.lib.awtextra.AbsoluteConstraints(620, 280, 67, 32));

        jButtonMoveMtr3ToPresetHigh.setText("Max Up  ");
        jButtonMoveMtr3ToPresetHigh.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonMoveMtr3ToPresetHighActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonMoveMtr3ToPresetHigh, new org.netbeans.lib.awtextra.AbsoluteConstraints(463, 350, 84, 58));

        jButtonStepUpMtr3.setText("step up");
        jButtonStepUpMtr3.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonStepUpMtr3ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonStepUpMtr3, new org.netbeans.lib.awtextra.AbsoluteConstraints(460, 410, 90, -1));

        jButtonStepDownMtr3.setText("step down");
        jButtonStepDownMtr3.setPreferredSize(new java.awt.Dimension(72, 22));
        jButtonStepDownMtr3.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonStepDownMtr3ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonStepDownMtr3, new org.netbeans.lib.awtextra.AbsoluteConstraints(460, 440, 90, -1));

        jButtonMoveMtr3ToPresetLow.setText("Max Down");
        jButtonMoveMtr3ToPresetLow.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonMoveMtr3ToPresetLowActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonMoveMtr3ToPresetLow, new org.netbeans.lib.awtextra.AbsoluteConstraints(463, 470, 84, 51));

        jToggleButtonMtr3.setText("Lower Torque");
        jToggleButtonMtr3.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jToggleButtonMtr3ActionPerformed(evt);
            }
        });
        getContentPane().add(jToggleButtonMtr3, new org.netbeans.lib.awtextra.AbsoluteConstraints(450, 530, -1, -1));

        jButtonResetTorqueMtr3.setText("Reset Torque to Normal");
        jButtonResetTorqueMtr3.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonResetTorqueMtr3ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonResetTorqueMtr3, new org.netbeans.lib.awtextra.AbsoluteConstraints(450, 570, 100, -1));

        jButtonMoveMtr4ToPresetHigh.setText("Max Up  ");
        jButtonMoveMtr4ToPresetHigh.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonMoveMtr4ToPresetHighActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonMoveMtr4ToPresetHigh, new org.netbeans.lib.awtextra.AbsoluteConstraints(613, 350, 84, 58));

        jButtonStepUpMtr4.setText("Right");
        jButtonStepUpMtr4.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonStepUpMtr4ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonStepUpMtr4, new org.netbeans.lib.awtextra.AbsoluteConstraints(610, 410, 90, -1));

        jButtonStepDownMtr4.setText("Left");
        jButtonStepDownMtr4.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonStepDownMtr4ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonStepDownMtr4, new org.netbeans.lib.awtextra.AbsoluteConstraints(610, 440, 90, -1));

        jButtonMoveMtr4ToPresetLow.setText("Max Down");
        jButtonMoveMtr4ToPresetLow.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonMoveMtr4ToPresetLowActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonMoveMtr4ToPresetLow, new org.netbeans.lib.awtextra.AbsoluteConstraints(613, 470, 84, 51));

        jToggleButtonMtr4.setText("Lower Torque");
        jToggleButtonMtr4.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jToggleButtonMtr4ActionPerformed(evt);
            }
        });
        getContentPane().add(jToggleButtonMtr4, new org.netbeans.lib.awtextra.AbsoluteConstraints(600, 530, -1, -1));

        jButtonResetTorqueMtr4.setText("Reset Torque to Normal");
        jButtonResetTorqueMtr4.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonResetTorqueMtr4ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonResetTorqueMtr4, new org.netbeans.lib.awtextra.AbsoluteConstraints(600, 570, 100, -1));

        jButtonEnableTorqueAllMotors.setText("enable all torque");
        jButtonEnableTorqueAllMotors.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonEnableTorqueAllMotorsActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonEnableTorqueAllMotors, new org.netbeans.lib.awtextra.AbsoluteConstraints(40, 640, -1, -1));

        jButtonDisableTorqueAllMotors.setText("diable all torque");
        jButtonDisableTorqueAllMotors.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonDisableTorqueAllMotorsActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonDisableTorqueAllMotors, new org.netbeans.lib.awtextra.AbsoluteConstraints(40, 610, 120, -1));

        jToggleButtonRecorYorN.setText("Record Positions");
        jToggleButtonRecorYorN.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jToggleButtonRecorYorNActionPerformed(evt);
            }
        });
        getContentPane().add(jToggleButtonRecorYorN, new org.netbeans.lib.awtextra.AbsoluteConstraints(10, 40, -1, -1));

        jButtonRecord1Position.setText("Record 1 Position");
        jButtonRecord1Position.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonRecord1PositionActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonRecord1Position, new org.netbeans.lib.awtextra.AbsoluteConstraints(10, 80, -1, -1));

        jButtonRunASequence1.setText("run ex1 sequence");
        jButtonRunASequence1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonRunASequence1ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonRunASequence1, new org.netbeans.lib.awtextra.AbsoluteConstraints(10, 120, 145, -1));

        jTextFieldRecordScenarioName.setText("record_scenario_name");
        getContentPane().add(jTextFieldRecordScenarioName, new org.netbeans.lib.awtextra.AbsoluteConstraints(10, 10, 156, -1));

        jButtonRunASequence2.setText("run ex2 sequence");
        jButtonRunASequence2.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonRunASequence2ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonRunASequence2, new org.netbeans.lib.awtextra.AbsoluteConstraints(10, 160, 145, -1));

        jTextFieldSpeedMotor1.setText("500");
        jTextFieldSpeedMotor1.setMinimumSize(new java.awt.Dimension(12, 25));
        jTextFieldSpeedMotor1.setPreferredSize(new java.awt.Dimension(25, 28));
        jTextFieldSpeedMotor1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jTextFieldSpeedMotor1ActionPerformed(evt);
            }
        });
        getContentPane().add(jTextFieldSpeedMotor1, new org.netbeans.lib.awtextra.AbsoluteConstraints(210, 190, 50, -1));

        jTextFieldSpeedMotor2.setText("20");
        jTextFieldSpeedMotor2.setMinimumSize(new java.awt.Dimension(12, 25));
        jTextFieldSpeedMotor2.setPreferredSize(new java.awt.Dimension(25, 28));
        jTextFieldSpeedMotor2.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jTextFieldSpeedMotor2ActionPerformed(evt);
            }
        });
        getContentPane().add(jTextFieldSpeedMotor2, new org.netbeans.lib.awtextra.AbsoluteConstraints(340, 190, 50, -1));

        jTextFieldSpeedMotor3.setText("15");
        jTextFieldSpeedMotor3.setMinimumSize(new java.awt.Dimension(12, 25));
        jTextFieldSpeedMotor3.setPreferredSize(new java.awt.Dimension(25, 28));
        jTextFieldSpeedMotor3.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jTextFieldSpeedMotor3ActionPerformed(evt);
            }
        });
        getContentPane().add(jTextFieldSpeedMotor3, new org.netbeans.lib.awtextra.AbsoluteConstraints(480, 190, 50, -1));

        jTextFieldSpeedMotor4.setText("8");
        jTextFieldSpeedMotor4.setMinimumSize(new java.awt.Dimension(12, 25));
        jTextFieldSpeedMotor4.setPreferredSize(new java.awt.Dimension(25, 28));
        jTextFieldSpeedMotor4.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jTextFieldSpeedMotor4ActionPerformed(evt);
            }
        });
        getContentPane().add(jTextFieldSpeedMotor4, new org.netbeans.lib.awtextra.AbsoluteConstraints(620, 190, 50, -1));

        jLabel1.setText("Speed");
        getContentPane().add(jLabel1, new org.netbeans.lib.awtextra.AbsoluteConstraints(130, 195, -1, -1));

        jTextField_IncrementMtr1.setText("250");
        jTextField_IncrementMtr1.setMinimumSize(new java.awt.Dimension(12, 25));
        jTextField_IncrementMtr1.setPreferredSize(new java.awt.Dimension(25, 28));
        getContentPane().add(jTextField_IncrementMtr1, new org.netbeans.lib.awtextra.AbsoluteConstraints(210, 220, 50, -1));

        jTextField_IncrementMtr2.setText("25");
        jTextField_IncrementMtr2.setMinimumSize(new java.awt.Dimension(12, 25));
        jTextField_IncrementMtr2.setPreferredSize(new java.awt.Dimension(25, 28));
        getContentPane().add(jTextField_IncrementMtr2, new org.netbeans.lib.awtextra.AbsoluteConstraints(340, 220, 50, -1));

        jTextField_IncrementMtr3.setText("25");
        jTextField_IncrementMtr3.setMinimumSize(new java.awt.Dimension(12, 25));
        jTextField_IncrementMtr3.setPreferredSize(new java.awt.Dimension(25, 28));
        getContentPane().add(jTextField_IncrementMtr3, new org.netbeans.lib.awtextra.AbsoluteConstraints(480, 220, 50, -1));

        jTextField_IncrementMtr4.setText("10");
        jTextField_IncrementMtr4.setMinimumSize(new java.awt.Dimension(12, 25));
        jTextField_IncrementMtr4.setPreferredSize(new java.awt.Dimension(25, 28));
        getContentPane().add(jTextField_IncrementMtr4, new org.netbeans.lib.awtextra.AbsoluteConstraints(620, 220, 50, -1));

        jLabel_Increment.setText("Increment");
        jLabel_Increment.setToolTipText("How much does the motor move on one click.");
        getContentPane().add(jLabel_Increment, new org.netbeans.lib.awtextra.AbsoluteConstraints(110, 225, 70, -1));

        jTextField_PositionMtr4.setText("Position");
        getContentPane().add(jTextField_PositionMtr4, new org.netbeans.lib.awtextra.AbsoluteConstraints(620, 255, 50, -1));

        jTextField_PositionMtr1.setText("Position");
        getContentPane().add(jTextField_PositionMtr1, new org.netbeans.lib.awtextra.AbsoluteConstraints(210, 255, 50, -1));

        jTextField_PositionMtr2.setText("Position");
        getContentPane().add(jTextField_PositionMtr2, new org.netbeans.lib.awtextra.AbsoluteConstraints(340, 255, 50, -1));

        jTextField_PositionMtr3.setText("Position");
        getContentPane().add(jTextField_PositionMtr3, new org.netbeans.lib.awtextra.AbsoluteConstraints(480, 255, 50, -1));

        jLabel2.setText("Pos Read");
        getContentPane().add(jLabel2, new org.netbeans.lib.awtextra.AbsoluteConstraints(110, 255, -1, -1));

        jLabel3.setText("Target Position");
        getContentPane().add(jLabel3, new org.netbeans.lib.awtextra.AbsoluteConstraints(110, 280, -1, 30));

        jToggleButtonPrintDebugging.setText("Print Debugging");
        jToggleButtonPrintDebugging.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jToggleButtonPrintDebuggingActionPerformed(evt);
            }
        });
        getContentPane().add(jToggleButtonPrintDebugging, new org.netbeans.lib.awtextra.AbsoluteConstraints(40, 540, -1, -1));

        jLabel4.setText("Elbow");
        getContentPane().add(jLabel4, new org.netbeans.lib.awtextra.AbsoluteConstraints(480, 10, 50, 30));

        jToggleButton_FollowDriverButton.setText("Follow Driver Arm");
        jToggleButton_FollowDriverButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jToggleButton_FollowDriverButtonActionPerformed(evt);
            }
        });
        getContentPane().add(jToggleButton_FollowDriverButton, new org.netbeans.lib.awtextra.AbsoluteConstraints(760, 40, -1, -1));

        jLabel5.setText("Load");
        getContentPane().add(jLabel5, new org.netbeans.lib.awtextra.AbsoluteConstraints(110, 310, 70, 20));

        jLabel6.setText("End Effector");
        getContentPane().add(jLabel6, new org.netbeans.lib.awtextra.AbsoluteConstraints(210, 10, 60, 30));

        jLabel7.setText("Sholder");
        getContentPane().add(jLabel7, new org.netbeans.lib.awtextra.AbsoluteConstraints(635, 10, 60, 30));

        jTextField_LoadEndEffector.setText("Load");
        getContentPane().add(jTextField_LoadEndEffector, new org.netbeans.lib.awtextra.AbsoluteConstraints(200, 310, 75, -1));

        jTextField_Load_2.setText("Load");
        getContentPane().add(jTextField_Load_2, new org.netbeans.lib.awtextra.AbsoluteConstraints(340, 310, 75, -1));

        jTextField_Load_4.setText("Load");
        getContentPane().add(jTextField_Load_4, new org.netbeans.lib.awtextra.AbsoluteConstraints(620, 310, 75, -1));

        jTextField_Load_3.setText("Load");
        getContentPane().add(jTextField_Load_3, new org.netbeans.lib.awtextra.AbsoluteConstraints(470, 310, 75, -1));

        jLabel8.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        jLabel8.setText("Wrist");
        getContentPane().add(jLabel8, new org.netbeans.lib.awtextra.AbsoluteConstraints(320, 10, 70, 30));

        jScrollBarMtr4_Actual.setBlockIncrement(20);
        jScrollBarMtr4_Actual.setMaximum(700);
        jScrollBarMtr4_Actual.setMinimum(385);
        jScrollBarMtr4_Actual.setValue(500);
        jScrollBarMtr4_Actual.setFocusable(false);
        getContentPane().add(jScrollBarMtr4_Actual, new org.netbeans.lib.awtextra.AbsoluteConstraints(660, 53, 36, 130));

        jSlider4_ActualTemp.setMaximum(160);
        jSlider4_ActualTemp.setMinimum(110);
        jSlider4_ActualTemp.setMinorTickSpacing(10);
        jSlider4_ActualTemp.setOrientation(javax.swing.JSlider.VERTICAL);
        jSlider4_ActualTemp.setPaintLabels(true);
        jSlider4_ActualTemp.setPaintTicks(true);
        getContentPane().add(jSlider4_ActualTemp, new org.netbeans.lib.awtextra.AbsoluteConstraints(880, 100, 20, -1));

        jSlider1_ActualTemp.setMaximum(160);
        jSlider1_ActualTemp.setMinimum(110);
        jSlider1_ActualTemp.setMinorTickSpacing(10);
        jSlider1_ActualTemp.setOrientation(javax.swing.JSlider.VERTICAL);
        jSlider1_ActualTemp.setPaintLabels(true);
        jSlider1_ActualTemp.setPaintTicks(true);
        jSlider1_ActualTemp.setToolTipText("");
        getContentPane().add(jSlider1_ActualTemp, new org.netbeans.lib.awtextra.AbsoluteConstraints(784, 100, 20, -1));

        jSlider2_ActualTemp.setMaximum(160);
        jSlider2_ActualTemp.setMinimum(110);
        jSlider2_ActualTemp.setMinorTickSpacing(10);
        jSlider2_ActualTemp.setOrientation(javax.swing.JSlider.VERTICAL);
        jSlider2_ActualTemp.setPaintLabels(true);
        jSlider2_ActualTemp.setPaintTicks(true);
        getContentPane().add(jSlider2_ActualTemp, new org.netbeans.lib.awtextra.AbsoluteConstraints(816, 100, 20, -1));

        jSlider3_ActualTemp.setMaximum(160);
        jSlider3_ActualTemp.setMinimum(110);
        jSlider3_ActualTemp.setMinorTickSpacing(10);
        jSlider3_ActualTemp.setOrientation(javax.swing.JSlider.VERTICAL);
        jSlider3_ActualTemp.setPaintLabels(true);
        jSlider3_ActualTemp.setPaintTicks(true);
        getContentPane().add(jSlider3_ActualTemp, new org.netbeans.lib.awtextra.AbsoluteConstraints(848, 100, 20, -1));

        jTextArea1.setColumns(20);
        jTextArea1.setLineWrap(true);
        jTextArea1.setRows(5);
        jTextArea1.setText("Temperature Sensors. \nShutdown occurs at 158 F (70 C).");
        jTextArea1.setWrapStyleWord(true);
        jScrollPane1.setViewportView(jTextArea1);

        getContentPane().add(jScrollPane1, new org.netbeans.lib.awtextra.AbsoluteConstraints(750, 310, 230, 100));

        jTextField2.setText("140 F");
        getContentPane().add(jTextField2, new org.netbeans.lib.awtextra.AbsoluteConstraints(910, 170, -1, -1));

        jTextField3.setText("110 F");
        getContentPane().add(jTextField3, new org.netbeans.lib.awtextra.AbsoluteConstraints(910, 275, -1, -1));

        jTextField4.setText("160 F");
        getContentPane().add(jTextField4, new org.netbeans.lib.awtextra.AbsoluteConstraints(910, 100, -1, -1));

        jToggleButton1.setText("Run End Effector");
        jToggleButton1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jToggleButton1ActionPerformed(evt);
            }
        });
        getContentPane().add(jToggleButton1, new org.netbeans.lib.awtextra.AbsoluteConstraints(10, 350, 150, 50));

        jTextFieldTempMtr4.setText("jTextField1");
        getContentPane().add(jTextFieldTempMtr4, new org.netbeans.lib.awtextra.AbsoluteConstraints(875, 80, 40, -1));

        jTextFieldTempMtr3.setText("jTextField1");
        getContentPane().add(jTextFieldTempMtr3, new org.netbeans.lib.awtextra.AbsoluteConstraints(835, 80, 40, -1));

        jTextFieldTempMtr2.setText("jTextField1");
        getContentPane().add(jTextFieldTempMtr2, new org.netbeans.lib.awtextra.AbsoluteConstraints(795, 80, 40, -1));

        jTextFieldTempMtr1.setText("jTextField1");
        getContentPane().add(jTextFieldTempMtr1, new org.netbeans.lib.awtextra.AbsoluteConstraints(755, 80, 40, -1));

        buttonGroup1.add(jRadioButtonPriorActualPositions);
        jRadioButtonPriorActualPositions.setText("Use Prior Actual Positions");
        getContentPane().add(jRadioButtonPriorActualPositions, new org.netbeans.lib.awtextra.AbsoluteConstraints(810, 450, -1, -1));

        buttonGroup1.add(jRadioButtonSavedPositions1);
        jRadioButtonSavedPositions1.setText("Use Saved Position #1");
        jRadioButtonSavedPositions1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jRadioButtonSavedPositions1ActionPerformed(evt);
            }
        });
        getContentPane().add(jRadioButtonSavedPositions1, new org.netbeans.lib.awtextra.AbsoluteConstraints(810, 540, 150, -1));

        jButtonGoToDesiredStartPosition.setText("Start (go to Desired Position)");
        jButtonGoToDesiredStartPosition.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonGoToDesiredStartPositionActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonGoToDesiredStartPosition, new org.netbeans.lib.awtextra.AbsoluteConstraints(780, 630, 210, 40));

        buttonGroup1.add(jRadioButtonPriorTargetPositions);
        jRadioButtonPriorTargetPositions.setText("Use Prior Target Positions");
        getContentPane().add(jRadioButtonPriorTargetPositions, new org.netbeans.lib.awtextra.AbsoluteConstraints(810, 480, -1, -1));

        buttonGroup1.add(jRadioButtonHomePositions1);
        jRadioButtonHomePositions1.setText("Use Home Positions");
        jRadioButtonHomePositions1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jRadioButtonHomePositions1ActionPerformed(evt);
            }
        });
        getContentPane().add(jRadioButtonHomePositions1, new org.netbeans.lib.awtextra.AbsoluteConstraints(810, 510, -1, -1));

        jTextFieldSavedPosition1.setText("jTextField1");
        getContentPane().add(jTextFieldSavedPosition1, new org.netbeans.lib.awtextra.AbsoluteConstraints(981, 540, 80, -1));

        jButtonUpdateSavedPosition1.setText("Update");
        jButtonUpdateSavedPosition1.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonUpdateSavedPosition1ActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonUpdateSavedPosition1, new org.netbeans.lib.awtextra.AbsoluteConstraints(1070, 540, -1, -1));

        jButtonListAllSavedPositions.setText("List All Saved Positions");
        jButtonListAllSavedPositions.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonListAllSavedPositionsActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonListAllSavedPositions, new org.netbeans.lib.awtextra.AbsoluteConstraints(1040, 580, 210, -1));

        jButtonClearThisSavedPreference.setText("Clear This Saved Preference");
        jButtonClearThisSavedPreference.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jButtonClearThisSavedPreferenceActionPerformed(evt);
            }
        });
        getContentPane().add(jButtonClearThisSavedPreference, new org.netbeans.lib.awtextra.AbsoluteConstraints(1040, 610, 210, 30));

        jListSavedPositions.setModel(new javax.swing.AbstractListModel<String>() {
            String[] strings = { "Item 1", "Item 2", "Item 3", "Item 4", "Item 5" };
            public int getSize() { return strings.length; }
            public String getElementAt(int i) { return strings[i]; }
        });
        jScrollPane2.setViewportView(jListSavedPositions);

        getContentPane().add(jScrollPane2, new org.netbeans.lib.awtextra.AbsoluteConstraints(1040, 650, 290, 320));

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void jButtonMoveMtr1ToPresetHighActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonMoveMtr1ToPresetHighActionPerformed
        motor1.moveMotor((short)400);
        jLabelMotor1.setText(""+400);
    }//GEN-LAST:event_jButtonMoveMtr1ToPresetHighActionPerformed

    private void jButtonMoveMtr1ToPresetLowActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonMoveMtr1ToPresetLowActionPerformed
        motor1.moveMotor((short)150);
        jLabelMotor1.setText(""+150);
    }//GEN-LAST:event_jButtonMoveMtr1ToPresetLowActionPerformed

    private void jScrollBarMtr1_TargetAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_jScrollBarMtr1_TargetAdjustmentValueChanged
        short j = (short)jScrollBarMtr1_Target.getValue();
        motor1.moveMotor(j);
        TargetPosMtr1 = (int)j;
        jLabelMotor1.setText(""+j);
    }//GEN-LAST:event_jScrollBarMtr1_TargetAdjustmentValueChanged

    private void jScrollBarMtr2_TargetAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_jScrollBarMtr2_TargetAdjustmentValueChanged
        short j = (short)jScrollBarMtr2_Target.getValue();
        motor2.moveMotor(j);
        TargetPosMtr2 = (int)j;
        jLabelMotor2.setText(""+j);
    }//GEN-LAST:event_jScrollBarMtr2_TargetAdjustmentValueChanged

    private void jScrollBarMtr3_TargetAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_jScrollBarMtr3_TargetAdjustmentValueChanged
        short j = (short)jScrollBarMtr3_Target.getValue();
        motor3.moveMotor(j);
        TargetPosMtr3 = (int)j;
        jLabelMotor3.setText(""+j);
    }//GEN-LAST:event_jScrollBarMtr3_TargetAdjustmentValueChanged

    private void jScrollBarMtr4_TargetAdjustmentValueChanged(java.awt.event.AdjustmentEvent evt) {//GEN-FIRST:event_jScrollBarMtr4_TargetAdjustmentValueChanged
        short j = (short)Math.abs((short)jScrollBarMtr4_Target.getValue());
        motor4.moveMotor((short)j);
        TargetPosMtr4 = (int)j;
        jLabelMotor4.setText(""+j);
    }//GEN-LAST:event_jScrollBarMtr4_TargetAdjustmentValueChanged

    private void jButtonStepUpMtr1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonStepUpMtr1ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor1.getText());
        motor1.setMovingSpeed(val);
        
        int moveIncrement = -100;
        moveIncrement = -Integer.parseInt(jTextField_IncrementMtr1.getText());
        TargetPosMtr1 = TargetPosMtr1 + moveIncrement;
        jScrollBarMtr1_Target.setValue(TargetPosMtr1);
        motor1.moveMotor((short)(TargetPosMtr1));
        jLabelMotor1.setText(TargetPosMtr1+"");
    }//GEN-LAST:event_jButtonStepUpMtr1ActionPerformed

    private void jButtonStepDownMtr1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonStepDownMtr1ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor1.getText());
        motor1.setMovingSpeed(val);
        
        int moveIncrement = 100;
        moveIncrement = Integer.parseInt(jTextField_IncrementMtr1.getText());
        short currentPos = motor1.readPosition();
        TargetPosMtr1 = TargetPosMtr1 + moveIncrement;
        jScrollBarMtr1_Target.setValue(TargetPosMtr1);
        motor1.moveMotor((short)(TargetPosMtr1));
        jLabelMotor1.setText(TargetPosMtr1+"");
    }//GEN-LAST:event_jButtonStepDownMtr1ActionPerformed

    private void jButtonExitActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonExitActionPerformed
        dynamixel.closePort(port_num);
        try {
            sleep(750);
        } catch (InterruptedException ex) {
            Logger.getLogger(JFrameClass.class.getName()).log(Level.SEVERE, null, ex);
        }
        System.exit(0);
    }//GEN-LAST:event_jButtonExitActionPerformed

    private void jToggleButtonMtr1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jToggleButtonMtr1ActionPerformed
        motor1.setTorque(0);
        motor1.disableTorque();
    }//GEN-LAST:event_jToggleButtonMtr1ActionPerformed

    private void jButtonResetTorqueMtr1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonResetTorqueMtr1ActionPerformed
        motor1.enableTorque();
        motor1.setTorque();
    }//GEN-LAST:event_jButtonResetTorqueMtr1ActionPerformed

    private void jButtonMoveMtr2ToPresetHighActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonMoveMtr2ToPresetHighActionPerformed
        motor2.moveMotor((short)400);
        jLabelMotor2.setText(""+400);
    }//GEN-LAST:event_jButtonMoveMtr2ToPresetHighActionPerformed

    private void jButtonStepUpMtr2ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonStepUpMtr2ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor2.getText());
        motor2.setMovingSpeed(val);
        
        int move_direction = -1; // set this to 1 or -1 to switch direction
        int moveIncrement = 25;
        moveIncrement = move_direction * Integer.parseInt(jTextField_IncrementMtr2.getText());
        short currentPos = motor2.readPosition();
        TargetPosMtr2 = TargetPosMtr2 + moveIncrement;
        jScrollBarMtr2_Target.setValue(TargetPosMtr2);
        motor2.moveMotor((short)(TargetPosMtr2));
        jLabelMotor2.setText(TargetPosMtr2+"");
    }//GEN-LAST:event_jButtonStepUpMtr2ActionPerformed

    private void jButtonStepDownMtr2ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonStepDownMtr2ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor2.getText());
        motor2.setMovingSpeed(val);
        
        int move_direction = 1; // set this to 1 or -1 to switch direction
        int moveIncrement = 25;
        moveIncrement = move_direction * Integer.parseInt(jTextField_IncrementMtr2.getText());
        short currentPos = motor2.readPosition();
        TargetPosMtr2 = TargetPosMtr2 + moveIncrement;
        jScrollBarMtr2_Target.setValue(TargetPosMtr2);
        motor2.moveMotor((short)(TargetPosMtr2));
        jLabelMotor2.setText(TargetPosMtr2+"");
    }//GEN-LAST:event_jButtonStepDownMtr2ActionPerformed

    private void jButtonMoveMtr2ToPresetLowActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonMoveMtr2ToPresetLowActionPerformed
        motor2.moveMotor((short)150);
        jLabelMotor2.setText(""+150);
    }//GEN-LAST:event_jButtonMoveMtr2ToPresetLowActionPerformed

    private void jToggleButtonMtr2ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jToggleButtonMtr2ActionPerformed
        motor2.setTorque(0);
        motor2.disableTorque();
    }//GEN-LAST:event_jToggleButtonMtr2ActionPerformed

    private void jButtonResetTorqueMtr2ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonResetTorqueMtr2ActionPerformed
        motor2.enableTorque();
        motor2.setTorque();
    }//GEN-LAST:event_jButtonResetTorqueMtr2ActionPerformed

    private void jButtonMoveMtr3ToPresetHighActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonMoveMtr3ToPresetHighActionPerformed
        motor3.moveMotor((short)340);
        jLabelMotor3.setText(""+340);
    }//GEN-LAST:event_jButtonMoveMtr3ToPresetHighActionPerformed

    private void jButtonStepUpMtr3ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonStepUpMtr3ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor3.getText());
        motor3.setMovingSpeed(val);
        
        int move_direction = -1; // set this to 1 or -1 to switch direction
        int moveIncrement = 25;
        moveIncrement = move_direction * Integer.parseInt(jTextField_IncrementMtr3.getText());
        short currentPos = motor3.readPosition();
        TargetPosMtr3 = TargetPosMtr3 + moveIncrement;
        jScrollBarMtr3_Target.setValue(TargetPosMtr3);
        motor3.moveMotor((short)(TargetPosMtr3));
        jLabelMotor3.setText(TargetPosMtr3+"");
    }//GEN-LAST:event_jButtonStepUpMtr3ActionPerformed

    private void jButtonStepDownMtr3ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonStepDownMtr3ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor3.getText());
        motor3.setMovingSpeed(val);
        
        int move_direction = 1; // set this to 1 or -1 to switch direction
        int moveIncrement = 25;
        moveIncrement = move_direction * Integer.parseInt(jTextField_IncrementMtr3.getText());
        short currentPos = motor3.readPosition();
        TargetPosMtr3 = TargetPosMtr3 + moveIncrement;
        jScrollBarMtr3_Target.setValue(TargetPosMtr3);
        motor3.moveMotor((short)(TargetPosMtr3));
        jLabelMotor3.setText(TargetPosMtr3+"");
    }//GEN-LAST:event_jButtonStepDownMtr3ActionPerformed

    private void jButtonMoveMtr3ToPresetLowActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonMoveMtr3ToPresetLowActionPerformed
        motor3.moveMotor((short)720);
        jLabelMotor3.setText(""+720);
    }//GEN-LAST:event_jButtonMoveMtr3ToPresetLowActionPerformed

    private void jToggleButtonMtr3ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jToggleButtonMtr3ActionPerformed
        motor3.setTorque(0);
        motor3.disableTorque();
    }//GEN-LAST:event_jToggleButtonMtr3ActionPerformed

    private void jButtonResetTorqueMtr3ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonResetTorqueMtr3ActionPerformed
        motor3.enableTorque();
        motor3.setTorque();
    }//GEN-LAST:event_jButtonResetTorqueMtr3ActionPerformed

    private void jButtonMoveMtr4ToPresetHighActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonMoveMtr4ToPresetHighActionPerformed
        motor4.moveMotor((short)700);
        jLabelMotor4.setText(""+700);
    }//GEN-LAST:event_jButtonMoveMtr4ToPresetHighActionPerformed

    private void jButtonStepUpMtr4ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonStepUpMtr4ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor4.getText());
        motor4.setMovingSpeed(val);
        
        int moveIncrement = -0; // value is not set here. It is set by reading the jTextField_InrementalMtr4.getText().
        moveIncrement = -Integer.parseInt(jTextField_IncrementMtr4.getText())*-1;
        short currentPos = motor4.readPosition();
        TargetPosMtr4 = TargetPosMtr4 + moveIncrement;
        jScrollBarMtr4_Target.setValue(TargetPosMtr4*-1); // this scroll bar is negative so left/right look right on the GUI
        motor4.moveMotor((short)(TargetPosMtr4));
        jLabelMotor4.setText(TargetPosMtr4+"");
    }//GEN-LAST:event_jButtonStepUpMtr4ActionPerformed

    private void jButtonStepDownMtr4ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonStepDownMtr4ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor4.getText());
        motor4.setMovingSpeed(val);
        
        int moveIncrement = 0; // value is not set here. It is set by reading the jTextField_InrementalMtr4.getText().
        moveIncrement = Integer.parseInt(jTextField_IncrementMtr4.getText())*-1;
        short currentPos = motor4.readPosition();
        TargetPosMtr4 = TargetPosMtr4 + moveIncrement;
        jScrollBarMtr4_Target.setValue(TargetPosMtr4*-1);
        motor4.moveMotor((short)(TargetPosMtr4));
        jLabelMotor4.setText(TargetPosMtr4+"");
    }//GEN-LAST:event_jButtonStepDownMtr4ActionPerformed

    private void jButtonMoveMtr4ToPresetLowActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonMoveMtr4ToPresetLowActionPerformed
        motor4.moveMotor((short)250);
        jLabelMotor4.setText(""+250);
    }//GEN-LAST:event_jButtonMoveMtr4ToPresetLowActionPerformed

    private void jToggleButtonMtr4ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jToggleButtonMtr4ActionPerformed
        motor4.setTorque(0);
        motor4.disableTorque();
    }//GEN-LAST:event_jToggleButtonMtr4ActionPerformed

    private void jButtonResetTorqueMtr4ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonResetTorqueMtr4ActionPerformed
        motor4.enableTorque();
        motor4.setTorque();
    }//GEN-LAST:event_jButtonResetTorqueMtr4ActionPerformed

    private void jButtonEnableTorqueAllMotorsActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonEnableTorqueAllMotorsActionPerformed
        motor1.enableTorque();
        motor2.enableTorque();
        motor3.enableTorque();
        motor4.enableTorque();
    }//GEN-LAST:event_jButtonEnableTorqueAllMotorsActionPerformed

    private void jButtonDisableTorqueAllMotorsActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonDisableTorqueAllMotorsActionPerformed
        
        motor1.disableTorque();
        motor2.disableTorque();
        motor3.disableTorque();
        motor4.disableTorque();
    }//GEN-LAST:event_jButtonDisableTorqueAllMotorsActionPerformed

    private void jToggleButtonRecorYorNActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jToggleButtonRecorYorNActionPerformed
        if(jToggleButtonRecorYorN.isSelected()){
            recordPositions = 1;
        }
        if(!jToggleButtonRecorYorN.isSelected()){
            recordPositions = 0;
        }
    }//GEN-LAST:event_jToggleButtonRecorYorNActionPerformed

    private void jButtonRecord1PositionActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonRecord1PositionActionPerformed
        recordOnce = 1;
    }//GEN-LAST:event_jButtonRecord1PositionActionPerformed

    private void jButtonRunASequence1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonRunASequence1ActionPerformed
        String scenarioReadName = "a2";
        int loop = 0;
    }//GEN-LAST:event_jButtonRunASequence1ActionPerformed

    private void jButtonRunASequence2ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonRunASequence2ActionPerformed
        String scenarioReadName = "example2";
        
    }//GEN-LAST:event_jButtonRunASequence2ActionPerformed

    private void jTextFieldSpeedMotor1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jTextFieldSpeedMotor1ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor1.getText());
        System.out.println("speed set to: "+val);
        motor1.setMovingSpeed(val);
    }//GEN-LAST:event_jTextFieldSpeedMotor1ActionPerformed

    private void jTextFieldSpeedMotor2ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jTextFieldSpeedMotor2ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor2.getText());
        System.out.println("speed set to: "+val);
        motor2.setMovingSpeed(val);
    }//GEN-LAST:event_jTextFieldSpeedMotor2ActionPerformed

    private void jTextFieldSpeedMotor3ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jTextFieldSpeedMotor3ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor3.getText());
        System.out.println("speed set to: "+val);
        motor3.setMovingSpeed(val);
    }//GEN-LAST:event_jTextFieldSpeedMotor3ActionPerformed

    private void jTextFieldSpeedMotor4ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jTextFieldSpeedMotor4ActionPerformed
        int val = Integer.valueOf(jTextFieldSpeedMotor4.getText());
        System.out.println("speed set to: "+val);
        motor4.setMovingSpeed(val);
    }//GEN-LAST:event_jTextFieldSpeedMotor4ActionPerformed

    private void jToggleButtonPrintDebuggingActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jToggleButtonPrintDebuggingActionPerformed
        boolean togglevalue = jToggleButtonPrintDebugging.isSelected();
        motor1.printDebugging(togglevalue);
        motor2.printDebugging(togglevalue);
        motor3.printDebugging(togglevalue);
        motor4.printDebugging(togglevalue);
    }//GEN-LAST:event_jToggleButtonPrintDebuggingActionPerformed

    private void jToggleButton_FollowDriverButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jToggleButton_FollowDriverButtonActionPerformed
        if (FollowDriverFlag){
            FollowDriverFlag = false;
            jToggleButton_FollowDriverButton.setText("Not Following Arm");
            jToggleButton_FollowDriverButton.setOpaque(true);
            jToggleButton_FollowDriverButton.setBackground(Color.green);
        }
        else {
            FollowDriverFlag = true;
            System.out.println("");
            System.out.println("------------ Begin to follow other arm -----------------");
            jToggleButton_FollowDriverButton.setText("Following Arm");
            jToggleButton_FollowDriverButton.setOpaque(true);
            jToggleButton_FollowDriverButton.setBackground(Color.red);
        }
    }//GEN-LAST:event_jToggleButton_FollowDriverButtonActionPerformed

    private void jToggleButton1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jToggleButton1ActionPerformed
        Boolean OldRunEndEffectorContinuously = RunEndEffectorContinuously;
        if(OldRunEndEffectorContinuously)
            {RunEndEffectorContinuously=false;}
            else
            {RunEndEffectorContinuously=true;}
        
    }//GEN-LAST:event_jToggleButton1ActionPerformed

    private void jButtonGoToDesiredStartPositionActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonGoToDesiredStartPositionActionPerformed
        if(jRadioButtonPriorActualPositions.isSelected()){
            ActualsOrTarget = "Actuals";
        }
        else if(jRadioButtonPriorTargetPositions.isSelected()){
            ActualsOrTarget = "Target";
        }        
        else {
            ActualsOrTarget = "Home";
        }
    }//GEN-LAST:event_jButtonGoToDesiredStartPositionActionPerformed

    private void jButtonUpdateSavedPosition1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonUpdateSavedPosition1ActionPerformed
        String PositionName = jTextFieldSavedPosition1.getText();
        
        System.out.println("ActualPosMtr1 " + position1);
        System.out.println("ActualPosMtr2 " + position2);
        System.out.println("ActualPosMtr3 " + position3);
        System.out.println("ActualPosMtr4 " + position4);
        
        prefsGlobal.putInt(PositionName+"SavedPosition1motor1TargetPosition", position1);
        prefsGlobal.putInt(PositionName+"SavedPosition1motor2TargetPosition", position2);
        prefsGlobal.putInt(PositionName+"SavedPosition1motor3TargetPosition", position3);
        prefsGlobal.putInt(PositionName+"SavedPosition1motor4TargetPosition", position4);
        prefsGlobal.put(PositionName+"SavedPosition1_Name", PositionName);

    }//GEN-LAST:event_jButtonUpdateSavedPosition1ActionPerformed

    private void jRadioButtonSavedPositions1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jRadioButtonSavedPositions1ActionPerformed
        String PositionName = jTextFieldSavedPosition1.getText();
        System.out.println("Saved Preferences Used. Name = " + prefsGlobal.get(PositionName+"SavedPosition1_Name", "default"));
        
        TargetPosMtr1 = prefsGlobal.getInt(PositionName+"SavedPosition1motor1TargetPosition", 99999);
        TargetPosMtr2 = prefsGlobal.getInt(PositionName+"SavedPosition1motor2TargetPosition", 99999);
        TargetPosMtr3 = prefsGlobal.getInt(PositionName+"SavedPosition1motor3TargetPosition", 99999);
        TargetPosMtr4 = prefsGlobal.getInt(PositionName+"SavedPosition1motor4TargetPosition", 99999);
        System.out.println("TargetPosMtr1 " + TargetPosMtr1);
        System.out.println("TargetPosMtr2 " + TargetPosMtr2);
        System.out.println("TargetPosMtr3 " + TargetPosMtr3);
        System.out.println("TargetPosMtr4 " + TargetPosMtr4);
        if(TargetPosMtr1>1300 || TargetPosMtr1<=0){TargetPosMtr1=HomePosMtr1;} // if the value read from the stored preferences from a prior session is outside the range of the dynamixel assume it is an error and use the home position instead.
        if(TargetPosMtr2>1300 || TargetPosMtr2<=0){TargetPosMtr2=HomePosMtr2;} // if the value read from the stored preferences from a prior session is outside the range of the dynamixel assume it is an error and use the home position instead.
        if(TargetPosMtr3>1300 || TargetPosMtr3<=0){TargetPosMtr3=HomePosMtr3;} // if the value read from the stored preferences from a prior session is outside the range of the dynamixel assume it is an error and use the home position instead.
        if(TargetPosMtr4>1300 || TargetPosMtr4<=0){TargetPosMtr4=HomePosMtr4;} // if the value read from the stored preferences from a prior session is outside the range of the dynamixel assume it is an error and use the home position instead.
                    
        jScrollBarMtr1_Target.setValue(TargetPosMtr1); motor1.moveMotor((short)TargetPosMtr1); jLabelMotor1.setText(""+TargetPosMtr1);
        jScrollBarMtr2_Target.setValue(TargetPosMtr2); motor2.moveMotor((short)TargetPosMtr2); jLabelMotor2.setText(""+TargetPosMtr2);
        jScrollBarMtr3_Target.setValue(TargetPosMtr3); motor3.moveMotor((short)TargetPosMtr3); jLabelMotor3.setText(""+TargetPosMtr3);
        jScrollBarMtr4_Target.setValue(TargetPosMtr4*-1); motor4.moveMotor((short)TargetPosMtr4); jLabelMotor4.setText(""+TargetPosMtr4);
        
    }//GEN-LAST:event_jRadioButtonSavedPositions1ActionPerformed

    private void jRadioButtonHomePositions1ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jRadioButtonHomePositions1ActionPerformed
        if(ActualsOrTarget=="Undecided"){System.out.println("Press the Start Button to use this preset");/*on the initial startup; ignore this actionevent and require the 'start button' to be pressed to use the Home Settings*/ }
        else{
        System.out.println("Home Positions Used.");
        TargetPosMtr1 = HomePosMtr1;
        TargetPosMtr2 = HomePosMtr2;
        TargetPosMtr3 = HomePosMtr3;
        TargetPosMtr4 = HomePosMtr4;
        System.out.println("TargetPosMtr1 " + TargetPosMtr1);
        System.out.println("TargetPosMtr2 " + TargetPosMtr2);
        System.out.println("TargetPosMtr3 " + TargetPosMtr3);
        System.out.println("TargetPosMtr4 " + TargetPosMtr4);
        
        jScrollBarMtr2_Target.setValue(TargetPosMtr2); motor2.moveMotor((short)TargetPosMtr2); jLabelMotor2.setText(""+TargetPosMtr2);
        jScrollBarMtr3_Target.setValue(TargetPosMtr3); motor3.moveMotor((short)TargetPosMtr3); jLabelMotor3.setText(""+TargetPosMtr3);
        
        // move arm up and wait a bit before moving it left/right to the home position so it doesn't drag along the ground
        // also don't move the end effector until arm moved up
            try {
                sleep(1000);
            } catch (InterruptedException ex) {
                Logger.getLogger(JFrameClass.class.getName()).log(Level.SEVERE, null, ex);
            }
        jScrollBarMtr1_Target.setValue(TargetPosMtr1); motor1.moveMotor((short)TargetPosMtr1); jLabelMotor1.setText(""+TargetPosMtr1);
        jScrollBarMtr4_Target.setValue(TargetPosMtr4*-1); motor4.moveMotor((short)TargetPosMtr4); jLabelMotor4.setText(""+TargetPosMtr4);
        }
    }//GEN-LAST:event_jRadioButtonHomePositions1ActionPerformed

    private void jButtonListAllSavedPositionsActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonListAllSavedPositionsActionPerformed
        System.out.println("Node's keys: ");
        DefaultListModel listmodel=new DefaultListModel();         
        //listmodel.addElement("YOUR NEW DATA");          
        
        try {
            for (String s : prefsGlobal.keys()) {
                System.out.println(s + " : " + prefsGlobal.get(s, ""));
                listmodel.addElement(s);
            }   
        } catch (BackingStoreException ex) {
            Logger.getLogger(JFrameClass.class.getName()).log(Level.SEVERE, null, ex);
        }
      jListSavedPositions.setModel(listmodel);
    }//GEN-LAST:event_jButtonListAllSavedPositionsActionPerformed

    private void jButtonClearThisSavedPreferenceActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jButtonClearThisSavedPreferenceActionPerformed

        System.out.println("Selected Value = " + jListSavedPositions.getSelectedValue());
        prefsGlobal.remove(jListSavedPositions.getSelectedValue());

    }//GEN-LAST:event_jButtonClearThisSavedPreferenceActionPerformed

    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        /* Set the Nimbus look and feel */
        //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /* If Nimbus (introduced in Java SE 6) is not available, stay with the default look and feel.
         * For details see http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html 
         */
//        try {
//            for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager.getInstalledLookAndFeels()) {
//                if ("Nimbus".equals(info.getName())) {
//                    javax.swing.UIManager.setLookAndFeel(info.getClassName());
//                    break;
//                }
//            }
//        } catch (ClassNotFoundException ex) {
//            java.util.logging.Logger.getLogger(JFrameClass.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
//        } catch (InstantiationException ex) {
//            java.util.logging.Logger.getLogger(JFrameClass.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
//        } catch (IllegalAccessException ex) {
//            java.util.logging.Logger.getLogger(JFrameClass.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
//        } catch (javax.swing.UnsupportedLookAndFeelException ex) {
//            java.util.logging.Logger.getLogger(JFrameClass.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
//        }
        //</editor-fold>
        
        
    
        /* Create and display the form */
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                JFrameClass jFrameClassFrontEnd = new JFrameClass();
                jFrameClassFrontEnd.setVisible(true);
                jFrameClassFrontEnd.startupMethod();
        
            }
        });
        
    }

//    abstract class PausableSwingWorker<K, V> extends SwingWorker<K, V> {
//        private volatile boolean isPaused;
//        public final void pause() {
//            if (!isPaused() && !isDone()) {
//                System.out.println("Paused? swing worker----------------------------------------------------------------------");
//                isPaused = true;
//                firePropertyChange("paused", false, true);
//            }
//        }
//
//        public final void resume23() {
//            if (isPaused() && !isDone()) {
//                System.out.println("Paused? swing worker----------------------------------------------------------------------");
//                isPaused = false;
//                firePropertyChange("paused", true, false);
//            }
//        }
//
//        public final boolean isPaused() {
//            System.out.println("Paused? swing worker----------------------------------------------------------------------");
//                return isPaused;
//        }
//    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.ButtonGroup buttonGroup1;
    private javax.swing.JButton jButtonClearThisSavedPreference;
    private javax.swing.JButton jButtonDisableTorqueAllMotors;
    private javax.swing.JButton jButtonEnableTorqueAllMotors;
    private javax.swing.JButton jButtonExit;
    private javax.swing.JButton jButtonGoToDesiredStartPosition;
    private javax.swing.JButton jButtonListAllSavedPositions;
    private javax.swing.JButton jButtonMoveMtr1ToPresetHigh;
    private javax.swing.JButton jButtonMoveMtr1ToPresetLow;
    private javax.swing.JButton jButtonMoveMtr2ToPresetHigh;
    private javax.swing.JButton jButtonMoveMtr2ToPresetLow;
    private javax.swing.JButton jButtonMoveMtr3ToPresetHigh;
    private javax.swing.JButton jButtonMoveMtr3ToPresetLow;
    private javax.swing.JButton jButtonMoveMtr4ToPresetHigh;
    private javax.swing.JButton jButtonMoveMtr4ToPresetLow;
    private javax.swing.JButton jButtonRecord1Position;
    private javax.swing.JButton jButtonResetTorqueMtr1;
    private javax.swing.JButton jButtonResetTorqueMtr2;
    private javax.swing.JButton jButtonResetTorqueMtr3;
    private javax.swing.JButton jButtonResetTorqueMtr4;
    private javax.swing.JButton jButtonRunASequence1;
    private javax.swing.JButton jButtonRunASequence2;
    private javax.swing.JButton jButtonStepDownMtr1;
    private javax.swing.JButton jButtonStepDownMtr2;
    private javax.swing.JButton jButtonStepDownMtr3;
    private javax.swing.JButton jButtonStepDownMtr4;
    private javax.swing.JButton jButtonStepUpMtr1;
    private javax.swing.JButton jButtonStepUpMtr2;
    private javax.swing.JButton jButtonStepUpMtr3;
    private javax.swing.JButton jButtonStepUpMtr4;
    private javax.swing.JButton jButtonUpdateSavedPosition1;
    private javax.swing.JLabel jLabel1;
    private javax.swing.JLabel jLabel2;
    private javax.swing.JLabel jLabel3;
    private javax.swing.JLabel jLabel4;
    private javax.swing.JLabel jLabel5;
    private javax.swing.JLabel jLabel6;
    private javax.swing.JLabel jLabel7;
    private javax.swing.JLabel jLabel8;
    private javax.swing.JLabel jLabelMotor1;
    private javax.swing.JLabel jLabelMotor2;
    private javax.swing.JLabel jLabelMotor3;
    private javax.swing.JLabel jLabelMotor4;
    private javax.swing.JLabel jLabel_Increment;
    private javax.swing.JList<String> jListSavedPositions;
    private javax.swing.JRadioButton jRadioButtonHomePositions1;
    private javax.swing.JRadioButton jRadioButtonPriorActualPositions;
    private javax.swing.JRadioButton jRadioButtonPriorTargetPositions;
    private javax.swing.JRadioButton jRadioButtonSavedPositions1;
    private javax.swing.JScrollBar jScrollBarMtr1_Target;
    private javax.swing.JScrollBar jScrollBarMtr2_Target;
    private javax.swing.JScrollBar jScrollBarMtr3_Target;
    private javax.swing.JScrollBar jScrollBarMtr4_Actual;
    private javax.swing.JScrollBar jScrollBarMtr4_Target;
    private javax.swing.JScrollPane jScrollPane1;
    private javax.swing.JScrollPane jScrollPane2;
    private javax.swing.JSlider jSlider1_ActualTemp;
    private javax.swing.JSlider jSlider2_ActualTemp;
    private javax.swing.JSlider jSlider3_ActualTemp;
    private javax.swing.JSlider jSlider4_ActualTemp;
    private javax.swing.JTextArea jTextArea1;
    private javax.swing.JTextField jTextField2;
    private javax.swing.JTextField jTextField3;
    private javax.swing.JTextField jTextField4;
    private javax.swing.JTextField jTextFieldRecordScenarioName;
    private javax.swing.JTextField jTextFieldSavedPosition1;
    private javax.swing.JTextField jTextFieldSpeedMotor1;
    private javax.swing.JTextField jTextFieldSpeedMotor2;
    private javax.swing.JTextField jTextFieldSpeedMotor3;
    private javax.swing.JTextField jTextFieldSpeedMotor4;
    private javax.swing.JTextField jTextFieldTempMtr1;
    private javax.swing.JTextField jTextFieldTempMtr2;
    private javax.swing.JTextField jTextFieldTempMtr3;
    private javax.swing.JTextField jTextFieldTempMtr4;
    private javax.swing.JTextField jTextField_IncrementMtr1;
    private javax.swing.JTextField jTextField_IncrementMtr2;
    private javax.swing.JTextField jTextField_IncrementMtr3;
    private javax.swing.JTextField jTextField_IncrementMtr4;
    private javax.swing.JTextField jTextField_LoadEndEffector;
    private javax.swing.JTextField jTextField_Load_2;
    private javax.swing.JTextField jTextField_Load_3;
    private javax.swing.JTextField jTextField_Load_4;
    private javax.swing.JTextField jTextField_PositionMtr1;
    private javax.swing.JTextField jTextField_PositionMtr2;
    private javax.swing.JTextField jTextField_PositionMtr3;
    private javax.swing.JTextField jTextField_PositionMtr4;
    private javax.swing.JToggleButton jToggleButton1;
    private javax.swing.JToggleButton jToggleButtonMtr1;
    private javax.swing.JToggleButton jToggleButtonMtr2;
    private javax.swing.JToggleButton jToggleButtonMtr3;
    private javax.swing.JToggleButton jToggleButtonMtr4;
    private javax.swing.JToggleButton jToggleButtonPrintDebugging;
    private javax.swing.JToggleButton jToggleButtonRecorYorN;
    private javax.swing.JToggleButton jToggleButton_FollowDriverButton;
    // End of variables declaration//GEN-END:variables
}
