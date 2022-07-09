/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package javaapplication1;

import static java.lang.Thread.sleep;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Allan
 */
public class MotorClass {
    Dynamixel dynamixel = null;
    int error_counter                   = 0;
    int port_num                        = 0;
    int motorNumber                     = 0;
    short goalPosMotor                  = 1;
    String DEVICENAME                   = "COM3";  //linux -> "/dev/ttyUSB0";              
                                                   // Check which port is being used on your controller
                                                   // ex) "COM1"   Linux: "/dev/ttyUSB0"

    //int port_num = dynamixel.portHandler(DEVICENAME);
    // baudrate needs to be the same for all motors because it is set at the port level????
    private int[] BaudRate                       = new int[] {1000000,1000000,1000000,1000000,1000000,1000000,1000000,1000000};
    // baudrate needs to be the same for all motors because it is set at the port level????
    
    private String mBatch_time_stamp_into_mysql  = "initialized_in_MotorClass";
    private short[] ADDR_MX_TORQUE_ENABLE        = new short[]{24,24,24,24,24,24,24,24}; // Control table address is different in Dynamixel model. [index size]
    private byte[] DXL_ID                        = new byte[]{16,12,2,13, 6,5,1,3};    // Dynamixel ID: 16 = End Effector; 12 = wrist; 2 = elbow; 13 = sholder; 
    private int[] PROTOCOL_VERSION               = {1,1,1,1,1,1,1,1};   // See which protocol version is used in the Dynamixel
    private byte[] TORQUE_ENABLE                 = {1,1,1,1,1,1,1,1};           // Value for enabling the torque
    private byte[] TORQUE_DISABLE                = {0,0,0,0,0,0,0,0};                  // Value for disabling the torque
    private short[] ADDR_MX_GOAL_POSITION        = {30,30,30,30,30,30,30,30};  // 182
    private short dxl_goal_position              = 300;
    private short[] ADDR_MX_PRESENT_POSITION     = new short[] {36,36,36,36,36,36,36,36};
    private short[] ADDR_MX_PRESENT_LOAD         = new short[] {40,40,40,40,40,40,40,40};
    private short[] ADDR_MX_PresentTemperature   = {43,43,43,43,43,43,43,43};
    private int COMM_SUCCESS                     = 0;                   // Communication Success result value
    private int COMM_TX_FAIL                     = -1001;               // Communication Tx Failed
    private short[] ADDR_SET_RETURN_DELAY        = new short[]{5,5,5,5,5,5,5,5}; // NOTE - torque must be off (set to zero 0) before this will work (EEPROM area)
    private byte[] RETURN_DELAY                  = new byte[] {1,1,1,1,1,1,1,1}; // NOTE - torque must be off (set to zero 0) before this will work (EEPROM area)
    private byte dxl_error = 0;
    private int[] Target_Torque                  = {600,600,600,600,0,0,0,0}; // between 0 and 1023
    private int[] Target_MovingSpeed             = {150,20,15,8,0,0,0,0}; // between 0 and 1023
    
    private short[] ADDR_movingspeed             = {32,32,32,32,32,32,32,32};
    private short[] ADDR_SET_TORQUE              = {34,34,34,34,34,34,34,34};  // address to set Torque. 

    private boolean mTogglePrintDebugValue = false;
    
    private int dxl_comm_result = COMM_TX_FAIL;
           
    public void setBatchTime(String Batch_time_stamp_into_mysql){
        mBatch_time_stamp_into_mysql = Batch_time_stamp_into_mysql;
    }
    
    public void moveMotor(short dxl_goal_position) {
        this.dxl_goal_position = dxl_goal_position;
        
        if(mTogglePrintDebugValue){
        System.out.println("moving motor "+motorNumber+" to position "+dxl_goal_position); 
        }
        
        // current actual position:
//        short position = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_PRESENT_POSITION[motorNumber]);
//        System.out.print("; "+ motorNumber+" position = "+position);
        int tries = 0;
        while (tries<8){
            tries = tries + 1;
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_GOAL_POSITION[motorNumber], dxl_goal_position);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber])) != COMM_SUCCESS)
            {
                error_counter = error_counter+1;
                //if(mTogglePrintDebugValue){
                try {
                    sleep(6);
                } catch (InterruptedException ex) {
                    Logger.getLogger(MotorClass.class.getName()).log(Level.SEVERE, null, ex);
                }
                if(mTogglePrintDebugValue){
                    System.err.println("error msg: "+tries +" motorNumber " + motorNumber + " " + dxl_comm_result + " " + dynamixel.getTxRxResult(PROTOCOL_VERSION[motorNumber], dxl_comm_result) + " moveMotor (goal position) NOT done correctly ");
                    //System.err.println("error msg: " + motorNumber+" <- motorNumber.  moveMotor (goal position) NOT done correctly "+dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber]));
                    System.err.println("errors " + error_counter +"; motorNumber " + motorNumber);
                }
                //}
            }
            else {
                tries = 10;
                //System.out.println("Success");//. dxl_comm_result = " + dxl_comm_result);
            }
        }
      
        while(true){
            short moving    = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], (short)46);
            //short position  = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_PRESENT_POSITION[motorNumber]);
            //short load      = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_PRESENT_LOAD[motorNumber]);
            String movingText = "Not moving";
            if(moving==1){
                movingText = "Moving";
            }

            //if(mTogglePrintDebugValue){
            //System.out.println("Motor "+motorNumber+", DXL_ID "+DXL_ID[motorNumber]+": "+movingText +"; "+"Load = "+load+"; position "+position+ " Target_Torque "+Target_Torque[motorNumber]);
            //}

//            MysqlLogger.put(MysqlLogger.Type.BETTER, position, position, position, position, motorNumber, motorNumber, motorNumber, motorNumber, mBatch_time_stamp_into_mysql);
//            
            boolean stopNow = true; // set this to false if we want this to loop 'while' - if it loops it waits for one motor to finish moving before letting the next motor start. 
                                    //it's blocking longer than it needs to - technically it's still blocking but not very long.
            if(moving==0){
                stopNow=true;
            }
            if(stopNow){
                if(mTogglePrintDebugValue){
                System.err.println("break now for motor: "+motorNumber);
                }
                break;
            }
        }
        
        
    }
    
    /**
     * when mTogglePrintDebugValue = true the program will print out more debugging messages
     */
    public void printDebugging(boolean togglevalue) {
        mTogglePrintDebugValue = togglevalue;
    }
    
    /**
     * pings the dynamixel to determine the position (rotational position) of the device
     * @return 
     */
    public short readPosition() {
        //System.out.println("readPosition port_num = "+port_num+" "+PROTOCOL_VERSION[motorNumber]+" "+ DXL_ID[motorNumber]+" "+ADDR_MX_PRESENT_POSITION[motorNumber]);
        //port_num = dynamixel.portHandler(DEVICENAME);
        // current actual position:
        short position = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_PRESENT_POSITION[motorNumber]);
        if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber])) != COMM_SUCCESS)
        {
            if(mTogglePrintDebugValue){
            System.err.println(motorNumber+" <<motorNumber.  Position Not read correctly "+dxl_comm_result+" "+dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber]));
            }
        }
        //System.out.println("readPosition "+position);
        return position;
    }
        
    /**
     * pings the dynamixel to determine the load on the device
     * @return 
     */
    public Long readLoad() {
        //short load_short = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_PRESENT_LOAD[motorNumber]);
        double load_short = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_PRESENT_LOAD[motorNumber]);
        Long load = (long)load_short;
        load = load.valueOf(load);
        if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber])) != COMM_SUCCESS)
        {
            if(mTogglePrintDebugValue){
                System.err.println(load+ " <-load. "+motorNumber + " <<motorNumber. Load Not read correctly"+dxl_comm_result+" "+dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber]));
            }
            Long loadReadFail = (long)999999;
            return loadReadFail;
        }
        else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION[motorNumber])) != 0)
        {
          System.err.println(dxl_error+" dxl_error is here --------------------------------------");
          dynamixel.printRxPacketError(PROTOCOL_VERSION[motorNumber], dxl_error);
        }
        if(load>=1024){ // per documentation; load of 1024 or greater (up to 2,047) is negative (CW). Values from 0-1023 are increasingly positive (CCW). https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/#present-load
            load = 1024-load;
        }
        return load;
    }
    
    /**
     * pings the dynamixel to determine the temperature inside the device
     * @return 
     */
    public short readPresentTemperature() {
        short temperature = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_PresentTemperature[motorNumber]);
        if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber])) != COMM_SUCCESS)
        {
            if(mTogglePrintDebugValue){
                System.err.println(temperature+ " <-Temperature. "+motorNumber + " <<motorNumber. Temperature Not read correctly"+dxl_comm_result+" "+dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber]));
            }
            short ReadFail = (short)50; // some kind of error message (probably not reading temp correctly; but not sure). Setting to a normal value within a reasonable range to avoid triggering a high temp warning.
            return ReadFail;
        }
        else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION[motorNumber])) != 0)
        {
          System.err.println(dxl_error+" dxl_error temperature reading erro is here --------------------------------------");
          dynamixel.printRxPacketError(PROTOCOL_VERSION[motorNumber], dxl_error);
        }
        
        return temperature;
    }
    
    /**
     * uses the standardized movingSpeed value that is hard coded
     */
    public void setMovingSpeed(){
        // torque is how hard it pushes; speed is how fast speed 35 is ~3 RPM
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_movingspeed[motorNumber], (short)Target_MovingSpeed[motorNumber]); // moving speed set to 20
        int movingSpeedSetting2 = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_movingspeed[motorNumber]);
        
        if (mTogglePrintDebugValue){
        System.out.println("moving speed "+motorNumber+" set to "+movingSpeedSetting2);
        }
    }
 
    /**
     * supply a movingSpeed value
     * @param movingSpeed 
     */
    public void setMovingSpeed(int movingSpeed){
        // torque is how hard it pushes; speed is how fast speed 35 is ~3 RPM
        int tries = 0;
        while (tries<8){
            tries = tries + 1;  
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_movingspeed[motorNumber], (short)movingSpeed); // moving speed set to 20
        if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber])) != COMM_SUCCESS)
        {
                error_counter = error_counter+1;
                //if(mTogglePrintDebugValue){
                try {
                    sleep(10);
                } catch (InterruptedException ex) {
                    Logger.getLogger(MotorClass.class.getName()).log(Level.SEVERE, null, ex);
                }
                if(mTogglePrintDebugValue){
                    System.err.println("error msg: "+tries +" motorNumber " + motorNumber + " " + dxl_comm_result + " " + dynamixel.getTxRxResult(PROTOCOL_VERSION[motorNumber], dxl_comm_result) + " setSpeed NOT done correctly ");
                    System.err.println("errors " + error_counter +"; motorNumber " + motorNumber);
                }
            }
            else {
                tries = 10;
                //System.out.println("Success setting moving speed on motor # " + motorNumber);//. dxl_comm_result = " + dxl_comm_result);
            }
        }        
        int movingSpeedSetting2 = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_movingspeed[motorNumber]);
        System.out.println("moving speed "+motorNumber+" set to "+movingSpeedSetting2);
    }
    
    /**
     * will use the standard torque setting with this method
     */
    public void setTorque() { 
        //int port_num = dynamixel.portHandler(DEVICENAME);
        // torque is how hard it pushes; speed is how fast   
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_SET_TORQUE[motorNumber], (short) Target_Torque[motorNumber]);
        if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber])) != COMM_SUCCESS)
        {
            System.out.println("setTorque Error: "+dxl_comm_result+" "+dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber]));
            System.out.println("  details: " + dynamixel.getTxRxResult(PROTOCOL_VERSION[motorNumber], dxl_comm_result));
        }
        else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION[motorNumber])) != 0)
        {
            System.out.println("here2");
          dynamixel.printRxPacketError(PROTOCOL_VERSION[motorNumber], dxl_error);
        }
        
        int TorqueSetting = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_SET_TORQUE[motorNumber]);
        System.out.println("Torque "+motorNumber+" set to "+TorqueSetting);
    }
    
    /**
     * this will use a custom torque value supplied by the call to the method
     * @param torque 
     */
    public void setTorque(int torque) { 
        //int port_num = dynamixel.portHandler(DEVICENAME);
        int mtorque = torque; // torque is how hard it pushes; speed is how fast   
        dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_SET_TORQUE[motorNumber], (short) mtorque);
        if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber])) != COMM_SUCCESS)
        {
            System.out.println("setTorque(int torque) error "+dxl_comm_result+" "+dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber]));
            System.out.println("   details: "+dynamixel.getTxRxResult(PROTOCOL_VERSION[motorNumber], dxl_comm_result));
        }
        else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION[motorNumber])) != 0)
        {
          System.out.println("here2");
          dynamixel.printRxPacketError(PROTOCOL_VERSION[motorNumber], dxl_error);
        }
        
        int TorqueSetting = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_SET_TORQUE[motorNumber]);
        System.out.println("Torque "+motorNumber+" set to "+TorqueSetting);
    }
    
    public void getTorque(){
        int TorqueSetting = dynamixel.read2ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_SET_TORQUE[motorNumber]);
        System.out.println("Torque "+motorNumber+" set to "+TorqueSetting);
    }
    
    public void disableTorque() {
        int tries = 0;
        while (tries<8){
            tries = tries + 1;        
        
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_TORQUE_ENABLE[motorNumber], TORQUE_DISABLE[motorNumber]);
        if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber])) != COMM_SUCCESS)
            {
                error_counter = error_counter+1;
                //if(mTogglePrintDebugValue){
                try {
                    sleep(10);
                } catch (InterruptedException ex) {
                    Logger.getLogger(MotorClass.class.getName()).log(Level.SEVERE, null, ex);
                }
                if(mTogglePrintDebugValue){
                    System.err.println("error msg: "+tries +" motorNumber " + motorNumber + " " + dxl_comm_result + " " + dynamixel.getTxRxResult(PROTOCOL_VERSION[motorNumber], dxl_comm_result) + " moveMotor (goal position) NOT done correctly ");
                    //System.err.println("error msg: " + motorNumber+" <- motorNumber.  moveMotor (goal position) NOT done correctly "+dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber]));
                    System.err.println("errors " + error_counter +"; motorNumber " + motorNumber);
                }
                //}
            }
            else {
                tries = 10;
                System.out.println("Success disabling torque on motor # " + motorNumber);//. dxl_comm_result = " + dxl_comm_result);
            }
        }
            if (mTogglePrintDebugValue){
            System.out.println("here attempting to disable "+dynamixel.getTxRxResult(PROTOCOL_VERSION[motorNumber], dxl_comm_result) + " "+dxl_comm_result+" "+dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber]));
            }
        
        else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION[motorNumber])) != 0)
        {
          System.out.println("here2 attempting to disable");
          dynamixel.printRxPacketError(PROTOCOL_VERSION[motorNumber], dxl_error);
        }
    }
    
    public void enableTorque() {
        // Enable Dynamixel Torque
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_TORQUE_ENABLE[motorNumber], TORQUE_ENABLE[motorNumber]);
    }
    
    public void makeDynamix(Dynamixel dynamixel, int motorNumber) {
        this.dynamixel = dynamixel;
        this.motorNumber = motorNumber;
        // baudrate needs to be the same for all motors because it is set at the port level????
        //System.err.println("BaudRate: "+BaudRate[motorNumber]+"; motorNumber: "+motorNumber);
        //dynamixel.setBaudRate(port_num, BaudRate[motorNumber]);
        
        // set return delay (response speed). Torque must be off for this to be set/adjusted (EEPROM).
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_SET_RETURN_DELAY[motorNumber], RETURN_DELAY[motorNumber]);
        if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber])) != COMM_SUCCESS)
            {
              dynamixel.printTxRxResult(PROTOCOL_VERSION[motorNumber], dxl_comm_result);
            }
        else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION[motorNumber])) != 0)
            {
              dynamixel.printRxPacketError(PROTOCOL_VERSION[motorNumber], dxl_error);
            }
        else
            {
              System.out.printf("Dynamixel#%d return delay has been successfully set\n", motorNumber);
            }
        
        // Enable Dynamixel Torque
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION[motorNumber], DXL_ID[motorNumber], ADDR_MX_TORQUE_ENABLE[motorNumber], TORQUE_ENABLE[motorNumber]);
                if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION[motorNumber])) != COMM_SUCCESS)
            {
              dynamixel.printTxRxResult(PROTOCOL_VERSION[motorNumber], dxl_comm_result);
            }
        else if ((dxl_error = dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION[motorNumber])) != 0)
            {
              dynamixel.printRxPacketError(PROTOCOL_VERSION[motorNumber], dxl_error);
            }
        else
            {
              System.out.printf("Dynamixel#%d Torque has been successfully enabled\n", motorNumber);
            }
    }
}