import rxtxrobot.*;
import java.util.Scanner;

//These are the digital pins that are initially free:
// 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
// All 8 analog pins are initially free

//Digital Pins in use
// 3 Motor controller 
// 5 and 6 for DC Motors
// 9 for Servo
// 4 for Ping 1 (takes a digitial pin), in front of robot
// 11 for Ping 2 on either side of robot
// 10 for ping 3 on other side of robot
// 12 and 13 for conductivity

//Analog pins in use
// 0 for bump sensor
// 2 for yellow temp sensor
// 2 and 1 for green anemometer
// 4 and 5 for conductivity sensor

//for motor controller send the pulses with 3 breaks at 200 ms each.

public class sprint2 {
	public static void main(String[] args){

		//assumes the use of an Arduino Uno as of now
		//Can be shifted to use Arduino Nano as necessary

		//Create an instance of an Arduino Robot
		RXTXRobot myRobot = new ArduinoUno();

		//setPort to a literalvalue per docu, I used the one for macs
		//Value is currently calibrated with my Mac
		myRobot.setPort("/dev/tty.usbmodem1411");

		//connect robot using pre-writtenFunctionality
		myRobot.connect();

		//Attach the first DC motor to pin 5 per KNW documentation
		//The 0 indicates that this is the first motor
		//Also do the same (incremented) for second motor in case necessary
		//Program will error out if two motors are not connected though
		myRobot.attachMotor(0,5);
		myRobot.attachMotor(1,6);

		//Attach the servo for the servo tests to pin 9 per documentation
		//Only one servo should be necessary here.
		myRobot.attachServo(0,9);
		//myRobot.setOverrideValidation(true);

		//to drop ping pong ball if necessary
		//myRobot.runMotor(0, 230, 200);
		//From documentation:
			//Note that you do not have to call an attach() method for the ping sensors

		//call displayMenu which will drive the rest of the program
		displayMenu(myRobot);
		/*int condADCReading = 0;
		int tempCond = 0;
		int resistance = 0;
		for (int i = 1; i<=5; i++){
			//myRobot.refreshAnalogPins();
			//myRobot.refreshDigitalPins();
			tempCond = myRobot.getConductivity();
			
			System.out.println(tempCond);
			condADCReading  += tempCond;
		}
		condADCReading = condADCReading/5;
		System.out.println("average is: " + condADCReading);
		myRobot.close();
		System.exit(0);
*/
	}
	//Will edit once we decide to do conductivity or anenometer
	static void displayMenu(RXTXRobot myRobot){
		Scanner kbScanner = new Scanner(System.in);
		int userChoice = 0;
		while(userChoice!=7){
			displayChoices();
			userChoice = kbScanner.nextInt();
			switch(userChoice){
				case 1: moveServoWrapper(kbScanner, myRobot);
						break;
				case 2: moveForward(myRobot);
						break;
				case 3: System.out.println("Distance of: " + readDistance(myRobot) + " centimeters");
						break;
				case 4: runMotorindefinitely(myRobot);
						break;
				case 5: dipslayTemperature(myRobot);
						break;
				case 6: displayOtherSensor(myRobot);
						break;
				case 7: myRobot.close();
						System.exit(0);
						break;
				default: System.out.println("Invalid choice, please try again");
						break;
			}
		}
	}
	//Will edit once we decide to do conductivity or anenometer
	static void displayChoices(){
		System.out.println("Please enter a number between 1 and 6 to chose from the options below");
		System.out.println("1. Move a servo to a given angle");
		System.out.println("2. Move 3 Meters");
		System.out.println("3. Read the distance from the ping sensor");
		System.out.println("4. Run until the bump sensor is triggered");
		System.out.println("5. Display a temperature reading");
		System.out.println("6. Display a reading from the other sensor");
		System.out.println("7. Exit the program");
	}
	//angle in degrees
	static void moveServoWrapper(Scanner kbScanner, RXTXRobot myRobot){
		//System.out.println("Please enter the number of degrees you would like the servo to turn");
		
		//pre-defining this to 0 in order to move the boom arm autonomously 
		int angle = 0;
		//header for moveServo:
		//public void moveServo(int servo, int position) where position is a number of degrees
		myRobot.moveServo(RXTXRobot.SERVO1, angle);

		//have the boom arm up for 5 seconds by sleeping the main thread
		try {
			Thread.sleep(5000);                 //unit of time for .sleep call is milliseconds
		} catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}
		//if negatives are handled
		//myRobot.moveServo(0, -angle);
		myRobot.moveServo(0,85);
	}

	static void moveForward(RXTXRobot myRobot) {
		boolean blockerLifted = false;
		boolean initApproachFinished = false;
		boolean reachedRamp = false;
		boolean gapFoundInit = false;
		boolean gapFoundFin = false;
		Scanner kbScanner = new Scanner(System.in);
		//headers for runMotor are :
		//public void  runMotor(int motor, int speed, int time)
		//public void runMotor(int motor1, int speed1, int motor2, int speed2, int time)
		//we will have to play with speed/time to get to exactly 3 meters
		//25000 is 25 seconds for clarity

		//this command moves from the starting box
		myRobot.runMotor(0, 119, 1, 115, 1400); // 140 135 

		// this command turns torward the ramp
		myRobot.runMotor(0, 130, 1, 0, 1850); // 160, 0

		//this command is intended to run the robot straight onto the top of the ramp
		//Speed may need to be revised based on the charge of the battery, also the amount of time the robot is set to run for
		do{		
				if((!initApproachFinished) && readDistance(myRobot) > 30)
					myRobot.runMotor(0, 119, 1, 123, 500); //
				else if(readDistance(myRobot) <= 30) //75 is just a guess
					initApproachFinished = true;
				else
					blockerLifted = true;
			}
			while(!blockerLifted);

			//move straight for momentum up ramp
			myRobot.runMotor(0, 210, 1, 196, 2750);

		/*while(!reachedRamp){
			if (readDistance(myRobot,1) < 80)
				myRobot.runMotor(0, 119, 1, 123, 500);
			else
				reachedRamp = true;
		}*/
		//this command should tell the boom arm to raise then lower
		// NOT LIFTING BOOM ARM FOR NOW
		moveServoWrapper(kbScanner, myRobot);

		//this command tells the robot to move away from the ramp
		//myRobot.runMotor(0, 0, 1, 130, 2300);
		//myRobot.runMotor(0, 119, 1, 123, 2500);
		while(!gapFoundInit){
			if((readDistance(myRobot,1) < 50))
				myRobot.runMotor(0, 230, 1, 180, 70);
			else
				gapFoundInit = true;

		}
		myRobot.runMotor(0, 0, 1, 140, 2200);
		myRobot.runMotor(0, 119, 1, 123, 2500);
		myRobot.runMotor(0, 0, 1, 140, 2200);
		myRobot.runMotor(0, 119, 1, 123, 1200);
		myRobot.runMotor(0, 130, 1, 0, 2450); // 160, 0
		/*while(readDistance(myRobot) > 65)
			myRobot.runMotor(0, 230, 1, 180, 80);*/
		myRobot.runMotor(0, 120, 1, 135, 2000);
		/*while(readDistance(myRobot) < 118)
			myRobot.runMotor(0, -180, 1, -175, 80);*/
		myRobot.runMotor(0, 130, 1, 0, 2650); // 160, 0
		myRobot.runMotor(0, 210, 1, 196, 6000);
		/*while((readDistance(myRobot,1) > 35))
				myRobot.runMotor(0, 130, 1, 0, 70);*/

		//myRobot.runMotor(0, 240, 1, 230, 2800);

		
		//myRobot.runMotor(0, 150, 1, 115, 2000);
		//myRobot.runMotor(0, 200, 1, 150, 3500);
		//myRobot.runMotor(0, 499, 1, 383, 2500);
		//myRobot.runMotor(0, 50, 1, 150, 1800);

	}
	static int readDistance(RXTXRobot myRobot){
		//Per documentation:
			/* Whenever you want to get the
     		* value of a digital sensor you should call refreshDigitalPins first.*/
    myRobot.refreshDigitalPins();
    return myRobot.getPing(4); //where 4 is a pin
	} 
	static int readDistance(RXTXRobot myRobot, int flag){
		//Per documentation:
			/* Whenever you want to get the
     		* value of a digital sensor you should call refreshDigitalPins first.*/
    myRobot.refreshDigitalPins();
    return myRobot.getPing(11); //where 4 is a pin
	} 
	//goes until bump sensor value changes
	static void runMotorindefinitely(RXTXRobot myRobot){
	//again, speed could be faster, we will have to test. inputting 0 for time says run infinitely
	//until another function call
	myRobot.runMotor(0, 150, 1, 150, 0);
	//To make sure the first loop runs 
	int bumpVoltage = 501;
		while (bumpVoltage > 500){ 
		//took 550 for reading from code found on forums, 
		//we will probably need to test what a more exact reading would be, but this might work
			myRobot.refreshAnalogPins();
			//getValue is necessary here as analogPin is a type
			bumpVoltage = myRobot.getAnalogPin(0).getValue();
		}
	//Call runMotor again to stop movement per documentation
	myRobot.runMotor(0, 0, 1, 0, 1);
	}

	//This fxn is not as simple as it would appear at first
	//Keep in mind that  for analog sensors:
	//sensor reading = slope * voltage + intercept or something like that
	//Updated with most recent temp sensor calibration
	static void dipslayTemperature(RXTXRobot myRobot){
		//myRobot.refreshAnalogPins();
		//System.out.println(myRobot.getAnalogPin(4).getValue());
		System.out.println("Temperature is " + (getCalibratedAnalogReading(-9.17,804, myRobot, 2)) + " degrees celsius");
		return;
	}
	//This fxn is extremely similar to the temp fxn, just has placeholders until we calibrate/ deide which sensor we will use.
	//-7.08,457
	static void displayOtherSensor(RXTXRobot myRobot){
		System.out.println("Measurement is " + Math.abs((getCalibratedAnalogReading(-8.3, 766, myRobot, 4)) - (getCalibratedAnalogReading(-9.17,804, myRobot, 2) + 3.6)) + " Meters per seconds");
	}
	//Called by othser display sensor fxns
	static double getCalibratedAnalogReading (double slope, double intercept, RXTXRobot myRobot, int pin){
		double sumTemps = 0.0;
		double avgTemp = 0.0;
		double measurement = 0.0;
		//To get the best temperature reading, it makes sense to take the average of a bunch (10) of readings
			for(int i =0; i<10; i++){
				myRobot.refreshAnalogPins();
				sumTemps += myRobot.getAnalogPin(pin).getValue();
			}
		avgTemp = sumTemps/10;
		measurement = (avgTemp - intercept)/(slope); //formula for temp calibration (avgtemp - intercept)/(slope)
		return measurement;
	}
	/*static void navigate (RXTXRobot myRobot, boolean leftOrRight){
		jf(leftOrRight){
			boolean blockerLifted = false;

			//move right and forward until you get to the idling point 
			//need to calibrate still
			myRobot.runMotor(0, 115, 1, 170, 6500);
			do{
				if(readDistance(myRobot) < 75) //75 is just a guess
					blockerLifted = false;
			}
			while(!blockerLifted)

			//again just a guess for how long the robot might need to run in order to go the appropriate distance
			myRobot.runMotor(0, 115, 1, 150, 5000);
			//turn right again
			myRobot.runMotor(0, 115, 1, 170, 2000);
			//going forward again theoretically, this should put us at the beginning of the possible openings for the bridge.
			myRobot.runMotor(0, 115, 1, 150, 5000);

			//now finding the gap in the bridge method
			//should center the robot in between the gaps in the bridge 
			findGap(myRobot);
	}
	}*/

	static void findGap (RXTXRobot myRobot){


	}
}