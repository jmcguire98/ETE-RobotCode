import rxtxrobot.*;
import java.util.Scanner;

//These are the digital pins that are initially free:
// 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
// All 8 analog pins are initially free

//Digital Pins in use
// 5 and 6 for DC Motors
// 9 for Servo
// 4 for Ping (takes a digitial pin)

//Analog pins in use
// 0 for bump sensor


public class sprint2 {
	public static void main(String[] args){

		//assumes the use of an Arduino Uno as of now
		//Can be shifted to use Arduino Nano as necessary

		//Create an instance of an Arduino Robot
		RXTXRobot myRobot = new ArduinoUno();

		//setPort to a literalvalue per docu, I used the one for macs
		//May have to change value, can check using ls /dev | grep usb
		myRobot.setPort("/dev/tty.usbmodem411");

		//connect robot using prewrittenFunctionality
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

		//From documentation:
			//Note that you do not have to call an attach() method for the ping sensors

		//call displayMenu which will drive the rest of the program
		displayMenu(myRobot);
		
	}
	//Will edit once we decide to do conductivity or anenometer
	static void displayMenu(RXTXRobot myRobot){
		Scanner kbScanner = new Scanner(System.in);
		int userChoice = 0;
		while(userChoice !=6){
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
				case 5: dipslayTemperature();
						break;
				case 6: break;
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
		System.out.println("6. Exit the program");
	}
	//angle in degrees
	static void moveServoWrapper(Scanner kbScanner, RXTXRobot myRobot){
		System.out.println("Please enter the number of degrees you would like the servo to turn");
		int angle = kbScanner.nextInt();
		//header for moveServo:
		//public void moveServo(int servo, int position) where position is a number of degrees
		myRobot.moveServo(0, angle);
	}

	static void moveForward(RXTXRobot myRobot) {
		//headers for runMotor are :
		//public void  runMotor(int motor, int speed, int time)
		//public void runMotor(int motor1, int speed1, int motor2, int speed2, int time)
		//we will have to play with speed/time to get to exactly 3 meters
		//25000 is 25 seconds for clarity
		myRobot.runMotor(0, 150, 1, 150, 25000);

	}
	static int readDistance(RXTXRobot myRobot){
		//Per documentation:
			/* Whenever you want to get the
     		* value of a digital sensor you should call refreshDigitalPins first.*/
    myRobot.refreshDigitalPins();
    return myRobot.getPing(4); //where 4 is a pin
	} 
	//goes until bump sensor value changes
	static void runMotorindefinitely(RXTXRobot myRobot){
	//again, speed could be faster, we will have to test. inputting 0 for time says run infinitely
	//until another function call
	myRobot.runMotor(0, 150, 1, 150, 0);
	int bumpVoltage = 0;
		while (bumpVoltage < 550){ 
		//took 550 for reading from code found on forums, 
		//we will probably need to test what a more exact reading would be, but this might work
			myRobot.refreshAnalogPins();
			//getValue is necessary here as analogPin is a type
			bumpVoltage = myRobot.getAnalogPin(2).getValue();
		}
	//Call runMotor again to stop movement per documentation
	myRobot.runMotor(0, 0, 1, 0, 1);
	}

	static void dipslayTemperature(){
		int a = 0;
	}
}