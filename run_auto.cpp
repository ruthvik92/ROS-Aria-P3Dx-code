#include "botConnector.h"
#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <sstream>
#include <math.h>

using namespace cv;

BotConnector::BotConnector(int argc, char **argv)
{
	Aria::init();
	parser=new ArArgumentParser(&argc, argv);
	parser->loadDefaultArguments();
	robotConnector=new ArRobotConnector(parser, &robot);
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);
	gotoPoseAction = new ArActionGoto("goto");
	robot.addAction(gotoPoseAction,95);
}


/**
 * connect() take cares of all the things needed to connect to the bot.
 * @args: None
 * @return: Connection successfull or not.
 */
bool BotConnector::connect()
{
	bool success = true; //Let's be optimistic and not pesimistic.
	if(!robotConnector->connectRobot())
	{
		ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
		if(parser->checkHelpAndWarnUnparsed())
		{
			// -help not given
			Aria::logOptions();
			Aria::exit(1);
			success = false;
		}
	}
	else if (!Aria::parseArgs())
	{
		Aria::logOptions();
		Aria::shutdown();
		success = false;
	}

	return success;
}

/**
 * getReadings() returns readings for all the sonar present on the robot.
 * To get the relation between sonar number and direction see getAngles().
 * @args: None
 * @return: A std::vector<int> containing all the sonar readings.
 */
vector<int> BotConnector::getReadings()
{

	int count = robot.getNumSonar();
	vector<int> Z;

	for(int i=0; i<count;i++){
		int reading =  robot.getSonarRange(i);
		Z.push_back(reading);
		//        printf("sensor reading=%d\n",reading);
	}
	//    ArSensorReading *reading = robot.getSonarReading(0);
	//    unsigned int gr = reading->getRange();
	//    ArPose lPose = reading->getLocalPose(), pose=reading->getPose();
	//    double lx,ly;
	//    lx=pose.getX();
	//    ly=pose.getY();
	//    unsigned int dd = sqrt(lx*lx + ly*ly);
	//    printf("lx=%f, ly=%f, r=%u, calc=%u\n",lx,ly,gr, dd);
	return Z;
}

/**
 * getAngles() gives the angle for all the sonar present w.r.t the robot.
 * @args: None
 * @return: A std::vector<int> containing all the sonar angles.
 */
float x,y,th;
vector<int> BotConnector::getAngles()
{

	vector<int> angle;
	int count = robot.getNumSonar();
	const ArRobotParams *roboParams = robot.getRobotParams();
	x=robot.getX();
	y=robot.getY();
	th=robot.getTh();
//	printf("robot angle=%f, robot pose=(%f, %f)\n",robot.getTh(), robot.getX(), robot.getY());
	for(int i=0; i<count;i++)
	{
		int sonarDir = roboParams->getSonarTh(i)+round(robot.getTh());
		angle.push_back(sonarDir);
		//        printf("angle[%d]=%d\t",i,sonarDir);
	}
	printf("\n");
	return angle;
}

/**
 * moveRobotTo() rotates the robot by the mentioned theta and displaces the robot
 * by the given distance in the new heading.
 * @args:
 * 	double r - the distance in Millimeter by which robot is to be moved. Default=1000.
 * 	double th - the angle in degrees by which the robot is to be rotated. Default=45
 * @return: None
 */
void BotConnector::moveRobot(double r, double th)
{
	ArTime start;

	robot.runAsync(true);
	robot.enableMotors();

	//rotating robot
	//    robot.lock();
	//    robot.setDeltaHeading(th);
	//    robot.unlock();
	//    ArUtil::sleep(10000);

	if( th!=0 )
	{
		robot.lock();
		robot.setHeading(th+robot.getTh());
		robot.unlock();
		start.setToNow();
		while (1)
		{
			robot.lock();
			if (robot.isHeadingDone(1))
			{
				printf("directMotionExample: Finished turn\n");
				robot.unlock();
				break;
			}
			if (start.mSecSince() > 15000)
			{
				printf("directMotionExample: Turn timed out\n");
				robot.unlock();
				break;
			}
			robot.unlock();
			ArUtil::sleep(100);
		}
	}

	//moving robot
	if( r!=0 )
	{
		robot.lock();
		robot.move(r);
		robot.unlock();
		start.setToNow();
		while (1)
		{
			robot.lock();
			if (robot.isMoveDone())
			{
				printf("directMotionExample: Finished distance\n");
				robot.unlock();
				break;
			}
			if (start.mSecSince() > 15000)
			{
				printf("directMotionExample: Distance timed out\n");
				robot.unlock();
				break;
			}
			robot.unlock();
			ArUtil::sleep(50);
		}
	}

	//    ArLog::log(ArLog::Normal, "Going to four goals in turn for %d seconds, then cancelling goal and exiting.", duration/1000);
	//    ArTime start;
	//    start.setToNow();
	//    while(Aria::getRunning())
	//    {
	//        if(robot.isHeadingDone())
	//        {
	//            robot.unlock();
	//            printf("turned in %ld\n",start.mSecSince());
	//            break;
	//        }
	//        gotoPoseAction->setGoal(pos);

	//        if(gotoPoseAction->haveAchievedGoal())
	//        if(start.mSecSince() >= duration)
	//        {
	//            gotoPoseAction->cancelGoal();
	//            printf("time out :(\n");
	//            break;
	//        }
	//        else if(gotoPoseAction->haveAchievedGoal())
	//        {
	//            gotoPoseAction->cancelGoal();
	//            printf("task in time %ld\n",start.mSecSince());
	//            break;
	//        }
	//    }
	printf("Movement Done\n");
}

/**
 * setRobotVelocity() sets the robot's right and left wheel velocity for a specified time.
 * @args:
 * 	double vr - Velocity of right wheel in Milimeter/sec. Default = 200.
 * 	double vl - Velocity of left wheel in Milimeter/sec. Default = 200.
 * 	unsigned int time - Time in MiliSecond for which the robot is to move. Default = 2000.
 * @return: None.
 */
void BotConnector::setRobotVelocity(double vr, double vl, unsigned int time)
{
	robot.runAsync(true);
	robot.enableMotors();

	printf("Setting vl=%.2lf mm/sec, vr=%.2lf mm/sec, then sleeping for %d seconds\n", vl, vr, time/1000);
	robot.lock();
	robot.setVel2(vr, vl);
	robot.unlock();

	//Just to let you know that in Aria TIME exists!!
	//ArTime start;
	//start.setToNow();
	//while (start.secSince()<2)
	//{
	//    ArUtil::sleep(50);
	//}


	ArUtil::sleep(time);//argument passed is in MilliSecond.
	robot.lock();
	robot.stop();
	robot.unlock();
}

/**
 * disconnect() stops the robot and severs all the connections to it. Like Father disowns the son in hindi movies.
 * @args: None
 * @return: A std::vector<int> containing all the sonar angles.
 */
int BotConnector::disconnect()
{
	robot.stopRunning();
	robot.waitForRunExit();
	return 0;
}

int main(int argc, char **argv)
{
	float a,p;
	int k, q=0;
	BotConnector bc(argc, argv);
	if( bc.connect() )
	{
		printf( "Connected to Robot\n" );
	}
	else
	{
		printf( "Connection attempt to Robot failed, Sorry :(\n" );
		return EXIT_SUCCESS;
	}

	VideoCapture cap(0);
	if(!cap.isOpened())
		printf("lost connection with device\n");


	string c;
	
	char str[200];
	bc.moveRobot(10,1);
	ArUtil::sleep(100);
	int r = 0;
	int key;
	//strcpy(str,"/workspace/karthik/RRC/Project_2011_vlocal/test_run/tedataset/file1.txt");
	strcpy(str,"data/file1.txt");
	FILE *filep;
	float x1=0,y1=0;
	while(1){
		int count = 0;
		char comm;
		printf("Press c to start the run\n :");
		scanf("%c",&comm);
		if(comm=='c'){
		while(count<=25){
		Mat frame;
		namedWindow("display",1);
		cap >> frame;
		imshow("display",frame);
//		printf("Press c to capture the image\n");
//		key = waitKey(30);
//		if(key=='r')
//			printf("r is pressed\n");
		key = waitKey(3);
		printf("key is %d", key);
		
		long int interval = 0;
			while(interval<=1000000){
				interval++;
		
//		if(waitKey(30)==1048675){
			//			cout << "captured\n";
			stringstream ss;
			q++;
			//ss<<"/workspace/karthik/RRC/Project_2011_vlocal/test_run/tedataset/image_node"<<r<<"_"<<q<<".jpg";
			ss<<"data/node_"<<r<<"_"<<q<<".jpg";
			c=ss.str();
//			printf("1111\n");
			imwrite(c.c_str(),frame);

//			printf("enter position and orientation:\n");
//			scanf("%f %f",&p,&a);
			p = 0.2;
			a = 0;
			filep = fopen(str,"a");
			printf("p=%f, a=%f\n", p, a);
			fprintf(filep,"node%d\ngiven translation= %f rotation= %f\n",r,p,a);

			bc.moveRobot(p*1000,a);
			bc.getAngles();
			fprintf(filep,"odometry pose X= %f Y= %f A= %f translation= %f\n",x,y,th,(sqrt(pow(x-x1,2)+pow(y-y1,2)))/1000);
//			ArUtil::sleep(100);
			
			int k;
			printf("next node?\n");
//			cin>>k;
			x1 = x;
			y1 = y;
			fclose(filep);
//			scanf("%d",&k);
			k = 1;
			if(k==1){
				r++;
				q=0;
			}
			count++;
			
			}	

		}
	}
	}
	return bc.disconnect();
}
