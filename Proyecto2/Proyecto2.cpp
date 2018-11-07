#include "Aria.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

using namespace std;

int xMax;						 //tamaño del mapa en x
int yMax;						 //tamaño del mapa en y
int xSize;					 //numero de celdas en x
int ySize;					 //numero de celdas en y
vector<int> qi;				 //coordenadas punto inicial
vector<int> qf;				 //coordenadas punto final
vector<int> qLm;				 //coordenadas punto final
vector<int> qLf;				 //coordenadas punto final
int thi;						 //angulo inicial
int thf;						 //angulo final
int thLm;
int thLf;
int nObs;							 //numero de obstaculos
int filas;
vector<int> obsX, obsY;	
const int colMatrix = 2;

//funciones basicas
const double PI = 3.1416;
double degr2rad (int degree)	//transformacion de grados a radianes
{
	double radians = double (degree) * PI/180.0;
	return radians;
}

double* changeCoordinates (double xE, double yE, double thE)
{
	double xR, yR, thR;
	xR = yE - 450.0;
	yR = -xE + 450.0;
	thR = thE - 90.0;
	double coordinatesR[3] = {xR,yR,thR};
	
	return coordinatesR;
}

vector <int> getAxe (int sizeA)						 //calcula las celdas en x y y
{
	vector <int> coorE;

	for (int i=0; i < sizeA; i++)			 
		{
			coorE.push_back(250 + i*500);			 //celdas separadas cada 500 mm iniciando desde 250 mm
		}
	return coorE;
}

//obtiene la celda correspondiente a un punto en el plano del ambiente
int getPntObs (vector<int> arrayW, int sizeA, int pnt)		//(arreglo de x/y, tamaño de x/y, coordenada en x/y)
{
    for (int i = 0; i < sizeA; i++)
    {
        if (abs(arrayW[i]-pnt) <= 250)				//el punto pertenece a la celda si la resta entre el punto y la mitad de la celda es menor a 250 
        {
            return arrayW[i];
        }
        
    }
}

//getObs calcula las celdas de los puntos de los obstaculos
vector <vector <int>> getObs (vector <vector <int>> matrixOP, int filasM, vector <int> arrayX, vector <int> arrayY, int szX, int szY)		
{
	vector <vector <int>> mCeldasObs (filasM, vector<int>(2));
            
    for (int i = 0; i < filasM; i++)		 //itera sobre los puntos de los obstaculos 
    {
        for (int j = 0; j < 2; j++)			 //itera sobre los puntos en x o y de un sólo obstaculos
        {
            if (i==0 || i % 2 == 0)          //calcula la celda para las x (siempre estan en filas pares)
            {
                mCeldasObs[i][j] = getPntObs(arrayX, szX, matrixOP[i][j]);			
            }
            else							 //calcula la celda para las y (siempre estan en filas impares)
            {
                mCeldasObs[i][j] = getPntObs(arrayY, szY, matrixOP[i][j]);
            }
		}
    }
    return mCeldasObs;
}

int getCostMnht(int xcP1, int ycP1, int xcP2, int ycP2)		//calcula la funcion costo (distancia Manhattan)
{
	int distMnht = abs(xcP1 - xcP2) + abs(ycP1 - ycP2);
	return distMnht;
}

double getX (int xE, int yE, double theta)		//calcula la coordenada x en el sistema del robot
{
	//double xR = xE*cos(-theta) - yE*sin(-theta) + xt;
	double xR = (xE-qi[0])*cos(theta) + (yE-qi[1])*sin(theta);
	return xR;
}

double getY (int xE, int yE, double theta)		//calcula la coordenada y en el sistema del robot
{
	//double yR = xE*sin(-theta) + yE*cos(-theta) + yt;
	double yR = -(xE-qi[0])*sin(theta) + (yE-qi[1])*cos(theta);
	return yR;
}

double getTh (double thE)
{
	double thR = thE - thi;
	return thR;
}

//definicion de rotacion para correccion
double defRot (double distED, double distEI, double rotVlc)	//recibe la distacia del extremo derecho y la del extremo izquierdo
{
	double rotation;

	if (distED>distEI && abs(distED-distEI) > 0.02)
	{
		rotation = rotVlc;
	}
	else if (distED < distEI && abs(distED-distEI) > 0.02)
	{
		rotation = -rotVlc;
	}
	else
	{
		rotation = 0;
	}

	return rotation;
}

int main(int argc, char **argv)
{
	// LECTURA DE ARCHIVO DE CONFIGURACION .TXT
	string header;
	string espacio;
	string coorx;
	string coory;
	string theta;
	string numObs;
	int lines=0;
	
	ifstream ip("data.txt");	//objeto ip para manipular el archivo
	if (!ip.is_open()) cout << "El archivo no existe" << endl;
	
	while (ip.good())
	{
		if (lines == 0)
		{
			getline(ip,header,':');
			getline(ip,espacio,' ');
			getline(ip,coorx,',');
			getline(ip,coory,'\n');

			xMax = stoi(coorx);
			yMax = stoi(coory);
		}
		else if (lines < 5)
		{
			getline(ip,header,':');
			getline(ip,espacio,' ');
			getline(ip,coorx,',');
			getline(ip,coory,',');
			getline(ip,theta,'\n');
		
			switch (lines){
				case 1:
					qi.push_back(stoi(coorx));
					qi.push_back(stoi(coory));				         //coordenadas punto inicial
					thi = stoi(theta);								 //angulo inicial
					break;
				case 2:
					qf.push_back(stoi(coorx));
					qf.push_back(stoi(coory));						 //coordenadas punto final
					thf = stoi(theta);								 //angulo final
					break;
				case 3:
					qLm.push_back(stoi(coorx));
					qLm.push_back(stoi(coory));						 //coordenadas punto final
					thLm = stoi(theta);								 //angulo final corredor 1
					break;
				case 4:
					qLf.push_back(stoi(coorx));
					qLf.push_back(stoi(coory));						 //coordenadas punto final
					thLf = stoi(theta);								 //angulo final corredor 2
					break;
				default:
					cout << "Error asignando variables" << endl;
					break;
			}
		}
		else if (lines == 5)
		{
			getline(ip,header,':');
			getline(ip,espacio,' ');
			getline(ip,numObs,'\n');
			nObs = stoi(numObs);
			filas = nObs * 2;
		}
		else if (lines > 5)
		{
			getline(ip,header,':');
			getline(ip,espacio,' ');
			getline(ip,coorx,',');
			getline(ip,coory,'\n');
			
			obsX.push_back(stoi(coorx));
			obsY.push_back(stoi(coory));
		}
		else
		{
			break;
		}
		lines ++;
	}

	cout << "CONFIGURACION --------" << endl;
	cout << "tamano " << xMax << " x " << yMax << endl;
	cout << "qi " << qi[0] << ", " << qi[1] << ", " << thi << endl;
	cout << "qf " << qf[0] << ", " << qf[1] << ", " << thf << endl;
	cout << "qLm " << qLm[0] << ", " << qLm[1] << ", " << thLm << endl;
	cout << "qLf " << qLf[0] << ", " << qLf[1] << ", " << thLf << endl;
	cout << "numero de obstaculos " << nObs << endl;
	cout << "primer obs " << obsX[0] << ", " << obsY[0] << endl;
	cout << "ultimo obs " << obsX[7] << ", " << obsY[7] << endl;
	cout << "----------------------" << endl;

	//areglo de obstaculos al formato usado
	vector <vector <int>> pntsObs (filas, vector<int>(colMatrix));
	for (int i = 0; i < filas; i+=2)
    {
		pntsObs[i][0] = obsX[i];
		pntsObs[i][1] = obsX[i+1];
	}

	for (int i = 1; i < filas; i+=2)
    {
		pntsObs[i][0] = obsY[i-1];
		pntsObs[i][1] = obsY[i];
	}
	
	// CREACION DEL ESPACIO DE CONFIGURACION
	const int xSize = ceil(double(xMax)/500);					 //numero de celdas en x
	const int ySize = ceil(double(yMax)/500);					 //numero de celdas en y
	vector<int> xCoorE = getAxe(xSize);			//calculo celdas del eje x
	vector<int> yCoorE = getAxe(ySize);			//calculo celdas del eje y
	
	// calculo de los qi y qf en la matriz
	const int qixM = (getPntObs(xCoorE, xCoorE.size(), qi[0])-250)/500;
	const int qiyM = ySize-1-(getPntObs (yCoorE, ySize, qi[1])-250)/500;        //ajuste en y para visualizar en la matriz
    const int qfxM = (getPntObs (xCoorE, xSize, qf[0])-250)/500;
    const int qfyM = ySize-1-(getPntObs (yCoorE, ySize, qf[1])-250)/500;        //ajuste en y para visualizar en la matriz

	vector <vector <int>> mCObjetos = getObs(pntsObs,filas,xCoorE,yCoorE,xSize,ySize);			//matriz con puntos de obstaculos como celdas
	
	vector <vector <int>> cSpace(yCoorE.size(), vector<int>(xCoorE.size(), 0)); // initialized to 0.0

	for (int icO = 0; icO < xCoorE.size(); icO++) // correr sobre las x
    {
        for (int icS = 0; icS < filas; icS+= 2) // correr sobre los obstáculos
        {
            if (xCoorE[icO] >= mCObjetos[icS][0] && xCoorE[icO] <= mCObjetos[icS][1])
            {
                for (int icOy = 0; icOy < ySize;  icOy++)
                {
                    if (yCoorE[icOy]  >= mCObjetos[icS+1][0] && yCoorE[icOy]  <= mCObjetos[icS+1][1])
                    {
                        cSpace[ySize-1-icOy][icO] = 1;
                    }
                }
            }            
        }
    }
            
    cSpace[qiyM][qixM] = 5;         //visaulizacion qi
    cSpace[qfyM][qfxM] = 7;         //visaulizacion qf
    
    for (int icS = 0; icS < ySize; icS++)		//visualizacion del espacio de configuracion en consola
    {
      for (int jcS = 0; jcS < xSize; jcS++)
        {
          cout << cSpace[icS][jcS] << " ";
        }
        cout << endl;
    }

	//PLANEACIÓN DE CAMINO
	//calculo del camino desde qi hasta qf
    int xAdv = qixM;
    int yAdv = qiyM;
    const int nVecinos = 8;
    int minD = 100;
    bool posibleChoque = false;
    int dist, bestX, bestY;
    vector<int> xPath, yPath;
    cout << endl;
    
    while (xAdv != qfxM || yAdv != qfyM)
    {
        int xVecinos[nVecinos] = {xAdv, xAdv-1, xAdv, xAdv+1, xAdv+1, xAdv-1, xAdv-1, xAdv+1};
        int yVecinos[nVecinos] = {yAdv+1, yAdv, yAdv-1, yAdv, yAdv+1, yAdv+1, yAdv-1, yAdv-1};
        
        for (int v = 0; v < nVecinos; v++)
        {
			if (xVecinos[v]>=0 && xVecinos[v]<xSize && yVecinos[v]>=0 && yVecinos[v]<ySize)
            {
                if (v>3)		//revisa vecinos en diagonal
                {
					//si los vecinos de la diagonal tienen vecinos horizontales o verticales ocupados
					//hay un posible choque que impide seguir la ruta en diagonal
                    if (cSpace[yVecinos[v]][xAdv] == 1 || cSpace[yAdv][xVecinos[v]])
                    {
                        posibleChoque = true;
                    }
                }
                
                dist = getCostMnht(xVecinos[v], yVecinos[v], qfxM, qfyM);	//calculo de la funcion costo
                
				if (minD > dist && cSpace[yVecinos[v]][xVecinos[v]]!=1 && posibleChoque == false)
                {
					// si la distancia del vecino revisado es menor, se guarda el vecino
                    minD = dist;
                    bestX = xVecinos[v];
                    bestY = yVecinos[v];
                }
				posibleChoque = false;		//se reinicia la bandera de choque, el cambia aplica solo a los vecinos de la diagonal
                
            }
        }
        
		//al revisar los vecinos se guarda el que tiene menor distancia (mas cerca al objetivo) guarda la posicion en la matriz
        xAdv = bestX;		
        yAdv = bestY;		
        cSpace[bestY][bestX] = 2;
        xPath.push_back(xCoorE[bestX]);		//se guardan las coordenadas del ambiente
        yPath.push_back(yCoorE[ySize-1-bestY]);
        //cout<< "(" << bestX << ", " << bestY << ")" << endl;
        
    } 
    
	//visualizacion de la ruta
    for (int icS = 0; icS < ySize; icS++)
    {
      for (int jcS = 0; jcS < xSize; jcS++)
        {
          cout << cSpace[icS][jcS] << " ";
        }
        cout << endl;
    }

	//transformacion de las coordenadas del camino al sistema del robot
	
	//cambio de coordenadas de la escena al robot
	const double thi_r = degr2rad(thi);
	const double thf_r = degr2rad(thf);
	const double thLm_r = degr2rad(thLm);
	const double thLf_r = degr2rad(thLf);
	const double xt = qi[0]*cos(thi_r) - qi[1]*sin(thi_r);		//traslacion en x 
	const double yt = qi[0]*sin(thi_r) + qi[1]*cos(thi_r);		//traslacion en y
	
	vector<double> xR, yR;
    for (int ixP = 0; ixP < xPath.size(); ixP++)
    {
		xR.push_back(getX(xPath[ixP],yPath[ixP],thi_r));
		yR.push_back(getY(xPath[ixP],yPath[ixP],thi_r));
		cout<< "(" << xPath[ixP] << ", " << yPath[ixP] << ") -> " <<  "(" << getX(xPath[ixP],yPath[ixP],thi_r) << ", " << getY(xPath[ixP],yPath[ixP],thi_r) << ")" << endl;
    }
	xR.at(xPath.size()-1) = getX(qf[0],qf[1],thi_r);	//coloca punto final en los puntos por los que el robot debe pasar
	yR.at(yPath.size()-1) = getY(qf[0],qf[1],thi_r);
	cout<< "Punto objetivo coordenas del robot: (" << xR[xPath.size()-1] << ", " << yR[yPath.size()-1] << ")" << endl;

	//MOVIMIENTO DEL ROBOT
	Aria::init();
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobot robot;
	//ArAnalogGyro gyro(&robot);
	//ArSonarDevice sonar;
	ArRobotConnector robotConnector(&parser, &robot);
	ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
    
	// Connect to the robot, get some initial data from it such as type and name,
	// and then load parameter files for this robot.
	if(!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "gotoActionExample: Could not connect to the robot.");
		if(parser.checkHelpAndWarnUnparsed())
		{
			// -help not given
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	if(!laserConnector.connectLasers())
	{
		ArLog::log(ArLog::Terse, "Could not connect to configured lasers. Exiting.");
		Aria::exit(3);
		return 3;
	}
    // Allow some time to read laser data
	ArUtil::sleep(500);
	ArLog::log(ArLog::Normal, "Connected to all lasers.");
	
	Aria::init();

	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
	}

	ArLog::log(ArLog::Normal, "gotoActionExample: Connected to robot.");

	//robot.addRangeDevice(&sonar);
	robot.runAsync(true);

	// Make a key handler, so that escape will shut down the program
	// cleanly
	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	robot.attachKeyHandler(&keyHandler);
	printf("You may press escape to exit\n");

	// Collision avoidance actions at higher priority
	ArActionLimiterForwards limiterAction("speed limiter near", 10, 15, 250);
	ArActionLimiterForwards limiterFarAction("speed limiter far", 30, 60, 400);
	ArActionLimiterTableSensor tableLimiterAction;
	robot.addAction(&tableLimiterAction, 20);
	robot.addAction(&limiterAction, 20);
	robot.addAction(&limiterFarAction, 20);

	// Goto action at lower priority
	ArActionGoto gotoPoseAction("goto");
	robot.addAction(&gotoPoseAction, 50);
  
	// Stop action at lower priority, so the robot stops if it has no goal
	ArActionStop stopAction("stop");
	robot.addAction(&stopAction, 40);

	// turn on the motors, turn off amigobot sounds
	robot.enableMotors();
	robot.comInt(ArCommands::SOUNDTOG, 0);
   
	
	//Recorrido del camino por puntos calculados
	
	for (int iPath=0; iPath < xR.size(); iPath++)
	{
		//ArLog::log(ArLog::Normal, "---x of robot %f, y of robot %f, theta of robot %f.", coorXR, coorYR, thetaR);
		ArLog::log(ArLog::Normal, "My position %.0f %.0f %.0f",robot.getX(), robot.getY(), robot.getTh());

		robot.lock();
		cout << xR[iPath] << "  " << yR[iPath] << "  "; 
		gotoPoseAction.setGoal(ArPose(xR[iPath], yR[iPath]));
		gotoPoseAction.setSpeed(100.0);
		ArLog::log(ArLog::Normal, "Going to next goal at %.2f %.2f", gotoPoseAction.getGoal().getX(), gotoPoseAction.getGoal().getY());
		robot.unlock();
	  
		while (gotoPoseAction.haveAchievedGoal()==false)
		{
			;
		}

		stopAction.activate();
		//robot.unlock();

		ArLog::log(ArLog::Normal, "Closest distance: %.2f", gotoPoseAction.getCloseDist());
		ArLog::log(ArLog::Normal, "Achieved: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f",
			robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel());
		
		if (robot.getTh() != thf && iPath == xR.size()-1)
		{
			robot.lock();
			ArLog::log(ArLog::Normal, "Angule to change %f.", getTh(thf) - robot.getTh());
			robot.setDeltaHeading(getTh(thf) - robot.getTh());
			robot.unlock();
			ArUtil::sleep(5000);
			ArLog::log(ArLog::Normal,"waiting 5 seconds 90");
		}

		ArUtil::sleep(1000);
		ArLog::log(ArLog::Normal,"waiting 5 seconds 90");
	}
	
	//Posicion final
	//cout << "Posicion final del robot (" << robot.getX() << ", " << robot.getY() << ", " << robot.getTh() << ")"  << endl;
	//cout << "Posicion final teorica (" << getX(qf[0],qf[1],thf_r) << ", " << getY(qf[0],qf[1],thf_r) << ", " << getTh(thf) << ")" << endl;
	

	//CORRECCION DE POSICION
	//lectura de sensores
	//captura de distancias del sensor
	std::map<int, ArLaser*> *lasers = robot.getLaserMap();
	ArLaser* laser = lasers->begin()->second;
	const double l_rs = 0;	//corrimiento del sensor respecto al centro del robot
	double angle;
	double distances[3];
	int rotVel;
	double xCorrect;
	double yCorrect;
	double thCorrect;
	double rotVelocity;
	double pntsCorrXRC1[20];		//puntos del corredor 1 en coordenadas del robot
	double pntsCorrYRC1[20];	
	double pntsCorrXRC2[30];		//puntos del corredor 2 en coordenadas del robot
	double pntsCorrYRC2[30];
	double smallDist;
	double diffDistances;
	double diffDistancesRot;
	const double stepsLm = (sizeof(pntsCorrXRC1)/sizeof(*pntsCorrXRC1));
	const double stepsLf = (sizeof(pntsCorrXRC2)/sizeof(*pntsCorrXRC2));
	

	//lectura de distancias a 0
	angle = 0;
	ArUtil::sleep(500);
	
	laser->lockDevice();
	distances[0] = laser->currentReadingPolar(angle-2, angle-1) - l_rs;
	distances[1] = laser->currentReadingPolar(angle, angle+1) - l_rs;
	distances[2] = laser->currentReadingPolar(angle+2, angle+3) - l_rs;
	laser->unlockDevice();

	//correccion en y del ambiente
	if (distances[1] - l_rs > 350)
	{
		while (distances[1] - l_rs > 350)
		{
			robot.lock();
			robot.setRotVel(0);
			robot.setVel(50);
			robot.unlock();
			ArUtil::sleep(500);
			laser->lockDevice();
			distances[1] = laser->currentReadingPolar(angle, angle+1) - l_rs;
			laser->unlockDevice();
		}
	}
	else
	{
		robot.lock();
		robot.setTransAccel(50);
		robot.move(distances[1] - l_rs - 350);
		robot.unlock();
		laser->lockDevice();
		distances[1] = laser->currentReadingPolar(angle, angle+1) - l_rs;
		laser->unlockDevice();
		while (robot.isMoveDone()==0)
		{
			;
		}
	}
	ArUtil::sleep(500);
	robot.stop();

	//correccion en rotacion
	rotVel = defRot(distances[0], distances[2], 1.0);
	
	cout << "rotacion " << rotVel << endl;
	while (abs(distances[0]-distances[1]) > 2 || abs(distances[2]-distances[1]) > 2 || distances[1] > 500)
	{
		robot.lock();
		robot.setRotVel(rotVel);

		laser->lockDevice();
		distances[0] = laser->currentReadingPolar(angle-2, angle-1) - l_rs;
		distances[1] = laser->currentReadingPolar(angle, angle+1) - l_rs;
		distances[2] = laser->currentReadingPolar(angle+2, angle+3) - l_rs;
		cout << "lectura " << angle-2 << ": " << distances[0] << " lectura " << angle << ": " << distances[1] << " lectura " << angle+2 << ": " << distances[2] << endl;
		cout << "diferencias " << distances[1]-distances[0] << "  " << distances[2]-distances[0] << endl;
		laser->unlockDevice();

		robot.unlock();
		ArUtil::sleep(500);
	}
	
	// Medición obstáculo derecha del robot
	ArUtil::sleep(500);
	laser->lockDevice();
	angle = -90;
	distances[0] = laser->currentReadingPolar(angle-2, angle+2);
	angle = 0;
	distances[1] = laser->currentReadingPolar(angle-2, angle+2) - l_rs;
	laser->unlockDevice();
	cout << distances[0] << endl;

	//Posicion final
	cout << "Posicion final del robot (" << robot.getX() << ", " << robot.getY() << ", " << robot.getTh() << ")"  << endl;
	cout << "Posicion final teorica (" << getX(qf[0],qf[1],thf_r) << ", " << getY(qf[0],qf[1],thf_r) << ", " << getTh(thf) << ")" << endl;
	xCorrect = getX(4200 + distances[0], distances[1], thi_r);
	yCorrect = getY(4200 + distances[0], distances[1], thi_r);
	thCorrect = getTh(thf);
	cout << "Posicion corregida (" << xCorrect << "," << yCorrect << "," << thCorrect << ")" << endl;
	robot.moveTo(ArPose(xCorrect, yCorrect, thCorrect));
	cout << "Posicion del robot despues del cambio (" << robot.getX() << ", " << robot.getY() << ", " << robot.getTh() << ")"  << endl;
	
	robot.lock();
	robot.setRotAccel(50);
	cout << "angulo de cambio " << getTh(0) - robot.getTh() << endl;
	robot.setDeltaHeading(getTh(0) - robot.getTh());
	robot.unlock();
	while(robot.isHeadingDone()==0)
	{
		;
	}
	
	//MOVIMIENTO POR CORREDOR
	//CORREDOR 1
	//Calculo de los puntos por donde debe pasar
	angle = -90;
	const double qLmXR = getX(qLm[0], qLm[1], thi_r);
	const double qLmYR = getY(qLm[0], qLm[1], thi_r);
	const double lStep = sqrt(pow(qLmXR - robot.getX(),2) + pow(qLmYR - robot.getY(),2))/stepsLm;		//lStep = (getY(qLm[0], qLm[1], thi_r) - yCorrect)/stepsLm;
	
	for (int iPCH = 0; iPCH < stepsLm+1; iPCH++)
	{
		pntsCorrXRC1[iPCH] = xCorrect;
		pntsCorrYRC1[iPCH] = yCorrect+lStep*(iPCH);
		cout << "(" << pntsCorrXRC1[iPCH] << ", " << pntsCorrYRC1[iPCH] << ")" << endl;
	}

	//Medicion de distancias
	ArUtil::sleep(500);
	laser->lockDevice();	distances[0] = laser->currentReadingPolar(angle-2, angle-1);
	distances[1] = laser->currentReadingPolar(angle, angle+1);
	distances[2] = laser->currentReadingPolar(angle+2, angle+3);
	laser->unlockDevice();
	cout << "lectura " << angle-2 << ": " << distances[0] << " lectura " << angle << ": " << distances[1] << " lectura " << angle+2 << ": " << distances[2] << endl;
	cout << "diferencias " << distances[1]-distances[0] << "  " << distances[2]-distances[0] << endl;
	
	
	//Recorrido por los puntos
	for (int iPnts=1 ; iPnts < stepsLm+1 ; iPnts++)
	{	
		ArLog::log(ArLog::Normal, "My position %.0f %.0f %.0f",robot.getX(), robot.getY(), robot.getTh());
		cout<<"Goal "<< pntsCorrXRC1[0] <<" , "<< pntsCorrYRC1[iPnts]<<endl;
		smallDist = pntsCorrYRC1[iPnts] - pntsCorrYRC1[iPnts-1];		
		cout<<"diff "<< smallDist << endl;

		robot.lock();
		robot.setVel(100.0);
		robot.setTransAccel(50);
		robot.move(smallDist);
		robot.unlock();

		while (robot.isMoveDone()==0)
		{
			;
		}
		robot.stop();
		ArUtil::sleep(500);

		//Correccion de giro (despegarse de los obstaculos)
		laser->lockDevice();
		distances[0] = laser->currentReadingPolar(angle-2, angle-1);
		distances[1] = laser->currentReadingPolar(angle, angle+1);
		distances[2] = laser->currentReadingPolar(angle+2, angle+3);
		laser->unlockDevice();
		rotVelocity = defRot(distances[0], distances[2], 5.0);
		diffDistances = (distances[0] - distances[2])/abs(distances[0] - distances[2]);
		diffDistancesRot = diffDistances;
		//cout << "lectura " << angle-2 << ": " << distances[0] << " lectura " << angle << ": " << distances[1] << " lectura " << angle+2 << ": " << distances[2] << endl;
		//cout << "diferencias " << diffDistances << endl;
		cout << "rotacion " << rotVelocity << endl;
		cout << "punto " << iPnts << endl;
		robot.setRotVel(rotVelocity);
		
		while ((diffDistances+diffDistancesRot)!= 0)
		{
			robot.lock();
			robot.setRotVel(rotVelocity);
			
			//ArUtil::sleep(500);
			laser->lockDevice();
			distances[0] = laser->currentReadingPolar(angle-2, angle-1);
			distances[1] = laser->currentReadingPolar(angle, angle+1);
			distances[2] = laser->currentReadingPolar(angle+2, angle+3);
			diffDistancesRot = (distances[0] - distances[2]) / abs(distances[0] - distances[2]);
			//cout << "lectura " << angle-2 << ": " << distances[0] << " lectura " << angle << ": " << distances[1] << " lectura " << angle+2 << ": " << distances[2] << endl;
			//cout << (distances[0] - distances[2]) << endl;
			//ArLog::log(ArLog::Normal, "Diferencias %f ", diffDistancesRot);
			if (diffDistances+diffDistancesRot == 0)
			{
				cout << "diferencias " << (diffDistances) << " " << (diffDistancesRot) << endl;
			}
			laser->unlockDevice();

			robot.unlock();
			
		}
		robot.stop();
		ArUtil::sleep(500);
	}
		
	//CORREDOR 2
	//lectura de distancias a 0
	angle = 0;
	ArUtil::sleep(500);
	
	laser->lockDevice();
	distances[0] = laser->currentReadingPolar(angle-2, angle-1) - l_rs;
	distances[1] = laser->currentReadingPolar(angle, angle+1) - l_rs;
	distances[2] = laser->currentReadingPolar(angle+2, angle+3) - l_rs;
	laser->unlockDevice();

	//correccion en x del ambiente
	if (distances[1] - l_rs > 280)
	{
		while (distances[1]>280)
		{
			robot.lock();
			robot.setRotVel(0);
			robot.setVel(50);
			robot.unlock();
			ArUtil::sleep(500);
			laser->lockDevice();
			distances[1] = laser->currentReadingPolar(angle, angle+1) - l_rs;
			laser->unlockDevice();
		}
	}
	else
	{
		robot.lock();
		robot.setTransAccel(50);
		robot.move(distances[1]-l_rs - 280);
		robot.unlock();
		laser->lockDevice();
		distances[1] = laser->currentReadingPolar(angle, angle+1) - l_rs;
		laser->unlockDevice();
		while (robot.isMoveDone()==0)
		{
			;
		}
	}
	ArUtil::sleep(500);
	robot.stop();
		
	//correccion en rotacion
	rotVel = defRot(distances[0],distances[2],1.0);
	
	cout << "rotacion " << rotVel << endl;
	while (abs(distances[0]-distances[1]) > 1.5 || abs(distances[2]-distances[1]) > 1.5 || distances[1] > 500)
	{
		robot.lock();
		robot.setRotVel(rotVel);

		laser->lockDevice();
		distances[0] = laser->currentReadingPolar(angle-2, angle-1) - l_rs;
		distances[1] = laser->currentReadingPolar(angle, angle+1) - l_rs;
		distances[2] = laser->currentReadingPolar(angle+2, angle+3) - l_rs;
		cout << "lectura " << angle-2 << ": " << distances[0] << " lectura " << angle << ": " << distances[1] << " lectura " << angle+2 << ": " << distances[2] << endl;
		cout << "diferencias " << distances[1]-distances[0] << "  " << distances[2]-distances[0] << endl;
		laser->unlockDevice();

		robot.unlock();
		ArUtil::sleep(500);
	}
	
	// Medición obstáculo derecha del robot
	ArUtil::sleep(500);
	laser->lockDevice();
	angle = -90;
	distances[0] = laser->currentReadingPolar(angle-2, angle+2);
	angle = 0;
	distances[1] = laser->currentReadingPolar(angle-2, angle+2);
	laser->unlockDevice();
	cout << distances[0] << endl;

	//Posicion final
	cout << "Posicion final del robot (" << robot.getX() << ", " << robot.getY() << ", " << robot.getTh() << ")"  << endl;
	cout << "Posicion final teorica (" << getX(qLm[0],qLm[1],thi_r) << ", " << getY(qLm[0],qLm[1],thi_r) << ", " << getTh(thLm) << ")" << endl;
	xCorrect = getX(8530 - distances[1] - l_rs, distances[0], thi_r);
	yCorrect = getY(8530 - distances[1] - l_rs, distances[0], thi_r);
	thCorrect = getTh(thLm_r);
	cout << "Posicion corregida (" << xCorrect << "," << yCorrect << "," << thCorrect << ")" << endl;
	robot.moveTo(ArPose(xCorrect, yCorrect, thCorrect));
	cout << "Posicion del robot despues del cambio (" << robot.getX() << ", " << robot.getY() << ", " << robot.getTh() << ")"  << endl;
	
	robot.lock();
	robot.setRotAccel(50);
	cout << "angulo de cambio " << getTh(thLm) - robot.getTh() << endl;
	robot.setDeltaHeading(getTh(thLf) - robot.getTh());
	robot.unlock();
	while(robot.isHeadingDone()==0)
	{
		;
	}
		
	//Calculo de los puntos por donde debe pasar
	const double qLfXR = getX(qLf[0], qLf[1], thi_r);
	const double qLfYR = getY(qLf[0], qLf[1], thi_r);
	const double lStepLf = sqrt(pow(qLfXR - robot.getX(),2) + pow(qLfYR - robot.getY(),2))/stepsLf;		//(robot.getX() - getX(qLf[0], qLf[1], thi_r))/stepsLf;
	rotVelocity = 0.0;
	angle = -90;

	cout << "Punto final objetivo (" << getX(qLf[0], qLf[1], thi_r) << ", " << getY(qLf[0], qLf[1], thi_r) << ")" << endl;
	cout << lStepLf << endl;

	for (int iPCH = 0; iPCH < stepsLf+1; iPCH++)
	{
		pntsCorrYRC2[iPCH] = robot.getY();
		pntsCorrXRC2[iPCH] = robot.getX()-lStepLf*(iPCH);
		cout << "(" << pntsCorrXRC2[iPCH] << ", " << pntsCorrYRC2[iPCH] << ")" << endl;
	}

	//Medicion de distancias
	ArUtil::sleep(500);
	laser->lockDevice();
	distances[0] = laser->currentReadingPolar(angle-2, angle-1);
	distances[1] = laser->currentReadingPolar(angle, angle+1);
	distances[2] = laser->currentReadingPolar(angle+2, angle+3);
	laser->unlockDevice();
	cout << "lectura " << angle-2 << ": " << distances[0] << " lectura " << angle << ": " << distances[1] << " lectura " << angle+2 << ": " << distances[2] << endl;
	cout << "diferencias " << distances[1]-distances[0] << "  " << distances[2]-distances[0] << endl;
	
	//Recorrido por los puntos
	for (int iPnts=1 ; iPnts < stepsLf+1 ; iPnts++)
	{	
		ArLog::log(ArLog::Normal, "My position %.0f %.0f %.0f",robot.getX(), robot.getY(), robot.getTh());
		cout<<"Goal "<< pntsCorrXRC2[iPnts] <<" , "<< pntsCorrYRC2[iPnts]<<endl;
		smallDist = pntsCorrXRC2[iPnts-1] - pntsCorrXRC2[iPnts];
		cout<< "diff "<< smallDist << endl;

		robot.lock();
		robot.setVel(100.0);
		robot.setTransAccel(50);
		robot.move(smallDist);
		robot.unlock();

		while (robot.isMoveDone()==0)
		{
			;
		}
		robot.stop();
		ArUtil::sleep(500);

		//Correccion de giro (despegarse de los obstaculos)
		laser->lockDevice();
		distances[0] = laser->currentReadingPolar(angle-2, angle-1);
		distances[1] = laser->currentReadingPolar(angle, angle+1);
		distances[2] = laser->currentReadingPolar(angle+2, angle+3);
		laser->unlockDevice();
		rotVelocity = defRot(distances[0], distances[2], 5.0);
		diffDistances = (distances[0] - distances[2])/abs(distances[0] - distances[2]);
		diffDistancesRot = diffDistances;
		//cout << "lectura " << angle-2 << ": " << distances[0] << " lectura " << angle << ": " << distances[1] << " lectura " << angle+2 << ": " << distances[2] << endl;
		//cout << "diferencias " << diffDistances << endl;
		cout << "rotacion " << rotVelocity << endl;
		cout << "punto " << iPnts << endl;
		robot.setRotVel(rotVelocity);
		
		while ((diffDistances+diffDistancesRot)!= 0)
		{
			robot.lock();
			robot.setRotVel(rotVelocity);

			//ArUtil::sleep(500);
			laser->lockDevice();
			distances[0] = laser->currentReadingPolar(angle-2, angle-1);
			distances[1] = laser->currentReadingPolar(angle, angle+1);
			distances[2] = laser->currentReadingPolar(angle+2, angle+3);
			diffDistancesRot = (distances[0] - distances[2]) / abs(distances[0] - distances[2]);
			//cout << "lectura " << angle-2 << ": " << distances[0] << " lectura " << angle << ": " << distances[1] << " lectura " << angle+2 << ": " << distances[2] << endl;
			//cout << (distances[0] - distances[2]) << endl;
			//ArLog::log(ArLog::Normal, "Diferencias %f ", diffDistancesRot);
			if (diffDistances+diffDistancesRot == 0)
			{
				cout << "diferencias " << (diffDistances) << " " << (diffDistancesRot) << endl;
			}
			laser->unlockDevice();

			robot.unlock();
			
		}
		robot.stop();
		ArUtil::sleep(500);
		
	}

	//Ajuste para llegar al punto
	//Correccion de giro (despegarse de los obstaculos)
	angle = -90;
	laser->lockDevice();
	distances[0] = laser->currentReadingPolar(angle-2, angle-1);
	distances[1] = laser->currentReadingPolar(angle, angle+1);
	distances[2] = laser->currentReadingPolar(angle+2, angle+3);
	laser->unlockDevice();
	rotVelocity = defRot(distances[0], distances[2], 5.0);
	diffDistances = (distances[0] - distances[2])/abs(distances[0] - distances[2]);
	diffDistancesRot = diffDistances;
	//cout << "lectura " << angle-2 << ": " << distances[0] << " lectura " << angle << ": " << distances[1] << " lectura " << angle+2 << ": " << distances[2] << endl;
	//cout << "diferencias " << diffDistances << endl;
	cout << "rotacion " << rotVelocity << endl;
	robot.setRotVel(rotVelocity);
		
	while ((diffDistances+diffDistancesRot)!= 0)
	{
		robot.lock();
		robot.setRotVel(rotVelocity);
		//ArUtil::sleep(500);
		laser->lockDevice();
		distances[0] = laser->currentReadingPolar(angle-2, angle-1);
		distances[1] = laser->currentReadingPolar(angle, angle+1);
		distances[2] = laser->currentReadingPolar(angle+2, angle+3);
		diffDistancesRot = (distances[0] - distances[2]) / abs(distances[0] - distances[2]);
		//cout << "lectura " << angle-2 << ": " << distances[0] << " lectura " << angle << ": " << distances[1] << " lectura " << angle+2 << ": " << distances[2] << endl;
		//cout << (distances[0] - distances[2]) << endl;
		//ArLog::log(ArLog::Normal, "Diferencias %f ", diffDistancesRot);
		if (diffDistances+diffDistancesRot == 0)
		{
			cout << "diferencias " << (diffDistances) << " " << (diffDistancesRot) << endl;
		}
		laser->unlockDevice();

		robot.unlock();
			
	}
	
	//distancia recorrida
	angle = 0;
	ArUtil::sleep(500);
	laser->lockDevice();
	distances[1] = laser->currentReadingPolar(angle, angle+1);
	laser->unlockDevice();
	
	if (distances[1] - l_rs > 900)
	{
		while (distances[1]>900)
		{
			robot.lock();
			robot.setRotVel(0);
			robot.setVel(50);
			robot.unlock();
			ArUtil::sleep(500);
			laser->lockDevice();
			distances[1] = laser->currentReadingPolar(angle, angle+1) - l_rs;
			laser->unlockDevice();
		}
	}
	else
	{
		robot.lock();
		robot.setTransAccel(50);
		robot.move(distances[1]-l_rs - 900);
		robot.unlock();
		laser->lockDevice();
		distances[1] = laser->currentReadingPolar(angle, angle+1) - l_rs;
		laser->unlockDevice();
		while (robot.isMoveDone()==0)
		{
			;
		}
	}

		//ArUtil::sleep(500);
		//ArLog::log(ArLog::Normal,"waiting 5	 seconds 90");

	
	
	
	
	

	Aria::exit(0);
  return 0;
}
