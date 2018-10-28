#include "Aria.h"
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

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

//obtiene la celda correspondiente a un punto en el plano del ambiente
int getPntObs (int array[], int sizeA, int pnt)		//(arreglo de x/y, tamaño de x/y, coordenada en x/y)
{
    for (int i = 0; i < sizeA; i++)
    {
        if (abs(array[i]-pnt) <= 250)				//el punto pertenece a la celda si la resta entre el punto y la mitad de la celda es menor a 250 
        {
            return array[i];
        }
        
    }
}

// Declaracion de variables
const int xMax = 8500;						 //tamaño del mapa en x
const int yMax = 4000;						 //tamaño del mapa en y
const int xSize = xMax/500;					 //numero de celdas en x
const int ySize = yMax/500;					 //numero de celdas en y
const int qi[2] = {600,3300};				 //coordenadas punto inicial
const int qf[2] = {5100,300};				 //coordenadas punto final
const int thi = -90;						 //angulo inicial
const int thf = -90;						 //angulo final
const int nObs = 2;							 //numero de obstaculos
const int filas = nObs * 4;					 //puntos totales de obstaculos (2 x obs)
int pntsObs[filas][2] = {{0,4200},{0,900},{5700,8000},{600,3900},{0,600},{900,2100},{5100,5700},{2700,3900}};  //matriz de puntos de los obs 
//Puntos obstaculos
//[[obs1px1  obs1px2],
// [obs1py1  obs1py2],
// [obs2px1  obs2px2],
// [obs2py1  obs2py2]]

int* getAxe (int sizeA)						 //calcula las celdas en x y y
{
	int* coorE = new int[sizeA];

	for (int i=0; i < sizeA; i++)			 
		{
			coorE[i] = 250 + i*500;			 //celdas separadas cada 500 mm iniciando desde 250 mm
		}
	return coorE;
}

//getObs calcula las celdas de los puntos de los obstaculos
int ** getObs (int matrixOP[filas][2], int filasM, int arrayX[], int arrayY[], int szX, int szY)		
{
    const int nFilas = filasM;
    int** mCeldasObs = 0;
    mCeldasObs = new int*[nFilas];
    
    for (int i = 0; i < nFilas; i++)		 //itera sobre los puntos de los obstaculos 
    {
        mCeldasObs[i] = new int[2];
        
        for (int j = 0; j < 2; j++)			 //itera sobre los puntos en x o y de un sólo obstaculos
        {
            if (i==0 || i % 2 == 0)          //calcula la celda para las x (siempre estan en filas pares)
            {
                mCeldasObs[i][j] = getPntObs(arrayX, szX, matrixOP[i][j]);			
            }
            else							 //calcula la celda para las y (siempre estan en filas impares)
            {
                mCeldasObs[i][j] = getPntObs(arrayY, szX, matrixOP[i][j]);
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

//cambio de coordenadas de la escena al robot
const double thi_r = degr2rad(thi);
const double xt = qi[0]*cos(thi_r) - qi[1]*sin(thi_r);		//traslacion en x 
const double yt = qi[0]*sin(thi_r) + qi[1]*cos(thi_r);		//traslacion en y

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


int main(int argc, char **argv)
{
	int* xCoorE = getAxe(xSize);			//calculo celdas del eje x
	int* yCoorE = getAxe(ySize);			//calculo celdas del eje y
	
	// calculo de los qi y qf en la matriz
	const int qixM = (getPntObs (xCoorE, xSize, qi[0])-250)/500;
    const int qiyM = ySize-1-(getPntObs (yCoorE, ySize, qi[1])-250)/500;        //ajuste en y para visualizar en la matriz
    
    const int qfxM = (getPntObs (xCoorE, xSize, qf[0])-250)/500;
    const int qfyM = ySize-1-(getPntObs (yCoorE, ySize, qf[1])-250)/500;        //ajuste en y para visualizar en la matriz

    int** mCObjetos = getObs(pntsObs,filas,xCoorE,yCoorE,xSize,ySize);			//matriz con puntos de obstaculos como celdas
  
    int cSpace[ySize][xSize] = {}; // initialized to 0.0
    
   
    for (int icO = 0; icO < xSize; icO++) // correr sobre las x
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
    
    for (int icS = 0; icS < ySize; icS++)
    {
      for (int jcS = 0; jcS < xSize; jcS++)
        {
          cout << cSpace[icS][jcS] << " ";
        }
        cout << endl;
    }
    
    //cout << getCostMnht(qixM,qiyM,qfxM,qfyM) << endl;
    
    //calculo del camino desde qi hasta qf
    int xAdv = qixM;
    int yAdv = qiyM;
    const int nVecinos = 8;
    int minD = 100;
    bool posibleChoque = false;
    int dist, bestX, bestY;
    vector<int> xPath, yPath;
    cout<< "qf (" << qfxM << ", " << qfyM << ")" << endl;
    
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
		robot.unlock();

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

	Aria::exit(0);
  return 0;
}
