#include "Aria.h"
#include <iostream>

using namespace std;

double* changeCoordinates (double xE, double yE, double thE)
{
	double xR, yR, thR;
	xR = yE - 450.0;
	yR = -xE + 450.0;
	thR = thE - 90.0;
	double coordinatesR[3] = {xR,yR,thR};
	
	return coordinatesR;
}

double getX (double yE)
{
	double xR = yE - 450.0;
	return xR;
}

double getY (double xE)
{
	double yR = -xE + 450.0;
	return yR;
}

double getTh (double thE)
{
	double thR = thE - 90.0;
	return thR;
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

int getCostMnht(int xcP1, int ycP1, int xcP2, int ycP2)
{
	int distMnht = abs(xcP1 - xcP2) + abs(ycP1 - ycP2);
	return distMnht;
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
    cout<< "qf (" << qfxM << ", " << qfyM << ")" << endl;
    
    while (xAdv != qfxM || yAdv != qfyM)
    {
        int xVecinos[nVecinos] = {xAdv, xAdv-1, xAdv, xAdv+1, xAdv+1, xAdv-1, xAdv-1, xAdv+1};
        int yVecinos[nVecinos] = {yAdv+1, yAdv, yAdv-1, yAdv, yAdv+1, yAdv+1, yAdv-1, yAdv-1};
        
        for (int v = 0; v < nVecinos; v++)
        {
			if (xVecinos[v]>=0 && xVecinos[v]<xSize && yVecinos[v]>=0 && yVecinos[v]<ySize)
            {
                if (v>3)
                {
                    if (cSpace[yVecinos[v]][xAdv] == 1 || cSpace[yAdv][xVecinos[v]])
                    {
                        posibleChoque = true;
                    }
                }
                
                dist = getCostMnht(xVecinos[v], yVecinos[v], qfxM, qfyM);
                //cout<< "vecino" << v << " (" << xVecinos[v] << ", " << yVecinos[v] << ") distancia " << dist << " celda " << cSpace[yVecinos[v]][xVecinos[v]] << endl;
                if (minD > dist && cSpace[yVecinos[v]][xVecinos[v]]!=1 && posibleChoque == false)
                {
                    minD = dist;
                    bestX = xVecinos[v];
                    bestY = yVecinos[v];
                }
				posibleChoque = false;
                
            }
        }
        
        xAdv = bestX;
        yAdv = bestY;
        cSpace[bestY][bestX] = 2;
        //cout<< "(" << bestX << ", " << bestY << ")" << endl;
        
    } 
    
    for (int icS = 0; icS < ySize; icS++)
    {
      for (int jcS = 0; jcS < xSize; jcS++)
        {
          cout << cSpace[icS][jcS] << " ";
        }
        cout << endl;
    }
    
  return 0;
}
