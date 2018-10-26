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

int getPntObs (int array[], int sizeA, int pnt)		//obtiene la celda correspondiente a un punto en el plano del ambiente
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


int main(int argc, char **argv)
{
	int* xCoorE = getAxe(xSize);			//calculo celdas del eje x
	int* yCoorE = getAxe(ySize);			//calculo celdas del eje y

    int** mCObjetos = getObs(pntsObs,filas,xCoorE,yCoorE,xSize,ySize);			//matriz con puntos de obstaculos como celdas
  
    int cSpace[ySize][xSize] = {}; // initialized to 0.0
    
   
    for (int icO = 0; icO < xSize; icO++) // correr sobre las x
    {
        
        for (int icS = 0; icS < filas; icS+= 2) // correr sobre los obstáculos
        {
            cout << "menor " << mCObjetos[icS][0] << " mayor " << mCObjetos[icS][1] << endl;
            if (xCoorE[icO] >= mCObjetos[icS][0] && xCoorE[icO] <= mCObjetos[icS][1])
            {
                cout << xCoorE[icO] << endl;
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
            
       
    for (int icS = 0; icS < ySize; icS++)
    {
      for (int jcS = 0; jcS < xSize; jcS++)
        {
          cout << cSpace[icS][jcS] << "   ";
        }
        cout << endl;
    }
  return 0;
}
