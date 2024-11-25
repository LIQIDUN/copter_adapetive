#include "Geometric_Trajectory_Generate.h"


void Trajectory_Generate_LINE(float timeInThisRun,
                             Vector3f *targetPos, 
                             Vector3f *targetVel, 
                             Vector3f *targetAcc, 
                             Vector3f *targetJerk, 
                             Vector3f *targetSnap, 
                             Vector2f *targetYaw, 
                             Vector2f *targetYaw_dot, 
                             Vector2f *targetYaw_ddot){
    float PolyCoef[8] = {0, 0, 0, 0, 0, 0, -2, 0};
    if (timeInThisRun >=0 ) {
        *targetPos = (Vector3f){ -polyEval(PolyCoef, timeInThisRun, 8),0, -1};
        
        *targetVel = (Vector3f){ -polyDiffEval(PolyCoef, timeInThisRun, 8),0, 0};

        *targetAcc = (Vector3f){ -polyDiff2Eval(PolyCoef, timeInThisRun, 8),0, 0};

        *targetJerk = (Vector3f){-polyDiff3Eval(PolyCoef, timeInThisRun, 8),0, 0 };

        *targetSnap = (Vector3f){ -polyDiff4Eval(PolyCoef, timeInThisRun, 8),0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};                             
    }
}

void Trajectory_Generate_POS(
                             Vector3f *targetPos, 
                             Vector3f *targetVel, 
                             Vector3f *targetAcc, 
                             Vector3f *targetJerk, 
                             Vector3f *targetSnap, 
                             Vector2f *targetYaw, 
                             Vector2f *targetYaw_dot, 
                             Vector2f *targetYaw_ddot){
    
    if (1) {
        
        *targetPos = (Vector3f){0,0, -1};
        
        *targetVel = (Vector3f){ 0,0, 0};

        *targetAcc = (Vector3f){ 0,0, 0};

        *targetJerk = (Vector3f){0,0, 0 };

        *targetSnap = (Vector3f){ 0,0, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};                             
    }
}





float polyEval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N; i++)
    {
        result += polyCoef[i] * powf(x, N - 1 - i);
    }
    return result;
}

float polyDiffEval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 1; i++)
    {
        result += polyCoef[i] * powf(x, N - 2 - i) * (float)(N - 1 - i);
    }
    return result;
}

float polyDiff2Eval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's 2nd derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 2; i++)
    {
        result += polyCoef[i] * powf(x, N - 3 - i) * (float)(N - 1 - i) * (float)(N - 2 - i);
    }
    return result;
}

float polyDiff3Eval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's 3rd derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 3; i++)
    {
        result += polyCoef[i] * powf(x, N - 4 - i) * (float)(N - 1 - i) * (float)(N - 2 - i) * (float)(N - 3 - i);
    }
    return result;
}

float polyDiff4Eval(float polyCoef[], float x, int N)
{
    // Evaluate the polynomial's 4th derivative given by polyCoef at variable x.
    // The order of polynomial is N.
    float result = 0;
    for (int i = 0; i < N - 4; i++)
    {
        result += polyCoef[i] * powf(x, N - 5 - i) * (float)(N - 1 - i) * (float)(N - 2 - i) * (float)(N - 3 - i) * (float)(N - 4 - i);
    }
    return result;
}
