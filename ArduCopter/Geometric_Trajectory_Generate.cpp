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

void Trajectory_Generate_SINWAVE(float timeInThisRun,
                             Vector3f *targetPos, 
                             Vector3f *targetVel, 
                             Vector3f *targetAcc, 
                             Vector3f *targetJerk, 
                             Vector3f *targetSnap, 
                             Vector2f *targetYaw, 
                             Vector2f *targetYaw_dot, 
                             Vector2f *targetYaw_ddot){
    float PolyCoef[8] = {0, 0, 0, 0, 0, 0, -1, 0};
    if (timeInThisRun >=0 ) {
        *targetPos = (Vector3f){ -polyEval(PolyCoef, timeInThisRun, 8),cosf(timeInThisRun)-1, -1};
        
        *targetVel = (Vector3f){ -polyDiffEval(PolyCoef, timeInThisRun, 8),-sinf(timeInThisRun), 0};

        *targetAcc = (Vector3f){ -polyDiff2Eval(PolyCoef, timeInThisRun, 8),-cosf(timeInThisRun), 0};

        *targetJerk = (Vector3f){-polyDiff3Eval(PolyCoef, timeInThisRun, 8),sinf(timeInThisRun), 0 };

        *targetSnap = (Vector3f){ -polyDiff4Eval(PolyCoef, timeInThisRun, 8),cosf(timeInThisRun), 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};                             
    }                                

}

void Trajectory_Generate_BIGSINWAVE(float timeInThisRun,
                             Vector3f *targetPos, 
                             Vector3f *targetVel, 
                             Vector3f *targetAcc, 
                             Vector3f *targetJerk, 
                             Vector3f *targetSnap, 
                             Vector2f *targetYaw, 
                             Vector2f *targetYaw_dot, 
                             Vector2f *targetYaw_ddot){
    float PolyCoef[8] = {0, 0, 0, 0, 0, 0, -1, 0};
    float scale_index=2;
    float time_factor=0.5;
    if (timeInThisRun >=0 ) {
        *targetPos = (Vector3f){ -polyEval(PolyCoef, timeInThisRun, 8),scale_index*(cosf(timeInThisRun*time_factor)-1), -1};
        
        *targetVel = (Vector3f){ -polyDiffEval(PolyCoef, timeInThisRun, 8),-sinf(timeInThisRun*time_factor)*scale_index, 0};

        *targetAcc = (Vector3f){ -polyDiff2Eval(PolyCoef, timeInThisRun, 8),-cosf(timeInThisRun*time_factor)*scale_index, 0};

        *targetJerk = (Vector3f){-polyDiff3Eval(PolyCoef, timeInThisRun, 8),sinf(timeInThisRun*time_factor)*scale_index, 0 };

        *targetSnap = (Vector3f){ -polyDiff4Eval(PolyCoef, timeInThisRun, 8),cosf(timeInThisRun*time_factor)*scale_index, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};                             
    }                                

}

void Trajectory_Generate_CIRCLE(float timeInThisRun,
                             Vector3f *targetPos, 
                             Vector3f *targetVel, 
                             Vector3f *targetAcc, 
                             Vector3f *targetJerk, 
                             Vector3f *targetSnap, 
                             Vector2f *targetYaw, 
                             Vector2f *targetYaw_dot, 
                             Vector2f *targetYaw_ddot){
    int8_t r_circle = 20;
    float T_circle = 100;
    float w_circle = 2*M_PI/T_circle;
    if (timeInThisRun >=0 ) {
        *targetPos = (Vector3f){sinf(w_circle*timeInThisRun)*r_circle,-r_circle+r_circle*cosf(w_circle*timeInThisRun),-1};
        
        *targetVel = (Vector3f){cosf(w_circle*timeInThisRun)*r_circle*w_circle,-sinf(w_circle*timeInThisRun)*r_circle*w_circle,0 };

        *targetAcc = (Vector3f){-sinf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle, -cosf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle,0};

        *targetJerk = (Vector3f){-cosf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle*w_circle,sinf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle*w_circle,0 };

        *targetSnap = (Vector3f){sinf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle*w_circle*w_circle,cosf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle*w_circle*w_circle, 0};

        *targetYaw = (Vector2f){1, 0};
        *targetYaw_dot = (Vector2f){0, 0};
        *targetYaw_ddot = (Vector2f){0, 0};                             
    }
                                     
}

void Trajectory_Generate_EIGHT(float timeInThisRun,
                             Vector3f *targetPos, 
                             Vector3f *targetVel, 
                             Vector3f *targetAcc, 
                             Vector3f *targetJerk, 
                             Vector3f *targetSnap, 
                             Vector2f *targetYaw, 
                             Vector2f *targetYaw_dot, 
                             Vector2f *targetYaw_ddot){
    int8_t r_circle = 20;
    float T_circle = 40;
    float w_circle = 2*M_PI/T_circle;
    int8_t left_or_right=0;//left  1  right  -1
    if(sinf(w_circle*timeInThisRun*0.5)>=0){
        left_or_right=1;
    }
    else if(sinf(w_circle*timeInThisRun*0.5)<0){
        left_or_right=-1;
    }

    *targetPos = (Vector3f){sinf(w_circle*timeInThisRun)*r_circle,left_or_right*(-r_circle+r_circle*cosf(w_circle*timeInThisRun)),-1};
        
    *targetVel = (Vector3f){cosf(w_circle*timeInThisRun)*r_circle*w_circle,left_or_right*(-sinf(w_circle*timeInThisRun)*r_circle*w_circle),0 };

    *targetAcc = (Vector3f){-sinf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle, left_or_right*(-cosf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle),0};

    *targetJerk = (Vector3f){-cosf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle*w_circle,left_or_right*(sinf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle*w_circle),0 };

    *targetSnap = (Vector3f){sinf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle*w_circle*w_circle,left_or_right*(cosf(w_circle*timeInThisRun)*r_circle*w_circle*w_circle*w_circle*w_circle), 0};

    *targetYaw = (Vector2f){1, 0};
    *targetYaw_dot = (Vector2f){0, 0};
    *targetYaw_ddot = (Vector2f){0, 0}; 


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
        
        *targetPos = (Vector3f){0,0, 0};
        
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
