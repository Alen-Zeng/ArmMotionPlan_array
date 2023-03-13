/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    MotionController.h
  * @author  ZengXilang chenmoshaoalen@126.com
  * @brief   Header file 
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#pragma once


/* Includes ------------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#define MaxPointAmount 100    //轨迹点最大数量
#define JointAmount 6         //关节数量
/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单个关节数据包
 * @note 包含的关节数据有：一条轨迹点中的关节位置&该位置关节的速度、电机转速&关节速度的比值
 * 
 */
typedef struct _JointDataPackStructdef
{
    double JointPosition[MaxPointAmount];          //关节位置
    double JointVelocity[MaxPointAmount];          //关节速度，与位置一一对应
    double MotorJointSpeedRatio;    //电机关节速度比（电机转速与关节速度的比值，需要考虑机械结构和减速箱减速比）
}JointDataPackStructdef;


/**
 * @brief 一系列轨迹点的三次插值原式系数
 * @note 把(xk+1-x)^3称为第一项，(x-xk)^3称为第二项，(xk+1-x)称为第三项，(x-xk)称为第四项
 */
typedef struct _InterpolaCoeStructdef
{
    double FirstCoe[MaxPointAmount-1];       //第一项系数
    double SecCoe[MaxPointAmount-1];         //第二项系数
    double ThirdCoe[MaxPointAmount-1];       //第三项系数
    double FourthCoe[MaxPointAmount-1];      //第四项系数
}InterpolaCoeStructdef;


/**
 * @brief 运动控制器
 * 
 */
class MotionControllerClassdef
{
private:
    /* data */
    int PointNum;                        //轨迹点数量
    JointDataPackStructdef JointDataPack[JointAmount];                  //每一个关节的数据
    double TimefromStart[MaxPointAmount];      //到达每个轨迹点的时间点

    /* 三次样条插值中间变量 */
    double Hk[MaxPointAmount-1];               //使用时间间隔进行计算
    double Uk[MaxPointAmount-1];
    double LAMBDAk[MaxPointAmount-1];
    double Dk[MaxPointAmount];
    double Bk[MaxPointAmount];               //三次样条插值的情形I，对角线全为2
    /* 追赶法中间变量 */
    double pk[MaxPointAmount-1];
    double qk[MaxPointAmount];
    double yk[MaxPointAmount];
    /* 三次样条关键系数,（参见石瑞民数值计算page99） */
    double Mk[MaxPointAmount];
public:
    InterpolaCoeStructdef JointInterCoe[JointAmount];                  //每个时间点的各个电机的三次插值化简式系数

    MotionControllerClassdef();
    MotionControllerClassdef(double* _MotorJointSpeedRatios);
    ~MotionControllerClassdef(){};

    void ReceiveTracjectory(double _JointsPosition[JointAmount][MaxPointAmount],double _JointsVelocity[JointAmount][MaxPointAmount],double* _TimefromStart,int _PointNum);
    void Interpolation();
    void ChaseLUFactorization(double* bk,double* ak,double* ck,double* xk,double* dk,int size);
    void PrintInterCoe();
    // void GetCoe(int JointNO,int CoeNO,double* Datapool);
};




/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
